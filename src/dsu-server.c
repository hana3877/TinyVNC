
	// check if we have incoming packets
	struct timeval t = {0};
	fd_set read_fds;
	FD_ZERO(&read_fds);
	FD_SET(server->socket, &read_fds);
	while (1) {
		if (select(server->socket+1, &read_fds, 0, 0, &t) < 0) {
			snprintf(server->lasterrmsg, DSU_ERRMSG_LENGTH, "select failed");
			server->lasterrno = errno;
			return -1;
		}
		if(!FD_ISSET(server->socket, &read_fds)) break;
			
		// receive packet and work on it
		struct sockaddr_in cli_addr;
		socklen_t clilen = sizeof(cli_addr);
		n = recvfrom(server->socket, buffer, BUFSIZE, 0, (struct sockaddr*)&cli_addr, &clilen);
		if (n<0) {
			snprintf(server->lasterrmsg, DSU_ERRMSG_LENGTH, "recvfrom failed");
			server->lasterrno = errno;
			return -1;
		}
//hex_dump((char*)buffer,n,"packet received");
		dsu_process_message(server, buffer, n, (struct sockaddr*)&cli_addr);
	}
	return 0;
}

static u32 dsu_server_transform(int a, int dead, int max, u32 thresh)
{
	if (thresh) a = a - (a % thresh) * (a < 0 ? -1 : 1); // quantize down to threshold quant
	if (abs(a) < dead) a=0;	// set dead zone to zero
	if (max && a > max) a = max;	// clamp between -max and max
	if (max && a < -max) a= -max;
	return a;
}

// all parameters can be NULL except server
int dsu_server_update(struct dsu_server *server, u32 but, circlePosition *posCp, circlePosition *posStk, touchPosition *touch, accelVector *accel, angularRate *gyro)
{
	static u64 lastupdate = 0;
	static int count = 0;
	static struct {
		u32 but;
		int posCp_x;
		int posCp_y;
		int posStk_x;
		int posStk_y;
		int touch_x;
		int touch_y;
		int touch_dx;
		int touch_dy;
		int gyro_x;
		int gyro_y;
		int gyro_z;
		int accel_x;
		int accel_y;
		int accel_z;
	} data = {0};

	// collect data until we trigger an update
	data.but |= but;
	if (posCp) {
		data.posCp_x += posCp->dx;
		data.posCp_y += posCp->dy;
	}
	if (posStk) {
		data.posStk_x += posStk->dx;
		data.posStk_y += posStk->dy;
	}
	if (touch) {
		static int oldx=0, oldy=0, oldstate=0;
		if (but & KEY_TOUCH) {
			if (!oldstate) {
				oldx = touch->px;
				oldy = touch->py;
			}
			data.touch_x += touch->px;
			data.touch_y += touch->py;
			data.touch_dx += touch->px - oldx;
			data.touch_dy += touch->py - oldy;
			oldx = touch->px;
			oldy = touch->py;
		} else {
			data.touch_x += oldx;
			data.touch_y += oldy;
		}
		oldstate = but & KEY_TOUCH;
	}
	if (gyro) {
		data.gyro_x += gyro->x;
		data.gyro_y += gyro->y;
		data.gyro_z += gyro->z;
	}
	if (accel) {
		data.accel_x += accel->x;
		data.accel_y += accel->y;
		data.accel_z += accel->z;
	}
	++count;

	u64 now = getmicrotime();
	if (!lastupdate) lastupdate = now;
	if (now - lastupdate > server->interval) { // done collecting data - now send!
		lastupdate = now;

		static u8 oldacc[12]={0};
		static u64 accts = 0;

		u8 sbuf[100] = {0};
		// Neutral positions for both sticks, including absent O3DS C-Stick.
		// Without this, an absent stick is reported as (0,0), not (128,128).
		sbuf[40] = 128;
		sbuf[41] = 128;
		sbuf[42] = 128;
		sbuf[43] = 128;
		int i;

		*((u32*)(sbuf+16)) = 0x100002;
		dsu_fill_controller_info(0, sbuf + 20);
		
		sbuf[31] = 1;	// Is controller connected (1 if connected, 0 if not)
		//*((u32*)(sbuf+32)) = 0; // packet number for this client - done below in client loop
		// DPAD_LEFT, DPAD_DOWN, DPAD_RIGHT, DPAD_UP, +, RSTCK_PUSH, LSTCK_PUSH, -
		if (data.but & server->button_meta) {
			sbuf[36] =
				(data.but & server->button_plus ? 0x08 : 0) |
				(data.but & server->button_RSTCK_PUSH ? 0x04 : 0) |
				(data.but & server->button_LSTCK_PUSH ? 0x02 : 0) |
				(data.but & server->button_minus ? 0x01 : 0);
		} else {
			sbuf[36] =
				(data.but & KEY_DLEFT ? 0x80 : 0) |
				(data.but & KEY_DDOWN ? 0x40 : 0) |
				(data.but & KEY_DRIGHT ? 0x20 : 0) |
				(data.but & KEY_DUP ? 0x10 : 0);
			sbuf[44] = data.but & KEY_DLEFT ? 255 : 0;	// Analog D-Pad Left (0 or 255)
			sbuf[45] = data.but & KEY_DDOWN ? 255 : 0;	// Analog D-Pad Down (0 or 255)
			sbuf[46] = data.but & KEY_DRIGHT ? 255 : 0;	// Analog D-Pad Right (0 or 255)
			sbuf[47] = data.but & KEY_DUP ? 255 : 0;		// Analog D-Pad Up (0 or 255)
		}
		sbuf[37] =	// Y, B, A, X, R, L, ZR, ZL
			(data.but & KEY_Y ? 0x80 : 0) |
			(data.but & KEY_B ? 0x40 : 0) |
			(data.but & KEY_A ? 0x20 : 0) |
			(data.but & KEY_X ? 0x10 : 0) |
			(data.but & KEY_R ? 0x08 : 0) |
			(data.but & KEY_L ? 0x04 : 0) |
			(data.but & KEY_ZR ? 0x02 : 0) |
			(data.but & KEY_ZL ? 0x01 : 0);
		sbuf[38] = // HOME Button (0 or 1)
			data.but & KEY_SELECT && data.but & KEY_START ? 1 : 0;
		sbuf[39] = // Touch Button (0 or 1)
			data.but & KEY_TOUCH ? 1 : 0;
		if (posCp) {
			// Left stick X (plus rightward, 128=neutral)
			i=dsu_server_transform(data.posCp_x / count, server->coeffs.cp_deadzone, server->coeffs.cp_max, server->coeffs.cp_threshold);
			i=((i+server->coeffs.cp_max) * 128) / server->coeffs.cp_max;
			sbuf[40] = i>255?255:i;
			// Left stick Y (plus upward, 128=neutral)
			i=dsu_server_transform(data.posCp_y / count, server->coeffs.cp_deadzone, server->coeffs.cp_max, server->coeffs.cp_threshold);
			i=((i+server->coeffs.cp_max) * 128) / server->coeffs.cp_max;
			sbuf[41] = i>255?255:i;
		}
		if (posStk) {
			// Right stick X (plus rightward, 128=neutral)
			i=dsu_server_transform(data.posStk_x / count, server->coeffs.cs_deadzone, server->coeffs.cs_max, server->coeffs.cs_threshold);
			i=((i+server->coeffs.cs_max) * 128) / server->coeffs.cs_max;
			sbuf[42] = i>255?255:i;
			// Right stick Y (plus upward, 128=neutral)
			i=dsu_server_transform(data.posStk_y / count, server->coeffs.cs_deadzone, server->coeffs.cs_max, server->coeffs.cs_threshold);
			i=((i+server->coeffs.cs_max) * 128) / server->coeffs.cs_max;
			sbuf[43] = i>255?255:i;
		}
		sbuf[48] = data.but & KEY_Y ? 255 : 0;		// Analog Y (0 or 255)
		sbuf[49] = data.but & KEY_B ? 255 : 0;		// Analog B (0 or 255)
		sbuf[50] = data.but & KEY_A ? 255 : 0;		// Analog A (0 or 255)
		sbuf[51] = data.but & KEY_X ? 255 : 0;		// Analog X (0 or 255)
		sbuf[52] = data.but & KEY_R ? 255 : 0;		// Analog R (0 or 255)
		sbuf[53] = data.but & KEY_L ? 255 : 0;		// Analog L (0 or 255)
		sbuf[54] = data.but & KEY_ZR ? 255 : 0;		// Analog ZR (0 or 255)
		sbuf[55] = data.but & KEY_ZL ? 255 : 0;		// Analog ZL (0 or 255)
		sbuf[56] = data.but & KEY_TOUCH ? 1 : 0;		// first touch? Is touch active (1 if active, else 0)
		if (touch) {
			// Touch id (should be the same for one continious touch)
			static u8 laststate=0, lastid=0;
			const u8 state=sbuf[56];
			if (state && !laststate) do {++lastid;} while (!lastid);
			laststate = state;
			sbuf[57] = state ? lastid : 0;

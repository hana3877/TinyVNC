
/*
Cemuhook Server implementation for N3DS
(c) 2022 Sebastian Weber

https://v1993.github.io/cemuhook-protocol/

Old 3DS compatibility fix:
Initialize both analog sticks to neutral (128,128)
before applying SELECT-based swapping.
*/

#include <3ds.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <math.h>
#include <errno.h>
#include <unistd.h>
#include "dsu-server.h"
#include "utilities.h"

#define BUFSIZE 4096
#define MAX_VERSION 1001
#define DSU_DEFAULT_PORT 26760
#define CLIENT_TIMEOUT 5

static u32 serverID=0;

int dsu_server_init(struct dsu_server *server, int port)
{
    if (port==0) port=DSU_DEFAULT_PORT;

    bzero(server, sizeof(*server));
    struct sockaddr_in servaddr;

    if ((server->socket = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        snprintf(server->lasterrmsg, DSU_ERRMSG_LENGTH, "socket creation failed");
        server->lasterrno = errno;
        return -1;
    }

    bzero(&servaddr, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(port);

    if (bind(server->socket, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        snprintf(server->lasterrmsg, DSU_ERRMSG_LENGTH, "bind failed");
        server->lasterrno = errno;
        return -1;
    }

    if (!serverID) {
        srand(time(NULL));
        while (!serverID)
            serverID = ((rand() & 0x0000FFFF) | (rand() << 16));
    }

    struct in_addr sin_addr;
    sin_addr.s_addr = gethostid();
    inet_ntop(AF_INET, &sin_addr, server->ipstr, INET_ADDRSTRLEN);
    server->port = port;

    server->button_meta = KEY_SELECT;
    server->button_plus = KEY_DUP;
    server->button_minus = KEY_DDOWN;
    server->button_LSTCK_PUSH = KEY_DLEFT;
    server->button_RSTCK_PUSH = KEY_DRIGHT;
    server->interval = 75;

    server->coeffs.cp_deadzone = 20;
    server->coeffs.cp_max = 150;
    server->coeffs.cp_threshold = 0;

    server->coeffs.cs_deadzone = 10;
    server->coeffs.cs_max = 100;
    server->coeffs.cs_threshold = 0;

    server->coeffs.acc_deadzone = 20;
    server->coeffs.acc_threshold = 10;

    server->coeffs.gy_deadzone = 20;
    server->coeffs.gy_threshold = 10;
    HIDUSER_GetGyroscopeRawToDpsCoefficient(&(server->coeffs.gy_raw2dps));

    return 0;
}

int dsu_server_shutdown(struct dsu_server *server)
{
    if (server->socket >= 0) close(server->socket);
    server->socket = -1;
    return 0;
}

unsigned int crc32calc(unsigned char *message, int msglen)
{
    int i, j;
    unsigned int byte, crc, mask;
    static unsigned int table[256];

    if (table[1] == 0) {
        for (byte = 0; byte <= 255; byte++) {
            crc = byte;
            for (j = 7; j >= 0; j--) {
                mask = -(crc & 1);
                crc = (crc >> 1) ^ (0xEDB88320 & mask);
            }
            table[byte] = crc;
        }
    }

    i = 0;
    crc = 0xFFFFFFFF;
    while (i < msglen) {
        crc = (crc >> 8) ^ table[(crc ^ message[i]) & 0xFF];
        i++;
    }
    return ~crc;
}

static void dsu_fill_controller_info(u8 nr, u8 *buf)
{
    buf[0] = nr;

    if (nr==0) {
        buf[1] = 2;
        buf[2] = 2;

        static u8 batt, charge;
        static u64 batttime = 0;
        u64 now = time(NULL);

        if (now - batttime > 5) {
            PTMU_GetBatteryLevel(&batt);
            PTMU_GetBatteryChargeState(&charge);
            batttime = now;
        }

        if (charge) {
            buf[10] = batt==5 ? 0xEF : 0xEE;
        } else {
            buf[10] = batt < 3 ? batt + 1 : batt;
        }
    }
}

static int dsu_send_packet(struct dsu_server *server, struct sockaddr *cli_addr, u8 *buf, int len)
{
    buf[0]='D';
    buf[1]='S';
    buf[2]='U';
    buf[3]='S';

    *((u16*)(buf+4)) = 1001;
    *((u16*)(buf+6)) = len - 16;
    *((u32*)(buf+12)) = serverID;
    *((u32*)(buf+8)) = 0;
    *((u32*)(buf+8)) = crc32calc(buf, len);

    socklen_t clilen = sizeof(struct sockaddr_in);
    int n = sendto(server->socket, buf, len, 0, cli_addr, clilen);

    if (n != len) {
        if (n < 0) {
            snprintf(server->lasterrmsg, DSU_ERRMSG_LENGTH, "sendto: %s", strerror(errno));
            server->lasterrno = errno;
        } else {
            snprintf(server->lasterrmsg, DSU_ERRMSG_LENGTH, "Packet sent incomplete: %d/%d bytes", n, len);
            server->lasterrno = -1;
        }
        return -1;
    }
    return 0;
}

static int dsu_process_message(struct dsu_server *server, u8 *buf, int len, struct sockaddr *cli_addr)
{
    int idx=0;

    if (len < 20) return 0;

    if (buf[0]!='D' || buf[1]!='S' || buf[2]!='U' || buf[3]!='C')
        return 0;
    idx += 4;

    u16 version = *((u16*)(buf+idx));
    if (version > MAX_VERSION) return 0;
    idx += 2;

    u16 length = *((u16*)(buf+idx));
    length += 16;
    if (len < length) return 0;
    len = length;
    idx += 2;

    u32 crc32a = *((u32*)(buf+idx));
    buf[idx++] = 0;
    buf[idx++] = 0;
    buf[idx++] = 0;
    buf[idx++] = 0;

    u32 crc32b = crc32calc(buf, len);
    if (crc32a != crc32b) return 0;

    u32 clientID = *((u32*)(buf+idx));
    idx += 4;

    u32 mType = *((u32*)(buf+idx));
    idx += 4;

    if (mType == 0x100000) {
        u8 sbuf[22] = {0};
        *((u32*)(sbuf+16)) = 0x100000;
        *((u16*)(sbuf+20)) = MAX_VERSION;
        dsu_send_packet(server, cli_addr, sbuf, 22);

    } else if (mType == 0x100001) {

        int numPadRequests = *((int*)(buf+idx));
        idx += 4;

        if (numPadRequests < 1 || numPadRequests > 4)
            return 0;

        for (int i=0; i<numPadRequests; i++) {
            u8 currRequest = buf[idx+i];
            if (currRequest > 3) return 0;
        }

        u8 sbuf[32];
        for (u8 i=0; i<numPadRequests; i++) {
            bzero(sbuf, 32);
            *((u32*)(sbuf+16)) = 0x100001;
            dsu_fill_controller_info(buf[idx+i], sbuf+20);
            dsu_send_packet(server, cli_addr, sbuf, 32);
        }

    } else if (mType == 0x100002) {

        u8 regFlags = buf[idx++];
        if (regFlags == 2) return 0;

        u8 idToReg = buf[idx++];
        if (regFlags == 0 && idToReg != 0)
            return 0;

        struct dsu_client *c;

        for (c=server->cli_first; c!=NULL; c=c->next)
            if (c->id == clientID) break;

        if (!c) {
            c = malloc(sizeof(*c));
            bzero(c, sizeof(*c));
            c->id = clientID;
            c->prev = server->cli_last;

            if (!server->cli_first) {
                server->cli_first = c;
            } else {
                server->cli_last->next = c;
            }
            server->cli_last = c;
        }

        c->timestamp = time(NULL);
        c->addr = *((struct sockaddr_in *)cli_addr);
    }

    return 0;
}

int dsu_server_run(struct dsu_server *server)
{
    int n;
    static u8 buffer[BUFSIZE];

    if (server->socket < 0) return -1;

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

        if (!FD_ISSET(server->socket, &read_fds)) break;

        struct sockaddr_in cli_addr;
        socklen_t clilen = sizeof(cli_addr);

        n = recvfrom(server->socket, buffer, BUFSIZE, 0,
                     (struct sockaddr*)&cli_addr, &clilen);

        if (n < 0) {
            snprintf(server->lasterrmsg, DSU_ERRMSG_LENGTH, "recvfrom failed");
            server->lasterrno = errno;
            return -1;
        }

        dsu_process_message(server, buffer, n,
                            (struct sockaddr*)&cli_addr);
    }

    return 0;
}

static u32 dsu_server_transform(int a, int dead, int max, u32 thresh)
{
    if (thresh)
        a = a - (a % thresh) * (a < 0 ? -1 : 1);

    if (abs(a) < dead) a=0;

    if (max && a > max) a=max;
    if (max && a < -max) a=-max;

    return a;
}

// All parameters can be NULL except server.
int dsu_server_update(struct dsu_server *server, u32 but,
                      circlePosition *posCp,
                      circlePosition *posStk,
                      touchPosition *touch,
                      accelVector *accel,
                      angularRate *gyro)
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

    // Collect controller data.
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

    if (now - lastupdate > server->interval) {
        lastupdate = now;

        static u8 oldacc[12] = {0};
        static u64 accts = 0;

        u8 sbuf[100] = {0};
        int i;

        *((u32*)(sbuf+16)) = 0x100002;
        dsu_fill_controller_info(0, sbuf+20);

        sbuf[31] = 1;

        // FIX: Both sticks must start at neutral.
        // Old 3DS has no physical C-Stick.
        sbuf[40] = 128;
        sbuf[41] = 128;
        sbuf[42] = 128;
        sbuf[43] = 128;

        // D-Pad and SELECT combinations.
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

            sbuf[44] = data.but & KEY_DLEFT ? 255 : 0;
            sbuf[45] = data.but & KEY_DDOWN ? 255 : 0;
            sbuf[46] = data.but & KEY_DRIGHT ? 255 : 0;
            sbuf[47] = data.but & KEY_DUP ? 255 : 0;
        }

        sbuf[37] =
            (data.but & KEY_Y ? 0x80 : 0) |
            (data.but & KEY_B ? 0x40 : 0) |
            (data.but & KEY_A ? 0x20 : 0) |
            (data.but & KEY_X ? 0x10 : 0) |
            (data.but & KEY_R ? 0x08 : 0) |
            (data.but & KEY_L ? 0x04 : 0) |
            (data.but & KEY_ZR ? 0x02 : 0) |
            (data.but & KEY_ZL ? 0x01 : 0);

        sbuf[38] =
            (data.but & KEY_SELECT && data.but & KEY_START) ? 1 : 0;

        sbuf[39] = data.but & KEY_TOUCH ? 1 : 0;

        if (posCp) {
            i = dsu_server_transform(
                data.posCp_x/count,
                server->coeffs.cp_deadzone,
                server->coeffs.cp_max,
                server->coeffs.cp_threshold);

            i = ((i+server->coeffs.cp_max)*128) /
                server->coeffs.cp_max;
            sbuf[40] = i>255 ? 255 : i;

            i = dsu_server_transform(
                data.posCp_y/count,
                server->coeffs.cp_deadzone,
                server->coeffs.cp_max,
                server->coeffs.cp_threshold);

            i = ((i+server->coeffs.cp_max)*128) /
                server->coeffs.cp_max;
            sbuf[41] = i>255 ? 255 : i;
        }

        if (posStk) {
            i = dsu_server_transform(
                data.posStk_x/count,
                server->coeffs.cs_deadzone,
                server->coeffs.cs_max,
                server->coeffs.cs_threshold);

            i = ((i+server->coeffs.cs_max)*128) /
                server->coeffs.cs_max;
            sbuf[42] = i>255 ? 255 : i;

            i = dsu_server_transform(
                data.posStk_y/count,
                server->coeffs.cs_deadzone,
                server->coeffs.cs_max,
                server->coeffs.cs_threshold);

            i = ((i+server->coeffs.cs_max)*128) /
                server->coeffs.cs_max;
            sbuf[43] = i>255 ? 255 : i;
        }

        sbuf[48] = data.but & KEY_Y ? 255 : 0;
        sbuf[49] = data.but & KEY_B ? 255 : 0;
        sbuf[50] = data.but & KEY_A ? 255 : 0;
        sbuf[51] = data.but & KEY_X ? 255 : 0;
        sbuf[52] = data.but & KEY_R ? 255 : 0;
        sbuf[53] = data.but & KEY_L ? 255 : 0;
        sbuf[54] = data.but & KEY_ZR ? 255 : 0;
        sbuf[55] = data.but & KEY_ZL ? 255 : 0;

        sbuf[56] = data.but & KEY_TOUCH ? 1 : 0;

        if (touch) {
            static u8 laststate=0, lastid=0;
            const u8 state=sbuf[56];

            if (state && !laststate) {
                do { ++lastid; } while (!lastid);
            }

            laststate = state;
            sbuf[57] = state ? lastid : 0;

            *((u16*)(sbuf+58)) = data.touch_x/count;
            *((u16*)(sbuf+60)) = data.touch_y/count;
        }

        if (accel) {
            i = dsu_server_transform(
                data.accel_x/count,
                server->coeffs.acc_deadzone,
                0,
                server->coeffs.acc_threshold);
            *((float*)(sbuf+76)) = -(float)i/510.0f;

            i = dsu_server_transform(
                data.accel_y/count,
                server->coeffs.acc_deadzone,
                0,
                server->coeffs.acc_threshold);
            *((float*)(sbuf+80)) = (float)i/510.0f;

            i = dsu_server_transform(
                data.accel_z/count,
                server->coeffs.acc_deadzone,
                0,
                server->coeffs.acc_threshold);
            *((float*)(sbuf+84)) = -(float)i/510.0f;

            if (memcmp(oldacc, sbuf+76, 12)) {
                memcpy(oldacc, sbuf+76, 12);
                accts = now;
            }
            *((u64*)(sbuf+68)) = accts;
        }

        if (gyro) {
            i = dsu_server_transform(
                data.gyro_x/count,
                server->coeffs.gy_deadzone,
                0,
                server->coeffs.gy_threshold);
            *((float*)(sbuf+88)) =
                -(float)i/server->coeffs.gy_raw2dps;

            i = dsu_server_transform(
                data.gyro_z/count,
                server->coeffs.gy_deadzone,
                0,
                server->coeffs.gy_threshold);
            *((float*)(sbuf+92)) =
                -(float)i/server->coeffs.gy_raw2dps;

            i = dsu_server_transform(
                data.gyro_y/count,
                server->coeffs.gy_deadzone,
                0,
                server->coeffs.gy_threshold);
            *((float*)(sbuf+96)) =
                (float)i/server->coeffs.gy_raw2dps;
        }

        // Original SELECT mapping:
        // Circle Pad <-> C-Stick
        // L <-> ZL
        // R <-> ZR
        if (data.but & server->button_meta) {
            u16 swap;

            swap = *((u16*)(sbuf+40));
            *((u16*)(sbuf+40)) = *((u16*)(sbuf+42));
            *((u16*)(sbuf+42)) = swap;

            swap = *((u16*)(sbuf+52));
            *((u16*)(sbuf+52)) = *((u16*)(sbuf+54));
            *((u16*)(sbuf+54)) = swap;

            sbuf[37] =
                (sbuf[37] & 0xf0) |
                ((sbuf[37] & 0x03) << 2) |
                ((sbuf[37] & 0x0c) >> 2);
        }

        count = 0;
        bzero(&data, sizeof(data));

        struct dsu_client *c,*c1;
        u64 tim = time(NULL);

        for (c=server->cli_first; c!=NULL; ) {
            if (tim - c->timestamp > CLIENT_TIMEOUT) {
                if (server->cli_first == c)
                    server->cli_first = c->next;
                if (server->cli_last == c)
                    server->cli_last = c->prev;
                if (c->prev)
                    c->prev->next = c->next;
                if (c->next)
                    c->next->prev = c->prev;

                c1 = c->next;
                free(c);
                c = c1;
                continue;
            }

            *((u32*)(sbuf+32)) = 0;

            if (memcmp(sbuf+20, c->oldpacket, 80)) {
                memcpy(c->oldpacket, sbuf+20, 80);
                *((u32*)(sbuf+32)) = ++(c->packet_count);
                dsu_send_packet(
                    server,
                    (struct sockaddr*)&c->addr,
                    sbuf,
                    100);
            }

            c = c->next;
        }
    }

    return 0;
}

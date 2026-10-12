// pmc_socketcan.c
//
// Minimal raw-CANopen control of a PUSIROBOT PMC006C3 stepper drive over
// SocketCAN. No CANopen stack - just expedited SDO frames built by hand.
//
// Assumes an already-open, already-bound SocketCAN raw socket (sockfd):
//     int sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
//     ioctl(...SIOCGIFINDEX...); bind(...);   // caller's job, not here
//
// Build: gcc -Wall -O2 pmc_socketcan.c -o pmctest   (Linux, needs <linux/can.h>)
//
// Expedited SDO download (write) frame, 8 bytes:
//   [cmd] [idxLSB] [idxMSB] [sub] [d0] [d1] [d2] [d3]
//   cmd by payload length: 1B=0x2F, 2B=0x2B, 3B=0x27, 4B=0x23
//   data little-endian, zero-padded to 4 bytes.
//   reply: cmd 0x60 = success ; cmd 0x80 = abort (+ 4-byte LE abort code).

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>
#include <poll.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#define PMC_SDO_TIMEOUT_MS  500

/* ------------------------------------------------------------------ */
/* Monotonic millisecond clock for a bounded total-wait timeout.       */
/* ------------------------------------------------------------------ */
static long pmc_now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long)ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

/* ------------------------------------------------------------------ */
/* Expedited SDO write + wait for the confirming reply.                */
/* Returns 0 on confirmed success (cmd 0x60), -1 on timeout / abort /   */
/* malformed reply. Prints the abort code on an SDO abort (cmd 0x80).   */
/* ------------------------------------------------------------------ */
static int pmc_sdo_write(int sockfd, uint8_t node_id, uint16_t index,
                         uint8_t subindex, uint32_t value, uint8_t nbytes)
{
    uint8_t cmd;
    switch (nbytes) {
        case 1: cmd = 0x2F; break;
        case 2: cmd = 0x2B; break;
        case 3: cmd = 0x27; break;
        case 4: cmd = 0x23; break;
        default:
            fprintf(stderr, "pmc: invalid nbytes %u\n", (unsigned)nbytes);
            return -1;
    }

    struct can_frame tx;
    memset(&tx, 0, sizeof(tx));
    tx.can_id  = (canid_t)(0x600u + node_id);   /* standard 11-bit request COB-ID */
    tx.can_dlc = 8;
    tx.data[0] = cmd;
    tx.data[1] = (uint8_t)(index & 0xFF);
    tx.data[2] = (uint8_t)(index >> 8);
    tx.data[3] = subindex;
    tx.data[4] = (uint8_t)(value & 0xFF);
    tx.data[5] = (uint8_t)((value >> 8) & 0xFF);
    tx.data[6] = (uint8_t)((value >> 16) & 0xFF);
    tx.data[7] = (uint8_t)((value >> 24) & 0xFF);

    if (write(sockfd, &tx, sizeof(tx)) != (ssize_t)sizeof(tx)) {
        perror("pmc: CAN write");
        return -1;
    }

    const canid_t expect_id = (canid_t)(0x580u + node_id);   /* response COB-ID */
    const long    deadline  = pmc_now_ms() + PMC_SDO_TIMEOUT_MS;
    struct pollfd pfd = { .fd = sockfd, .events = POLLIN };

    for (;;) {
        long remaining = deadline - pmc_now_ms();
        if (remaining <= 0) {
            fprintf(stderr, "pmc: SDO reply timeout (idx 0x%04X sub 0x%02X)\n",
                    index, subindex);
            return -1;
        }

        int pr = poll(&pfd, 1, (int)remaining);
        if (pr < 0) {
            if (errno == EINTR) continue;
            perror("pmc: poll");
            return -1;
        }
        if (pr == 0) continue;   /* remaining will hit <=0 on the next loop */

        struct can_frame rx;
        ssize_t n = read(sockfd, &rx, sizeof(rx));
        if (n < 0) {
            if (errno == EINTR) continue;
            perror("pmc: CAN read");
            return -1;
        }
        if (n < (ssize_t)sizeof(rx)) continue;             /* short read */
        if (rx.can_id & CAN_EFF_FLAG) continue;            /* extended = not our SDO */
        if ((rx.can_id & CAN_SFF_MASK) != expect_id) continue;  /* other node/traffic */
        if (rx.can_dlc < 8) continue;                      /* malformed SDO reply */

        uint8_t rcmd = rx.data[0];
        if (rcmd == 0x60) {
            return 0;                                      /* write confirmed */
        }
        if (rcmd == 0x80) {                                /* SDO abort */
            uint32_t abort_code = (uint32_t)rx.data[4]         |
                                  ((uint32_t)rx.data[5] << 8)  |
                                  ((uint32_t)rx.data[6] << 16) |
                                  ((uint32_t)rx.data[7] << 24);
            fprintf(stderr, "pmc: SDO ABORT idx 0x%04X sub 0x%02X code 0x%08X\n",
                    index, subindex, abort_code);
            return -1;
        }
        fprintf(stderr, "pmc: unexpected SDO reply cmd 0x%02X (idx 0x%04X)\n",
                rcmd, index);
        return -1;
    }
}

/* ------------------------------------------------------------------ */
/* Public: rotate continuously at speed_pps (sign = direction).        */
/*   1) enter speed mode  (0x6005 = 1, U8)                             */
/*   2) set run speed     (0x6003 = speed_pps, S32)                    */
/* Returns 0 on success, -1 on any failure (message already printed).   */
/* ------------------------------------------------------------------ */
int pmc_run_continuous(int sockfd, uint8_t node_id, int32_t speed_pps)
{
    if (speed_pps > 200000 || speed_pps < -200000) {
        fprintf(stderr, "pmc_run_continuous: speed %d out of range (+/-200000)\n",
                speed_pps);
        return -1;
    }

    /* 1. Velocity/speed mode. */
    if (pmc_sdo_write(sockfd, node_id, 0x6005, 0x00, 1u, 1) != 0) {
        fprintf(stderr, "pmc_run_continuous: could not enter speed mode (node %u)\n",
                (unsigned)node_id);
        return -1;
    }

    /* 2. Run speed (signed; drive keeps spinning at this until 0x6003=0 / 0x6020=0). */
    if (pmc_sdo_write(sockfd, node_id, 0x6003, 0x00, (uint32_t)speed_pps, 4) != 0) {
        fprintf(stderr, "pmc_run_continuous: could not set speed %d (node %u)\n",
                speed_pps, (unsigned)node_id);
        return -1;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* Public: stop. hard != 0 -> 0x6020=0 (immediate, no ramp).           */
/*               hard == 0 -> 0x6003=0 (ramped stop, speed mode).      */
/* Returns 0 on success, -1 on failure.                                */
/* ------------------------------------------------------------------ */
int pmc_stop(int sockfd, uint8_t node_id, int hard)
{
    if (hard) {
        return pmc_sdo_write(sockfd, node_id, 0x6020, 0x00, 0u, 1);  /* hard stop */
    }
    return pmc_sdo_write(sockfd, node_id, 0x6003, 0x00, 0u, 4);      /* ramped stop */
}

/* ------------------------------------------------------------------ */
/* Optional standalone demo:  gcc -DPMC_DEMO_MAIN ... ./pmctest can0 3 5000  */
/* Opens can0, spins node 3 at 5000 pps for 3 s, then hard-stops.      */
/* ------------------------------------------------------------------ */
#ifdef PMC_DEMO_MAIN
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>

int main(int argc, char **argv)
{
    if (argc < 4) {
        fprintf(stderr, "usage: %s <ifname> <node_id> <speed_pps>\n", argv[0]);
        return 2;
    }
    const char *ifname = argv[1];
    uint8_t node = (uint8_t)atoi(argv[2]);
    int32_t spd  = (int32_t)atoi(argv[3]);

    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) { perror("socket"); return 1; }

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) { perror("SIOCGIFINDEX"); return 1; }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) { perror("bind"); return 1; }

    if (pmc_run_continuous(s, node, spd) != 0) { close(s); return 1; }
    printf("spinning node %u at %d pps for 3 s...\n", (unsigned)node, spd);
    sleep(3);
    pmc_stop(s, node, 1);
    printf("stopped.\n");
    close(s);
    return 0;
}
#endif

/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause) */
/*
 * isotpsend.c
 *
 * Copyright (c) 2008 Volkswagen Group Electronic Research
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of Volkswagen nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * Alternatively, provided that this notice is retained in full, this
 * software may be distributed under the terms of the GNU General
 * Public License ("GPL") version 2, in which case the provisions of the
 * GPL apply INSTEAD OF those given above.
 *
 * The provided data structures and external interfaces from this code
 * are not restricted to be used by modules with a GPL compatible license.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 * Send feedback to <linux-can@vger.kernel.org>
 *
 */

#include <libgen.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <stdint.h>
#include <linux/can.h>
#include "../../../../MTB_CE_EV-ECU_FOTA/ECU_FOTA/Kernel_Modules/can_isotp/isotp.h"
#include <fcntl.h>
#include <netinet/in.h>
#include <errno.h>

#define NO_CAN_ID 0xFFFFFFFFU
#define BUFSIZE 5000 /* size > 4095 to check socket API internal checks */
int s;

int guard(int n, char * err) { if (n == -1) { perror(err); exit(1); } return n; }

int can_isotp_connect(uint32_t sourceAddr, uint32_t destAddr, bool txPadding, uint8_t txPaddingByte)
{
    struct sockaddr_can addr;
    static struct can_isotp_options opts;
    static struct can_isotp_ll_options llopts;
    static struct can_isotp_fc_options fcopts;
    __u32 force_tx_stmin = 0;
    struct ifreq ifr;

    addr.can_addr.tp.tx_id = addr.can_addr.tp.rx_id = NO_CAN_ID;
    addr.can_addr.tp.tx_id = sourceAddr;
    addr.can_addr.tp.rx_id = destAddr;
    if(sourceAddr > 0x7FF) 
    {
       addr.can_addr.tp.tx_id |= CAN_EFF_FLAG;
       addr.can_addr.tp.rx_id |= CAN_EFF_FLAG;
    }
    opts.frame_txtime = 0;
    if (txPadding)
    {
        opts.flags |= CAN_ISOTP_TX_PADDING;
        opts.ext_address = txPaddingByte;
    }

    if ((addr.can_addr.tp.tx_id == NO_CAN_ID) ||
        ((addr.can_addr.tp.rx_id == NO_CAN_ID) &&
         (!(opts.flags & CAN_ISOTP_SF_BROADCAST))))
    {
        // LE_INFO("Invalid options for ISO-TP");
        return -1;
    }

    if ((s = socket(PF_CAN, SOCK_DGRAM, CAN_ISOTP)) < 0)
    {
        // LE_INFO("Unable to create socket for iso-tp CAN communication");
        return -1;
    }

    setsockopt(s, SOL_CAN_ISOTP, CAN_ISOTP_OPTS, &opts, sizeof(opts));
    setsockopt(s, SOL_CAN_ISOTP, CAN_ISOTP_RECV_FC, &fcopts, sizeof(fcopts));

    if (llopts.tx_dl)
    {
        if (setsockopt(s, SOL_CAN_ISOTP, CAN_ISOTP_LL_OPTS, &llopts, sizeof(llopts)) < 0)
        {
            // LE_INFO("Link layer socket option error");
        }
    }

    if (opts.flags & CAN_ISOTP_FORCE_TXSTMIN)
        setsockopt(s, SOL_CAN_ISOTP, CAN_ISOTP_TX_STMIN, &force_tx_stmin, sizeof(force_tx_stmin));

    strcpy(ifr.ifr_name, "can0");
    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        // LE_INFO("Error while binding the socket");
        close(s);
        return -1;
    }

    int flags = guard(fcntl(s, F_GETFL), "could not get file flags");
    guard(fcntl(s, F_SETFL, flags | O_NONBLOCK), "could not set file flags");
    return 0;
}

int can_isotp_send(int buflen, uint8_t *buf)
{
    int retval = 0;

    retval = write(s, buf, buflen);
    if (retval < 0)
    {
        // LE_INFO("Error while writing the data \t retval = %d", retval);
        return retval;
    }

    if (retval != buflen)
        // LE_INFO("Wrote only %d from %d byte\n", retval, buflen);

    /* 
     * due to a Kernel internal wait queue the PDU is sent completely
     * before close() returns.
     */
    // close(s);
    return 0;
}

int can_isotp_recv(uint8_t *buf, uint16_t *noOfBytesRead)
{

    uint8_t retryCount = 0;
    int nbytes;
    retryCount = 0;
    do
    {
        nbytes = 0;
        nbytes = read(s, buf, BUFSIZE);
        retryCount++;
	//printf("%d ",retryCount);
    } while (buf[0] == 0x7F && buf[2] == 0x78 && retryCount < 6);
    
    //printf("read done\n");

    if (nbytes > 0 && nbytes < BUFSIZE)
    {
        *noOfBytesRead = nbytes;
        // close(s);
        return 0;
    }
    // close(s);
    return -1;
}
int can_isotp_close()
{
    close(s);
    return 0;
}

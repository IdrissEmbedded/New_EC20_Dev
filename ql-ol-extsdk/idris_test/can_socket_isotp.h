#ifndef _CAN_ISOTP_H_
#define _CAN_ISOTP_H_

#include <stdint.h>

int can_isotp_connect(uint32_t sourceAddr, uint32_t destAddr, bool txPadding, uint8_t txPaddingByte);
int can_isotp_send(int buflen, uint8_t *buf);
int can_isotp_recv(uint8_t *buf, uint16_t *noOfBytesRead);
int can_isotp_close();

#endif
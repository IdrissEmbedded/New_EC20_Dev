/*
 * LSM_cnf.h
 *
 *  Created on: 21-Feb-2022
 *      Author: idris
 */
/*
 * Author: Idris Salim
 * Description: Header for rtc.c contains register macros and functions.
 *
 *
 *
 */
#ifndef RTC_H_
#define RTC_H_


#include "ql_oe.h"
//#include "./fsInfo.h"
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <errno.h>
////////////////////////////////RTC config and functions
///
///


///////////////
///
/// register address macros

//control
#define RTC_ADDR       0x51
#define CONTROL_1      0x00
#define CONTROL_2      0x01
#define OFFSET         0x02
#define RAM_BYTE       0x03
#define ONE_BYTE                0x01
#define TWO_BYTE                0x02
#define MAX_UNSGN_16BIT         0x7FFF
#define SENSOR_DATA_SIZE        0x0C
//#define RESET_VALUE             0x00
#define RTC_SLAVE_ADDRESS 0x51

//Time and Date
#define SECONDS        0x04
#define MINUTES        0x05
#define HOURS          0x06
#define DAYS           0x07
#define WEEKDAYS       0x08
#define MONTHS         0x09
#define YEARS          0x0A
#define THIS_MILLENNIA 2000   //just added a safety to avoid Y2K issues 980 years from now ;)

//Misc
#define RTC_DATA_SIZE    (7u)


//Generic Functions
int RTC_config(int* fd_i2c);
int RTC_reset(int* fd_i2c);
int RTC_set_DT(int*fd_i2c, uint8_t* dt);
int RTC_get_DT(int*fd_i2c, uint8_t *dateTime);
int validate_RTC_date_time(uint8_t *dtime, int dtlen);
int check_rtc(int* fd_i2c);


#endif

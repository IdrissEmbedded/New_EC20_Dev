/*
 * RTC_rw_data.c

 *
 *  Created on: 14-Mar-2022
 *      Author: Idris Salim
 */


#include <stdio.h>
#include "rtc.h"


const uint8_t offset = 7;  //offset to start reading DT string from
const uint8_t ascii_offset = 0x30;

#define _LEAP_YEAR(year)  (((year) > 0) && !((year) % 4) && (((year) % 100) || !((year) % 400)))
#define _LEAP_COUNT(year) ((((year) - 1) / 4) - (((year) - 1) / 100) + (((year) - 1) / 400))

const int yeardays[2][13] = {
    { -1, 30, 58, 89, 119, 150, 180, 211, 242, 272, 303, 333, 364 },
    { -1, 30, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365 }
};
const int monthdays[2][13] = {
    { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 },
    { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 }
};


int utoi(uint8_t ch);
int weekday(int year, int month, int day);
int validate_RTC_date(uint8_t *dtime, int dtlen);
int validate_RTC_time(uint8_t *dtime, int dtlen);

int utoi(uint8_t ch)
{
      int r = 0;
       switch(ch)
         {
             case 0x30:  r = 0; break;
             case 0x31:  r = 1; break;
             case 0x32:  r = 2; break;
             case 0x33:  r = 3; break;
             case 0x34:  r = 4; break;
             case 0x35:  r = 5; break;
             case 0x36:  r = 6; break;
             case 0x37:  r = 7; break;
             case 0x38:  r = 8; break;
             case 0x39:  r = 9; break;
             default  :  r = 0; break;
         }
         return r;
}

int weekday(int year, int month, int day)
{
    int ydays, mdays, base_dow;
    /* Correct out of range months by shifting them into range (in the same year) */
    month = (month < 1) ? 1 : month;
    month = (month > 12) ? 12 : month;
    mdays = monthdays[_LEAP_YEAR(year)][month - 1];
    /* Correct out of range days by shifting them into range (in the same month) */
    day = (day < 1) ? 1 : day;
    day = (day > mdays) ? mdays : day;
    /* Find the number of days up to the requested date */
    ydays = yeardays[_LEAP_YEAR(year)][month - 1] + day;
    /* Find the day of the week for January 1st */
    base_dow = (year * 365 + _LEAP_COUNT(year)) % 7;
    return (base_dow + ydays) % 7;
}

int check_rtc(int* fd_i2c)
{

    int iRet;
    uint8_t RTC_msg[2];

    RTC_msg[0] = 0x4F;
    RTC_msg[1] = 0;      //write a value
    Ql_I2C_Write(*fd_i2c, RTC_SLAVE_ADDRESS, RAM_BYTE ,RTC_msg ,1);

    unsigned char rdBuff[2] = {0, 0};
    iRet = Ql_I2C_Read(*fd_i2c, RTC_SLAVE_ADDRESS, RAM_BYTE, rdBuff, 1);
    printf("\n< read rtc on i2c iRet=%d, value=0x%x%x >\n", iRet, rdBuff[1], rdBuff[0]);//Little endian
}



int RTC_reset(int* fd_i2c)
{
    
    uint8_t val = 0x58;  //CONTROL_1   
    Ql_I2C_Write(*fd_i2c, RTC_SLAVE_ADDRESS, CONTROL_1 ,(uint8_t*)0x58 ,1);
    return 1;

}



///
/// "rtcRST|ddmmyyyyhhmmss;"    (23 byte)
///

int RTC_get_DT(int* fd_i2c, uint8_t *dateTime)
{
    uint8_t tmp[10] = {0x00};
    uint8_t temp[3]  , y = 0x20;
    int iRet;
    int i =0;

    //write start register
    static uint8_t read_start_reg = SECONDS;
    iRet = Ql_I2C_Read(*fd_i2c, RTC_SLAVE_ADDRESS, read_start_reg ,tmp ,7);


    memset(temp, '\0', sizeof temp);
    sprintf((char *)temp , "%x", tmp[3]);
    if(strlen((char *)temp)== 1)
    {
      strcpy((char *)dateTime, "0");
      strcat((char *)dateTime,(char *)temp);
    }
    else
        strcpy((char *)dateTime,(char *)temp);


    memset(temp, '\0', sizeof temp);
    sprintf((char *)temp , "%x", tmp[5]);
    if(strlen((char *)temp)== 1)
        strcat((char *)dateTime, "0");
    strcat((char *)dateTime,(char *)temp);

    memset(temp, '\0', sizeof temp);
    sprintf((char *)temp , "%x", y);
    strcat((char *)dateTime,(char *)temp);

    memset(temp, '\0', sizeof temp);
    sprintf((char *)temp , "%x", tmp[6]);
    if(strlen((char *)temp)== 1)
        strcat((char *)dateTime, "0");
    strcat((char *)dateTime,(char *)temp);

    memset(temp, '\0', sizeof temp);
    sprintf((char *)temp , "%x", tmp[2]);
    if(strlen((char *)temp)== 1)
        strcat((char *)dateTime, "0");
    strcat((char *)dateTime,(char *)temp);

    memset(temp, '\0', sizeof temp);
    sprintf((char *)temp , "%x", tmp[1]);
    if(strlen((char *)temp)== 1)
        strcat((char *)dateTime, "0");
    strcat((char *)dateTime,(char *)temp);

    memset(temp, '\0', sizeof temp);
    sprintf((char *)temp , "%x", tmp[0]);
    if(strlen((char *)temp)== 1)
        strcat((char *)dateTime,"0");
    strcat((char *)dateTime,(char *)temp);


    return iRet;



}


///create an RTC data buffer from a formatted DT string. Pass in buffer from main thread

///
/// DTstring: "rtcRST|ddmmyyyyhhmmss;"    (23 byte)  21042023123000
///


int RTC_set_DT(int* fd_i2c, uint8_t* dt)
{

    uint8_t tb[25] , temp[5] ;
    int dd=0,mm=0,yy=0 , l=0;

    memset(tb, '\0' , sizeof tb);
    memset(temp, '\0' , sizeof temp);
    temp[0] = dt[0];
    temp[1] = dt[1];
    dd = atoi((char *)temp);
    memset(temp, '\0' , sizeof temp);
    temp[0] = dt[2];
    temp[1] = dt[3];
    mm = atoi((char *)temp);
    memset(temp, '\0' , sizeof temp);
    temp[0] = dt[4];
    temp[1] = dt[5];
    temp[2] = dt[6];
    temp[3] = dt[7];
    yy = atoi((char *)temp);
    l = 0;
    l = weekday(yy,mm,dd);

    tb[0] = SECONDS;
    tb[1] = ((uint8_t)(utoi(dt[12])) << 4) | (uint8_t)(utoi(dt[13]));
    tb[2] = (uint8_t)((utoi(dt[10])) << 4) | (utoi(dt[11]));
    tb[3] = (uint8_t)((utoi(dt[8])) << 4) | (utoi(dt[9]));
    tb[4] = (uint8_t)((utoi(dt[0])) << 4) | (utoi(dt[1]));
    tb[5] = (uint8_t)l;  //0x00;  //weekday
    tb[6] = (uint8_t)((utoi(dt[2])) << 4) | (utoi(dt[3]));
    tb[7] = (uint8_t)((utoi(dt[6])) << 4) | (utoi(dt[7]));


    return Ql_I2C_Write(*fd_i2c, RTC_SLAVE_ADDRESS, SECONDS,tb+1 ,RTC_DATA_SIZE);

}


int validate_RTC_time(uint8_t *dtime, int dtlen)
{
   unsigned int ret = 1,  ret1 = 1, i = 0;
   int hrs = 0, mins=0, secs=0;
   uint8_t temp1[5] = "";

   if(dtlen == 14)
   {
            // check if time contains only numerals
       for(i = 8; i < (uint8_t)dtlen; i++)
        {
            if(dtime[i] > 0x39 || dtime[i] < 0x30)
            {
                ret = 0;
                break;
            }
        }

        if(ret == 1)
        {
                memset(temp1, '\0' , sizeof temp1);
                temp1[0] = dtime[8];
                temp1[1] = dtime[9];
                hrs = atoi((char *)temp1);
                if(hrs >=0 && hrs <= 23)
                {
                     memset(temp1, '\0' , sizeof temp1);
                     temp1[0] = dtime[10];
                     temp1[1] = dtime[11];
                     mins = atoi((char *)temp1);
                     if(mins >= 0 && mins <= 59)
                     {
                         memset(temp1, '\0' , sizeof temp1);
                         temp1[0] = dtime[12];
                         temp1[1] = dtime[13];
                         secs = atoi((char *)temp1);
                         if(secs >= 0 && secs <= 59)
                                ret1 = 1;
                         else
                                 ret1 = 0;
                     }
                     else
                         ret1 = 0;
                     }
                    else
                        ret1 = 0;
                }
                else if (ret == 0)
                     ret1 = 0;

             }
             else
                 ret1 = 0;

         return (int)ret1;
}



int validate_RTC_date(uint8_t *dtime, int dtlen)
{

     unsigned int ret = 1, ret1 = 1, i = 0;
     int day = 0, mnth = 0, year = 0;
     uint8_t temp1[5] = "";

     if(dtlen == 14)
     {
                        // check if date contains only numerals
        for(i = 0; i < 8; i++)
        {
            if(dtime[i] > 0x39 || dtime[i] < 0x30)
            {
                ret = 0;
                break;
            }
        }

        if(ret == 1)
        {
            // date format : For ex      05062016
            memset(temp1, '\0' , sizeof temp1);
            temp1[0] = dtime[0];
            temp1[1] = dtime[1];
            day = atoi((char *)temp1);
            if(day > 0 && day <= 31)
            {
                  memset(temp1, '\0' , sizeof temp1);
                  temp1[0] = dtime[2];
                  temp1[1] = dtime[3];
                  mnth = atoi((char *)temp1);
                  if(mnth > 0 && mnth <=12)
                  {
                        memset(temp1, '\0' , sizeof temp1);
                        temp1[0] = dtime[4];
                        temp1[1] = dtime[5];
                        temp1[2] = dtime[6];
                        temp1[3] = dtime[7];
                        year = atoi((char *)temp1);
                        if(year >= 2020 && year <= 2040)
                        {
                             if(year == 2020 && mnth < 4)
                               ret1 = 0;
                             else
                               ret1 = 1;
                        }
                        else
                           ret1 = 0;
                   }
                   else
                      ret1 = 0;
                }
                else
                    ret1 = 0;
        }
        else
             ret1 = 0;



     }
     else
          ret1 = 0;


     return (int)
             ret1;
}



int validate_RTC_date_time(uint8_t *dtime, int dtlen)
{
  int ret = 0, ret1 = 0;


  ret = validate_RTC_time(dtime, dtlen);
  ret1 = validate_RTC_date(dtime, dtlen);

  if(ret && ret1)
             return 1;


  return 0;
}


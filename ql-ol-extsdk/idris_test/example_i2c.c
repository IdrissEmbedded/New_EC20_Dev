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
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <dirent.h>
#include <sys/stat.h>
#include "accel.h"
#include "rtc.h"
#include "curses.h"
#include "can_socket_isotp.h"





#define I2C_DEV         "/dev/i2c-2"	//i2c-2 on EC20xx, i2c-4 on AG35
#define I2C_SLAVE_ADDR  0x6B	//codec 3104
//#define WHO_AM_I        0x0F
#define WHO_AM_I_VALUE  0x6A

//#define RAM_BYTE       0x03

#define MEMS_READ_BUFF_SIZE  12
#define STARTUP_G_THRESH 0.7  //gravitation accelaration percieved on the sensor

const double rad_to_deg = 57.295779505601046646705075978956;

bool kill_accel_thread = false, recording = false, event_detect = false; //testing inputs

void machineEpsilon(float EPS);
int floatCompare(const void* f1, const void* f2);
void pollPidISO();
clock_t t_stamp;

uint8_t MEMS_rec_buff[12] = {0x00} , date_time[12] = {0x00};
float dataBuff[60] = {0x00};
double AccelX = 0;
double AccelY = 0;
double AccelZ = 0;

double gyroX = 0;
double gyroY = 0;
double gyroZ = 0;

double roll;
double pitch;

const double accel_scaling_factor = 0.00006103515625f;
const double gyro_scaling_factor = 0.004375;
const double gyro_offset_X = 0.1366527123;
const double gyro_offset_Y = -0.3121025943;
const double gyro_offset_Z = -0.1323354953;



double prev_epsilon;



int16_t AccelX_shifted = 0;
int16_t AccelY_shifted = 0;
int16_t AccelZ_shifted = 0;

int16_t gyroX_shifted = 0;
int16_t gyroY_shifted = 0;
int16_t gyroZ_shifted = 0;   //left shited values, check for sign

float ACCX_BUF1[1024], ACCY_BUF1[1024], ACCZ_BUF1[1024];
float GYRX_BUF1[1024], GYRY_BUF1[1024], GYRZ_BUF1[1024];

float ACCX_BUF2[10], ACCY_BUF2[10], ACCZ_BUF2[10];
float GYRX_BUF2[10], GYRY_BUF2[10], GYRZ_BUF2[10];

uint8_t bufWrite[4500], readBuf[4500];

static uint16_t BUF1_index = 0, BUF2_index = 0;

pthread_mutex_t kb_lock;

pthread_t accel_tid;
void* tret;
int fd_i2c;
char dt[14] = "21042023123001";

FILE *fp;

struct tms {  
    clock_t tms_utime;              /* User CPU time.  */    
    clock_t tms_stime;              /* System CPU time.  */   
    clock_t tms_cutime;             /* User CPU time of terminated children.  */  
    clock_t tms_cstime;             /* System CPU time of terminated children.  */    
}tms0;




int main(int argc, char* argv[])  
{
    int iRet;
    int err; 
	int x;      
    void* arg;
    int can_ret = 0;
	static char key_press;
    /*
     * Open I2C device with read-write mode,
     * and write slave address.
     */
    fd_i2c = Ql_I2C_Init(I2C_DEV);
    printf("< Ql_I2C_Init=%d >\n", fd_i2c);
    if (fd_i2c < 0)  
    {
        printf("< Fail to open i2c >\n");
        return -1;  
    }  

    /*
     * Read the response for "who am i".
     */
    unsigned char rdBuff[2] = {0, 0};
    iRet = Ql_I2C_Read(fd_i2c, I2C_SLAVE_ADDR, WHO_AM_I, rdBuff, 1);
    printf("< read i2c iRet=%d, value=0x%x%x >\n", iRet, rdBuff[1], rdBuff[0]);//Little endian


    iRet = check_rtc(&fd_i2c);
    if(iRet>1)

    {
        RTC_set_DT(&fd_i2c, (uint8_t*)dt);

        //RTC_reset(&fd_i2c);
        usleep(2000 * 1000);
        iRet = RTC_get_DT(&fd_i2c, date_time);
        printf("read date time: %s \n", date_time);
    }

    Ql_I2C_Deinit(fd_i2c);
    fd_i2c = Ql_I2C_Init(I2C_DEV);

    fp = fopen ("/home/root/IMU.csv", "w+"); //open new IMU file
    fprintf(fp,"Tick, ACCELX, ACCELY, ACCELZ, GYRX, GYRY, GYRZ, Event\n");


    if (pthread_mutex_init(&kb_lock, NULL) != 0) {
        printf("\n mutex init has failed\n");
        return 1;
    }


    /*//CAN ISO TP
    //ISO
    can_ret = can_isotp_connect(0x7DF, 0x7E8, true, 0xFF);
    if(can_ret)
        printf("\nISO init success");
    else
    {
        printf("\nISO init failed");
        can_isotp_close();
        return 0;
    }*/


    if(rdBuff[0] == WHO_AM_I_VALUE)
    {
        printf("\n\raccel check OK, starting accel thread\n");

        machineEpsilon(0.001);

        //ioctl(fd_i2c, I2C_SLAVE, I2C_SLAVE_ADDR);

        err = MEMS_config_data(&fd_i2c);
        if(err<0)
        {
            printf("config failed\n");
            return 0;
        }
        else
        {
            MEMS_read_config(&fd_i2c);
            printf("config OK, starting\n");
        }
    
        createAccelReadThread();
    }
    else 
        return 0;



    while(1)
    {
        //for testing

        pthread_mutex_lock(&kb_lock); //CS enter

        key_press = fgetc(stdin);

        if(key_press == 'q')
        { 
            printf("IMU thread exit");
            kill_accel_thread = true;
            fclose(fp);
            can_isotp_close();
            break;
        }

        if(key_press == 'w')
        {
            recording = true;
            printf("\n /////////recording////////////");
        }

        else if(key_press == 'x')
        {
            recording = false;
            printf("\n /////////stop////////////");
        }
        

        else if(key_press == 'r')
        {
            event_detect = true;
            printf("\n /////////EVENT!////////////");
        }

        pthread_mutex_unlock(&kb_lock);  //CS exit
    }
    
    return 0;  
}

int killAccelReadThread(void)
{
    int ret = 0;

        
        if (pthread_join(accel_tid, &tret) != 0)
        {
            printf("ACCEL: Join Accel thread error!\n");
        }
        else
        {
            printf("ACCEL: Accel Threadexit code: %d\n", *((int *)tret));
            ret = 1;
        }
    
    return ret;
}

void* accelRead_func(void* arg)
{  


    int iRet;
    static uint32_t callback_count = 0;
    
    
    /*thread entry point*/
    while(1)
    {       

        //if(callback_count == UINT16_MAX);
            //callback_count = 0;

        callback_count++;

        //printf("\nbuf1 index = %d", BUF1_index);

        iRet = MEMS_Read(&fd_i2c, MEMS_rec_buff);
        if(iRet<0)
        {
            printf("i2c read failed\n");
        }
        if(kill_accel_thread)
            pthread_exit(0); 

        


        MEMS_process_data(MEMS_rec_buff); // convert data to float
        //iRet = RTC_get_DT(&fd_i2c, date_time);
        //printf("read date time: %s \n", date_time);
        //average window = 8, step = 5, freq = 100ms/20ms = 5
        MEMS_process_moving_avg(8, 5, 5, callback_count);
        //detection window = 25, step = 25, freq = 500ms/20ms = 25
        //4 states of acc>thresh = harsh accel
                                    //  +X     -X   X_min  +-Y   +-Z   Zmin
        MEMS_process_harsh_accel(25, 5, 0.2, -0.25, 0.05 , 0.1 , 4.5 , 1 ,  5, callback_count);

        //if(callback_count == 0) // every 2 sec 
            //pollPidISO();


        usleep(20 * 1000);


    }

    return 0;
   
}


void MEMS_process_harsh_accel(int window /*avg window size*/, int step, double x_threshold_pos /*accel*/
                            ,double x_threshold_neg /*braking*/, double x_threshold_lower, double y_threshold
                            ,double gyr_z_threshold , double gyr_z_threshold_lower , int cbk_cnt_min, int cbk_global)
{
    //step = 5, window = 25, freq = 100ms/20ms = 5 

    int i = 0;
    int j = 0;

    static double prev_min_X, prev_max_X, prev_min_Y, prev_max_Y;
    static int X_event , X_event_brake = 0, Y_event = 0;

    static int X_event_ongoing = 0, Y_event_ongoing = 0; // 1 for event in progress

    static bool sign_invert = 0;   //flag to store sign inversion
    static int prev_sign = 0;   //sign flag 1 for pos and -1 for neg and 0 for no threshold crossed

    static uint16_t sort_count= 0;
    if(cbk_global%cbk_cnt_min != 0)  //frequency
        return;

    if(BUF1_index < (window + step)) //overflow condition
        return;

    //printf("\ncbk_global:%d, sort_count:%d",cbk_global, sort_count);

    //BUF1_index - 25(window) + 5(step)
    for(i = BUF1_index - window + step,j=0; j<5 ; i+=5,j++) //take every 5th value from BUF1 to BUF2
    {
        ACCX_BUF2[j] = ACCX_BUF1[i];
        ACCY_BUF2[j] = ACCY_BUF1[i];
        ACCZ_BUF2[j] = ACCZ_BUF1[i];

        GYRX_BUF2[j] = GYRX_BUF1[i];
        GYRY_BUF2[j] = GYRY_BUF1[i];
        GYRZ_BUF2[j] = GYRZ_BUF1[i];
    }


    qsort(ACCX_BUF2, 5, sizeof(float), floatCompare);
    qsort(ACCY_BUF2, 5, sizeof(float), floatCompare);
    qsort(ACCZ_BUF2, 5, sizeof(float), floatCompare);
    qsort(GYRZ_BUF2, 5, sizeof(float), floatCompare);




    //acceleration start
    if(ACCX_BUF2[2] > x_threshold_pos && !X_event_ongoing && !prev_sign)  //median
    {
        prev_sign = 1; //accel start
        X_event_ongoing = 1;

    }

    //braking start
    else if(ACCX_BUF2[2] < x_threshold_neg && !X_event_ongoing  &&  !prev_sign)
    {
        prev_sign = -1; //braking start
        X_event_ongoing = 1;  
    }


    //if under lower thresh, evaulate event end
    else if(fabs(ACCX_BUF2[2])<x_threshold_lower && prev_sign!= 0 && X_event_ongoing)
    {
        //harsh braking/acceleration
        X_event += 1;
        if(X_event> 4)
        {   //TBD
            //event detected (hard thresh)
            X_event = 0;

            if(prev_sign == 1)
            {
                printf("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< HARD ACCELERATION EVENT >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
                if(recording)  //start recording
                {
                    t_stamp = times(&tms0);
                    fprintf(fp, "%ld,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d\n",t_stamp,AccelX,AccelY,AccelZ,gyroX,gyroY,gyroZ, 1);
                }

            }

            else if(prev_sign == -1)
            {
                printf("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< HARD BRAKING EVENT >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
                if(recording)  //start recording
                {
                    t_stamp = times(&tms0);
                    fprintf(fp, "%ld,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d\n",t_stamp,AccelX,AccelY,AccelZ,gyroX,gyroY,gyroZ, -1);
                }
            }

            prev_sign = 0;
            X_event_ongoing = 0;
        }
    }





    //hard turn event
    if(fabs(ACCY_BUF2[2]) > y_threshold && fabs(GYRZ_BUF2[2])>gyr_z_threshold  &&  !Y_event_ongoing)  
    {
      Y_event_ongoing = 1;  //harsh turn start
    }

    else if(fabs(ACCY_BUF2[2]) < y_threshold && fabs(GYRZ_BUF2[2])<gyr_z_threshold_lower  &&  Y_event_ongoing)
    {
       Y_event += 1; 
       if(Y_event>4)
       {
            Y_event = 0;
            Y_event_ongoing = 0;
            printf("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< HARD TURN EVENT >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
            if(recording)  //start recording
            {
                t_stamp = times(&tms0);
                fprintf(fp, "%ld,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d\n",t_stamp,AccelX,AccelY,AccelZ,gyroX,gyroY,gyroZ, 2);
            }
       }
    }

    sort_count++;



}




void MEMS_process_data(uint8_t* MEMS_r_buff)
{


        gyroX_shifted = (int16_t)MEMS_r_buff[1];
        gyroX_shifted = (gyroX_shifted * 256) + (int16_t)(MEMS_r_buff[0]);

        gyroY_shifted = (int16_t)MEMS_r_buff[3];
        gyroY_shifted = (gyroY_shifted * 256) + (int16_t)(MEMS_r_buff[2]);

        gyroZ_shifted = (int16_t)MEMS_r_buff[5];
        gyroZ_shifted = (gyroZ_shifted * 256) + (int16_t)(MEMS_r_buff[4]);


        AccelX_shifted = (int16_t)MEMS_r_buff[7];
        AccelX_shifted = (AccelX_shifted * 256) + (int16_t)(MEMS_r_buff[6]);

        AccelY_shifted = (int16_t)MEMS_r_buff[9];
        AccelY_shifted = (AccelY_shifted * 256) + (int16_t)(MEMS_r_buff[8]);

        AccelZ_shifted = (int16_t)MEMS_r_buff[11];
        AccelZ_shifted = (AccelZ_shifted * 256) + (int16_t)(MEMS_r_buff[10]);


        AccelX = AccelX_shifted * accel_scaling_factor;
        AccelY = AccelY_shifted * accel_scaling_factor;
        AccelZ = AccelZ_shifted * accel_scaling_factor;

        gyroX = (gyroX_shifted * gyro_scaling_factor) - gyro_offset_X;
        gyroY = (gyroY_shifted * gyro_scaling_factor) - gyro_offset_Y;
        gyroZ = (gyroZ_shifted * gyro_scaling_factor) - gyro_offset_Z;


        if(recording)  //start recording
        {
            t_stamp = times(&tms0);
            fprintf(fp, "%ld,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d\n",t_stamp,AccelX,AccelY,AccelZ,gyroX,gyroY,gyroZ,0);
        }

        if(event_detect)
        {
            event_detect = false;
            //insert special character
            fprintf(fp,"\n\n");
        }


        if(BUF1_index+1 == 1023)  //bounds check
            BUF1_index = 0;


       ACCX_BUF1[BUF1_index] = AccelX;  //add to buffer
       ACCY_BUF1[BUF1_index] = AccelY;
       ACCZ_BUF1[BUF1_index] = AccelZ;

       GYRX_BUF1[BUF1_index] = gyroX;
       GYRY_BUF1[BUF1_index] = gyroY;
       GYRZ_BUF1[BUF1_index] = gyroZ;

       BUF1_index++;



}


int createAccelReadThread(void)
{

    if (pthread_create(&accel_tid, NULL, accelRead_func, NULL) != 0)
    {
        printf("ACCEL: Create Accel thread error!\n");
        return 0;
    }
    printf("ACCEL: Accel thread TID: %lu\n", accel_tid);
    return 1;


}

void pollPidISO()
{
    static int i = 0;
    uint16_t noOfBytesRead;
    //while(i < 8)
    //{
        bufWrite[0] = 0x01;
        bufWrite[1] = 0x0D;
        can_isotp_send(2, bufWrite);
        printf("%d Tx %.2X %.2X %.2X %.2X %.2X %.2X %.2X\n", i, bufWrite[0],bufWrite[1],bufWrite[2], bufWrite[3], bufWrite[4],bufWrite[5],bufWrite[6]);

        memset(readBuf, 0, sizeof readBuf);
        can_isotp_recv(readBuf, &noOfBytesRead);
        //read(s, buf, BUFSIZE);

        if(noOfBytesRead > 0)
        {
            printf("%d Rx %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X\n\n", i, noOfBytesRead, readBuf[0],readBuf[1],readBuf[2],readBuf[3],readBuf[4],readBuf[5],readBuf[6]);
        }
        //else continue;
        
        //sleep(2);
        i++;
    //}
    printf("done\n");

}


float get_roll(float AccelX, float AccelY, float AccelZ)    //get roll
{

    return atan2(AccelY, (sqrt(pow(AccelX,2)  +  pow(AccelZ,2) )))  *   rad_to_deg;

}

float get_pitch(float AccelX, float AccelY, float AccelZ)
{
    return atan2( -AccelX,-AccelZ )  * rad_to_deg;
}

int MEMS_Read(int* fd_i2c ,uint8_t* MEMS_r_buff)               //read gyroscope and accelaration values
{
 // for read sequence, refer datasheet.

     //* A write to the starting register followed by subsequent reads. SDA line needs to be high on 'SR' state
     //* between write and read.

    int iRet =-1;
    int i;

    //check buffer pointer
    if(NULL == MEMS_r_buff)
    {
        return -1;
    }

    memset(MEMS_r_buff, 0x00, sizeof MEMS_r_buff); 

    iRet = Ql_I2C_Read(*fd_i2c, I2C_SLAVE_ADDR, OUTX_L_G, MEMS_r_buff, MEMS_READ_BUFF_SIZE);

    /*for(i =0; i<MEMS_READ_BUFF_SIZE; i++)
    {
        iRet = Ql_I2C_Read(*fd_i2c, I2C_SLAVE_ADDR, OUTX_L_G+i, &MEMS_r_buff[i], 1);
    }*/

    return iRet;

}


int MEMS_config_data(int* fd_i2c)
{

        int iRet;
        uint8_t MEMS_msg;

        /*MEMS_msg= 0b00000000;
        iRet =  Ql_I2C_Write(*fd_i2c, I2C_SLAVE_ADDR, ORIENT_CFG_G, &MEMS_msg , 1);
        if(iRet < 0)
            printf("MEMS config fail err = %d\n", iRet);
        usleep(100 * 1000);*/

       //control register
        MEMS_msg = 0x60;                //0b 0101 0000;     //104Hz rate, +-2G range 400Hz anti aliasing
        iRet =  Ql_I2C_Write(*fd_i2c, I2C_SLAVE_ADDR, CTRL1_XL, &MEMS_msg , 1);
        if(iRet < 0)
            printf("MEMS config fail err = %d\n", iRet);
        usleep(10 * 1000);


        //control register
        MEMS_msg = 0x68;          //0b 0110 1000;
        iRet =  Ql_I2C_Write(*fd_i2c, I2C_SLAVE_ADDR, CTRL2_G, &MEMS_msg , 1);
        if(iRet < 0)
            printf("MEMS config fail err = %d\n", iRet);
        usleep(10 * 1000);


        
        MEMS_msg = 0b01000100;     //Intterrupt active low, increment reg, lower byte LSB
        iRet =  Ql_I2C_Write(*fd_i2c, I2C_SLAVE_ADDR, CTRL3_C , &MEMS_msg , 1);
        if(iRet < 0)
            printf("MEMS config fail err = %d\n", iRet);
        usleep(10 * 1000);
 

        MEMS_msg = 0b00000000;
        iRet =  Ql_I2C_Write(*fd_i2c, I2C_SLAVE_ADDR, CTRL4_C, &MEMS_msg , 1);
        if(iRet < 0)
            printf("MEMS config fail err = %d\n", iRet);
            usleep(10 * 1000);
       

    
        /*MEMS_msg = 0b01100000;     //circular rounding self test disable
        iRet =  Ql_I2C_Write(*fd_i2c, I2C_SLAVE_ADDR, CTRL5_C, &MEMS_msg , 1);
        if(iRet < 0)
            printf("MEMS config fail err = %d\n", iRet);
            usleep(100 * 1000);



        MEMS_msg = 0b00000000;
        iRet =  Ql_I2C_Write(*fd_i2c, I2C_SLAVE_ADDR, CTRL6_C, &MEMS_msg , 1);
        if(iRet < 0)
            printf("MEMS config fail err = %d\n", iRet);
            usleep(100 * 1000);


        MEMS_msg = 0b01001000;
        iRet =  Ql_I2C_Write(*fd_i2c, I2C_SLAVE_ADDR, CTRL7_G, &MEMS_msg , 1);
        if(iRet < 0)
            printf("MEMS config fail err = %d\n", iRet);
            usleep(100 * 1000);



        MEMS_msg = 0b10000000;
        iRet =  Ql_I2C_Write(*fd_i2c, I2C_SLAVE_ADDR, CTRL8_XL, &MEMS_msg , 1);
        if(iRet < 0)
            printf("MEMS config fail err = %d\n", iRet);
            usleep(100 * 1000);


        MEMS_msg = 0b00111000;
        iRet =  Ql_I2C_Write(*fd_i2c, I2C_SLAVE_ADDR, CTRL9_XL, &MEMS_msg , 1);
        if(iRet < 0)
            printf("MEMS config fail err = %d\n", iRet);
            usleep(100 * 1000);


        MEMS_msg = 0b00111100;
        iRet =  Ql_I2C_Write(*fd_i2c, I2C_SLAVE_ADDR, CTRL10_C, &MEMS_msg , 1);
        if(iRet < 0)
            printf("MEMS config fail err = %d\n", iRet);
            usleep(100 * 1000);


       //control register
        /*MEMS_msg = 0b00000000;
        iRet =  Ql_I2C_Write(*fd_i2c, I2C_SLAVE_ADDR, MASTER_CONFIG, &MEMS_msg , 1);
        if(iRet < 0)
            printf("MEMS config fail err = %d", iRet);
            usleep(100 * 1000);*/


        return iRet;
}

void MEMS_read_config(int* fd_i2c)
{
    uint8_t temp;
    int iRet = -1;
    int i=0;

    
    /*if(iRet<0)
        printf("\n\rread failed");*/

    for(i=0; i<15;i++)
    {
        iRet =  Ql_I2C_Read(*fd_i2c, I2C_SLAVE_ADDR, ORIENT_CFG_G+i, &temp , 1);
        printf("MEMS config reg addr: 0x%X val= 0x%X\n", (0x0B +i), temp);
    }

}



int MEMS_config_sleep(int* fd_i2c) // set MEMS option registers. Enable accel and gyro
{

    int iRet;
    uint8_t MEMS_msg;

           //set 410Hz frequency//2g
    MEMS_msg = 0x60;
    iRet =  Ql_I2C_Write(*fd_i2c, I2C_SLAVE_ADDR, CTRL1_XL, &MEMS_msg , 1);
        if(iRet < 0)
            printf("MEMS config fail err = %d\n", iRet);
        //enable tap on x,y,z axis
    MEMS_msg = 0x8E;
    iRet =  Ql_I2C_Write(*fd_i2c, I2C_SLAVE_ADDR, TAP_CFG, &MEMS_msg , 1);
        if(iRet < 0)
            printf("MEMS config fail err = %d\n", iRet);

      //set zero wake up duration
    MEMS_msg = 0x00;
    iRet =  Ql_I2C_Write(*fd_i2c, I2C_SLAVE_ADDR, WAKE_UP_DUR, &MEMS_msg , 1);
        if(iRet < 0)
            printf("MEMS config fail err = %d\n", iRet);

      //set zero wake up threshold
    MEMS_msg = 0x00;
    iRet =  Ql_I2C_Write(*fd_i2c, I2C_SLAVE_ADDR, WAKE_UP_THS, &MEMS_msg , 1);
        if(iRet < 0)
            printf("MEMS config fail err = %d\n", iRet);

            // set single tap interupt(INT1)
    MEMS_msg = 0x40;
    iRet =  Ql_I2C_Write(*fd_i2c, I2C_SLAVE_ADDR, MD1_CFG, &MEMS_msg , 1);
        if(iRet < 0)
            printf("MEMS config fail err = %d\n", iRet);

         //set maximum time of an overthreshold signal detection
    MEMS_msg = 0x06;
    iRet =  Ql_I2C_Write(*fd_i2c, I2C_SLAVE_ADDR, INT_DUR2, &MEMS_msg , 1);
        if(iRet < 0)
            printf("MEMS config fail err = %d\n", iRet);

      //6D orientation detection enable & Threshold set by 62.3mg
    MEMS_msg = 0x41;
    iRet =  Ql_I2C_Write(*fd_i2c, I2C_SLAVE_ADDR, TAP_THS_6D, &MEMS_msg , 1);
        if(iRet < 0)
            printf("MEMS config fail err = %d\n", iRet);

    return iRet;

}



//############################# ###############################//
/*
 * Get device orientation. Buffer some values before reading to determine device orientation.
 * Device can either be vertical or horizontal (all four possibilities in both)
 * Check for g on all axes:
 * if z= +-g, device is horizontal (Roll and pitch axes are undeterminable).
 * if x or y = +-g, device is vertical. set roll and pitch axes accordingly.
 *
 * args: read values in a data buffer on main thread, pass accelaration buffer pointer and size
 *
 * return
 * -1         failed to get orientation
 * 0          horizontal orientation
 * 1          vertical with g on +-x
 * 2          vertical with g on +-y
 */


int get_device_orientation(float dataBuff[], int buff_size)
{
 float AvgAccelX=0 , AvgAccelY=0 , AvgAccelZ=0;  //get average accelaration values from buffer

 int i;
 for(i =0; i< buff_size/3; i+=3)
 {
     AvgAccelX += dataBuff[i];
     AvgAccelY += dataBuff[i+1];
     AvgAccelZ += dataBuff[i+2];
 }

 AvgAccelX /=3;
 AvgAccelY /=3;
 AvgAccelZ /=3;


 if(AvgAccelZ > STARTUP_G_THRESH || AvgAccelZ< -STARTUP_G_THRESH)
 {
     return 0;
 }

 else if(AvgAccelX> STARTUP_G_THRESH  ||  AvgAccelX< -STARTUP_G_THRESH )
 {
     return 1;
 }

 else if(AvgAccelY> STARTUP_G_THRESH  ||  AvgAccelY<  -STARTUP_G_THRESH )
  {
      return 2;
  }

 else
     return -1;

}

void MEMS_process_moving_avg(int window, int step, int cbk_cnt_min, int cbk_global)
{
    
    int i = 0;
    float sumAccX = 0, sumAccY =0, sumAccZ = 0;
    float sumGyrX = 0, sumGyrY =0, sumGyrZ = 0;

    if(cbk_global%cbk_cnt_min != 0)
        return;
    
    if(BUF1_index < window)
        return; //not enough entries

        

        for(i =0; i< window; i++)
        {
            sumAccX += ACCX_BUF1[BUF1_index-window+i];
            sumAccY += ACCY_BUF1[BUF1_index-window+i];
            sumAccZ += ACCZ_BUF1[BUF1_index-window+i];

            sumGyrX += GYRX_BUF1[BUF1_index-window+i];
            sumGyrY += GYRY_BUF1[BUF1_index-window+i];
            sumGyrZ += GYRZ_BUF1[BUF1_index-window+i];

        }

        for(i =0; i< window; i++)
        {
            ACCX_BUF1[BUF1_index-window+i] = sumAccX/window;
            ACCY_BUF1[BUF1_index-window+i] = sumAccY/window;
            ACCZ_BUF1[BUF1_index-window+i] = sumAccZ/window;

            GYRX_BUF1[BUF1_index-window+i] = sumGyrX/window;
            GYRY_BUF1[BUF1_index-window+i] = sumGyrY/window;
            GYRZ_BUF1[BUF1_index-window+i] = sumGyrZ/window;  
        }

        //printf("\n\r GYRX = %.6f , GYRY = %.6f , GYRZ = %.6f   \n\r" , GYRX_BUF1[BUF1_index-1], GYRY_BUF1[BUF1_index-1], GYRZ_BUF1[BUF1_index-1]);
        printf("\n\r ACCELX = %.3f , ACCELY = %.3f , ACCELZ = %.3f   \n\r" , ACCX_BUF1[BUF1_index], ACCY_BUF1[BUF1_index], ACCZ_BUF1[BUF1_index]);
        //printf("\n\r sum1 = %.3f , sum2 = %.3f , sum3 = %.3f   \n\r" , sumAccX, sumAccY, sumAccZ);
    
}




void machineEpsilon(float EPS)  //calculate machine epsilon
{
    // taking a floating type variable
    
    // run until condition satisfy
    while ((1+EPS) != 1)
    {
        // copying value of epsilon into previous epsilon
        prev_epsilon = EPS;
  
        // dividing epsilon by 2
        EPS /=2;
    }
  
    printf("\nmachine Epsilon is %E \n", prev_epsilon);
   
}

int floatCompare(const void* f1, const void* f2) 
{  
    float a = *(float*) f1;  
    float b = *(float*) f2;  
    if(a>b && fabs(a-b)>prev_epsilon)  //a greater
    {  
        return 1;  
    }  
    else if(a<b && fabs(a-b)>prev_epsilon)        //b greater
    {  
        return -1;  
    }  

    else if(fabs(a-b)<= prev_epsilon)      //equal
    return 0;  
}  

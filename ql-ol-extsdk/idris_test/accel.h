//#define _SVID_SOURCE 1
/*
 * LSM_cnf.h
 *
 *  Created on: 21-Feb-2022
 *      Author: idris
 */
/*
 * Author: Idris Salim
 * Description: Header for LSM_cnf.c contains register macros and functions.
 *
 *
 *
 */
#ifndef ACCEL_H_
#define ACCEL_H_


///////////////////////////////////////////LSM config and read adresses
/// As defined in ST datasheet
/// link: https://www.mouser.in/datasheet/2/389/dm00345518-2449349.pdf
///


// Macro definitions
#define ONE_BYTE                0x01
#define TWO_BYTE                0x02
#define MAX_UNSGN_16BIT         0x7FFF
#define SENSOR_DATA_SIZE        0x0C
#define RESET_VALUE             0x00
#define DEVICE_ID               0x69

//#define UINT16_MAX              65535





//Address definitions
#define LSM_ADDR_BASE        0x35 //base address of LSM [0:6]
#define LSM_ADDR_FULL1       0x6A //addr option 1 [0]
#define LSM_ADDR_FULL2       0x6B //addr option 2 [1]
#define LSM_ADDR_FULL1_R     0xD5 //addr1 read
#define LSM_ADDR_FULL1_W     0xD4 //addr1 write
#define LSM_ADDR_FULL2_R     0xD7 //addr2 read
#define LSM_ADDR_FULL2_W     0xD6 //addr2 write


//control and output register addresses

#define SENSOR_SYNC_TIME_FRAME   0x04 //Sensor sync configuration register
#define FIFO_CTRL1               0x06
#define FIFO_CTRL2               0x07
#define FIFO_CTRL3               0x08
#define FIFO_CTRL4               0x09
#define FIFO_CTRL5               0x0A
#define ORIENT_CFG_G             0x0B
#define INT1_CTRL                0x0D
#define INT2_CTRL                0x0E
#define WHO_AM_I                 0x0F
#define CTRL1_XL                 0x10
#define CTRL2_G                  0x11
#define CTRL3_C                  0x12
#define CTRL4_C                  0x13
#define CTRL5_C                  0x14
#define CTRL6_C                  0x15
#define CTRL7_G                  0x16
#define CTRL8_XL                 0x17
#define CTRL9_XL                 0x18
#define CTRL10_C                 0x19
#define MASTER_CONFIG            0x1A
#define WAKE_UP_SRC              0x1B
#define TAP_SRC                  0x1C
#define D6D_SRC                  0x1D
#define STATUS_REG               0x1E
#define OUT_TEMP_L               0x20
#define OUT_TEMP_H               0x21
#define OUTX_L_G                 0x22
#define OUTX_H_G                 0x23
#define OUTY_L_G                 0x24
#define OUTY_H_G                 0x25
#define OUTZ_L_G                 0x26
#define OUTZ_H_G                 0x27
#define OUTX_L_XL                0x28
#define OUTX_H_XL                0x29
#define OUTY_L_XL                0x2A
#define OUTY_H_XL                0x2B
#define OUTZ_L_XL                0x2C
#define OUTZ_H_XL                0x2D


//sensor bank is skipped


#define FIFO_STATUS1             0x3A
#define FIFO_STATUS2             0x3B
#define FIFO_STATUS3             0x3C
#define FIFO_STATUS4             0x3D
#define FIFO_DATA_OUT_L          0x3E
#define FIFO_DATA_OUT_H          0x3F
#define TIMESTAMP0_REG           0x40
#define TIMESTAMP1_REG           0x41
#define TIMESTAMP2_REG           0x42

//sensor bank skipped

#define FUNC_SRC                 0x53
#define TAP_CFG                  0x58
#define TAP_THS_6D               0x59
#define INT_DUR2                 0x5A
#define WAKE_UP_THS              0x5B
#define WAKE_UP_DUR              0x5C
#define FREE_FALL                0x5D
#define MD1_CFG                  0x5E
#define MD2_CFG                  0x5F
#define OUT_MAG_RAW_X_L          0x66
#define OUT_MAG_RAW_X_H          0x67
#define OUT_MAG_RAW_Y_L          0x68
#define OUT_MAG_RAW_Y_H          0x69
#define OUT_MAG_RAW_Z_L          0x6A
#define OUT_MAG_RAW_Z_H          0x6B
#define CTRL_SPIAux              0x70


//Function declarations

void MEMS_process_data(uint8_t* MEMS_r_buff);
int MEMS_config_sleep(int* fd_i2c);// set MEMS option registers. Enable accel and gyro
int get_device_orientation(float dataBuff[], int buff_size);
int MEMS_Read(int* fd_i2c ,uint8_t* MEMS_r_buff);
float get_pitch(float AccelX, float AccelY, float AccelZ);
float get_roll(float AccelX, float AccelY, float AccelZ);
int createAccelReadThread(void);
void* accelRead_func(void* arg);
int killAccelReadThread(void); 
void MEMS_read_config(int* fd_i2c);
int MEMS_config_data(int* fd_i2c);
int check_rtc(int* fd_i2c);


void MEMS_process_moving_avg(int window, int step, int cbk_cnt_min, int cbk_global);
void MEMS_process_harsh_accel(int window, int step, double x_threshold,double y_threshold,
double gyr_Z_threshold, int cbk_cnt_min, int cbk_global);

#endif

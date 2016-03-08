/*
 * Definitions for dmard10 compass chip.
 */
#ifndef DMT10_H
#define DMT10_H

#include <linux/ioctl.h>


// only used reg was redefined by zhangyx 20110930

/* Default register settings */
#define RBUFF_SIZE		12	/* Rx buffer size */

#define REG_ACTR 				0x00
#define REG_WDAL 				0x01
#define REG_TAPNS				0x0f
#define REG_MISC2				0x1f
#define REG_AFEM 				0x0c
#define REG_CKSEL 				0x0d
#define REG_INTC 				0x0e
#define REG_STADR 				0x12
#define REG_STAINT 				0x1C
#define REG_PD					0x21
#define REG_TCGYZ				0x26
#define REG_X_OUT 				0x41

#define MODE_Off				0x00
#define MODE_ResetAtOff			0x01
#define MODE_Standby			0x02
#define MODE_ResetAtStandby		0x03
#define MODE_Active				0x06
#define MODE_Trigger			0x0a
#define MODE_ReadOTP			0x12
#define MODE_WriteOTP			0x22
#define MODE_WriteOTPBuf		0x42
#define MODE_ResetDataPath		0x82

#define VALUE_STADR					0x55
#define VALUE_STAINT 				0xAA
#define VALUE_AFEM_AFEN_Normal		0x8f// AFEN set 1 , ATM[2:0]=b'000(normal),EN_Z/Y/X/T=1
#define VALUE_AFEM_Normal			0x0f// AFEN set 0 , ATM[2:0]=b'000(normal),EN_Z/Y/X/T=1
#define VALUE_INTC					0x00// INTC[6:5]=b'00 
#define VALUE_INTC_Interrupt_En		0x20// INTC[6:5]=b'01 (Data ready interrupt enable, active high at INT0)
#define VALUE_CKSEL_ODR_0_204		0x04// ODR[3:0]=b'0000 (0.78125Hz), CCK[3:0]=b'0100 (204.8kHZ)
#define VALUE_CKSEL_ODR_1_204		0x14// ODR[3:0]=b'0001 (1.5625Hz), CCK[3:0]=b'0100 (204.8kHZ)
#define VALUE_CKSEL_ODR_3_204		0x24// ODR[3:0]=b'0010 (3.125Hz), CCK[3:0]=b'0100 (204.8kHZ)
#define VALUE_CKSEL_ODR_6_204		0x34// ODR[3:0]=b'0011 (6.25Hz), CCK[3:0]=b'0100 (204.8kHZ)
#define VALUE_CKSEL_ODR_12_204		0x44// ODR[3:0]=b'0100 (12.5Hz), CCK[3:0]=b'0100 (204.8kHZ)
#define VALUE_CKSEL_ODR_25_204		0x54// ODR[3:0]=b'0101 (25Hz), CCK[3:0]=b'0100 (204.8kHZ)
#define VALUE_CKSEL_ODR_50_204		0x64// ODR[3:0]=b'0110 (50Hz), CCK[3:0]=b'0100 (204.8kHZ)
#define VALUE_CKSEL_ODR_100_204		0x74// ODR[3:0]=b'0111 (100Hz), CCK[3:0]=b'0100 (204.8kHZ)

#define VALUE_TAPNS_NoFilter	0x00	// TAP1/TAP2	NO FILTER
#define VALUE_TAPNS_Ave_2		0x11	// TAP1/TAP2	Average 2
#define VALUE_TAPNS_Ave_4		0x22	// TAP1/TAP2	Average 4
#define VALUE_TAPNS_Ave_8		0x33	// TAP1/TAP2	Average 8
#define VALUE_TAPNS_Ave_16		0x44	// TAP1/TAP2	Average 16
#define VALUE_TAPNS_Ave_32		0x55	// TAP1/TAP2	Average 32
#define VALUE_MISC2_OSCA_EN		0x08
#define VALUE_PD_RST			0x52


//#define DMARD10_REG_INTSU        0x47
//#define DMARD10_REG_MODE        0x44
//#define DMARD10_REG_SR               0x44


#define DMARD10_REG_DS      0X49
#define DMARD10_REG_ID       0X0F
#define DMARD10_REG_IT       0X4D
#define DMARD10_REG_INTSRC1_C       0X4A
#define DMARD10_REG_INTSRC1_S       0X4B
#define MMAIO				'a'

// IOCTLs for DMARD10 library 
#define ECS_IOCTL_INIT                  _IO(MMAIO, 0x01)
#define ECS_IOCTL_RESET      	        _IO(MMAIO, 0x04)
#define ECS_IOCTL_CLOSE		        _IO(MMAIO, 0x02)
#define ECS_IOCTL_START		        _IO(MMAIO, 0x03)
#define ECS_IOCTL_GETDATA               _IOR(MMAIO, 0x08, char[RBUFF_SIZE+1])
#define SENSOR_CALIBRATION   		_IOWR(MMAIO, 0x05 , int[SENSOR_DATA_SIZE])
	 
// IOCTLs for APPs 
#define ECS_IOCTL_APP_SET_RATE		_IOW(MMAIO, 0x10, char)

 //rate
#define DMARD10_RANGE						2000000

#define DMARD10_RATE_32         32
/*
#define DMARD10_RATE_64         64
#define DMARD10_RATE_120        128
#define DMARD10_RATE_MIN		DMARD10_RATE_1
#define DMARD10_RATE_MAX		DMARD10_RATE_120
*/
/*status*/
#define DMARD10_OPEN               1
#define DMARD10_CLOSE              0
#define DMARD10_NORMAL	      	   2
#define DMARD10_LOWPOWER  	   3



#define DMARD10_IIC_ADDR 	    0x18  
#define DMARD10_REG_LEN         11
#define DMARD10_RANGE						2000000
//#define DMARD10_PRECISION       10
#define DMARD10_BOUNDARY        (0x1 << (DMARD10_PRECISION - 1))
//#define DMARD10_GRAVITY_STEP    DMARD10_RANGE/DMARD10_BOUNDARY


#define DMARD10_FATOR	15 


#define DMARD10_X_OUT 		0x41
#define SENSOR_DATA_SIZE 3
#define DMARD10_SENSOR_RATE_1   0
#define DMARD10_SENSOR_RATE_2   1
#define DMARD10_SENSOR_RATE_3   2
#define DMARD10_SENSOR_RATE_4   3

#define POWER_OR_RATE 1
#define SW_RESET 1
#define DMARD10_INTERRUPUT 1
#define DMARD10_POWERDOWN 0 
#define DMARD10_POWERON 1 

//g-senor layout configuration, choose one of the following configuration

#define AVG_NUM 			16
#define SENSOR_DATA_SIZE 		3 
#define DEFAULT_SENSITIVITY 		1024


//#define DMARD10_TOTAL_TIME      10



/*struct DMARD10_platform_data {
	int reset;
	int clk_on;
	int intr;
};*/


struct dmard10_platform_data {
	int reset;
	int swap_xy;
	int	swap_xyz;
	signed char orientation[9];
	int clk_on;
	int intr;
};

typedef union {
	struct {
		s16	x;
		s16	y;
		s16	z;
	} u;
	s16	v[SENSOR_DATA_SIZE];
} raw_data;

#define  GSENSOR_DEV_PATH    "/dev/dmard10"
#endif


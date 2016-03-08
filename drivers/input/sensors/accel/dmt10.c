/* drivers/input/sensors/accel/dmt10.c
 *
 * Copyright (C) 2012-2015 ROCKCHIP.
 * Author: luowei <lw@rock-chips.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <mach/gpio.h>
#include <mach/board.h> 
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/sensor-dev.h>
#include "dmt10.h"

#define DMARD10_ENABLE		1

#define DMARD10_REG_X_OUT       0x12
#define DMARD10_REG_Y_OUT       0x1
#define DMARD10_REG_Z_OUT       0x2
#define DMARD10_REG_TILT        0x3
#define DMARD10_REG_SRST        0x4
#define DMARD10_REG_SPCNT       0x5
#define DMARD10_REG_INTSU       0x6
#define DMARD10_REG_MODE        0x7
#define DMARD10_REG_SR          0x8
#define DMARD10_REG_PDET        0x9
#define DMARD10_REG_PD          0xa

#define DMARD10_REG_LEN         11
#define DMARD10_GRAVITY_STEP    47

#define DMARD10_PRECISION       6//   ?? 
#define DMARD10_BOUNDARY        (0x1 << (DMARD10_PRECISION  - 1)) 
/*
struct sensor_axis_average {
		int x_average;
		int y_average;
		int z_average;
		int count;
};

static struct sensor_axis_average axis_average;
*/
static raw_data	offset;
static int firsttime = 0;

char DMT_OffsetFileName[] = "/data/misc/dmt/offset.txt";	/* FILE offset.txt */
int DMT_read_offset_from_file(void)
{
	unsigned int orgfs;
	char data[18];
	struct file *fp;
	int ux,uy,uz;
	orgfs = get_fs();
	/* Set segment descriptor associated to kernel space */
	set_fs(KERNEL_DS);

	fp = filp_open(DMT_OffsetFileName, O_RDWR , 0);

	if(IS_ERR(fp)){
		set_fs(orgfs);
		//GSE_FUN();
		return (-1);
	} else{
		printk("filp_open %s SUCCESS!!.\n",DMT_OffsetFileName);
		fp->f_op->read(fp,data,18, &fp->f_pos);
		printk("filp_read result %s\n",data);
		sscanf(data,"%d %d %d",&ux,&uy,&uz);
		offset.v[0] = ux;
		offset.v[1] = uy;
		offset.v[2] = uz;
		//axis_offset.offset_found = 1;
		filp_close(fp,NULL);
	}
	set_fs(orgfs);
	return 0;
}

void DMT_write_offset_to_file(void){
	char data[18];
	unsigned int orgfs;
	struct file *fp;

	sprintf(data,"%5d %5d %5d", offset.v[0], offset.v[1], offset.v[2]);
	orgfs = get_fs();
	/* Set segment descriptor associated to kernel space */
	set_fs(KERNEL_DS);
	fp = filp_open(DMT_OffsetFileName, O_RDWR | O_CREAT, 0777);
	if(IS_ERR(fp)){
		printk("filp_open %s error!!.\n",DMT_OffsetFileName);
	}
	else{
		printk("filp_open %s SUCCESS!!.\n",DMT_OffsetFileName);
		fp->f_op->write(fp,data,18, &fp->f_pos);
 		filp_close(fp,NULL);
	}
	set_fs(orgfs);
}

int gsensor_reset(struct i2c_client *client){
	char buffer[7], buffer2[2];
	/* 1. check D10 , VALUE_STADR = 0x55 , VALUE_STAINT = 0xAA */
	buffer[0] = REG_STADR;
	buffer2[0] = REG_STAINT;
	
	sensor_rx_data(client, buffer, 2);
	sensor_rx_data(client, buffer2, 2);
		
	if( buffer[0] == VALUE_STADR || buffer2[0] == VALUE_STAINT){
		printk(KERN_INFO " REG_STADR_VALUE = %d , REG_STAINT_VALUE = %d\n", buffer[0], buffer2[0]);
		printk(KERN_INFO " %s DMT_DEVICE_NAME registered I2C driver!\n",__FUNCTION__);
	}
	else{
		printk(KERN_INFO " %s gsensor I2C err @@@ REG_STADR_VALUE = %d , REG_STAINT_VALUE = %d \n", __func__, buffer[0], buffer2[0]);
		return -1;
	}
	/* 2. Powerdown reset */
	buffer[0] = REG_PD;
	buffer[1] = VALUE_PD_RST;
	sensor_tx_data(client, buffer, 2);
	/* 3. ACTR => Standby mode => Download OTP to parameter reg => Standby mode => Reset data path => Standby mode */
	buffer[0] = REG_ACTR;
	buffer[1] = MODE_Standby;
	buffer[2] = MODE_ReadOTP;
	buffer[3] = MODE_Standby;
	buffer[4] = MODE_ResetDataPath;
	buffer[5] = MODE_Standby;
	sensor_tx_data(client, buffer, 6);
	/* 4. OSCA_EN = 1 ,TSTO = b'000(INT1 = normal, TEST0 = normal) */
	buffer[0] = REG_MISC2;
	buffer[1] = VALUE_MISC2_OSCA_EN;
	sensor_tx_data(client, buffer, 2);
	/* 5. AFEN = 1(AFE will powerdown after ADC) */
	buffer[0] = REG_AFEM;
	buffer[1] = VALUE_AFEM_AFEN_Normal;	
	buffer[2] = VALUE_CKSEL_ODR_100_204;	
	buffer[3] = VALUE_INTC;	
	buffer[4] = VALUE_TAPNS_Ave_2;
	buffer[5] = 0x00;	// DLYC, no delay timing
	buffer[6] = 0x07;	// INTD=1 (push-pull), INTA=1 (active high), AUTOT=1 (enable T)
	sensor_tx_data(client, buffer, 7);
	/* 6. write TCGYZ & TCGX */
	buffer[0] = REG_WDAL;	// REG:0x01
	buffer[1] = 0x00;		// set TC of Y,Z gain value
	buffer[2] = 0x00;		// set TC of X gain value
	buffer[3] = 0x03;		// Temperature coefficient of X,Y,Z gain
	sensor_tx_data(client, buffer, 4);
	
	buffer[0] = REG_ACTR;			// REG:0x00
	buffer[1] = MODE_Standby;		// Standby
	buffer[2] = MODE_WriteOTPBuf;	// WriteOTPBuf 
	buffer[3] = MODE_Standby;		// Standby
	sensor_tx_data(client, buffer, 4);	
	//buffer[0] = REG_TCGYZ;
	//device_i2c_rxdata(client, buffer, 2);
	//GSE_LOG(" TCGYZ = %d, TCGX = %d  \n", buffer[0], buffer[1]);
	
	/* 7. Activation mode */
	buffer[0] = REG_ACTR;
	buffer[1] = MODE_Active;
	sensor_tx_data(client, buffer, 2);
	printk("\n dmard10 gsensor_reset SUCCESS!!\n");
	return 0;
}

/****************operate according to sensor chip:start************/

static int sensor_active(struct i2c_client *client, int enable, int rate)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	int result = 0;
	int status = 0;

	sensor->ops->ctrl_data = sensor_read_reg(client, sensor->ops->ctrl_reg);
	//register setting according to chip datasheet		
	if(enable)
	{	
		status = DMARD10_ENABLE;	//dmard10
		sensor->ops->ctrl_data |= status;	
	}
	else
	{
		status = ~DMARD10_ENABLE;	//dmard10
		sensor->ops->ctrl_data &= status;
	}

	DBG("%s:reg=0x%x,reg_ctrl=0x%x,enable=%d\n",__func__,sensor->ops->ctrl_reg, sensor->ops->ctrl_data, enable);
	result = sensor_write_reg(client, sensor->ops->ctrl_reg, sensor->ops->ctrl_data);
	if(result)
		printk("%s:fail to active sensor\n",__func__);
	
	return result;

}

static int sensor_init(struct i2c_client *client)
{	
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	int result = 0;
	
	result = sensor->ops->active(client,0,0);
	if(result)
	{
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return result;
	}
	
	sensor->status_cur = SENSOR_OFF;

	DBG("%s:DMARD10_REG_TILT=0x%x\n",__func__,sensor_read_reg(client, DMARD10_REG_TILT));

	result = sensor_write_reg(client, DMARD10_REG_SR, (0x01<<5)| 0x02);	//32 Samples/Second Active and Auto-Sleep Mode
	if(result)
	{
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return result;
	}

	if(sensor->pdata->irq_enable)	//open interrupt
	{
		result = sensor_write_reg(client, DMARD10_REG_INTSU, 1<<4);//enable int,GINT=1
		if(result)
		{
			printk("%s:line=%d,error\n",__func__,__LINE__);
			return result;
		}
	}
	
	sensor->ops->ctrl_data = 1<<6;	//Interrupt output INT is push-pull
	result = sensor_write_reg(client, sensor->ops->ctrl_reg, sensor->ops->ctrl_data);
	if(result)
	{
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return result;
	}
	firsttime =0 ;
	gsensor_reset(client);
	
	//memset(&axis_average, 0, sizeof(struct sensor_axis_average));

	return result;
}

/*
static int sensor_convert_data(struct i2c_client *client, char high_byte, char low_byte)
{
    s64 result;
	
	//struct sensor_private_data *sensor =
	//    (struct sensor_private_data *) i2c_get_clientdata(client);	
	//int precision = sensor->ops->precision;
//	result=(u16)high_byte) << 8) | (u16)low_byte;
//	result&=0x
			
			result = ((int)high_byte << 8)|((int)low_byte);
			result<<=3;
			return result;

}
*/
static int gsensor_report_value(struct i2c_client *client, struct sensor_axis *axis)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	if(!firsttime){
		printk("firsttime = %d\n", firsttime);	
		msleep(4444);
		DMT_read_offset_from_file();
	 	firsttime = 1;	
	}
	/* Report acceleration sensor information */
	input_report_abs(sensor->input_dev, ABS_X, axis->x);
	input_report_abs(sensor->input_dev, ABS_Y, axis->y);
	input_report_abs(sensor->input_dev, ABS_Z, axis->z);
	input_sync(sensor->input_dev);

	return 0;
}

static int D10_i2c_read_xyz(struct i2c_client *client, s16 *xyz_p){
	struct sensor_private_data *sensor =
		(struct sensor_private_data *) i2c_get_clientdata(client);	
	//struct sensor_platform_data *pdata = sensor->pdata;
	int i,ret = 0;
	char buffer[8] = {0};	
	if(sensor->ops->read_len < 3)	//sensor->ops->read_len = 3
	{
		return -1;
	}
	
	memset(buffer, 0, 8);
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */	
	do {
		*buffer = sensor->ops->read_reg;
		ret = sensor_rx_data(client, buffer, sensor->ops->read_len);
		if (ret < 0)
		return ret;
	} while (0);

	/* merge xyz high/low bytes & 1g = 128 becomes 1g = 1024 */
	for(i = 0; i < SENSOR_DATA_SIZE; ++i){
		xyz_p[i] = ((int16_t)((buffer[2*(i+1)+1] << 8)) | buffer[2*(i+1)] ) << 3;

	}
	/* axis = RawData - Offset */
	for(i = 0; i < SENSOR_DATA_SIZE; ++i)
		xyz_p[i] -= offset.v[i];
	return 0;
}

#define DMARD10_COUNT_AVERAGE 2
#define GSENSOR_MIN  		2
static int sensor_report_value(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
		(struct sensor_private_data *) i2c_get_clientdata(client);	
	struct sensor_platform_data *pdata = sensor->pdata;
	int i,ret = 0;
	struct sensor_axis axis;
	char value = 0;
	raw_data xyz;
	int x,y,z;
		
	ret = D10_i2c_read_xyz(client, (s16 *)&xyz.v);
	if (ret < 0)
		return ret;
		
	x = (xyz.v[0])<<10;
	y = (xyz.v[1])<<10;
	z = (xyz.v[2])<<10;

	axis.x = (pdata->orientation[0])*x + (pdata->orientation[1])*y + (pdata->orientation[2])*z;
	axis.y = (pdata->orientation[3])*x + (pdata->orientation[4])*y + (pdata->orientation[5])*z; 
	axis.z = (pdata->orientation[6])*x + (pdata->orientation[7])*y + (pdata->orientation[8])*z;
	 
	if((abs(sensor->axis.x - axis.x) > GSENSOR_MIN) || (abs(sensor->axis.y - axis.y) > GSENSOR_MIN) || (abs(sensor->axis.z - axis.z) > GSENSOR_MIN))
		{
			gsensor_report_value(client, &axis);
			//mutex_lock(&(sensor->data_mutex) );
			//sensor->axis = axis;
			//mutex_unlock(&(sensor->data_mutex) );
			DBG( "axis = %d  %d  %d \n", axis.x, axis.y, axis.z);
		}
/*
	
	axis_average.x_average += axis.x;
	axis_average.y_average += axis.y;
	axis_average.z_average += axis.z;
	axis_average.count++;
	
	if(axis_average.count >= DMARD10_COUNT_AVERAGE)
	{
		axis.x = axis_average.x_average / axis_average.count;	
		axis.y = axis_average.y_average / axis_average.count;	
		axis.z = axis_average.z_average / axis_average.count;
		
		DBG( "%s: axis = %d  %d  %d \n", __func__, axis.x, axis.y, axis.z);
		
		memset(&axis_average, 0, sizeof(struct sensor_axis_average));
		
		//Report event only while value is changed to save some power
		if((abs(sensor->axis.x - axis.x) > GSENSOR_MIN) || (abs(sensor->axis.y - axis.y) > GSENSOR_MIN) || (abs(sensor->axis.z - axis.z) > GSENSOR_MIN))
		{
			gsensor_report_value(client, &axis);

			mutex_lock(&(sensor->data_mutex) );
			sensor->axis = axis;
			mutex_unlock(&(sensor->data_mutex) );
		}
	}
	
	if((sensor->pdata->irq_enable)&& (sensor->ops->int_status_reg >= 0))	//read sensor intterupt status register
	{
		
		value = sensor_read_reg(client, sensor->ops->int_status_reg);
		DBG("%s:sensor int status :0x%x\n",__func__,value);
	}
*/	
	return ret;
}

static int sensor_calibration(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
		(struct sensor_private_data *) i2c_get_clientdata(client);	
	//struct sensor_platform_data *pdata = sensor->pdata;
	raw_data avg;
	long xyz_acc[SENSOR_DATA_SIZE]; 
	s16 xyz[SENSOR_DATA_SIZE];
	int i, j, ret = 0;

	/* initialize the offset value */
	for(i = 0; i < SENSOR_DATA_SIZE; ++i)
		offset.v[i] = 0;
	/* initialize the accumulation buffer */
	for(i = 0; i < SENSOR_DATA_SIZE; ++i)
		xyz_acc[i] = 0;
	/* get rawdata of AVG_NUM */
	for(i = 0; i < AVG_NUM; i++) {      
		ret = D10_i2c_read_xyz(client, (s16 *)&xyz);
		if (ret < 0)
			return ret;
		for(j = 0; j < SENSOR_DATA_SIZE; ++j)
			xyz_acc[j] += xyz[j];
	}

	/* calculate averages */
	for(i = 0; i < SENSOR_DATA_SIZE; i++)
		avg.v[i] = (s16) (xyz_acc[i] / AVG_NUM);
	/* calculate offset */
	offset.v[0] = avg.v[0]; 
	offset.v[1] = avg.v[1]; 
	if (avg.v[2] < 0)
		offset.v[2] = (avg.v[2] + DEFAULT_SENSITIVITY); 
	else
		offset.v[2] = (avg.v[2] - DEFAULT_SENSITIVITY); 

	DMT_write_offset_to_file();
	return 0;
}


struct sensor_operate gsensor_dmard10_ops = {
	.name				= "dmard10",
	.type				= SENSOR_TYPE_ACCEL,			//sensor type and it should be correct
	.id_i2c				= ACCEL_ID_DMARD10,			//i2c id number
	.read_reg			= DMARD10_REG_X_OUT,			//read data
	.read_len			= 8,					//data length
	.id_reg				= SENSOR_UNKNOW_DATA,			//read device id from this register
	.id_data 			= SENSOR_UNKNOW_DATA,			//device id
	.precision			= DMARD10_PRECISION,			//12 bit
	.ctrl_reg 			= DMARD10_REG_MODE,			//enable or disable 	
	.int_status_reg 		= SENSOR_UNKNOW_DATA,			//intterupt status register
	.range				= {-DMARD10_RANGE,DMARD10_RANGE},	//range
	.trig				= IRQF_TRIGGER_LOW|IRQF_ONESHOT,
	.active				= sensor_active,	
	.init				= sensor_init,
	.report 			= sensor_report_value,
	//.calibration		= sensor_calibration,
};

/****************operate according to sensor chip:end************/

//function name should not be changed
static struct sensor_operate *gsensor_get_ops(void)
{
	return &gsensor_dmard10_ops;
}


static int __init gsensor_dmard10_init(void)
{
	struct sensor_operate *ops = gsensor_get_ops();
	int result = 0;
	int type = ops->type;
	result = sensor_register_slave(type, NULL, NULL, gsensor_get_ops);	
	return result;
}

static void __exit gsensor_dmard10_exit(void)
{
	struct sensor_operate *ops = gsensor_get_ops();
	int type = ops->type;
	sensor_unregister_slave(type, NULL, NULL, gsensor_get_ops);
}


module_init(gsensor_dmard10_init);
module_exit(gsensor_dmard10_exit);




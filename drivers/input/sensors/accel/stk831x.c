/* drivers/input/sensors/access/stk831x.c
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
//#include <linux/stk831x.h>
#include <linux/sensor-dev.h>

//#define STK_ALLWINNER_PLATFORM
#define STK_ACC_DRIVER_VERSION	"1.6.1"
/*choose interrupt mode*/
#define ADDITIONAL_GPIO_CFG 1
#define STK_INT_PIN	RK30_PIN4_PC0


#ifdef CONFIG_SENSORS_STK8313
	#include "stk8313.h"
#elif defined CONFIG_SENSORS_STK8312
	#include "stk8312.h"
#else
	#error "What's your stk accelerometer?"
#endif

#define STK831X_INIT_ODR		1		//0:100Hz, 1:50Hz, 2:25Hz
#define STK831X_SAMPLE_TIME_BASE		2
#define STK831X_SAMPLE_TIME_NO		4


#define MMA8452_ENABLE		1
#define STK831X_RANGE			2000000


static int STK831x_ReadByteOTP(struct i2c_client *client,char rReg, char *value)
{
	int redo = 0;
	int result;
	char buffer[2] = "";
	*value = 0;
	
	result = sensor_write_reg(client,0x3D,rReg);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed\n", __func__);
		goto eng_i2c_r_err;
	}
	result = sensor_write_reg(client,0x3F,0x02);
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed\n", __func__);
		goto eng_i2c_r_err;
	}
	
	msleep(1);	
	do {
		buffer[0] = 0x3F;
		result = sensor_rx_data(client, &buffer[0],1);	
		if (result < 0) 
		{
			printk(KERN_ERR "%s:failed\n", __func__);
			goto eng_i2c_r_err;
		}
		if(buffer[0]& 0x80)
		{
			break;
		}		
		msleep(1);
		redo++;
	}while(redo < 5);
	
	if(redo == 5)
	{
		printk(KERN_ERR "%s:OTP read repeat read 5 times! Failed!\n", __func__);
		return -1;
	}	
	buffer[0] = 0x3E;
	result = sensor_rx_data(client, &buffer[0],1);	
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed\n", __func__);
		goto eng_i2c_r_err;
	}	
	*value = buffer[0];
	return 0;
	
eng_i2c_r_err:	
	return -1;	
}

static int STK831X_SetVD(struct i2c_client *client)
{
	int result;
	char buffer[2] = "";
	char reg24;
	
	msleep(2);
	result = STK831x_ReadByteOTP(client,0x70, &reg24);
	if(result < 0)
	{
		printk(KERN_ERR "%s: read back error, result=%d\n", __func__, result);
		return result;
	}
	
	if(reg24 != 0)
	{
		result = sensor_write_reg(client,0x24,reg24);
		if (result < 0) 
		{
			printk(KERN_ERR "%s:failed\n", __func__);
			return result;
		}
	}	
	else
	{
		//printk(KERN_INFO "%s: reg24=0, do nothing\n", __func__);
		return 0;
	}
	
	buffer[0] = 0x24;
	result = sensor_rx_data(client,&buffer[0], 1);	
	if (result < 0) 
	{
		printk(KERN_ERR "%s:failed\n", __func__);
		return result;
	}				
	if(buffer[0] != reg24)
	{
		printk(KERN_ERR "%s: error, reg24=0x%x, read=0x%x\n", __func__, reg24, buffer[0]);
		return -1;
	}
	printk(KERN_INFO "%s: successfully", __func__);
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
		sensor->ops->ctrl_data &= 0xF8;
		sensor->ops->ctrl_data |= 1;
	}
	else
	{
		sensor->ops->ctrl_data &= 0xF8;
	}

	DBG("%s:reg=0x%x,reg_ctrl=0x%x,enable=%d\n",__func__,sensor->ops->ctrl_reg, sensor->ops->ctrl_data, enable);
	result = sensor_write_reg(client, sensor->ops->ctrl_reg, sensor->ops->ctrl_data);
	if(result)
		printk("%s:fail to active sensor\n",__func__);
	
	if(enable)
	{	
		STK831X_SetVD(client);
	}
	return result;

}

static int sensor_init(struct i2c_client *client)
{
	int tmp;
	int ret = 0;
	char buffer[2] = {0};
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	
	ret = sensor->ops->active(client,0,0);
	if(ret)
	{
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return ret;
	}
	
	sensor->status_cur = SENSOR_OFF;
	
	sensor_write_reg(client,STK831X_RESET,0x00);

	/* int pin is active high, psuh-pull */
	sensor_write_reg(client,STK831X_MODE,0xC0);
	
	/* 50 Hz ODR */
	sensor_write_reg(client,STK831X_SR,3);
	
	/* enable GINT, int after every measurement */
	sensor_write_reg(client,STK831X_INTSU,0x10);
	
	/* +- 6g mode */
#ifdef CONFIG_SENSORS_STK8312	
	sensor_write_reg(client,STK831X_STH,0x42);
#elif defined CONFIG_SENSORS_STK8313
	sensor_write_reg(client,STK831X_STH,0x82);
#endif	
	
#ifdef STK_TUNE	
	stk_tune_offset[0] = 0;
	stk_tune_offset[1] = 0;
	stk_tune_offset[2] = 0;	
	stk_tune_done = 0;
#endif	
	return ret;
}

static int gsensor_report_value(struct i2c_client *client, struct sensor_axis *axis)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	

	/* Report acceleration sensor information */
	input_report_abs(sensor->input_dev, ABS_X, axis->x);
	input_report_abs(sensor->input_dev, ABS_Y, axis->y);
	input_report_abs(sensor->input_dev, ABS_Z, axis->z);
	input_sync(sensor->input_dev);
	DBG("Gsensor x==%d  y==%d z==%d\n",axis->x,axis->y,axis->z);

	return 0;
}

#define GSENSOR_MIN  		10

#ifdef CONFIG_SENSORS_STK8312
static int sensor_report_value(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
		(struct sensor_private_data *) i2c_get_clientdata(client);	
	struct sensor_platform_data *pdata = sensor->pdata;
	int ret = 0;
	int x,y,z;
	struct sensor_axis axis;
	char buffer[3] = {0};	
	int acc_xyz[3] = {0};
	char value = 0;
	
	if(sensor->ops->read_len < 3)	//sensor->ops->read_len = 6
	{
		printk("%s:lenth is error,len=%d\n",__func__,sensor->ops->read_len);
		return -1;
	}
	
	memset(buffer, 0, 3);
	
	do {
		*buffer = sensor->ops->read_reg;
		ret = sensor_rx_data(client, buffer, sensor->ops->read_len);
		if (ret < 0)
		return ret;
	} while (0);

	if (buffer[0] & 0x80)
		acc_xyz[0] = buffer[0] - 256;
	else
		acc_xyz[0] = buffer[0];
	if (buffer[1] & 0x80)
		acc_xyz[1] = buffer[1] - 256;
	else
		acc_xyz[1] = buffer[1];
	if (buffer[2] & 0x80)
		acc_xyz[2] = buffer[2] - 256;
	else
		acc_xyz[2] = buffer[2];

	//this gsensor need 6 bytes buffer
	//x = (acc_xyz[0]*STK831X_RANGE)>>7;
	//y = (acc_xyz[1]*STK831X_RANGE)>>7;
	//z = (acc_xyz[2]*STK831X_RANGE)>>7;
	x = acc_xyz[0]<<15;
	y = acc_xyz[1]<<15;
	z = acc_xyz[2]<<15;
	printk("x = %02d, y = %02d, z = %02d \n",acc_xyz[0],acc_xyz[1],acc_xyz[2]);
	//printk("x = %02d, y = %02d, z = %02d \n",buffer[0],buffer[1],buffer[2]);
	//printk("x = %02d, y = %02d, z = %02d \n",x,y,z);

	axis.x = (pdata->orientation[0])*x + (pdata->orientation[1])*y + (pdata->orientation[2])*z;
	axis.y = (pdata->orientation[3])*x + (pdata->orientation[4])*y + (pdata->orientation[5])*z; 
	axis.z = (pdata->orientation[6])*x + (pdata->orientation[7])*y + (pdata->orientation[8])*z;

	DBG( "%s: axis = %d  %d  %d \n", __func__, axis.x, axis.y, axis.z);

	//Report event only while value is changed to save some power
	if((abs(sensor->axis.x - axis.x) > GSENSOR_MIN) || (abs(sensor->axis.y - axis.y) > GSENSOR_MIN) || (abs(sensor->axis.z - axis.z) > GSENSOR_MIN))
	{
		gsensor_report_value(client, &axis);

		/* 互斥地缓存数据. */
		mutex_lock(&(sensor->data_mutex) );
		sensor->axis = axis;
		mutex_unlock(&(sensor->data_mutex) );
	}

	if((sensor->pdata->irq_enable)&& (sensor->ops->int_status_reg >= 0))	//read sensor intterupt status register
	{
		
		value = sensor_read_reg(client, sensor->ops->int_status_reg);
		DBG("%s:sensor int status :0x%x\n",__func__,value);
	}
	
	return ret;
}
#elif defined CONFIG_SENSORS_STK8313
static int sensor_report_value(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
		(struct sensor_private_data *) i2c_get_clientdata(client);	
	struct sensor_platform_data *pdata = sensor->pdata;
	int ret = 0;
	int x,y,z;
	struct sensor_axis axis;
	char buffer[6] = {0};	
	int acc_xyz[3] = {0};
	char value = 0;
	
	if(sensor->ops->read_len < 6)	//sensor->ops->read_len = 6
	{
		printk("%s:lenth is error,len=%d\n",__func__,sensor->ops->read_len);
		return -1;
	}
	
	memset(buffer, 0, 6);
	
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */	
	do {
		*buffer = sensor->ops->read_reg;
		ret = sensor_rx_data(client, buffer, sensor->ops->read_len);
		if (ret < 0)
		return ret;
	} while (0);

	if (buffer[0] & 0x80)
		acc_xyz[0] = ((int)buffer[0]<<4) + (buffer[1]>>4) - 4096;
	else
		acc_xyz[0] = ((int)buffer[0]<<4) + (buffer[1]>>4);
	if (buffer[2] & 0x80)
		acc_xyz[1] = ((int)buffer[2]<<4) + (buffer[3]>>4) - 4096;
	else
		acc_xyz[1] = ((int)buffer[2]<<4) + (buffer[3]>>4);
	if (buffer[4] & 0x80)
		acc_xyz[2] = ((int)buffer[4]<<4) + (buffer[5]>>4) - 4096;
	else
		acc_xyz[2] = ((int)buffer[4]<<4) + (buffer[5]>>4);

	//this gsensor need 6 bytes buffer
	x = (acc_xyz[0]*STK831X_RANGE)>>9;
	y = (acc_xyz[1]*STK831X_RANGE)>>9;
	z = (acc_xyz[2]*STK831X_RANGE)>>9;

	axis.x = (pdata->orientation[0])*x + (pdata->orientation[1])*y + (pdata->orientation[2])*z;
	axis.y = (pdata->orientation[3])*x + (pdata->orientation[4])*y + (pdata->orientation[5])*z; 
	axis.z = (pdata->orientation[6])*x + (pdata->orientation[7])*y + (pdata->orientation[8])*z;

	DBG( "%s: axis = %d  %d  %d \n", __func__, axis.x, axis.y, axis.z);

	//Report event only while value is changed to save some power
	if((abs(sensor->axis.x - axis.x) > GSENSOR_MIN) || (abs(sensor->axis.y - axis.y) > GSENSOR_MIN) || (abs(sensor->axis.z - axis.z) > GSENSOR_MIN))
	{
		gsensor_report_value(client, &axis);

		/* 互斥地缓存数据. */
		mutex_lock(&(sensor->data_mutex) );
		sensor->axis = axis;
		mutex_unlock(&(sensor->data_mutex) );
	}

	if((sensor->pdata->irq_enable)&& (sensor->ops->int_status_reg >= 0))	//read sensor intterupt status register
	{
		
		value = sensor_read_reg(client, sensor->ops->int_status_reg);
		DBG("%s:sensor int status :0x%x\n",__func__,value);
	}
	
	return ret;
}
#endif

struct sensor_operate gsensor_stk831x_ops = {
	.name				= "stk831x",
	.type				= SENSOR_TYPE_ACCEL,			//sensor type and it should be correct
	.id_i2c				= ACCEL_ID_STK831X,			//i2c id number
	.read_reg			= STK831X_XOUT,		//read data
#ifdef CONFIG_SENSORS_STK8312
	.read_len			= 3,					//data length
#elif defined CONFIG_SENSORS_STK8313
	.read_len			= 6,					//data length
#endif
	.id_reg				= STK831X_DEVID,			//read device id from this register
#ifdef CONFIG_SENSORS_STK8312
	.id_data 			= 0x58,			//device id
#elif defined CONFIG_SENSORS_STK8313
	.id_data 			= SENSOR_UNKNOW_DATA,			//device id
#endif
#ifdef CONFIG_SENSORS_STK8312
	.precision			= 8,			//8 bit
#elif defined CONFIG_SENSORS_STK8313
	.precision			= 10,			//10 bit
#endif
	.ctrl_reg 			= STK831X_MODE,		//enable or disable 	
	.int_status_reg 		= SENSOR_UNKNOW_DATA,			//intterupt status register
	.range				= {-128,127},	//range
	.trig				= IRQF_TRIGGER_RISING,		
	.active				= sensor_active,	
	.init				= sensor_init,
	.report 			= sensor_report_value,
};

/****************operate according to sensor chip:end************/

//function name should not be changed
static struct sensor_operate *gsensor_get_ops(void)
{
	return &gsensor_stk831x_ops;
}


static int __init gsensor_stk831x_init(void)
{
	struct sensor_operate *ops = gsensor_get_ops();
	int result = 0;
	int type = ops->type;
	result = sensor_register_slave(type, NULL, NULL, gsensor_get_ops);	
	return result;
}

static void __exit gsensor_stk831x_exit(void)
{
	struct sensor_operate *ops = gsensor_get_ops();
	int type = ops->type;
	sensor_unregister_slave(type, NULL, NULL, gsensor_get_ops);
}


module_init(gsensor_stk831x_init);
module_exit(gsensor_stk831x_exit);




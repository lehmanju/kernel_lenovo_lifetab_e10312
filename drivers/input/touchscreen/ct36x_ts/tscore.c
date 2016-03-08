/* 
 * drivers/input/touchscreen/ct36x_ts.c
 *
 * VTL ct36x TouchScreen driver. 
 *
 * Copyright (c) 2010  VTL tech Ltd.
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
 *
 * George Chen, 2012-06-15
 */

// ****************************************************************************
// Includes
// ****************************************************************************

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/input.h>
// for android 4.x only
#include <linux/input/mt.h>

#include <linux/gpio.h>


#include "chip.h"
#include "tscore.h"
#include "platform.h"


#ifdef CONFIG_POWER_ON_CHARGER_DISPLAY
extern int charger_in_logo;
#endif

// ****************************************************************************
// Globel or static variables
// ****************************************************************************
static struct ct36x_ts_info	ct36x_ts;


// ****************************************************************************
// Function declaration
// ****************************************************************************
int ct36x_cmd_list_ind[] = { 
	CT36X_TS_CHIP_ID, 
	CT36X_TS_CHIP_RESET, 
	CT36X_TS_FW_VER, 
	CT36X_TS_FW_CHKSUM,
	CT36X_TS_FW_UPDATE,
	CT36X_TS_BIN_LOAD,
	CT36X_TS_BIN_VER,
	CT36X_TS_BIN_CHKSUM,
};

char ct36x_cmd_list_cmd[] = { 'i','r','v','c','u','l','b','k',0, };

static int ct36x_fs_load_bin(char *buffer, size_t count)
{
	int ret = -1;
	struct file* filp = NULL;
	loff_t offset = 0;
	mm_segment_t old_fs = get_fs();
	set_fs(get_ds());
	//fd = sys_open("/etc/firmware/ct36x_ts.bin", O_RDONLY, 0);
	filp = filp_open("/mnt/external_sd/firmware/ct36x_ts.bin", O_RDONLY, 0);
	if ( !IS_ERR(filp) ) {
		ret = vfs_read(filp, buffer, count, &offset);
		filp_close(filp, NULL);
	}
	set_fs(old_fs);

	return ret;
}

static int ct36x_fs_find_cmd(char *cmdlist, const char cmd)
{
	int i = 0;

	// search cmd
	while ( cmdlist[i] ) {
		if ( cmd == cmdlist[i] ) 
			return ct36x_cmd_list_ind[i];
		i++;
	}

	return -1;
}

static int ct36x_fs_open(struct inode *inode, struct file *file)
{
	if ( CT36X_TS_CORE_DEBUG )
	printk(">>>>> %s() called <<<<< \n", __FUNCTION__);
	
	return 0;
}

static int ct36x_fs_close(struct inode *inode, struct file *file)
{
	if ( CT36X_TS_CORE_DEBUG )
	printk(">>>>> %s() called <<<<< \n", __FUNCTION__);
	
	return 0;
}

static ssize_t ct36x_fs_write(struct file *file, const char __user *buffer, size_t count, loff_t *offset)
{
	int cmd = 0;
	int rslt = 0;
	
	if ( CT36X_TS_CORE_DEBUG ) {
	printk(">>>>> %s() called <<<<< \n", __FUNCTION__);
	}

	/* search cmd */
	cmd = ct36x_fs_find_cmd(ct36x_cmd_list_cmd, buffer[0]);

	/* execute cmd */
	if ( ct36x_ts.state == CT36X_STATE_NORMAL )
	switch ( cmd ) {
		case CT36X_TS_CHIP_ID:
		break;

		case CT36X_TS_CHIP_RESET:
		printk("%s(): CT36X_TS_CHIP_RESET\n", __FUNCTION__);
		ct36x_platform_hw_reset(&ct36x_ts);
		break;

		case CT36X_TS_FW_VER:
		break;

		case CT36X_TS_FW_CHKSUM:
		printk("%s(): CT36X_TS_FW_CHKSUM\n", __FUNCTION__);
		rslt = ct36x_chip_get_fwchksum(ct36x_ts.client, ct36x_ts.data.buf);
		printk("%s(): Fw checksum: 0x%x\n", __FUNCTION__, rslt);
		break;

		case CT36X_TS_FW_UPDATE:
		printk("%s(): CT36X_TS_FW_UPDATE\n", __FUNCTION__);
		ct36x_chip_go_bootloader(ct36x_ts.client, ct36x_ts.data.buf);
		break;

		case CT36X_TS_BIN_LOAD:
		printk("%s(): CT36X_TS_BIN_LOAD\n", __FUNCTION__);
		ct36x_fs_load_bin(binary_data, CT36X_CHIP_FLASH_SECTOR_NUM * CT36X_CHIP_FLASH_SECTOR_SIZE);
		break;

		case CT36X_TS_BIN_VER:
		break;

		case CT36X_TS_BIN_CHKSUM:
		printk("%s(): CT36X_TS_BIN_CHKSUM\n", __FUNCTION__);
		rslt = ct36x_chip_get_binchksum(ct36x_ts.data.buf);
		printk("%s(): bin checksum: 0x%x\n", __FUNCTION__, rslt);
		break;

		default:
		printk("%s(): No such command (0x%x). \n", __FUNCTION__, buffer[0]);
		break;
	}

	return count;
}

static ssize_t ct36x_fs_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
	int err = -1;
	
	if ( CT36X_TS_CORE_DEBUG ) {
	printk(">>>>> %s() called <<<<< \n", __FUNCTION__);
	printk("%s(): count=0x%x \n", __FUNCTION__, count);
	}

	if ( ct36x_ts.state == CT36X_STATE_NORMAL ) {
	//ct36x_ts_reg_read(ct36x_ts->client, buf[0], buf+1, buf[]);
	}

	return count;
}

static struct file_operations ct36x_ts_fops = {
	.owner = THIS_MODULE,
	.open = ct36x_fs_open,
	.release = ct36x_fs_close,
	.write = ct36x_fs_write,
	.read = ct36x_fs_read,
};

static irqreturn_t ct36x_ts_irq(int irq, void *dev)
{
	irqreturn_t ret = IRQ_NONE;
	struct ct36x_ts_info *ts;

	if ( CT36X_TS_CORE_DEBUG )
	printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	if ( ct36x_platform_is_ts_irq() ) {
		ts = (struct ct36x_ts_info *)dev;

		// touch device is ready??
		if ( ts->state == CT36X_STATE_NORMAL ) {
			// Disable ts interrupt
			disable_irq_nosync(ts->irq);
			queue_work(ts->workqueue, &ts->event_work);
		}
		ret = IRQ_HANDLED;
	}

	return ret;
}

static void ct36x_ts_workfunc(struct work_struct *work)
{
	int iter;
	int sync;
	int x, y;
	struct ct36x_ts_info *ts;

	if ( CT36X_TS_CORE_DEBUG )
	printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	ts = container_of(work, struct ct36x_ts_info, event_work);

	/* read touch points */
	ct36x_ts_reg_read(ts->client, ts->i2c_address, (char *) ts->data.pts, sizeof(struct ct36x_finger_info) * CT36X_TS_POINT_NUM);

	/* report points */
	sync = 0; ts->press = 0;
	for ( iter = 0; iter < CT36X_TS_POINT_NUM; iter++ ) {
		if ( ts->data.pts[iter].xhi != 0xFF && ts->data.pts[iter].yhi != 0xFF &&
		     (ts->data.pts[iter].status == 1 || ts->data.pts[iter].status == 2) ) {
		#ifdef CONFIG_TOUCHSCREEN_CT36X_MISC_XY_SWAP
			x = (ts->data.pts[iter].yhi<<4)|(ts->data.pts[iter].ylo&0xF);
			y = (ts->data.pts[iter].xhi<<4)|(ts->data.pts[iter].xlo&0xF);
		#else
			x = (ts->data.pts[iter].xhi<<4)|(ts->data.pts[iter].xlo&0xF);
		 	y = (ts->data.pts[iter].yhi<<4)|(ts->data.pts[iter].ylo&0xF);
		#endif
		#ifdef CONFIG_TOUCHSCREEN_CT36X_MISC_X_REVERSE
			x = CT36X_TS_ABS_X_MAX - x;
		#endif
		#ifdef CONFIG_TOUCHSCREEN_CT36X_MISC_Y_REVERSE
			y = CT36X_TS_ABS_Y_MAX - y;
		#endif
		
			if ( CT36X_TS_EVENT_DEBUG ) {
				printk("ID:       %d\n", ts->data.pts[iter].id);
				printk("status:   %d\n", ts->data.pts[iter].status);
				printk("X Lo:     %d\n", ts->data.pts[iter].xlo);
				printk("Y Lo:     %d\n", ts->data.pts[iter].ylo);
				printk("X Hi:     %d\n", ts->data.pts[iter].xhi);
				printk("Y Hi:     %d\n", ts->data.pts[iter].yhi);
				printk("X:        %d\n", x);
				printk("Y:        %d\n", y);
			}

		#if 1
			input_mt_slot(ts->input, ts->data.pts[iter].id - 1);
			input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
			input_report_abs(ts->input, ABS_MT_POSITION_X, x);
			input_report_abs(ts->input, ABS_MT_POSITION_Y, y);
			input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, 30);
			input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, 128);
		#else
			input_report_abs(ts->input, ABS_MT_TRACKING_ID, ts->data.pts[iter].id - 1);
			input_report_abs(ts->input, ABS_MT_POSITION_X,  x);
			input_report_abs(ts->input, ABS_MT_POSITION_Y,  y);
			input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, 30);
			input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, 128);
			
			input_mt_sync(ts->input);
		#endif

		
			sync = 1;
			ts->press |= 0x01 << (ts->data.pts[iter].id - 1);
		}
	}

	ts->release &= ts->release ^ ts->press;
	for ( iter = 0; iter < CT36X_TS_POINT_NUM; iter++ ) {
		if ( ts->release & (0x01<<iter) ) {
		#if 1	// android 4.x
			input_mt_slot(ts->input, iter);
			input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
		#else	// android 2.x / others
			input_report_abs(ts->input, ABS_MT_TRACKING_ID, iter);
			input_report_abs(ts->input, ABS_MT_POSITION_X,  x);
			input_report_abs(ts->input, ABS_MT_POSITION_Y,  y);
			input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, 0);

			input_mt_sync(ts->input);
		#endif
			sync = 1;
		}
	}
	ts->release = ts->press;

	if ( sync ) input_sync(ts->input);

	// Enable ts interrupt
	enable_irq(ts->irq);

}

static void ct36x_ts_adapter(int state)
{
	if ( CT36X_TS_CORE_DEBUG )
	printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	if ( ct36x_ts.state == CT36X_STATE_NORMAL ) {
		if ( state )
		ct36x_chip_set_adapter_on(ct36x_ts.client, ct36x_ts.data.buf);
		else
		ct36x_chip_set_adapter_off(ct36x_ts.client, ct36x_ts.data.buf);
	}
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ct36x_early_suspend(struct early_suspend *handler)
{
	struct ct36x_ts_info *ts;

	if (CT36X_TS_CORE_DEBUG)
	printk(">>>>> %s() called <<<<< \n", __FUNCTION__);
	
	ts = container_of(handler, struct ct36x_ts_info, early_suspend);

	ct36x_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void ct36x_early_resume(struct early_suspend *handler)
{
	struct ct36x_ts_info *ts;

	if (CT36X_TS_CORE_DEBUG)
	printk(">>>>> %s() called <<<<< \n", __FUNCTION__);
	
	ts = container_of(handler, struct ct36x_ts_info, early_suspend);
	
	ct36x_ts_resume(ts->client);
}
#endif

int ct36x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = -1;
	int binchksum, fwchksum;
	int updcnt;
	struct ct36x_ts_info *ts;
	//struct ct36x_platform_data *pdata;
	struct device *dev;
	unsigned char buf[10];

	if ( CT36X_TS_CORE_DEBUG )
	printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

#ifdef CONFIG_POWER_ON_CHARGER_DISPLAY
    if (charger_in_logo == 1) {
        return -ENODEV;
    }
#endif

	ct36x_ts.state = CT36X_STATE_INIT;

	dev = &client->dev;

	ct36x_ts.pdata = dev->platform_data;

	/* this code segment is platform dependent */
#ifdef CONFIG_TOUCHSCREEN_CT36X_PLATFORM_GENERIC
	if ( ct36x_ts.pdata ) {
		/* for platform data driver */
		ct36x_ts.client = client;

		ct36x_platform_get_cfg(&ct36x_ts);
		i2c_set_clientdata(client, &ct36x_ts);
	}
#endif

#ifdef CONFIG_TOUCHSCREEN_CT36X_PLATFORM_ROCKCHIP
	/* for all-in-one driver */
	printk("No platform data for device %s.\n", DRIVER_NAME);
#endif

#ifdef CONFIG_TOUCHSCREEN_CT36X_PLATFORM_ALLWINNER
	/* for allwinner platform driver */
	ct36x_ts.client = client;
	i2c_set_clientdata(client, &ct36x_ts);
#endif
	/* this code segment is platform dependent */

	ts = (struct ct36x_ts_info *)i2c_get_clientdata(client);

	/* Create Proc Entry File */
	ts->proc_entry = create_proc_entry(DRIVER_NAME, 0666/*S_IFREG | S_IRUGO | S_IWUSR*/, NULL);
	if ( ts->proc_entry == NULL ) {
		dev_err(dev, "Failed creating proc dir entry file.\n");
	} else {
		ts->proc_entry->proc_fops = &ct36x_ts_fops;
	}

	/* register early suspend */
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = ct36x_early_suspend;
	ts->early_suspend.resume = ct36x_early_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	/* Check I2C Functionality */
	err = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
	if ( !err ) {
		dev_err(dev, "Check I2C Functionality Failed.\n");
		goto ERR_I2C_CHK;
	}

	/* Request platform resources (gpio/interrupt pins) */
	err = ct36x_platform_get_resource(ts);
	if ( err ) {
		dev_err(dev, "Unable to request platform resource for device %s.\n", DRIVER_NAME);
		goto ERR_PLAT_RSC;
	}

	if (ct36x_chip_get_ver(client, buf) == 0xFF) {
		dev_err(dev, "Unable to get version for device %s.\n", DRIVER_NAME);
		err = -ENOMEM;
		goto ERR_PLAT_RSC;
	}

	/* Hardware reset */
	ct36x_platform_hw_reset(ts);

	// Get binary Checksum
	binchksum = ct36x_chip_get_binchksum(ts->data.buf);
	if ( CT36X_TS_CORE_DEBUG )
	printk("Bin checksum: 0x%x\n", binchksum);

	// Get firmware Checksum
	fwchksum = ct36x_chip_get_fwchksum(client, ts->data.buf);
	if ( CT36X_TS_CORE_DEBUG )
	printk("Fw checksum: 0x%x\n", fwchksum);

	updcnt = 5;
	while ( binchksum != fwchksum && updcnt--) {
		/* Update Firmware */
		ct36x_chip_go_bootloader(client, ts->data.buf);

		/* Hardware reset */
		ct36x_platform_hw_reset(ts);

		// Get firmware Checksum
		fwchksum = ct36x_chip_get_fwchksum(client, ts->data.buf);
		if ( CT36X_TS_CORE_DEBUG )
		printk("Fw checksum: 0x%x\n", fwchksum);
	}

	printk("Fw update %s. 0x%x, 0x%x\n", binchksum != fwchksum ? "Failed" : "Success", binchksum, fwchksum);
	if (binchksum != fwchksum) {
		dev_err(dev, "Unable to update firmware for device %s.\n", DRIVER_NAME);
		err = -ENOMEM;
		goto ERR_PLAT_RSC;
	}

	/* Hardware reset */
	ct36x_platform_hw_reset(ts);

	/* allocate input device */
	ts->input = input_allocate_device();
	if ( !ts->input ) {
		dev_err(dev, "Unable to allocate input device for device %s.\n", DRIVER_NAME);
		err = -ENOMEM;
		goto ERR_INPUT_ALLOC;
	}

	/* config input device */
	__set_bit(EV_SYN, ts->input->evbit);
	__set_bit(EV_KEY, ts->input->evbit);
	__set_bit(EV_ABS, ts->input->evbit);

	// For android 4.x only
	__set_bit(INPUT_PROP_DIRECT, ts->input->propbit);

	// For android 4.x only
	input_mt_init_slots(ts->input, CT36X_TS_POINT_NUM);

	input_set_abs_params(ts->input, ABS_MT_POSITION_X, 0, CT36X_TS_ABS_X_MAX, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y, 0, CT36X_TS_ABS_Y_MAX, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);

	ts->input->name = DRIVER_NAME;
	ts->input->id.bustype =	BUS_I2C;

	/* register input device */
	err = input_register_device(ts->input);
	if ( err ) {
		dev_err(dev, "Unable to register input device for device %s.\n", DRIVER_NAME);
		goto ERR_INPUT_REGIS;
	}

	/* Create work queue */
	INIT_WORK(&ts->event_work, ct36x_ts_workfunc);
	ts->workqueue = create_singlethread_workqueue(dev_name(&client->dev));

	/* Init irq */
	/* this code segment is platform dependent */
	// irq request for generic platform (enabled when unsure)
	err = request_irq(ts->irq, ct36x_ts_irq, IRQF_TRIGGER_FALLING, DRIVER_NAME, ts);

	// irq request for amlogic platform (enabled for amlogic)
	//err = request_irq(ts->irq, ct36x_ts_irq, IRQF_DISABLED, DRIVER_NAME, ts);

	// irq request for allwinner platform (enabled for allwinner)
	//err = request_irq(ts->irq, ct36x_ts_irq, IRQF_TRIGGER_FALLING | IRQF_SHARED, DRIVER_NAME, ts);
	/* this code segment is platform dependent */
	if ( err ) {
		dev_err(dev, "Unable to request irq for device %s.\n", DRIVER_NAME);
		goto ERR_IRQ_REQ;
	}

	/* Set device is ready */
	ts->state = CT36X_STATE_NORMAL;

	/* power denoisy*/
	//ct36x_chip_set_adapter_on(client, ts->data.buf);
	//ct36x_chip_set_adapter_off(client, ts->data.buf);
	ct36x_ts_adapter(0);
	
	return 0;

	ERR_IRQ_REQ:
	destroy_workqueue(ts->workqueue);
	ERR_INPUT_REGIS:
	input_free_device(ts->input);
	ERR_INPUT_ALLOC:
	ERR_PLAT_RSC:
	ct36x_platform_put_resource(ts);
	ERR_I2C_CHK:
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif
	remove_proc_entry(DRIVER_NAME, NULL);
	return err;
}

void ct36x_ts_shutdown(struct i2c_client *client)
{
	struct ct36x_ts_info *ts;

	if (CT36X_TS_CORE_DEBUG)
	printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	ts = (struct ct36x_ts_info *)i2c_get_clientdata(client);

	ct36x_chip_go_sleep(client, ts->data.buf);
}

int ct36x_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct ct36x_ts_info *ts;

	if (CT36X_TS_CORE_DEBUG)
	printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	ts = (struct ct36x_ts_info *)i2c_get_clientdata(client);

	if ( ts->state == CT36X_STATE_NORMAL ) {
		ts->state = CT36X_STATE_SLEEP;
		disable_irq(ts->irq);
		//cancel_work_sync(&ts->event_work);
		ct36x_chip_go_sleep(client, ts->data.buf);
	}

	return 0;
}

int ct36x_ts_resume(struct i2c_client *client)
{
	struct ct36x_ts_info *ts;

	if (CT36X_TS_CORE_DEBUG)
	printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	ts = (struct ct36x_ts_info *)i2c_get_clientdata(client);

	if ( ts->state == CT36X_STATE_SLEEP ) {
		/* Hardware reset */
		ct36x_platform_hw_reset_resume(ts);
		enable_irq(ts->irq);
		ts->state = CT36X_STATE_NORMAL;
	}

	return 0;
}

int __devexit ct36x_ts_remove(struct i2c_client *client)
{
	struct ct36x_ts_info *ts;

	if (CT36X_TS_CORE_DEBUG)
	printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	ts = (struct ct36x_ts_info *)i2c_get_clientdata(client);

	/* Driver clean up */
	disable_irq(ts->irq);
	cancel_work_sync(&ts->event_work);
	destroy_workqueue(ts->workqueue);
	input_free_device(ts->input);
	free_irq(ts->irq, ts);
	ct36x_platform_put_resource(ts);
	i2c_unregister_device(client);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif
	remove_proc_entry(DRIVER_NAME, NULL);

	return 0;
}

int __init ct36x_ts_init(void)
{
	int err = -1;

	printk("VTL ct36x TouchScreen driver, <george.chen@vtl.com.cn>.\n");

#ifdef CONFIG_POWER_ON_CHARGER_DISPLAY
    if (charger_in_logo == 1) {
        return -ENODEV;
    }
#endif

	ct36x_ts.state = CT36X_STATE_UNKNOWN;

	ct36x_platform_get_cfg(&ct36x_ts);

	err = ct36x_platform_set_dev(&ct36x_ts);
	if ( err ) goto ERR_INIT;

	err = i2c_add_driver(&ct36x_ts_driver);
	if ( err ) goto ERR_INIT;

	return 0;

	ERR_INIT:
	return err;
}

void __exit ct36x_ts_exit(void)
{
	i2c_del_driver(&ct36x_ts_driver);
}

module_init(ct36x_ts_init);
module_exit(ct36x_ts_exit);

MODULE_AUTHOR("<george.chen@vtl.com>");
MODULE_DESCRIPTION("VTL ct36x TouchScreen driver");
MODULE_LICENSE("GPL");



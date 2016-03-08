
#include <linux/i2c.h>
#include <linux/delay.h>

#include <mach/irqs.h>
#include <mach/system.h>
#include <mach/hardware.h>
#include <mach/sys_config.h>

#include "tscore.h"
#include "allwinner.h"


union union_i2c_addr {
	unsigned short dirty_addr_buf[2];
	const unsigned short normal_i2c[2];
};


static int int_cfg_addr[] = {
	PIO_INT_CFG0_OFFSET, PIO_INT_CFG1_OFFSET,
	PIO_INT_CFG2_OFFSET, PIO_INT_CFG3_OFFSET
};

void* __iomem gpio_addr = NULL;
__u32 twi_addr = 0x00;
__u32 twi_id = 0;

static union union_i2c_addr i2c_addr = {{0x00},};

static struct i2c_device_id ct36x_ts_id[] = {
	{ DRIVER_NAME, 0 },
	{ }
};

struct i2c_driver ct36x_ts_driver  = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.owner	= THIS_MODULE,
		.name	= DRIVER_NAME
	},
	.id_table	= ct36x_ts_id,
	.probe      = ct36x_ts_probe,
	.suspend	= ct36x_ts_suspend,
	.resume	    = ct36x_ts_resume,
	.remove 	= __devexit_p(ct36x_ts_remove),
	.address_list	= i2c_addr.normal_i2c,
};

void ct36x_ts_reg_read(struct i2c_client *client, unsigned short addr, char *buf, int len)
{
	struct i2c_msg msgs;

	msgs.addr = addr;
	msgs.flags = 0x01;  // 0x00: write 0x01:read 
	msgs.len = len;
	msgs.buf = buf;
	//msgs.scl_rate = CT36X_TS_I2C_SPEED;

	i2c_transfer(client->adapter, &msgs, 1);
}

void ct36x_ts_reg_write(struct i2c_client *client, unsigned short addr, char *buf, int len)
{
	struct i2c_msg msgs;

	msgs.addr = addr;
	msgs.flags = 0x00;  // 0x00: write 0x01:read 
	msgs.len = len;
	msgs.buf = buf;
	//msgs.scl_rate = CT36X_TS_I2C_SPEED;

	i2c_transfer(client->adapter, &msgs, 1);
}

void ct36x_platform_get_cfg(struct ct36x_ts_info *ts)
{
	int cpt = -1;
	char name[I2C_NAME_SIZE];
	script_parser_value_type_t type = SCIRPT_PARSER_VALUE_TYPE_STRING;

	if ( script_parser_fetch("ctp_para", "ctp_used", &cpt, 1) != SCRIPT_PARSER_OK ) {
		printk("script_parser_fetch failed for %s %s. \n", "ctp_para", "ctp_used");
		return;
	}
	if ( cpt != 1 ) {
		printk("no ctp used. \n");
		return;
	}

	if( script_parser_fetch_ex("ctp_para", "ctp_name", (int *)(name), &type, sizeof(name)/sizeof(int)) != SCRIPT_PARSER_OK ){
		printk("script_parser_fetch failed for %s %s. \n", "ctp_para", "ctp_name");
		return;
	}
	if ( strcmp(DRIVER_NAME, name) ) {
		printk("ctp_name no match, %s %s. \n", DRIVER_NAME, name);
		return;	
	}

	if ( script_parser_fetch("ctp_para", "ctp_twi_addr", &twi_addr, sizeof(twi_addr)/sizeof(__u32)) != SCRIPT_PARSER_OK ) {
		printk("script_parser_fetch failed for %s %s. \n", "ctp_para", "ctp_twi_addr");
		return;
	}
	// big-endian or small-endian?
	i2c_addr.dirty_addr_buf[0] = twi_addr;
	i2c_addr.dirty_addr_buf[1] = I2C_CLIENT_END;

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_twi_id", &twi_id, sizeof(twi_id)/sizeof(__u32))){
		//pr_err("%s: script_parser_fetch err. \n", name);
		printk("script_parser_fetch failed for %s %s. \n", "ctp_para", "ctp_twi_id");
		return;
	}

	// optional
#if 0
	if ( script_parser_fetch("ctp_para", "ctp_screen_max_x", &screen_max_x, 1) != SCRIPT_PARSER_OK ) {
		printk("script_parser_fetch failed for %s %s. \n", "ctp_para", "ctp_screen_max_x");
		return -1;
	}

	if ( script_parser_fetch("ctp_para", "ctp_screen_max_y", &screen_max_y, 1) != SCRIPT_PARSER_OK ) {
		printk("script_parser_fetch failed for %s %s. \n", "ctp_para", "ctp_screen_max_y");
		return -1;
	}

	if ( script_parser_fetch("ctp_para", "ctp_revert_x_flag", &revert_x_flag, 1) != SCRIPT_PARSER_OK ) {
		printk("script_parser_fetch failed for %s %s. \n", "ctp_para", "ctp_revert_x_flag");
		return -1;
	}

	if ( script_parser_fetch("ctp_para", "ctp_revert_y_flag", &revert_y_flag, 1) != SCRIPT_PARSER_OK ) {
		printk("script_parser_fetch failed for %s %s. \n", "ctp_para", "ctp_revert_y_flag");
		return -1;
	}

	if ( script_parser_fetch("ctp_para", "ctp_exchange_x_y_flag", &exchange_x_y_flag, 1) != SCRIPT_PARSER_OK ) {
        	printk("script_parser_fetch failed for %s %s. \n", "ctp_para", "ctp_exchange_x_y_flag");
		return -1;
	}
#endif

	/* I2C config */
	ts->i2c_address = twi_addr;
	ts->i2c_bus = twi_id;

	/* GPIO config */

	/* IRQ config*/
	ts->irq = SW_INT_IRQNO_PIO;
}

static int ct36x_platform_det_dev(struct i2c_client *client, struct i2c_board_info *info)
{
	int ret = -ENODEV;
	struct i2c_adapter *adapter = client->adapter;

	if ( twi_id == adapter->nr ) {
		//pr_info("%s: Detected chip %s at adapter %d, address 0x%02x\n",
		//	 __func__, CTP_NAME, i2c_adapter_id(adapter), client->addr);

		strlcpy(info->type, DRIVER_NAME, I2C_NAME_SIZE);

		ret = 0;
	}

	return ret;
}

int ct36x_platform_set_dev(struct ct36x_ts_info *ts)
{
	ct36x_ts_driver.detect = ct36x_platform_det_dev;

	return 0;
}

int ct36x_platform_get_resource(struct ct36x_ts_info *ts)
{
	__u32 reg_num = 0;
	__u32 reg_addr = 0;
	__u32 reg_val = 0;

	gpio_addr = ioremap(PIO_BASE_ADDRESS, PIO_RANGE_SIZE);
	if ( !gpio_addr ) {
		return -EIO;
	}

	/*ts->pwr = gpio_request_ex("ctp_para", "ctp_pwr");
	if ( !ts->pwr ) {
		return -EIO;
	}
	gpio_write_one_pin_value(ts->pwr, 1, "ctp_pwr");
	*/
	ts->rst = gpio_request_ex("ctp_para", "ctp_reset");
	if ( !ts->rst ) {
		return -EIO;
	}
        gpio_write_one_pin_value(ts->rst, 1, "ctp_reset");

	ts->ss = gpio_request_ex("ctp_para", "ctp_int_port");
	if ( !ts->ss ) {
		return -EIO;
	}

	reg_num = CT36X_TS_IRQ_NO % 8;
	reg_addr = CT36X_TS_IRQ_NO / 8;
	// config falling edge triggered interrupt
	reg_val = readl(gpio_addr + int_cfg_addr[reg_addr]);
	reg_val &= (~(7 << (reg_num * 4)));
	reg_val |= (CT36X_TS_IRQ_MODE << (reg_num * 4));
	writel(reg_val, gpio_addr + int_cfg_addr[reg_addr]);

	// clear pending interrupt
	reg_val = readl(gpio_addr + PIO_INT_STAT_OFFSET);
	if((reg_val = (reg_val&(1<<(CT36X_TS_IRQ_NO))))){
		writel(reg_val, gpio_addr + PIO_INT_STAT_OFFSET);
	}

	// enable irq
	reg_val = readl(gpio_addr + PIO_INT_CTRL_OFFSET); 
	reg_val |= (1 << CT36X_TS_IRQ_NO);
	writel(reg_val, gpio_addr + PIO_INT_CTRL_OFFSET);

	return 0;
}

void ct36x_platform_put_resource(struct ct36x_ts_info *ts)
{
	gpio_release(ts->rst, 2);
	gpio_release(ts->ss, 2);
}

void ct36x_platform_hw_reset(struct ct36x_ts_info *ts)
{
	mdelay(500);
	gpio_write_one_pin_value(ts->rst, 0, "ctp_reset");
	mdelay(50);
	gpio_write_one_pin_value(ts->rst, 1, "ctp_reset");
	mdelay(500);
}

int ct36x_platform_is_ts_irq(void)
{
	int ret = 0;
	int reg_val;

	reg_val = readl(gpio_addr + PIO_INT_STAT_OFFSET);
	if ( reg_val & (1 << (CT36X_TS_IRQ_NO)) ) {
		writel( reg_val & (1 << (CT36X_TS_IRQ_NO)), gpio_addr + PIO_INT_STAT_OFFSET);
		ret = 1;
	}

	return ret;
}

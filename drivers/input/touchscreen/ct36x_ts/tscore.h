#ifndef TSCORE_H
#define TSCORE_H

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include "chip.h"

// ****************************************************************************
// Defines
// ****************************************************************************
#define CT36X_TS_CORE_DEBUG				0
#define CT36X_TS_EVENT_DEBUG				0


#define DRIVER_NAME		"ct36x_ts"

enum enum_ct36x_state {
	CT36X_STATE_UNKNOWN,
	CT36X_STATE_INIT,
	CT36X_STATE_NORMAL,
	CT36X_STATE_SLEEP,
	CT36X_STATE_UPDATE,
};

enum enum_ct36x_ts_cmds {
	CT36X_TS_CHIP_ID,
	CT36X_TS_CHIP_RESET,
	CT36X_TS_FW_VER,
	CT36X_TS_FW_CHKSUM,
	CT36X_TS_FW_UPDATE,
	CT36X_TS_BIN_LOAD,
	CT36X_TS_BIN_VER,
	CT36X_TS_BIN_CHKSUM,
};

// Platform data
struct ct36x_platform_data {
	int 				pwr;
	int 				rst;
	int 				ss;
};

union ct36x_i2c_data {
	struct ct36x_finger_info	pts[CT36X_TS_POINT_NUM];
	unsigned char			buf[CT36X_TS_POINT_NUM * sizeof(struct ct36x_finger_info)];
};

struct ct36x_ts_info {
	/* Chip ID */
	int				chip_id;

	/* platform data */
	struct ct36x_platform_data	*pdata;

	/* Communication settings */
	int				i2c_bus;
	unsigned short			i2c_address;
	struct i2c_client		*client;

	/* input devices */
	struct input_dev		*input;

	/* hw config */
	int 				irq;
	int 				pwr;
	int 				rst;
	int 				ss;
	int				state;

	// Early suspend
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend		early_suspend;
#endif

	// Proc Control
	struct proc_dir_entry		*proc_entry;

	// Work thread settings
	struct work_struct		event_work;
	struct workqueue_struct 	*workqueue;
	
	/* touch event data & status */
	union ct36x_i2c_data		data;
	int				press;
	int				release;
};

//////////////////////////////////////////////

int ct36x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id);
void ct36x_ts_shutdown(struct i2c_client *client);
int ct36x_ts_suspend(struct i2c_client *client, pm_message_t mesg);
int ct36x_ts_resume(struct i2c_client *client);
int __devexit ct36x_ts_remove(struct i2c_client *client);

int __init ct36x_ts_init(void);
void __exit ct36x_ts_exit(void);

#endif



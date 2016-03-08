
#ifndef GENERIC_H
#define GENERIC_H

#include <mach/gpio_data.h>
#include <mach/gpio.h>
#include <mach/irqs.h>

/*
** board-m6g24.c **

// Platform data
struct ct36x_platform_data {
	int 				rst;
	int 				ss;
};

#define GPIO_TS_RST  PAD_GPIOC_3
#define GPIO_TS_IRQ  PAD_GPIOA_16
#define TS_IRQ     INT_GPIO_0

static struct ct36x_platform_data ts_pdata = {
    .rst = GPIO_TS_RST,
    .ss = GPIO_TS_IRQ,
};

static struct i2c_board_info __initdata aml_i2c_bus_info_a[] = {
...
    {
        I2C_BOARD_INFO("ct36x_ts", 0x01),
	.platform_data = (void *)&ts_pdata,
    },
...
};
*/

#define GPIO_TS_RST  PAD_GPIOC_3
#define GPIO_TS_IRQ  PAD_GPIOA_16
#define TS_IRQ     INT_GPIO_0

#endif


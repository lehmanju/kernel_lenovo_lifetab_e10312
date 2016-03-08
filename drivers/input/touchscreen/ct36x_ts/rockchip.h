
#ifndef ROCKCHIP_H
#define ROCKCHIP_H

#include <linux/gpio.h>

#define CT36X_TS_I2C_BUS			2	// I2C Bus
#define CT36X_TS_I2C_ADDRESS			0x01
#define CT36X_TS_I2C_SPEED			400000

#define CT36X_TS_PWR_PIN			RK30_PIN2_PC2
#define CT36X_TS_IRQ_PIN			RK30_PIN4_PC2
#define CT36X_TS_RST_PIN			RK30_PIN6_PB0

#endif


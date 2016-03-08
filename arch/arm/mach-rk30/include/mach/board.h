#ifndef __MACH_BOARD_H
#define __MACH_BOARD_H

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/types.h>
#include <linux/timer.h>
#include <linux/notifier.h>
#include <asm/setup.h>
#include <plat/board.h>
#include <mach/sram.h>
#include <linux/i2c-gpio.h>

#if defined (CONFIG_TOUCHSCREEN_FT5X06)
struct ft5x0x_platform_data{
	  u16     model;
    int     (*get_pendown_state)(void);
    int     (*init_platform_hw)(void);
    int     (*ft5x0x_platform_sleep)(void);
    int     (*ft5x0x_platform_wakeup)(void);
    void    (*exit_platform_hw)(void);
};
#endif

#if defined (CONFIG_TOUCHSCREEN_ELAN_TP)
struct ktf2k_platform_data{
	  u16     model;
    int     (*get_pendown_state)(void);
    int     (*init_platform_hw)(void);
    int     (*ktf2k_platform_sleep)(void);
    int     (*ktf2k_platform_wakeup)(void);
    void    (*exit_platform_hw)(void);
    int			rst_pin;
};
#endif

#if defined(CONFIG_TOUCHSCREEN_BYD693X)
struct byd_platform_data {
	u16     model;
	int     pwr_pin;
	int	  int_pin;
	int     rst_pin;
	int		pwr_on_value;
	int (*get_probe_state)(void);
	void   (*set_probe_state)(int );
	int 	*tp_flag;

	uint16_t screen_max_x;
	uint16_t screen_max_y;
	u8 swap_xy :1;
	u8 xpol :1;
	u8 ypol :1;	
};
#endif

#if defined (CONFIG_TOUCHSCREEN_BF6931A)
struct bf6931a_platform_data{
	  u16     model;
    int     (*get_pendown_state)(void);
    int     (*init_platform_hw)(void);
    int     (*bf6931a_platform_sleep)(void);
    int     (*bf6931a_platform_wakeup)(void);
    void    (*exit_platform_hw)(void);
};
#endif

#if defined (CONFIG_TOUCHSCREEN_NOVATEK)
struct novatek_i2c_platform_data {
	uint32_t version;		/* Use this entry for panels with */

	int (*ts_init_platform_hw)(void);
	int (*ts_exit_platform_hw)(void);
	int (*get_probe_state)(void);
	void (*set_probe_state)(int);	
	int gpio_rst;
	int gpio_irq;
	bool irq_edge; 		/* 0:rising edge, 1:falling edge */
	uint16_t touch_max_x;
	uint16_t touch_max_y;
	uint16_t screen_max_x;
	uint16_t screen_max_y;
	u8 swap_xy :1;
	u8 xpol :1;
	u8 ypol :1;
};
#endif

void __init rk30_map_common_io(void);
void __init rk30_init_irq(void);
void __init rk30_map_io(void);
struct machine_desc;
void __init rk30_fixup(struct machine_desc *desc, struct tag *tags, char **cmdline, struct meminfo *mi);
void __init rk30_clock_data_init(unsigned long gpll,unsigned long cpll,u32 flags);

#ifdef CONFIG_RK30_PWM_REGULATOR
void  rk30_pwm_suspend_voltage_set(void);
void  rk30_pwm_resume_voltage_set(void);
void __sramfunc rk30_pwm_logic_suspend_voltage(void);
 void __sramfunc rk30_pwm_logic_resume_voltage(void);
#endif

extern struct sys_timer rk30_timer;

enum _periph_pll {
	periph_pll_1485mhz = 148500000,
	periph_pll_297mhz = 297000000,
	periph_pll_300mhz = 300000000,
	periph_pll_384mhz = 384000000,
	periph_pll_594mhz = 594000000,
	periph_pll_1188mhz = 1188000000, /* for box*/
};
enum _codec_pll {
	codec_pll_360mhz = 360000000, /* for HDMI */
	codec_pll_408mhz = 408000000,
	codec_pll_456mhz = 456000000,
	codec_pll_504mhz = 504000000,
	codec_pll_552mhz = 552000000, /* for HDMI */
	codec_pll_594mhz = 594000000, /* for HDMI */
	codec_pll_600mhz = 600000000,
	codec_pll_742_5khz = 742500000,
	codec_pll_768mhz = 768000000,
	codec_pll_798mhz = 798000000,
	codec_pll_1188mhz = 1188000000,
	codec_pll_1200mhz = 1200000000,
};

//has extern 27mhz
#define CLK_FLG_EXT_27MHZ 			(1<<0)
//max i2s rate
#define CLK_FLG_MAX_I2S_12288KHZ 	(1<<1)
#define CLK_FLG_MAX_I2S_22579_2KHZ 	(1<<2)
#define CLK_FLG_MAX_I2S_24576KHZ 	(1<<3)
#define CLK_FLG_MAX_I2S_49152KHZ 	(1<<4)
//uart 1m\3m
#define CLK_FLG_UART_1_3M			(1<<5)
#define CLK_CPU_HPCLK_11				(1<<6)
#define CLK_GPU_GPLL				(1<<7)
#define CLK_GPU_CPLL				(1<<8)


#ifdef CONFIG_RK29_VMAC

#define RK30_CLOCKS_DEFAULT_FLAGS (CLK_FLG_MAX_I2S_12288KHZ/*|CLK_FLG_EXT_27MHZ*/)
#define periph_pll_default periph_pll_300mhz
#define codec_pll_default codec_pll_1188mhz

#else


#define RK30_CLOCKS_DEFAULT_FLAGS (CLK_FLG_MAX_I2S_12288KHZ/*|CLK_FLG_EXT_27MHZ*/)

#if (RK30_CLOCKS_DEFAULT_FLAGS&CLK_FLG_UART_1_3M)
#define codec_pll_default codec_pll_768mhz
#define periph_pll_default periph_pll_297mhz

#else

#ifdef CONFIG_ARCH_RK3066B
#define codec_pll_default codec_pll_594mhz
#define periph_pll_default periph_pll_384mhz

#else 
#define codec_pll_default codec_pll_1200mhz
#define periph_pll_default periph_pll_297mhz
#endif

#endif

#endif

#endif

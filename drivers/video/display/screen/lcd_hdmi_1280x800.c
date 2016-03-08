#include <linux/delay.h>
#include <mach/gpio.h>
#include <mach/iomux.h>
#include <mach/board.h>
#include <linux/rk_fb.h>
#if defined(CONFIG_RK_HDMI)
#include "../../rockchip/hdmi/rk_hdmi.h"
#endif
#ifdef CONFIG_RK610_LVDS
#include "../transmitter/rk610_lcd.h"
#endif



/* Base */
#define OUT_TYPE		SCREEN_LVDS

#define LVDS_FORMAT      	LVDS_8BIT_2

#define OUT_FACE		OUT_D888_P666  
#define OUT_CLK			71000000
#define LCDC_ACLK        500000000//312000000           //29 lcdc axi DMA ÆµÂÊ


/* Timing */
#define H_PW			32
#define H_BP			80
#define H_VD			1280
#define H_FP			48

#define V_PW			6
#define V_BP			15
#define V_VD			800
#define V_FP			2

#define LCD_WIDTH       216
#define LCD_HEIGHT      135

int dsp_lut[256] ={
		0x0, 0x0, 0x10101, 0x10101, 0x10101, 0x10101, 0x20202, 0x20202, 
		0x30303, 0x30303, 0x40404, 0x40404, 0x50505, 0x50505, 0x60606, 0x60606, 
		0x70707, 0x80808, 0x80808, 0x90909, 0xa0a0a, 0xa0a0a, 0xb0b0b, 0xc0c0c, 
		0xc0c0c, 0xd0d0d, 0xe0e0e, 0xf0f0f, 0xf0f0f, 0x101010, 0x111111, 0x121212, 
		0x121212, 0x131313, 0x141414, 0x151515, 0x161616, 0x171717, 0x171717, 0x181818, 
		0x191919, 0x1a1a1a, 0x1b1b1b, 0x1c1c1c, 0x1d1d1d, 0x1e1e1e, 0x1f1f1f, 0x202020, 
		0x202020, 0x212121, 0x222222, 0x232323, 0x242424, 0x252525, 0x262626, 0x272727, 
		0x282828, 0x292929, 0x2a2a2a, 0x2b2b2b, 0x2c2c2c, 0x2d2d2d, 0x2e2e2e, 0x303030, 
		0x313131, 0x323232, 0x333333, 0x343434, 0x353535, 0x363636, 0x373737, 0x383838, 
		0x393939, 0x3a3a3a, 0x3c3c3c, 0x3d3d3d, 0x3e3e3e, 0x3f3f3f, 0x404040, 0x414141, 
		0x424242, 0x444444, 0x454545, 0x464646, 0x474747, 0x484848, 0x494949, 0x4b4b4b, 
		0x4c4c4c, 0x4d4d4d, 0x4e4e4e, 0x4f4f4f, 0x515151, 0x525252, 0x535353, 0x545454, 
		0x565656, 0x575757, 0x585858, 0x595959, 0x5b5b5b, 0x5c5c5c, 0x5d5d5d, 0x5f5f5f, 
		0x606060, 0x616161, 0x626262, 0x646464, 0x656565, 0x666666, 0x686868, 0x696969, 
		0x6a6a6a, 0x6c6c6c, 0x6d6d6d, 0x6e6e6e, 0x707070, 0x717171, 0x727272, 0x747474, 
		0x757575, 0x777777, 0x787878, 0x797979, 0x7b7b7b, 0x7c7c7c, 0x7d7d7d, 0x7f7f7f, 
		0x808080, 0x828282, 0x838383, 0x848484, 0x868686, 0x878787, 0x888888, 0x8a8a8a, 
		0x8b8b8b, 0x8c8c8c, 0x8e8e8e, 0x8f8f8f, 0x919191, 0x929292, 0x939393, 0x959595, 
		0x969696, 0x979797, 0x999999, 0x9a9a9a, 0x9b9b9b, 0x9d9d9d, 0x9e9e9e, 0x9f9f9f, 
		0xa0a0a0, 0xa2a2a2, 0xa3a3a3, 0xa4a4a4, 0xa5a5a5, 0xa7a7a7, 0xa8a8a8, 0xa9a9a9, 
		0xababab, 0xacacac, 0xadadad, 0xaeaeae, 0xb0b0b0, 0xb1b1b1, 0xb2b2b2, 0xb3b3b3, 
		0xb4b4b4, 0xb6b6b6, 0xb7b7b7, 0xb8b8b8, 0xb9b9b9, 0xbababa, 0xbcbcbc, 0xbcbcbc, 
		0xbebebe, 0xbfbfbf, 0xc0c0c0, 0xc1c1c1, 0xc2c2c2, 0xc3c3c3, 0xc5c5c5, 0xc6c6c6, 
		0xc7c7c7, 0xc8c8c8, 0xc9c9c9, 0xcacaca, 0xcbcbcb, 0xcccccc, 0xcdcdcd, 0xcecece, 
		0xcfcfcf, 0xd1d1d1, 0xd2d2d2, 0xd3d3d3, 0xd3d3d3, 0xd5d5d5, 0xd6d6d6, 0xd6d6d6, 
		0xd8d8d8, 0xd9d9d9, 0xd9d9d9, 0xdbdbdb, 0xdbdbdb, 0xdddddd, 0xdddddd, 0xdfdfdf, 
		0xdfdfdf, 0xe1e1e1, 0xe1e1e1, 0xe3e3e3, 0xe3e3e3, 0xe4e4e4, 0xe5e5e5, 0xe6e6e6, 
		0xe6e6e6, 0xe8e8e8, 0xe9e9e9, 0xe9e9e9, 0xeaeaea, 0xebebeb, 0xececec, 0xededed, 
		0xeeeeee, 0xeeeeee, 0xefefef, 0xf0f0f0, 0xf0f0f0, 0xf1f1f1, 0xf2f2f2, 0xf3f3f3, 
		0xf3f3f3, 0xf4f4f4, 0xf5f5f5, 0xf6f6f6, 0xf6f6f6, 0xf7f7f7, 0xf7f7f7, 0xf8f8f8, 
		0xf9f9f9, 0xf9f9f9, 0xfafafa, 0xfbfbfb, 0xfbfbfb, 0xfbfbfb, 0xfbfbfb, 0xfcfcfc, 
		0xfdfdfd, 0xfdfdfd, 0xfdfdfd, 0xfefefe, 0xfefefe, 0xffffff, 0xffffff, 0xffffff,
};
extern char GetSNSectorInfoBeforeNandInit(char * pbuf);

/* scaler Timing    */
//1920*1080*60

#define S_OUT_CLK		SCALE_RATE(148500000,67500000) //m=20 n=11 no=4
#define S_H_PW			10
#define S_H_BP			10
#define S_H_VD			1280
#define S_H_FP			50

#define S_V_PW			6
#define S_V_BP			15
#define S_V_VD			800
#define S_V_FP			12

#define S_H_ST			450
#define S_V_ST			12

//1920*1080*50
#define S1_OUT_CLK		SCALE_RATE(148500000,66000000)  //m=16 n=9 no=4 
#define S1_H_PW			32
#define S1_H_BP			80
#define S1_H_VD			1280
#define S1_H_FP			192

#define S1_V_PW			6
#define S1_V_BP			15
#define S1_V_VD			800
#define S1_V_FP			12

#define S1_H_ST			528
#define S1_V_ST			12

//1280*720*60
#define S2_OUT_CLK		SCALE_RATE(74250000,67500000)  //m=40 n=11 no=4
#define S2_H_PW			10
#define S2_H_BP			10
#define S2_H_VD			1280
#define S2_H_FP			50

#define S2_V_PW			6
#define S2_V_BP			10
#define S2_V_VD			800
#define S2_V_FP			17

#define S2_H_ST			450
#define S2_V_ST			12

//1280*720*50

#define S3_OUT_CLK		SCALE_RATE(74250000,66000000)   // m=32 n=9 no=4
#define S3_H_PW			32
#define S3_H_BP			80
#define S3_H_VD			1280
#define S3_H_FP			192

#define S3_V_PW			6
#define S3_V_BP			10
#define S3_V_VD			800
#define S3_V_FP			17

#define S3_H_ST			528
#define S3_V_ST			12

//720*576*50
#define S4_OUT_CLK		SCALE_RATE(27000000,65625000)  //m=175 n=9 no=8
#define S4_H_PW			32
#define S4_H_BP			80
#define S4_H_VD			1280
#define S4_H_FP			120

#define S4_V_PW			6
#define S4_V_BP			15
#define S4_V_VD			800
#define S4_V_FP			47

#define S4_H_ST			84
#define S4_V_ST			32

//720*480*60
#define S5_OUT_CLK		SCALE_RATE(27000000,75000000)  //m=100 n=9 no=4
#define S5_H_PW			32
#define S5_H_BP			80
#define S5_H_VD			1280
#define S5_H_FP			38

#define S5_V_PW			6
#define S5_V_BP			15
#define S5_V_VD			800
#define S5_V_FP			53

#define S5_H_ST			476
#define S5_V_ST			25

#define S_DCLK_POL       1

/* Other */
#define DCLK_POL	1
#define DEN_POL		0
#define VSYNC_POL	0
#define HSYNC_POL	0

#define SWAP_RB		0
#define SWAP_RG		0
#define SWAP_GB		0


#if  ( defined(CONFIG_ONE_LCDC_DUAL_OUTPUT_INF)&& defined(CONFIG_RK610_LVDS) ) || defined(CONFIG_HDMI_DUAL_DISP)
static int set_scaler_info(struct rk29fb_screen *screen, u8 hdmi_resolution)
{
	screen->s_clk_inv = S_DCLK_POL;
	screen->s_den_inv = 0;
	screen->s_hv_sync_inv = 0;
	switch(hdmi_resolution){
	case HDMI_1920x1080p_60Hz:
		/* Scaler Timing    */
		screen->hdmi_resolution = hdmi_resolution;
		screen->s_pixclock = S_OUT_CLK;
		screen->s_hsync_len = S_H_PW;
		screen->s_left_margin = S_H_BP;
		screen->s_right_margin = S_H_FP;
		screen->s_hsync_len = S_H_PW;
		screen->s_upper_margin = S_V_BP;
		screen->s_lower_margin = S_V_FP;
		screen->s_vsync_len = S_V_PW;
		screen->s_hsync_st = S_H_ST;
		screen->s_vsync_st = S_V_ST;
		break;
	case HDMI_1920x1080p_50Hz:
		/* Scaler Timing    */
		screen->hdmi_resolution = hdmi_resolution;
		screen->s_pixclock = S1_OUT_CLK;
		screen->s_hsync_len = S1_H_PW;
		screen->s_left_margin = S1_H_BP;
		screen->s_right_margin = S1_H_FP;
		screen->s_hsync_len = S1_H_PW;
		screen->s_upper_margin = S1_V_BP;
		screen->s_lower_margin = S1_V_FP;
		screen->s_vsync_len = S1_V_PW;
		screen->s_hsync_st = S1_H_ST;
		screen->s_vsync_st = S1_V_ST;
		break;
	case HDMI_1280x720p_60Hz:
		/* Scaler Timing    */
		screen->hdmi_resolution = hdmi_resolution;
		screen->s_pixclock = S2_OUT_CLK;
		screen->s_hsync_len = S2_H_PW;
		screen->s_left_margin = S2_H_BP;
		screen->s_right_margin = S2_H_FP;
		screen->s_hsync_len = S2_H_PW;
		screen->s_upper_margin = S2_V_BP;
		screen->s_lower_margin = S2_V_FP;
		screen->s_vsync_len = S2_V_PW;
		screen->s_hsync_st = S2_H_ST;
		screen->s_vsync_st = S2_V_ST;
		break;
	case HDMI_1280x720p_50Hz:
		/* Scaler Timing    */
		screen->hdmi_resolution = hdmi_resolution;
		screen->s_pixclock = S3_OUT_CLK;
		screen->s_hsync_len = S3_H_PW;
		screen->s_left_margin = S3_H_BP;
		screen->s_right_margin = S3_H_FP;
		screen->s_hsync_len = S3_H_PW;
		screen->s_upper_margin = S3_V_BP;
		screen->s_lower_margin = S3_V_FP;
		screen->s_vsync_len = S3_V_PW;
		screen->s_hsync_st = S3_H_ST;
		screen->s_vsync_st = S3_V_ST;
		break;
	case HDMI_720x576p_50Hz_4_3:
	case HDMI_720x576p_50Hz_16_9:
		/* Scaler Timing    */
		screen->hdmi_resolution = hdmi_resolution;
		screen->s_pixclock = S4_OUT_CLK;
		screen->s_hsync_len = S4_H_PW;
		screen->s_left_margin = S4_H_BP;
		screen->s_right_margin = S4_H_FP;
		screen->s_hsync_len = S4_H_PW;
		screen->s_upper_margin = S4_V_BP;
		screen->s_lower_margin = S4_V_FP;
		screen->s_vsync_len = S4_V_PW;
		screen->s_hsync_st = S4_H_ST;
		screen->s_vsync_st = S4_V_ST;
		break;
	case HDMI_720x480p_60Hz_16_9:
	case HDMI_720x480p_60Hz_4_3:
		/* Scaler Timing    */
		screen->hdmi_resolution = hdmi_resolution;
		screen->s_pixclock = S5_OUT_CLK;
		screen->s_hsync_len = S5_H_PW;
		screen->s_left_margin = S5_H_BP;
		screen->s_right_margin = S5_H_FP;
		screen->s_hsync_len = S5_H_PW;
		screen->s_upper_margin = S5_V_BP;
		screen->s_lower_margin = S5_V_FP;
		screen->s_vsync_len = S5_V_PW;
		screen->s_hsync_st = S5_H_ST;
		screen->s_vsync_st = S5_V_ST;
		break;
	default :
		printk("%s lcd not support dual display at this hdmi resolution %d \n",__func__,hdmi_resolution);
		return -1;
		break;
	}
	
	return 0;
}
#else
static int set_scaler_info(struct rk29fb_screen *screen, u8 hdmi_resolution){return 0;}
#endif
void set_lcd_info(struct rk29fb_screen *screen,  struct rk29lcd_info *lcd_info )
{
    char snbuf[0x200] = {0};

    GetSNSectorInfoBeforeNandInit(snbuf);
    if (snbuf[0] >= 10) {
        screen->dsp_lut = dsp_lut;
        printk("lcd is B101EVN07.\n");
    }

	/* screen type & face */
	screen->type = OUT_TYPE;
	screen->face = OUT_FACE;
	screen->hw_format = LVDS_FORMAT;

	/* Screen size */
	screen->x_res = H_VD;
	screen->y_res = V_VD;

	screen->width = LCD_WIDTH;
	screen->height = LCD_HEIGHT;

	/* Timing */
	screen->lcdc_aclk = LCDC_ACLK;
	screen->pixclock = OUT_CLK;
	screen->left_margin = H_BP;
	screen->right_margin = H_FP;
	screen->hsync_len = H_PW;
	screen->upper_margin = V_BP;
	screen->lower_margin = V_FP;
	screen->vsync_len = V_PW;

	/* Pin polarity */
	screen->pin_hsync = HSYNC_POL;
	screen->pin_vsync = VSYNC_POL;
	screen->pin_den = DEN_POL;
	screen->pin_dclk = DCLK_POL;

	/* Swap rule */
	screen->swap_rb = SWAP_RB;
	screen->swap_rg = SWAP_RG;
	screen->swap_gb = SWAP_GB;
	screen->swap_delta = 0;
	screen->swap_dumy = 0;

	/* Operation function*/
	screen->init = NULL;
	screen->standby = NULL;
	screen->sscreen_get = set_scaler_info;
#ifdef CONFIG_RK610_LVDS
	screen->sscreen_set = rk610_lcd_scaler_set_param;
#endif
}


size_t get_fb_size(void)
{
	size_t size = 0;
	#if defined(CONFIG_THREE_FB_BUFFER)
		size = ((H_VD)*(V_VD)<<2)* 3; //three buffer
	#else
		size = ((H_VD)*(V_VD)<<2)<<1; //two buffer
	#endif
	return ALIGN(size,SZ_1M);
}


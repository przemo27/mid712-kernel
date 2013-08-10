#include <linux/fb.h>
#include <linux/delay.h>
#include <asm/arch/lcdcon.h>
#include <asm/arch/rk28_i2c.h>
#include <asm/arch/rk28_fb.h>
#include <asm/arch/gpio.h>
#include <asm/arch/iomux.h>
#include "screen.h"

//#define DEBUG

#ifdef DEBUG
#define D(fmt, arg...) printk("<7>%s:%d:%s: " fmt, __FILE__, __LINE__, __FUNCTION__, ##arg)
#else
#define D(fmt, arg...)
#endif
#define E(fmt, arg...) printk("<3>!!!%s:%d: " fmt, __FILE__, __LINE__, ##arg)


/* 基本属性 */
/* Base */
#define OUT_TYPE		SCREEN_RGB
#define OUT_FACE		OUT_P888
#define DCLK_POL		1
#define SWAP_RB			1

/* Cvbs NTSC时序 */
#define OUT_CLK         27              //输出CLK
#define H_PW            6
#define H_BP            116
#define H_VD            720             //水平有效分辨率
#define H_FP            16
#define V_PW            6
#define V_BP            25
#define V_VD            480             //垂直有效分辨率
#define V_FP            14

/* Cvbs PAL时序 */
#define OUT1_CLK        27              //输出CLK
#define H1_PW           6
#define H1_BP           126
#define H1_VD           720             //水平有效分辨率
#define H1_FP           12
#define V1_PW           6
#define V1_BP           37
#define V1_VD           576             //垂直有效分辨率
#define V1_FP           6

/* Ypbpr 480时序 */
#define OUT2_CLK        27              //输出CLK
#define H2_PW           64
#define H2_BP           55
#define H2_VD           720             //水平有效分辨率
#define H2_FP           19
#define V2_PW           5
#define V2_BP           37
#define V2_VD           480             //垂直有效分辨率
#define V2_FP           3

/* Ypbpr 576时序 */
#define OUT3_CLK        27              //输出CLK
#define H3_PW           64
#define H3_BP           68
#define H3_VD           720             //水平有效分辨率
#define H3_FP           11
#define V3_PW           6
#define V3_BP           38
#define V3_VD           576             //垂直有效分辨率
#define V3_FP           5

/* Ypbpr 720时序 */
#define OUT4_CLK        75              //输出CLK, 74.25M, rk28_fb 驱动里切换时做特殊处理
#define H4_PW           100
#define H4_BP           600
#define H4_VD           1280            //水平有效分辨率
#define H4_FP           5
#define V4_PW           5
#define V4_BP           20
#define V4_VD           720             //垂直有效分辨率
#define V4_FP           5

extern int rk1000_control_write_block(u8 addr, u8 *buf, u8 len);
extern int rk1000_tv_write_block(u8 addr, u8 *buf, u8 len);

int rk1000_tv_ntsc_init(void)
{
	uint8 Tv_encoder_regs[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8 Tv_encoder_control_regs[] = {0x43, 0x01};
	int i;
	int ret;
	
	D("enter\n");

	for(i=0; i<sizeof(Tv_encoder_regs); i++){
		ret = rk1000_tv_write_block(i, Tv_encoder_regs+i, 1);
		if(ret < 0){
			E("rk1000_tv_write_block err!\n");
			return ret;
		}
	}

	for(i=0; i<sizeof(Tv_encoder_control_regs); i++){
		ret = rk1000_control_write_block(i+3, Tv_encoder_control_regs+i, 1);
		if(ret < 0){
			E("rk1000_control_write_block err!\n");
			return ret;
		}
	}

	return 0;
}

int rk1000_tv_pal_init(void)
{
	uint8 Tv_encoder_regs[] = {0x06, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8 Tv_encoder_control_regs[] = {0x41, 0x01};
	int i;
	int ret;
	
	D("enter\n");

	for(i=0; i<sizeof(Tv_encoder_regs); i++){
		ret = rk1000_tv_write_block(i, Tv_encoder_regs+i, 1);
		if(ret < 0){
			E("rk1000_tv_write_block err!\n");
			return ret;
		}
	}

	for(i=0; i<sizeof(Tv_encoder_control_regs); i++){
		ret = rk1000_control_write_block(i+3, Tv_encoder_control_regs+i, 1);
		if(ret < 0){
			E("rk1000_control_write_block err!\n");
			return ret;
		}
	}

	return 0;
}

int rk1000_tv_Ypbpr480_init(void)
{
	uint8 Tv_encoder_regs[] = {0x00, 0x00, 0x40, 0x08, 0x00, 0x02, 0x17, 0x0A, 0x0A};
	uint8 Tv_encoder_control_regs[] = {0x00};
	int i;
	int ret;
	
	D("enter\n");

	for(i=0; i<sizeof(Tv_encoder_regs); i++){
		ret = rk1000_tv_write_block(i, Tv_encoder_regs+i, 1);
		if(ret < 0){
			E("rk1000_tv_write_block err!\n");
			return ret;
		}
	}

	for(i=0; i<sizeof(Tv_encoder_control_regs); i++){
		ret = rk1000_control_write_block(i+3, Tv_encoder_control_regs+i, 1);
		if(ret < 0){
			E("rk1000_control_write_block err!\n");
			return ret;
		}
	}

	return 0;
}

int rk1000_tv_Ypbpr576_init(void)
{
	uint8 Tv_encoder_regs[] = {0x06, 0x00, 0x40, 0x08, 0x00, 0x01, 0x17, 0x0A, 0x0A};
	uint8 Tv_encoder_control_regs[] = {0x00};
	int i;
	int ret;
	
	D("enter\n");

	for(i=0; i<sizeof(Tv_encoder_regs); i++){
		ret = rk1000_tv_write_block(i, Tv_encoder_regs+i, 1);
		if(ret < 0){
			E("rk1000_tv_write_block err!\n");
			return ret;
		}
	}

	for(i=0; i<sizeof(Tv_encoder_control_regs); i++){
		ret = rk1000_control_write_block(i+3, Tv_encoder_control_regs+i, 1);
		if(ret < 0){
			E("rk1000_control_write_block err!\n");
			return ret;
		}
	}

	return 0;
}

int rk1000_tv_Ypbpr720_init(void)
{
	uint8 Tv_encoder_regs[] = {0x06, 0x00, 0x40, 0x08, 0x00, 0x03, 0x17, 0x0A, 0x0A};
	uint8 Tv_encoder_control_regs[] = {0x00};
	int i;
	int ret;
	
	D("enter\n");

	for(i=0; i<sizeof(Tv_encoder_regs); i++){
		ret = rk1000_tv_write_block(i, Tv_encoder_regs+i, 1);
		if(ret < 0){
			E("rk1000_tv_write_block err!\n");
			return ret;
		}
	}

	for(i=0; i<sizeof(Tv_encoder_control_regs); i++){
		ret = rk1000_control_write_block(i+3, Tv_encoder_control_regs+i, 1);
		if(ret < 0){
			E("rk1000_control_write_block err!\n");
			return ret;
		}
	}

	return 0;
}

int rk1000_tv_standby(u8 enable)
{
	uint8 val1;
	uint8 val2;
	int ret;

	D("enter\n");

	if(enable){
		val1 = 0x00;
		val2 = 0x07;
		
		ret = rk1000_control_write_block(0x03, &val1, 1);
		if(ret < 0){
			E("rk1000_control_write_block err!\n");
			return ret;
		}
		
		ret = rk1000_tv_write_block(0x03, &val2, 1);
		if(ret < 0){
			E("rk1000_tv_write_block err!\n");
			return ret;
		}
	}

	return 0;
}

void set_tv_info(struct rk28fb_screen *screen)
{
	struct rk28fb_screen *screen1 = screen + 1;
	struct rk28fb_screen *screen2 = screen + 2;
	struct rk28fb_screen *screen3 = screen + 3;
	struct rk28fb_screen *screen4 = screen + 4;

	/* ****************** cvbs ntsc ******************* */
	/* screen type & face */
	screen->type = OUT_TYPE;
	screen->face = OUT_FACE;

	/* Screen size */
	screen->x_res = H_VD;
	screen->y_res = V_VD;

	/* Timing */
	screen->pixclock = OUT_CLK;
	screen->left_margin = H_BP;
	screen->right_margin = H_FP;
	screen->hsync_len = H_PW;
	screen->upper_margin = V_BP;
	screen->lower_margin = V_FP;
	screen->vsync_len = V_PW;

	/* Pin polarity */
	screen->pin_hsync = 0;
	screen->pin_vsync = 0;
	screen->pin_den = 0;
	screen->pin_dclk = DCLK_POL;

	/* Swap rule */
	screen->swap_rb = SWAP_RB;
	screen->swap_rg = 0;
	screen->swap_gb = 0;
	screen->swap_delta = 0;
	screen->swap_dumy = 0;

	/* Operation function*/
	screen->init = rk1000_tv_ntsc_init;
	screen->standby = rk1000_tv_standby;

	/* ****************** cvbs pal ******************* */
	/* screen type & face */
	screen1->type = OUT_TYPE;
	screen1->face = OUT_FACE;

	/* Screen size */
	screen1->x_res = H1_VD;
	screen1->y_res = V1_VD;

	/* Timing */
	screen1->pixclock = OUT1_CLK;
	screen1->left_margin = H1_BP;
	screen1->right_margin = H1_FP;
	screen1->hsync_len = H1_PW;
	screen1->upper_margin = V1_BP;
	screen1->lower_margin = V1_FP;
	screen1->vsync_len = V1_PW;

	/* Pin polarity */
	screen1->pin_hsync = 0;
	screen1->pin_vsync = 0;
	screen1->pin_den = 0;
	screen1->pin_dclk = DCLK_POL;

	/* Swap rule */
	screen1->swap_rb = SWAP_RB;
	screen1->swap_rg = 0;
	screen1->swap_gb = 0;
	screen1->swap_delta = 0;
	screen1->swap_dumy = 0;

	/* Operation function*/
	screen1->init = rk1000_tv_pal_init;
	screen1->standby = rk1000_tv_standby;

	/* ****************** Ypbpr 480p ******************* */
	/* screen type & face */
	screen2->type = OUT_TYPE;
	screen2->face = OUT_FACE;

	/* Screen size */
	screen2->x_res = H2_VD;
	screen2->y_res = V2_VD;

	/* Timing */
	screen2->pixclock = OUT2_CLK;
	screen2->left_margin = H2_BP;
	screen2->right_margin = H2_FP;
	screen2->hsync_len = H2_PW;
	screen2->upper_margin = V2_BP;
	screen2->lower_margin = V2_FP;
	screen2->vsync_len = V2_PW;

	/* Pin polarity */
	screen2->pin_hsync = 0;
	screen2->pin_vsync = 0;
	screen2->pin_den = 0;
	screen2->pin_dclk = DCLK_POL;

	/* Swap rule */
	screen2->swap_rb = SWAP_RB;
	screen2->swap_rg = 0;
	screen2->swap_gb = 0;
	screen2->swap_delta = 0;
	screen2->swap_dumy = 0;

	/* Operation function*/
	screen2->init = rk1000_tv_Ypbpr480_init;
	screen2->standby = rk1000_tv_standby;

	/* ****************** Ypbpr 576p ******************* */
	/* screen type & face */
	screen3->type = OUT_TYPE;
	screen3->face = OUT_FACE;

	/* Screen size */
	screen3->x_res = H3_VD;
	screen3->y_res = V3_VD;

	/* Timing */
	screen3->pixclock = OUT3_CLK;
	screen3->left_margin = H3_BP;
	screen3->right_margin = H3_FP;
	screen3->hsync_len = H3_PW;
	screen3->upper_margin = V3_BP;
	screen3->lower_margin = V3_FP;
	screen3->vsync_len = V3_PW;

	/* Pin polarity */
	screen3->pin_hsync = 0;
	screen3->pin_vsync = 0;
	screen3->pin_den = 0;
	screen3->pin_dclk = DCLK_POL;

	/* Swap rule */
	screen3->swap_rb = SWAP_RB;
	screen3->swap_rg = 0;
	screen3->swap_gb = 0;
	screen3->swap_delta = 0;
	screen3->swap_dumy = 0;

	/* Operation function*/
	screen3->init = rk1000_tv_Ypbpr576_init;
	screen3->standby = rk1000_tv_standby;

	/* ****************** Ypbpr 720 ******************* */
	/* screen type & face */
	screen4->type = OUT_TYPE;
	screen4->face = OUT_FACE;

	/* Screen size */
	screen4->x_res = H4_VD;
	screen4->y_res = V4_VD;

	/* Timing */
	screen4->pixclock = OUT4_CLK;
	screen4->left_margin = H4_BP;
	screen4->right_margin = H4_FP;
	screen4->hsync_len = H4_PW;
	screen4->upper_margin = V4_BP;
	screen4->lower_margin = V4_FP;
	screen4->vsync_len = V4_PW;

	/* Pin polarity */
	screen4->pin_hsync = 0;
	screen4->pin_vsync = 0;
	screen4->pin_den = 0;
	screen4->pin_dclk = DCLK_POL;

	/* Swap rule */
	screen4->swap_rb = SWAP_RB;
	screen4->swap_rg = 0;
	screen4->swap_gb = 0;
	screen4->swap_delta = 0;
	screen4->swap_dumy = 0;

	/* Operation function*/
	screen4->init = rk1000_tv_Ypbpr720_init;
	screen4->standby = rk1000_tv_standby;
}



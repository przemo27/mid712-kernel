#include <linux/fb.h>
#include <linux/delay.h>
#include <asm/arch/lcdcon.h>
#include <asm/arch/rk28_i2c.h>
#include <asm/arch/rk28_fb.h>
#include <asm/arch/gpio.h>
#include <asm/arch/iomux.h>
#include "screen.h"

/* Base */
#define OUT_TYPE		SCREEN_RGB
#define OUT_FACE		OUT_P888
#if 1  //LCD
#if(defined(CONFIG_BOARD_TD10D6))
#define OUT_CLK			66

/* Timing */
#define H_PW			10
#define H_BP			206
#define H_VD			1024  //800
#define H_FP			40

#define V_PW			10
#define V_BP			25
#define V_VD			768 //480
#define V_FP			10
#define DCLK_POL		0
#define SWAP_RB			1
#elif(defined(CONFIG_BOARD_IPAD100))
#define OUT_CLK			50

/* Timing */
#define H_PW			10
#define H_BP			206
#define H_VD			1024  //800
#define H_FP			40

#define V_PW			10
#define V_BP			25
#define V_VD			600//576 //480
#define V_FP			10
#define DCLK_POL		0
#define SWAP_RB			1
#else
#define OUT_CLK			40

/* Timing */
#define H_PW			10
#define H_BP			206
#define H_VD			1024  //800
#define H_FP			40

#define V_PW			10
#define V_BP			25
#define V_VD			600 //480
#define V_FP			10
#define DCLK_POL		0
#define SWAP_RB			1
#endif
#else	//hdmi
#define OUT_CLK        73              //输出CLK
#define H_PW           40
#define H_BP           260
#define H_VD           1280            //水平有效分辨率
#define H_FP           440
#define V_PW           5
#define V_BP           25
#define V_VD           720             //垂直有效分辨率
#define V_FP           5
#define DCLK_POL		1
#define SWAP_RB			1

#endif
/* Other */

int init(void);
int standby(u8 enable);

void set_lcd_info(struct rk28fb_screen *screen)
{
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
    screen->init = init;
    screen->standby = standby;
}


void spi_screenreg_set(uint32 Addr, uint32 Data)
{}


int init(void)
{
    return 0;
}

int standby(u8 enable)
{
    return 0;
}


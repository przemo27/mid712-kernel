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
#define OUT_CLK			24

/* Timing */
#define H_PW			2
#define H_BP			12
#define H_VD			800
#define H_FP			12

#define V_PW			1
#define V_BP			2
#define V_VD			480
#define V_FP			1
#define DCLK_POL		1
#define SWAP_RB			1
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
{
#define CS_OUT()        GPIOSetPinDirection(GPIOPortF_Pin4, GPIO_OUT)
#define CS_SET()        GPIOSetPinLevel(GPIOPortF_Pin4, GPIO_HIGH)
#define CS_CLR()        GPIOSetPinLevel(GPIOPortF_Pin4, GPIO_LOW)
#define CLK_OUT()       GPIOSetPinDirection(GPIOPortE_Pin7, GPIO_OUT)  //I2C0_SCL
#define CLK_SET()       GPIOSetPinLevel(GPIOPortE_Pin7, GPIO_HIGH)
#define CLK_CLR()       GPIOSetPinLevel(GPIOPortE_Pin7, GPIO_LOW)
#define TXD_OUT()       GPIOSetPinDirection(GPIOPortE_Pin6, GPIO_OUT)  //I2C0_SDA
#define TXD_SET()       GPIOSetPinLevel(GPIOPortE_Pin6, GPIO_HIGH)
#define TXD_CLR()       GPIOSetPinLevel(GPIOPortE_Pin6, GPIO_LOW)

#define DRVDelayUs(i)   udelay(i*2)

    uint32 i;
    uint32 Indexcomd=0x74;
    uint32 Datacomd=0x76;
    TXD_OUT();
    CLK_OUT();
    CS_OUT();
    DRVDelayUs(2);
    DRVDelayUs(2);

    CS_SET();
    TXD_SET();
    CLK_SET();
    DRVDelayUs(2);

	CS_CLR();
    for(i = 0; i < 8; i++)  //reg
	{
		if(Indexcomd &(1<<(7-i)))
			TXD_SET();
		else
			TXD_CLR();

		// \u6a21\u62dfCLK
		CLK_CLR();
		DRVDelayUs(2);
		CLK_SET();
		DRVDelayUs(2);
	}
      for(i = 0; i < 8; i++)  //reg
      {
      TXD_CLR();
      CLK_CLR();
      DRVDelayUs(2);
      CLK_SET();
      DRVDelayUs(2);
      }
	for(i = 0; i < 8; i++)  //reg
	{
		if(Addr &(1<<(7-i)))
			TXD_SET();
		else
			TXD_CLR();

		// \u6a21\u62dfCLK
		CLK_CLR();
		DRVDelayUs(2);
		CLK_SET();
		DRVDelayUs(2);
	}

	TXD_CLR();  //write

	// \u6a21\u62dfCLK
    CLK_CLR();
    DRVDelayUs(2);
    CLK_SET();
    DRVDelayUs(2);

	TXD_SET();  //highz

	// \u6a21\u62dfCLK
    CLK_CLR();
    DRVDelayUs(2);
    CLK_SET();
    DRVDelayUs(2);

         for(i = 0; i < 8; i++)  //data
         {
         if(Datacomd &(1<<(7-i)))
         	TXD_SET();
         else
         	TXD_CLR();
         
         // \u6a21\u62dfCLK
         CLK_CLR();
         DRVDelayUs(2);
         CLK_SET();
         DRVDelayUs(2);
         }
	for(i = 0; i < 16; i++)  //data
	{
		if(Data &(1<<(15-i)))
			TXD_SET();
		else
			TXD_CLR();

		// \u6a21\u62dfCLK
        CLK_CLR();
		DRVDelayUs(2);
		CLK_SET();
		DRVDelayUs(2);
	}

	CS_SET();
	CLK_CLR();
	TXD_CLR();
	DRVDelayUs(2);

}


int init(void)
{
 printk("init  sharp LCD--------------------------------------\n");
    rockchip_mux_api_set(GPIOE_U1IR_I2C1_NAME, IOMUXA_GPIO1_B1);

   
    spi_screenreg_set(0x01, 0x0102);
    spi_screenreg_set(0x02, 0x0400);
    spi_screenreg_set(0x03, 0x28e7);
    spi_screenreg_set(0x04, 0x4100);
    spi_screenreg_set(0x0B, 0x0019);
    spi_screenreg_set(0x0C, 0x700A);
    spi_screenreg_set(0x0D, 0x0200);
    

    spi_screenreg_set(0x11, 0x1101);
    spi_screenreg_set(0x12, 0x1020);
    spi_screenreg_set(0x13, 0x0320);
    spi_screenreg_set(0x14, 0x01E0);
    spi_screenreg_set(0x15, 0x03E8);
    spi_screenreg_set(0x16, 0x0C0B);
    spi_screenreg_set(0x17, 0x0001);
    spi_screenreg_set(0x18, 0x0000);
    spi_screenreg_set(0x30, 0x341F);
    spi_screenreg_set(0x31, 0x0D1B);

    spi_screenreg_set(0x32, 0x1301);
    spi_screenreg_set(0x33, 0x6634);
    spi_screenreg_set(0x34, 0x0034);
    spi_screenreg_set(0x35, 0x0300);
    spi_screenreg_set(0x36, 0x0D1B);
    spi_screenreg_set(0x37, 0x1508);
    spi_screenreg_set(0x38, 0x6634);
    spi_screenreg_set(0x39, 0x8276);
    spi_screenreg_set(0x28, 0x0006);
    spi_screenreg_set(0x22, 0x0090);
    spi_screenreg_set(0x3D, 0x0290);
    spi_screenreg_set(0x21, 0x0F5A);
    spi_screenreg_set(0x3F, 0x1B00);
    spi_screenreg_set(0x3E, 0x0080);	

    rockchip_mux_api_set(GPIOE_U1IR_I2C1_NAME, IOMUXA_I2C1);
    return 0;
}

int standby(u8 enable)
{
       return 0;
    rockchip_mux_api_set(GPIOE_U1IR_I2C1_NAME, IOMUXA_GPIO1_B1);
	if(enable) {
		//spi_screenreg_set(0x03, 0xde);
	} else {
		//spi_screenreg_set(0x03, 0x5f);
	}
    rockchip_mux_api_set(GPIOE_U1IR_I2C1_NAME, IOMUXA_I2C1);
    return 0;
}


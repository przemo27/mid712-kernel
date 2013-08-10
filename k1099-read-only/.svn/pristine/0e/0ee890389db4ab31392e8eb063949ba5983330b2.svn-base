/* arch/arm/mach-goldfish/board-ROCK28DEMO.c
**
** Copyright (C) 2009 ROCKCHIP, Inc.
**  
** This software is licensed under the terms of the GNU General Public
** License version 2, as published by the Free Software Foundation, and
** may be copied, distributed, and modified under those terms.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/irqflags.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/flash.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>
#include <asm/arch/hw_common.h>
#include <asm/arch/hardware.h>
#include <asm/arch/api_i2c.h>
#include <asm/arch/api_intc.h>
#include <asm/arch/intc.h>
#include <asm/arch/rk28_serial.h>
#include <asm/arch/rk28_fb.h>
#include <asm/arch/rk28_spim.h>
#include <asm/arch/iomux.h>
#include <asm/arch/rk28_i2c.h>
#if defined (CONFIG_RK1000_CONTROL)
#include "../drivers/rk1000/rk1000.h"
#endif
#if defined (CONFIG_SND_SOC_WM8988)
#include "../sound/soc/codecs/wm8988.h"
#endif
#include "../drivers/rtc/rtc-HYM8563.h"
#include <asm/arch/rk28_scu.h>
#include <asm/arch/gpio.h>
/********************************************************
 *	DEBUG
 * ******************************************************/
#define	RK28_PRINT 0 
#include <asm/arch/rk28_debug.h>

extern 	void __init rk28_irq_init(unsigned int priority[NR_RK28_IRQS]);
extern unsigned int int_priority[NR_RK28_IRQS];
extern struct sys_timer rockchip_timer;
extern   void  rk28_add_device_touchscreen(void);
extern void __init rk28_add_device_mmc(void);
extern 	void rock28_add_device_key(void);
extern 	void __init rk28_add_device_i2c(struct i2c_board_info *devices, int nr_devices);
extern void rk28_adddevice_nand(void);
extern void rk28_add_device_i2s(void);
extern void rk28_add_device_battery(void);
extern void rk28_add_device_backlight(void);
extern void rockchip_add_device_dsp(void);
extern void rk28_add_usb_devices(void);
extern void rk28_add_device_rk1000_control(void);
extern int uart_init(char uartCh, unsigned int baudRate);
extern void __rockchip_scu_init_hw( void );
extern void __init rockchip_timer_clock_source_init( int apb_clk );
void __init rk28_kld_init( void );
//IO映射方式描述 ，每个为一段线性连续映射
static struct map_desc rockchip_io_desc[] __initdata = {

	{
		.virtual	= AHB_BASEADD_VA,					//虚拟地址
		.pfn		= __phys_to_pfn(AHB_BASEADD_PA),    //物理地址，须与页表对齐
		.length 	= AHB_SIZE,							//长度
		.type		= MT_DEVICE							//映射方式
	},
	
	{
			.virtual	= APB_BASEADD_VA,
			.pfn		= __phys_to_pfn(APB_BASEADD_PA),
			.length 	= APB_SIZE,
			.type		= MT_DEVICE
	},

            {
			.virtual	= 0xff400000,           /* for itcm , vir = phy , for reboot */
			.pfn		= __phys_to_pfn(0xff400000),
			.length 	= APB_SIZE,
			.type		= MT_DEVICE
	}
		
};

 /******************************************************************************************
 * Serial port configuration.
 *    0 .. 1 = USART0 ,USART1
 ******************************************************************************************/
static struct rock_uart_config __initdata rockchip_uart_config = {  
	.console_tty	= 0,  //1,  //0,				//ttyS0
	.nr_tty		= 1,  //2,  //1,
	.tty_map	= { 0, 1 }		/// ttyS0,  ttyS1 
}; 



/*****************************************************************************************
 * SPI devices
 *author: lhh
 *****************************************************************************************/
static struct spi_board_info board_spi_devices[] = {
	{	/* net chip */
		.modalias	= "enc28j60",
		.chip_select	= 0,
		.max_speed_hz	= 12 * 1000 * 1000,
		.bus_num	= 0,
	},

  	{	/*spi touscreen (xpt2046) auth wqq 2009-05-16*/
		.modalias	= "xpt2046_ts",
		.chip_select	= 1,
		.max_speed_hz	= 1000000,
		.bus_num	= 0,
		.irq = IRQ_GPIO1,
	},

};

/**************************************************************************
* I2C devices
*author :wqq
*date : 2009-5-11
**************************************************************************/

static struct i2c_board_info __initdata board_i2c_devices[] = {
#if defined (CONFIG_RTC_HYM8563)
{
	.driver_name	= "rtc_HYM8563",
	.type			= "RTC",
	.addr			= 0xA2,

},
#endif


#if defined (CONFIG_RK28_I2C4310_TS)
{
	.driver_name	= "ra4310name",
	.type			= "TS",
	.addr			= 0x38,

},
#endif

#if defined (CONFIG_RK28_I2C_TS_GTT8205S)
{
	.driver_name  = "gtt8205s",
	.type	=   "TS",
	.addr 	=  0x15,
},
#endif
#if defined (CONFIG_SND_SOC_WM8988)
{
	.driver_name	= "WM8988 I2C Codec",
	.type			= "CODEC",
	.addr			= 0x34,
},
#endif

#if defined (CONFIG_GS_MMA7660)
{
	.driver_name	= "gs_mma7660",
	.type			= "GSENSOR",
	.addr			= 0x98,
},
#endif
#if defined (CONFIG_RK1000_CONTROL)
    {
        .driver_name    = "RK1000_CONTROL",
        .type           = "I2C_CONTROL",
        .addr           = 0x80,
    },
#endif
#if defined (CONFIG_RK1000_TVOUT)
    {
        .driver_name    = "RK1000_TVOUT",
        .type           = "TVOUT",
        .addr           = 0x84,
    },
#endif    
#if defined (CONFIG_SND_SOC_RK1000)
    {
        .driver_name    = "RK1000 I2C Codec",
        .type           = "CODEC",
        .addr           = 0xc0,
    },
#endif
#if defined (CONFIG_RTC_PT7C4337)
	{
		.driver_name	= "rtc_PT7C4337",
		.type			= "RTC",
		.addr			= 0xD0,
	
	},
#endif

};
/********************************************************************************
*	POWER	:gpio
*	Auth		:wqq
*	function	:set gpio when power on 
*************************************************************************************/
#ifdef CONFIG_RK28_I2C_TS_GTT8205S
static void rk28_gtt8205_init( void )
{
	/*reset touchscreen*/
	rockchip_mux_api_set(GPIOF0_UART1_CPWM0_NAME, IOMUXA_GPIO1_B0);
	GPIOSetPinDirection(GPIOPortF_Pin0,GPIO_OUT);
	
	GPIOSetPinLevel(GPIOPortF_Pin0,GPIO_HIGH);
	
	mdelay(100);
	
	GPIOSetPinLevel(GPIOPortF_Pin0,GPIO_LOW);
}
#else
 
	static void rk28_gtt8205_init( void ) {}

#endif

#if 1
static void rk28_codec_spcon_init( void )
{
	rockchip_mux_api_set(GPIOE_SPI1_SEL_NAME, IOMUXA_GPIO1_A1237);/*speaker disable pin PF7*/
	GPIOSetPinDirection(GPIOPortF_Pin7,GPIO_OUT);
	GPIOSetPinLevel(GPIOPortF_Pin7,GPIO_LOW);
	mdelay(10);
}
#else

static void rk28_codec_spcon_init()  {}

#endif
static void rk28_power_on(void)
{

	/*disable codec spcon*/
	rk28_codec_spcon_init( );
	
	/*set PLAY_ON and PWR_ON pins to gpio mode*/
	
	
	rockchip_mux_api_set(GPIOF1_UART1_CPWM1_NAME, IOMUXA_GPIO1_B1);	/*PWR_ON*/
	
	rockchip_mux_api_set(GPIOF0_UART1_CPWM0_NAME, IOMUXA_GPIO1_B0);	/*PLAY_ON*/
#if 1
	
/*set  PF1=1 PF2=1 for close backlight*/
	
	rockchip_mux_api_set(GPIOF2_APWM0_SEL_NAME, IOMUXB_GPIO1_B2);
	
	GPIOSetPinDirection(GPIOPortF_Pin2,GPIO_OUT);
	
	GPIOSetPinLevel(GPIOPortF_Pin2,GPIO_HIGH);
	
	mdelay(500);
#endif
	
/*enable power pin*/
		
	GPIOSetPinDirection(GPIOPortF_Pin1,GPIO_OUT);
	
	GPIOPullUpDown(GPIOPortF_Pin1,GPIOPullUp);
	
	GPIOSetPinLevel(GPIOPortF_Pin1,GPIO_HIGH);

	
	GPIOSetPinDirection(GPIOPortE_Pin2,GPIO_IN);
	
	GPIOPullUpDown(GPIOPortE_Pin2,GPIOPullUp);
	
/*init touchscreen*/	
	rk28_gtt8205_init();

}

/*************************************************************************************
 
*   POWER	:
 
*    Auth		: WQQ
 
*	function 	: set gpio when power down
 
*************************************************************************************/
 
static void rk28_power_off(void)
{
	local_irq_enable();
 	while( 1 ) {
  		debug_print("shut down the system ...\n");
		printk("power pin status == %d!!\n",GPIOGetPinLevel(GPIOPortF_Pin1));
		GPIOSetPinLevel(GPIOPortF_Pin1,GPIO_LOW);	/*power down*/
		if(GPIOGetPinLevel(GPIOPortF_Pin1) == 0)	/*avoid alway set gpio during poor battery*/
		{
			/*set charge status*/
			if(GPIOGetPinLevel(GPIOPortB_Pin1))	/*set battery charge*/
			{
	 			GPIOSetPinLevel(GPIOPortB_Pin0,GPIO_HIGH);
				local_irq_disable();
				break;
			}
		}
       	debug_print("retry to ");
    }
	printk("power pin status == %d!!\n",GPIOGetPinLevel(GPIOPortF_Pin1));
}


/* 初始化IO映射表*/
static void __init machine_rk28_mapio(void)
{

	
	iotable_init(rockchip_io_desc, ARRAY_SIZE(rockchip_io_desc));
 
    	uart_init(0 , 115200);
	
	__rockchip_scu_init_hw();
 
   	rockchip_timer_clock_source_init( rockchip_clk_get_apb() );
	
	rockchip_iomux_init();
	
/* Setup the serial ports and console*/
 
 	rockchip_init_serial(&rockchip_uart_config);
 
 	rk28_kld_init();
}

/*初始化IRQ*/
void machine_rk28_init_irq(void)
{	
	rk28_irq_init(int_priority);
}




//初始化各设备
void machine_rk28_board_init(void)
{
	
	/*Power on*/
	
rk28_power_on( );
	
pm_power_off = rk28_power_off;
	
/* Serial*/
 
	rockchip_add_device_serial();
	
/*LCD*/
 
 	rk28_add_device_lcdc();    
	
/*SDIO*/
	
	rk28_add_device_mmc();
	
/*AD button*/
	
	rock28_add_device_key();
	
 /* SPI */
	
	rockchip_add_device_spi_master(board_spi_devices, ARRAY_SIZE(board_spi_devices));
	
/*NAND*/
	
	rk28_adddevice_nand();
	
/*I2C*/
	
	rk28_add_device_i2c(board_i2c_devices,ARRAY_SIZE(board_i2c_devices));
	
//rockchip_add_device_i2c1(board_i2c1_devices,ARRAY_SIZE(board_i2c1_devices));
	
/*I2S*/
	
	rk28_add_device_i2s();

	
	rk28_add_device_battery();
	
	rk28_add_usb_devices();

	
	rk28_add_device_backlight();
	
	rockchip_add_device_dsp();
}

extern const char linux_banner[];
const char rockchip_version[] = 
       // "VERSION:2009-11-06 1.0.0 , with debug info,fix usb,ram,suspend,wifi some bug.\n"
       // "rockchip version 1.0.02 (ruiguan) #with debug infomation.\n"
      //  "rockchip version 1.0.03 (ruiguan merged) #with debug infomation.\n"
     //   "rockchip version 1.0.04 (ruiguan merged) #with debug infomation,update fb.\n"
     //   "rockchip version 1.0.05 (ruiguan merged) #with debug infomation,update tss8205,remove lowmem shrink.\n"
      //  "rockchip version 1.0.06 (ruiguan) #with debug infomation,modify andriod power ashmem sdmmc no gsensor.\n"
      	//  "rockchip version 1.2.25 (ruiguan) #with debug infomation,add change arm frequency when enter 1level sleep.\n"
    // "rockchip version 1.2.7 (ruiguan) #with debug infomation,mask codec noise &timer bug & battery status.\n"
	"rockchip version 1.2.8_swap (ruiguan) #with debug infomation,add swap parttion& fixed dsp driver bug, for A/V sync\n"
;        
int rk28_version( void )
{
        printk(linux_banner);
        printk(rockchip_version );
        return 0x30;
}

// don`t change the name "MACHINE_ROCK",  -lingzj
MACHINE_START(ROCKCHIP, "RK28board")	
	.phys_io	= AHB_BASEADD_PA,
	
	.io_pg_offst	= (AHB_BASEADD_VA >> 18) & 0xfffc,
	
	.boot_params	= ROCK_SDRAM_BASE + 0x100,
	
	//以上地址有待确认
	
	.timer		= &rockchip_timer,//此处注册一个系统timer,
	
	.map_io 	= machine_rk28_mapio,
	
	.init_irq	= machine_rk28_init_irq,
	
	.init_machine	= machine_rk28_board_init,
MACHINE_END


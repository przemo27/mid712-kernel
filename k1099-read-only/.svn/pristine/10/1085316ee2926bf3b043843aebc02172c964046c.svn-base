/*
 * arch/arm/mach-rockchip/rockchip_devices.c
 *
 *  Copyright (C) 2005 Thibaut VARENE <varenet@parisc-linux.org>
 *  Copyright (C) 2005 David Brownell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */  
#include <linux/kernel.h> 
#include <asm/mach/arch.h>  
#include <asm/mach/map.h>

#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <asm/arch/board.h>
#include <asm/arch/hardware.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/usb/android.h>

#include <asm/io.h>
#include <asm/arch/hw_common.h>
#include <asm/arch/iomux.h>
#include <asm/arch/api_intc.h>
#include <linux/spi/spi.h>
#include <asm/arch/rk28_serial.h>
#include <asm/arch/rk28_i2c.h>
#include <asm/arch/rk28_fb.h>

#include <asm/arch/rk28_sdmmc.h>
#include <asm/arch/rk28_macro.h>
#include <asm/arch/rk28_debug.h>
#include <asm/arch/gpio.h>
#include <asm/arch/iomux.h>
#include <asm/arch/rk28_backlight.h>

//#define RK28_PRINT	1
#include <asm/arch/rk28_debug.h>
/* --------------------------------------------------------------------
 *  lcdc
 *Author :nzy
 * -------------------------------------------------------------------- */
/*resource*/
static struct resource rk28_lcd_resource[] = {
	[0] = {
		.start = LCDC_BASE_ADDR,
		.end   = VIP_BASE_ADDR - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_LCDC,
		.end   = IRQ_LCDC,
		.flags = IORESOURCE_IRQ,
	},
};

/*gpio information*/
static struct rk28fb_gpio rk28_fb_gpio_info = {
#if 1
    .lcd_cs     = (GPIO_LOW<<16)|GPIOPortB_Pin3,
    .display_on = (GPIO_LOW<<16)|GPIOPortB_Pin2,
    .lcd_standby = 0,
#else
    .lcd_cs     = (GPIO_LOW<<16)|GPIOPortB_Pin3,
    .display_on = (GPIO_LOW<<16)|GPIOPortF_Pin5,
    .lcd_standby = (GPIO_HIGH<<16)|GPIOPortB_Pin2,
#endif
};
//#ifdef CONFIG_USB_ANDROID



static char android_serial_number[16] = "11223344";
static struct android_usb_platform_data android_usb_pdata = {
	//below is starndard android usb id
        /*.vendor_id      = 0x0bb4,
        .product_id     = 0x0c01,*/
        
	//below is normal usb mass storage id
        .vendor_id      = 0x05e3,
        .product_id     = 0x0726,

        .adb_product_id = 0x0c02,
        .version        = 0x0100,
        .product_name   = "Android Phone",
        .manufacturer_name = "HTC",
        .nluns = 2,
        .serial_number = android_serial_number,
};

static struct platform_device android_usb_device = {
        .name   = "android_usb",
        .id             = -1,
        .dev            = {
                .platform_data = &android_usb_pdata,
        },
};
//#endif

/*iomux information*/
static struct rk28fb_iomux rk28_fb_iomux_info = {
    .data16     = GPIOC_LCDC16BIT_SEL_NAME,
    .data18     = GPIOC_LCDC18BIT_SEL_NAME,
    .data24     = GPIOC_LCDC24BIT_SEL_NAME,
    .den        = CXGPIO_LCDDEN_SEL_NAME,
    .vsync      = CXGPIO_LCDVSYNC_SEL_NAME,
};

static struct rk28fb_mach_info fbmach_info = {
    .gpio = &rk28_fb_gpio_info,
    .iomux = &rk28_fb_iomux_info,
};

/*platform_device*/
struct platform_device rk28_device_lcd = {
	.name		  = "rk28-lcdc",
	.id		  = 3,
	.num_resources	  = ARRAY_SIZE(rk28_lcd_resource),
	.resource	  = rk28_lcd_resource,
	.dev            = {
		.platform_data  = &fbmach_info,
	}
};

void __init rk28_add_device_lcdc(void)
{
	
    	///printk("---->%s..%s..:%i\n",__FILE__,__FUNCTION__,__LINE__);
    	platform_device_register(&rk28_device_lcd);
}

/* --------------------------------------------------------------------
 *  UART
 *Author :lhh
 * -------------------------------------------------------------------- */
#if defined(CONFIG_ROCK_UART)

int uartportregs		= UART0_BASE_ADDR_VA;
int uartportregs1		= UART1_BASE_ADDR_VA;
static struct resource uart0_resources[] = {
	[0] = {
		.start	= UART0_BASE_ADDR,
		.end	= UART0_BASE_ADDR + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_UART0,
		.end	= IRQ_UART0,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device rockchip_uart0_device = {
	.name		= "rockchip_usart",  //"rockchip_uart0",
	.id		= 0,
	.resource	= uart0_resources,
	.num_resources	= ARRAY_SIZE(uart0_resources),
};

static struct resource uart1_resources[] = {
	[0] = {
		.start	= UART1_BASE_ADDR,
		.end	= UART1_BASE_ADDR + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_UART1,
		.end	= IRQ_UART1,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device rockchip_uart1_device = {
	.name		= "rockchip_usart",  //"rockchip_uart0",
	.id		= 1,
	.resource	= uart1_resources,
	.num_resources	= ARRAY_SIZE(uart1_resources),
};

static struct platform_device *rockchip_uarts[NR_PORTS];	/* the UARTs to use */
struct platform_device *rockchip_default_console_device;	/* the serial console device */


void __init __deprecated rockchip_init_serial(struct rock_uart_config *config)
{
	int i;
    /*
		__raw_writel(0x07,uartportregs+0x88);
		__raw_writel(0x00,uartportregs+0x04);
		__raw_writel(0x01,uartportregs+0x98);
		__raw_writel(0x03,uartportregs+0x9c);
		__raw_writel(0x01,uartportregs+0xa0);  //use uart1 init
		*/
	/* Fill in list of supported UARTs */
	for (i = 0; i <  config->nr_tty; i++) {
		switch (config->tty_map[i]) {
	    case 0:
				rockchip_mux_api_set(GPIOG1_UART0_MMC1WPT_NAME, IOMUXA_UART0_SOUT);
				rockchip_mux_api_set(GPIOG0_UART0_MMC1DET_NAME, IOMUXA_UART0_SIN);
				rockchip_uarts[i] = &rockchip_uart0_device;
				break;
			case 1:
				rockchip_mux_api_set(GPIOF1_UART1_CPWM1_NAME, IOMUXA_UART1_SOUT);
				rockchip_mux_api_set(GPIOF0_UART1_CPWM0_NAME, IOMUXA_UART1_SIN); 
				rockchip_uarts[i] = &rockchip_uart1_device;
				/*
				__raw_writel(0x83,uartportregs+0x0c);
				__raw_writel(0x0d,uartportregs);
				__raw_writel(0x00,uartportregs+0x04);
				__raw_writel(0x03,uartportregs+0x0c);//use uart1 init rate
				*/
				break;
			default:
				continue;
		}
		rockchip_uarts[i]->id = i;		/* update ID number to mapped ID */
	}
 	
	/* Set serial console device */
	if (config->console_tty<NR_PORTS)
          rockchip_default_console_device = rockchip_uarts[config->console_tty];
	if (!rockchip_default_console_device)
		printk(KERN_INFO "ROCKCHIP: No default serial console defined.\n");
	
}

void __init rockchip_register_uart(unsigned id, unsigned portnr, unsigned pins)
{
	struct platform_device *pdev;


	switch (id) {
		case IRQ_UART0:
			pdev = &rockchip_uart0_device;
			rockchip_mux_api_set(GPIOG1_UART0_MMC1WPT_NAME, IOMUXA_UART0_SOUT);
		  rockchip_mux_api_set(GPIOG0_UART0_MMC1DET_NAME, IOMUXA_UART0_SIN);
			break;
		case IRQ_UART1:
			pdev = &rockchip_uart1_device;
			rockchip_mux_api_set(GPIOF1_UART1_CPWM1_NAME, IOMUXA_UART1_SOUT);
		  rockchip_mux_api_set(GPIOF0_UART1_CPWM0_NAME, IOMUXA_UART1_SIN); 
			break;
		default:
			return;
	}
	pdev->id = portnr;		/* update to mapped ID */
      if (portnr < NR_PORTS) 
         rockchip_uarts[portnr] = pdev;      
	
}
void __init rockchip_set_serial_console(unsigned portnr)
{
	if (portnr < NR_PORTS) 
            rockchip_default_console_device = rockchip_uarts[portnr];
	if (!rockchip_default_console_device)
		printk(KERN_INFO "ROCKCHIP: No default serial console defined.\n");
} 

void __init rockchip_add_device_serial(void)
{
	int i; 
        for (i = 0; i< NR_PORTS; i++){
           if (rockchip_uarts[i])
			platform_device_register(rockchip_uarts[i]);	
			//platform_device_register(rockchip_uarts[1]);			
	}
} 
#else
void __init __deprecated rockchip_init_serial(struct rock_uart_config *config) {}
void __init rockchip_register_uart(unsigned id, unsigned portnr, unsigned pins) {}
void __init rockchip_set_serial_console(unsigned portnr) {}
void __init rockchip_add_device_serial(void) {}
#endif 



/****************************************************************************
*		                     I2C
*Author	: wy
*****************************************************************************/
#if defined(CONFIG_I2C_RK28)

static struct resource i2c_resources[] = {
	[0] = {
		.start	= RK28_BASE_I2C,
		.end	= RK28_BASE_I2C + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= RK28_ID_I2C,
		.end	= RK28_ID_I2C,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device rk28_i2c_device = {
	.name		= "rk28_i2c",
	.id 	= -1,
	.resource	= i2c_resources,
	.num_resources	= ARRAY_SIZE(i2c_resources),
};

unsigned int arm_freq_temp = 300000;
void i2c_delayus_temp(unsigned int count)
{
	unsigned int temp = count * arm_freq_temp;
	while(--temp){}
	return ;
}


#if defined(CONFIG_I2C_ROCKSOFT)

#define RK28_ID_i2c 16
static struct resource I2C_resources[] = {
		[0] = {
			.start	= I2C0_BASE_ADDR,
			.end	= I2C0_BASE_ADDR + SZ_4K - 1,
			.flags	= IORESOURCE_MEM,
		},
		[1] = {
			.start	= RK28_ID_i2c,
			.end	= RK28_ID_i2c,
			.flags	= IORESOURCE_IRQ,
		},
	};
	
	static struct platform_device rk28_i2c_device = {
		.name		= "rock-i2csoft",
		.id 	= 0,
		.resource	= I2C_resources,
		.num_resources	= ARRAY_SIZE(I2C_resources),
	};


#endif





void __init rk28_add_device_i2c(struct i2c_board_info *devices, int nr_devices)
{

	i2c_register_board_info(0, devices, nr_devices);
	platform_device_register(&rk28_i2c_device);

	
}
#else
void __init rk28_add_device_i2c(struct i2c_board_info *devices, int nr_devices) {}

#endif

/**************************************************************************************
 *	TouchScreen(xpt2046)
 *Author :wqq
 * ***********************************************************************************/

#if defined(CONFIG_RK28_GPIO_TS)

 static struct platform_device rk28_xpt2046_ts = {
	.name = "xpt2046_ts",
	.id = -1,
};

void __init rk28_add_device_touchscreen(void)
{
	unsigned int config;
	config = __raw_readl(REG_FILE_BASE_ADDR_VA+0x20);
	//rk28printk("%s....%s....iomux_a_con=%d\n",__FILE__,__FUNCTION__,config);
	platform_device_register(&rk28_xpt2046_ts);
}
#else
void __init rk28_add_device_touchscreen(void) { }

#endif

/******************************************************************************************
*	SDMMC
*Author :xbw
******************************************************************************************/

	 
#if defined(CONFIG_MMC_RK28) || defined(CONFIG_MMC_RK28_MODULE)

      //SDMMC0主要用于SD/MMC卡
        static struct rk28_mmc_data __initdata rk28_sdmmc0_data = {
		.det_pin	= (u8)GPIOPortF_Pin3,

     #if defined(CONFIG_SDMMC0_BUS_WIDTH_4)
        .wire4		= 1,
     #else
        .wire4		= 0,
     #endif
		 
	//	.wp_pin 	= ... not connected
	//	.vcc_pin	= ... not connected
	};

	
	
	static struct resource sdmmc0_resources[] = {
		[0] = {
			.start = SDMMC0_BASE_ADDR,
			.end   = SDMMC0_BASE_ADDR+ SZ_8K -1,
			.flags = IORESOURCE_MEM,
		},
		[1] = {
			 .start = IRQ_SDMMC0,  
			.end	= IRQ_SDMMC0, 
			.flags	= IORESOURCE_IRQ,
		},
	
	};
	
	
	static struct platform_device rk28_sdmmc0_device = {
		.name		= "rk28_sdmmc0",
		.id 	= -1,
		.dev		= {
					.platform_data		= &rk28_sdmmc0_data,
		},
		.resource	= sdmmc0_resources,
		.num_resources	= ARRAY_SIZE(sdmmc0_resources),
	};


	//SDMMC1主要用于SDIO设备，如Wifi.
	static struct rk28_mmc_data __initdata rk28_sdmmc1_data = {
        #if defined(CONFIG_SDMMC1_BUS_WIDTH_4)
            .wire4		= 1,
        #else
            .wire4		= 0,
        #endif

		//	.det_pin	= ... not connected
		//	.wp_pin 	= ... not connected
		//	.vcc_pin	= ... not connected
		};
	    
	 static struct resource sdmmc1_resources[] = {
		[0] = {
		    .start = SDMMC1_BASE_ADDR,
		    .end   = SDMMC1_BASE_ADDR + SZ_8K -1,
		    .flags = IORESOURCE_MEM,
		},
		[1] = {
		     .start = IRQ_SDMMC1,  
		    .end    = IRQ_SDMMC1, 
		    .flags  = IORESOURCE_IRQ,
		},
	    
	    
	    };
	    
	 static struct platform_device rk28_sdmmc1_device = {
		.name       = "rk28_sdmmc1",
		.id     = -1,
		.dev        = {
		            .platform_data      = &rk28_sdmmc1_data,
		},
		.resource   = sdmmc1_resources,
		.num_resources  = ARRAY_SIZE(sdmmc1_resources),
	    };
	    

	
	
	void __init rk28_add_device_mmc(void)
	{
			
           // printk("\n__init rk28_add_device_mmc:  register  the device of SD or MMC  *****************xbw******************\n\n");
    	    platform_device_register(&rk28_sdmmc0_device); //sdmmc0 主要用 在SD/MMC card

           // printk("\n__init rk28_add_device_mmc:  register  the device of SDIO  *****************xbw******************\n\n");
    	    platform_device_register(&rk28_sdmmc1_device);//sdmmc1 主要用在SDIO, 如Wifi等
  
	}
	
#else
	void __init rk28_add_device_mmc(void) {}
#endif
	
	



/****************************************************************************
*							 rk 28 nand
*****************************************************************************/
	
static struct resource nand_resources[] = {
		[0] = {
			.start	= NANDC_BASE_ADDR,
			.end	      = NANDC_BASE_ADDR  + SZ_4K - 1,
			.flags	= IORESOURCE_MEM,
		},

		[1] = {
			.start	= REG_FILE_BASE_ADDR,
			.end	      = REG_FILE_BASE_ADDR  + SZ_4K - 1,
			.flags	= IORESOURCE_MEM,
		},
	};
static struct platform_device rk28_nand_device = {
	.name		= "rk28xxnand",
	.id 	= -1,
	.resource	= nand_resources,
	.num_resources	= ARRAY_SIZE(nand_resources),
};


void rk28_adddevice_nand(void)
{
	//printk("\n____________rock28_adddevice_nand___________");
	platform_device_register(&rk28_nand_device);


}



/****************************************************************************
*							 key (adc key)
*****************************************************************************/
#define RK28_ID_ADC 15
static struct resource key_resources[] = {
		[0] = {
			.start	= ADC_BASE_ADDR,
			.end	= ADC_BASE_ADDR + SZ_4K - 1,
			.flags	= IORESOURCE_MEM,
		},
		[1] = {
			.start	= RK28_ID_ADC,
			.end	= RK28_ID_ADC,
			.flags	= IORESOURCE_IRQ,
		},
	};
	
	static struct platform_device rk28_key_device = {
		.name		= "rk28_AD_button",
		.id 	= -1,
		.resource	= key_resources,
		.num_resources	= ARRAY_SIZE(key_resources),
	};





void rock28_add_device_key(void)
{
	//printk("\n____________rock28_adddevice_key___________");
	platform_device_register(&rk28_key_device);


}

/***********************************************************
*	  Battery	 
*	author :colin	
*	data:2009-06-05
***************************************************************/
struct platform_device rk28_device_battery = {
		.name	= "rockchip_battery",
		.id 	= -1,
};


void rk28_add_device_battery(void)
{
	rockchip_mux_api_set(GPIOB0_SPI0CSN1_MMC1PCA_NAME, IOMUXA_GPIO0_B0);	/*check supply full pin*/
	platform_device_register(&rk28_device_battery);
}


/***********************************************************
*	  backlight	 
*	author :nzy	
*	data:2009-07-21
***************************************************************/
static struct rk28bl_info rk28_bl_info = {
        .pwm_id   = 0,
        .pw_pin   = GPIO_HIGH | (GPIOPortF_Pin1 << 8) ,
        .bl_ref   = 0,
};

static struct platform_device rk28_device_backlight = {
		.name	= "rk28_backlight",
		.id 	= -1,
        .dev    = {
           .platform_data  = &rk28_bl_info,
        }
};

void rk28_add_device_backlight(void)
{
	platform_device_register(&rk28_device_backlight);
}


/* --------------------------------------------------------------------
 *  SPI
 * -------------------------------------------------------------------- */

#if defined(CONFIG_SPIM_RK28) 

static struct resource spi0_resources[] = {
	[0] = {
		.start	= SPI_MASTER_BASE_ADDR,
		.end	= SPI_MASTER_BASE_ADDR + 0x100,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_SPIM,
		.end	= IRQ_SPIM,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device rockchip_spi0_device = {
	.name		= "rockchip_spi_master",
	.id		= 0,
	.resource	= spi0_resources,
	.num_resources	= ARRAY_SIZE(spi0_resources),
};


void __init rockchip_add_device_spi_master(struct spi_board_info *devices, int nr_devices)
{
	  int i;

	  /* Choose SPI chip-selects */
	  for (i = 0; i < nr_devices; i++) {
		  switch (i) {
		    case 0:
		      rockchip_mux_api_set(GPIOB_SPI0_MMC0_NAME, IOMUXA_SPI0);
		      rockchip_mux_api_set(GPIOB4_SPI0CS0_MMC0D4_NAME, IOMUXA_GPIO0_B4);
		     // __raw_writel(__raw_readl(GPIO1_BASE_ADDR_VA + 0x04) & (~0x40),(GPIO1_BASE_ADDR_VA + 0x04));//gpio1a6
		      //__raw_writel(__raw_readl(GPIO1_BASE_ADDR_VA + 0x38) | 0x40,(GPIO1_BASE_ADDR_VA + 0x38));
		      //__raw_writel(__raw_readl(GPIO1_BASE_ADDR_VA + 0x3c) & (~0x40),(GPIO1_BASE_ADDR_VA + 0x3c));		
		      //__raw_writel((__raw_readl(REG_FILE_BASE_ADDR_VA + 0x30) & (~0x3000)) | 0x1000,(REG_FILE_BASE_ADDR_VA + 0x30));	
		      break;
		    case 1:
		      rockchip_mux_api_set(GPIOB_SPI0_MMC0_NAME, IOMUXA_SPI0);
		      rockchip_mux_api_set(GPIOB0_SPI0CSN1_MMC1PCA_NAME, IOMUXA_GPIO0_B0);		   
		      break;
		    }
	  }

	  spi_register_board_info(devices, nr_devices);

		//rockchip_clock_associate("spi0_clk", &at91sam9261_spi0_device.dev, "spi_clk");
		platform_device_register(&rockchip_spi0_device);
}
#else
void __init rockchip_add_device_spi_master(struct spi_board_info *devices, int nr_devices) {}
#endif

void __init rk28_add_usb_devices(void)
{
//#ifdef CONFIG_USB_ANDROID
        platform_device_register(&android_usb_device);
//#endif
}

/* 
 *IIS
 */
#if defined(CONFIG_SND_ROCKCHIP_SOC_IIS)  
 
static struct resource rockchip_iis_resource[] = {
	[0] = {
		.start = I2S_BASE_ADDR,
		.end   = I2S_BASE_ADDR + 0x20,
		.flags = IORESOURCE_MEM,
	}
};

static u64 rockchip_device_iis_dmamask = 0xffffffffUL;

struct platform_device rockchip_device_iis = {
	.name		  = "rockchip-i2s",
	.id		  = 0,
	.num_resources	  = ARRAY_SIZE(rockchip_iis_resource),
	.resource	  = rockchip_iis_resource,
	.dev              = {
		.dma_mask = &rockchip_device_iis_dmamask,
		.coherent_dma_mask = 0xffffffffUL
	}
};

void rk28_add_device_i2s(void)
{
    platform_device_register(&rockchip_device_iis);
}
#else	
void rk28_add_device_i2s(void){}
#endif

static struct resource rockchip_dsp_resource[] = {
        [0] = {
                .start = DSP_BASE_ADDR,
                .end   = DSP_BASE_ADDR + 0x5fffff,
                .flags = IORESOURCE_DMA,
        },
        [1] = {
                .start  = IRQ_PIUCMD,
                .end    = IRQ_PIUCMD,
                .flags  = IORESOURCE_IRQ,
        },
};
static u64 rockchip_device_dsp_dmamask = 0xffffffffUL;
struct platform_device rockchip_device_dsp = {
        .name             = "rk28-dsp",
        .id               = 0,
        .num_resources    = ARRAY_SIZE(rockchip_dsp_resource),
        .resource         = rockchip_dsp_resource,
        .dev              = {
                .dma_mask = &rockchip_device_dsp_dmamask,
                .coherent_dma_mask = 0xffffffffUL
        }
};

void rockchip_add_device_dsp(void)
{
        platform_device_register(&rockchip_device_dsp);
}

/*----------------------------------------------------------------------- */


/*
 *  linux/drivers/video/sidsafb.c
 *
 *  Driver for SIDSA LCD Controller IP
 *
 *  Copyright (C) 2004 Atmel Norway AS
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#undef DEBUG
#undef IO_DEBUG

//#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/delay.h>

#include <asm/hardware.h>
//#include <asm/platform.h>

#include <asm/arch/gpio.h>
//#include <asm/arch/lcdc.h>

#include "sidsafb.h"
#include "at91sam9261_lcdc.h"

// LINGZJ ADD

struct at91_pioline {
        const char*	pin_name;		/* Name of the pin */
        unsigned int	pio_ctrl_va_base;	/* Virtual Address of the PIO controller*/
        unsigned char	pio_ctrl_id;		/* PIO controller ID */
        unsigned char	pin_num;		/* pin number */
        unsigned char	type;			/* type of the pin: PIO or Peripheral mode */
        unsigned char	direction;		/* if the pin is in PIO mode    --> selects input or output mode */ 
						/* if the pin is in Periph mode --> selects periph A or B */
        unsigned char	use_pullup;		/* pullup enable */
        unsigned char	use_filter;		/* glitch filter enable */
};
extern const struct platform_device *platform_get_device(const char *name, int id);

#define platform_num_resources(dev)     ((dev)->num_resources)
#define platform_resource_start(dev, i) ((dev)->resource[(i)].start)
#define platform_resource_end(dev, i)   ((dev)->resource[(i)].end)
#define platform_resource_flags(dev, i) ((dev)->resource[(i)].flags)
#define platform_resource_len(dev, i)                   \
        (platform_resource_end((dev), (i)) -            \
         platform_resource_start((dev), (i)) + 1)

#define AT91C_IO_PHYS_BASE	0xFFFFF000
#define AT91C_IO_VIRT_BASE	VMALLOC_END	
#define AT91C_BASE_PMC       	0xFFFFFC00 /**< PMC base address */

 /* Convert a physical IO address to virtual IO address */
//#define AT91_IO_P2V(x)	((x) - AT91C_IO_PHYS_BASE + AT91C_IO_VIRT_BASE)
#define AT91C_BASE_PIOA      	0xFFFFF400 /**< PIOA base address */

/*
 * Virtual to Physical Address mapping for IO devices.
 */
//#define AT91C_VA_BASE_EBI	AT91_IO_P2V(AT91C_BASE_EBI)
//#define AT91C_VA_BASE_AIC	AT91_IO_P2V(AT91C_BASE_AIC)
//#define AT91C_VA_BASE_SYS	AT91_IO_P2V(AT91C_BASE_AIC)
//#define AT91C_VA_BASE_DBGU	AT91_IO_P2V(AT91C_BASE_DBGU)
//////#define AT91C_VA_BASE_PIOA	AT91_IO_P2V(AT91C_BASE_PIOA)
//#define AT91C_VA_BASE_PIOB	AT91_IO_P2V(AT91C_BASE_PIOB)
//#define AT91C_VA_BASE_PIOC	AT91_IO_P2V(AT91C_BASE_PIOC)
//#define AT91C_VA_BASE_PITC	AT91_IO_P2V(AT91C_BASE_PITC)
//////#define AT91C_VA_BASE_PMC	AT91_IO_P2V(AT91C_BASE_PMC)
//#define AT91C_VA_BASE_SSC0	AT91_IO_P2V(AT91C_BASE_SSC0)
//#define AT91C_VA_BASE_SSC1	AT91_IO_P2V(AT91C_BASE_SSC1)
//#define AT91C_VA_BASE_SSC2	AT91_IO_P2V(AT91C_BASE_SSC2)
//#define AT91C_VA_BASE_TWI	AT91_IO_P2V(AT91C_BASE_TWI)
//#define AT91C_VA_BASE_UDP	AT91_IO_P2V(AT91_BASE_UDP)
//-----------------------------------------------------------------lingzj modify
#define AT91C_VA_BASE_PIOA	AT91_IO_P2V(AT91_PIOA)
#define AT91C_VA_BASE_PMC	AT91_IO_P2V(AT91_PMC)





#ifndef __ASM_ARCH_AT91SAM9261EK_H
#define __ASM_ARCH_AT91SAM9261EK_H


/* AT91SAM9261 clocks */
#define AT91C_MAIN_CLOCK	179712000	/* from 18.432 MHz crystal (18432000 / 4 * 39) */
#define AT91C_MASTER_CLOCK	99300000
#define AT91C_SLOW_CLOCK	32768		/* slow clock */
#define AT91_PLLB_VALUE		0x10483F0E	/* (18.432 / 14 * 73) /2 = 47.9714  */

/* FLASH */
#define AT91_NANDFLASH_BASE	0x40000000	// NCS0: Flash physical base address

/* SDRAM */
#define AT91_SDRAM_BASE		0x20000000	// NCS1: SDRAM physical base address

/* Internal SRAM base address */
#define AT91C_IRAM_BASE		0x00300000
#define AT91C_IRAM_SIZE		0x00028000

#define AT91C_CONSOLE_DEFAULT_BAUDRATE 115200	/* default serial console baud-rate */

#endif




#define PIO_PER 	(0x0000) 	/**< PIO Enable Register */
#define PIO_PDR 	(0x0004) 	/**< PIO Disable Register */
#define PIO_PSR 	(0x0008) 	/**< PIO Status Register */
#define PIO_OER 	(0x0010) 	/**< Output Enable Register */
#define PIO_ODR 	(0x0014) 	/**< Output Disable Registerr */
#define PIO_OSR 	(0x0018) 	/**< Output Status Register */
#define PIO_IFER 	(0x0020) 	/**< Input Filter Enable Register */
#define PIO_IFDR 	(0x0024) 	/**< Input Filter Disable Register */
#define PIO_IFSR 	(0x0028) 	/**< Input Filter Status Register */
#define PIO_SODR 	(0x0030) 	/**< Set Output Data Register */
#define PIO_CODR 	(0x0034) 	/**< Clear Output Data Register */
#define PIO_ODSR 	(0x0038) 	/**< Output Data Status Register */
#define PIO_PDSR 	(0x003C) 	/**< Pin Data Status Register */
#define PIO_IER 	(0x0040) 	/**< Interrupt Enable Register */
#define PIO_IDR 	(0x0044) 	/**< Interrupt Disable Register */
#define PIO_IMR 	(0x0048) 	/**< Interrupt Mask Register */
#define PIO_ISR 	(0x004C) 	/**< Interrupt Status Register */
#define PIO_MDER 	(0x0050) 	/**< Multi-driver Enable Register */
#define PIO_MDDR 	(0x0054) 	/**< Multi-driver Disable Register */
#define PIO_MDSR 	(0x0058) 	/**< Multi-driver Status Register */
#define PIO_PPUDR 	(0x0060) 	/**< Pull-up Disable Register */
#define PIO_PPUER 	(0x0064) 	/**< Pull-up Enable Register */
#define PIO_PPUSR 	(0x0068) 	/**< Pull-up Status Register */
#define PIO_ASR 	(0x0070) 	/**< Select A Register */
#define PIO_BSR 	(0x0074) 	/**< Select B Register */
#define PIO_ABSR 	(0x0078) 	/**< AB Select Status Register */
#define PIO_OWER 	(0x00A0) 	/**< Output Write Enable Register */
#define PIO_OWDR 	(0x00A4) 	/**< Output Write Disable Register */
#define PIO_OWSR 	(0x00A8) 	/**< Output Write Status Register */
/* -------------------------------------------------------- */
#define PMC_SCER 	(0x0000) 	/**< System Clock Enable Register */
#define PMC_SCDR 	(0x0004) 	/**< System Clock Disable Register */
#define PMC_SCSR 	(0x0008) 	/**< System Clock Status Register */
#define PMC_PCER 	(0x0010) 	/**< Peripheral Clock Enable Register */
#define PMC_PCDR 	(0x0014) 	/**< Peripheral Clock Disable Register */
#define PMC_PCSR 	(0x0018) 	/**< Peripheral Clock Status Register */
#define PMC_MOR 	(0x0020) 	/**< Main Oscillator Register */
#define PMC_MCFR 	(0x0024) 	/**< Main Clock  Frequency Register */
#define PMC_PLLAR 	(0x0028) 	/**< PLL A Register */
#define PMC_PLLBR 	(0x002C) 	/**< PLL B Register */
#define PMC_MCKR 	(0x0030) 	/**< Master Clock Register */
#define PMC_PCKR 	(0x0040) 	/**< Programmable Clock Register */
#define PMC_IER 	(0x0060) 	/**< Interrupt Enable Register */
#define PMC_IDR 	(0x0064) 	/**< Interrupt Disable Register */
#define PMC_SR 	(0x0068) 	/**< Status Register */
#define PMC_IMR 	(0x006C) 	/**< Interrupt Mask Register */
#define readreg_pmc(offset)             readl(AT91C_VA_BASE_PMC + offset)
#define writereg_pmc(value, offset)     writel(value, (AT91C_VA_BASE_PMC + offset))

#define AT91C_PMC_HCK1        (0x1 << 17) /**< (PMC) AHB LCDC Clock Output */
#define AT91C_ID_LCDC  	21 /**< LCD Controller id */
#define pin_to_mask(pin)        (1 << (pin))
#ifndef AT91C_ID_PIOA
#define AT91C_ID_PIOA  	 2 /**< Parallel IO Controller A id */
#endif /* AT91C_ID_PIOA */
#ifndef AT91C_ID_PIOB
#define AT91C_ID_PIOB  	 3 /**< Parallel IO Controller B id */
#endif /* AT91C_ID_PIOB */
#ifndef AT91C_ID_PIOC
#define AT91C_ID_PIOC  	 4 /**< Parallel IO Controller C id */
#endif /* AT91C_ID_PIOC */

#define TYPE_PIO	0
#define TYPE_PERIPH	1





void at91_gpio_set_level (unsigned int pio_va_base, unsigned int pin, unsigned int level)
{
        unsigned int mask = pin_to_mask(pin);

	// This is just a sanity action to force a pin 
	// to output before driving it.
        writel(mask, pio_va_base + PIO_PER);
	writel(mask, pio_va_base + PIO_OER);
	
	if (level)
	  writel(mask, pio_va_base + PIO_SODR);
	else // pin should be cleared
	  writel(mask, pio_va_base + PIO_CODR);

}


void at91_gpio_periph_enable (unsigned int pio_va_base, 
			      unsigned char pin, 
			      unsigned char peripheral, 
			      unsigned char use_pullup, 
			      unsigned char use_filter) 
{
	unsigned int mask = pin_to_mask(pin);

	writel(mask, (pio_va_base + PIO_IDR));	

	if(use_pullup)
		writel(mask, (pio_va_base + PIO_PPUER));
	else
		writel(mask, (pio_va_base + PIO_PPUDR));

	switch (peripheral) 
	{
		case 1:
			writel(mask, (pio_va_base + PIO_BSR));
		break;
		case 0:
		default:
			writel(mask, (pio_va_base + PIO_ASR));
		break;
	}

	/* The Peripheral controls the pin (pio disabled) */
	writel(mask, (pio_va_base + PIO_PDR));
}

void at91_gpio_configure (unsigned int pio_va_base, 
			  unsigned char pin, 
			  unsigned char in_out, 
			  unsigned char use_pullup, 
			  unsigned char use_filter) 
{
        unsigned int mask = pin_to_mask(pin);
	
	/* The PIO controls the pin (periph disabled) */
	writel(mask, (pio_va_base + PIO_PER));

	if(in_out) { /* the pin is an input */
        	writel(mask, (pio_va_base + PIO_ODR));
	}
	else {       /* the pin is an output */
        	writel(mask, (pio_va_base + PIO_OER));
	}

        if(use_pullup)
                writel(mask, (pio_va_base + PIO_PPUER));
        else
                writel(mask, (pio_va_base + PIO_PPUDR));
}

int at91_device_pio_setup (struct at91_pioline *pPin) {

	if(!pPin) {
		printk(KERN_ERR "Define the PIO muxing of this device first !!\n");
		return -ENODEV;
	}
#if 1
	/* Sets all the pio muxing of the corresponding device as defined in its platform_data struct */
	while (pPin->pin_name)
	{
		if ((pPin->pio_ctrl_id != AT91C_ID_PIOA) &&
		    (pPin->pio_ctrl_id != AT91C_ID_PIOB) &&
		    (pPin->pio_ctrl_id != AT91C_ID_PIOC)) {
			printk(KERN_ERR "Bad PIO controler ID %u, correct values are {%u, %u ,%u}\n", 
			       pPin->pio_ctrl_id, AT91C_ID_PIOA, AT91C_ID_PIOB, AT91C_ID_PIOC);
			return -ENODEV;
		}
		if (pPin->type == TYPE_PERIPH) // PIN is in PERIPH mode
			at91_gpio_periph_enable(pPin->pio_ctrl_va_base, pPin->pin_num, pPin->direction, pPin->use_pullup, pPin->use_filter);
		else // PIN is in PIO mode
			at91_gpio_configure(pPin->pio_ctrl_va_base, pPin->pin_num, pPin->direction, pPin->use_pullup, pPin->use_filter);
		pPin++;
	}
#endif
	return 0;
}


void at91_disable_periph_clock(unsigned int irq)
{
	/* Disable the corresponding peripheral clock */
	writereg_pmc(1 << irq, PMC_PCDR);
}

void at91_enable_periph_clock(unsigned int irq)
{
	printk( "----at91_enable_periph_clock!!\n");

	/* Enable the corresponding peripheral clock */
	writereg_pmc(1 << irq, PMC_PCER);
	
	printk( "----at91_enable_periph_clock done\n");
}

void at91_disable_system_clock(unsigned int mask)
{
	/* Disable the corresponding system clock */
	writereg_pmc(mask, PMC_SCDR);
}

void at91_enable_system_clock(unsigned int mask)
{
	printk( "----at91_enable_system_clock!!\n");

	/* Enable the corresponding system clock */
	writereg_pmc(mask, PMC_SCER);
	
	printk( "----at91_enable_system_clock done\n");
}


// Clock management functions 
static inline void at91_lcdc_clock_enable (void) 
{
        at91_enable_system_clock(AT91C_PMC_HCK1);
	at91_enable_periph_clock(AT91C_ID_LCDC);
}


static inline void at91_lcdc_clock_disable (void) 
{
        at91_disable_system_clock(AT91C_PMC_HCK1);
	at91_disable_periph_clock(AT91C_ID_LCDC);
}


// LCD Power up and down function
static inline void at91_lcdc_power_up (void)
{
       at91_gpio_set_level(AT91C_VA_BASE_PIOA, 14, 0); //su.2006.05.13
}


static inline void at91_lcdc_power_down (void)
{
       at91_gpio_set_level(AT91C_VA_BASE_PIOA, 14, 1);
}


//----------------------------------------------------------------------------------



#define LCDC_PUT_BA1(sinfo,v)		writel(v, ((sinfo)->mmio + LCDC_BA1))
#define LCDC_GET_FRMCFG(sinfo)		readl((sinfo)->mmio + LCDC_FRMCFG)
#define LCDC_PUT_FRMCFG(sinfo,v)	writel(v, ((sinfo)->mmio + LCDC_FRMCFG))
#define LCDC_GET_DMACON(sinfo)		readl((sinfo)->mmio + LCDC_DMACON)
#define LCDC_PUT_DMACON(sinfo,v)	writel(v, ((sinfo)->mmio + LCDC_DMACON))
#define LCDC_GET_DMA2DCFG(sinfo)	readl((sinfo)->mmio + LCDC_DMA2DCFG)
#define LCDC_PUT_DMA2DCFG(sinfo,v)	writel(v, ((sinfo)->mmio + LCDC_DMA2DCFG))
#define LCDC_GET_LCDCON1(sinfo)		readl((sinfo)->mmio + LCDC_LCDCON1)
#define LCDC_PUT_LCDCON1(sinfo,v)	writel(v, ((sinfo)->mmio + LCDC_LCDCON1))
#define LCDC_GET_LCDCON2(sinfo)		readl((sinfo)->mmio + LCDC_LCDCON2)
#define LCDC_PUT_LCDCON2(sinfo,v)	writel(v, ((sinfo)->mmio + LCDC_LCDCON2))
#define LCDC_GET_TIM1(sinfo)		readl((sinfo)->mmio + LCDC_TIM1)
#define LCDC_PUT_TIM1(sinfo,v)		writel(v, ((sinfo)->mmio + LCDC_TIM1))
#define LCDC_GET_TIM2(sinfo)		readl((sinfo)->mmio + LCDC_TIM2)
#define LCDC_PUT_TIM2(sinfo,v)		writel(v, ((sinfo)->mmio + LCDC_TIM2))
#define LCDC_GET_LCDFRCFG(sinfo)	readl((sinfo)->mmio + LCDC_LCDFRCFG)
#define LCDC_PUT_LCDFRCFG(sinfo,v)	writel(v, ((sinfo)->mmio + LCDC_LCDFRCFG))
#define LCDC_GET_FIFO(sinfo)		readl((sinfo)->mmio + LCDC_FIFO)
#define LCDC_PUT_FIFO(sinfo,v)		writel(v, ((sinfo)->mmio + LCDC_FIFO))
#define LCDC_GET_MVAL(sinfo)		readl((sinfo)->mmio + LCDC_MVAL)
#define LCDC_PUT_MVAL(sinfo,v)		writel(v, ((sinfo)->mmio + LCDC_MVAL))
#define LCDC_GET_PWRCON(sinfo)		readl((sinfo)->mmio + LCDC_PWRCON)
#define LCDC_PUT_PWRCON(sinfo,v)	writel(v, (sinfo)->mmio + LCDC_PWRCON)
#define LCDC_GET_CTRSTCON(sinfo)	readl((sinfo)->mmio + LCDC_CTRSTCON)
#define LCDC_PUT_CTRSTCON(sinfo,v)	writel(v, ((sinfo)->mmio + LCDC_CTRSTCON))
#define LCDC_GET_CTRSTVAL(sinfo)	readl((sinfo)->mmio + LCDC_CTRSTVAL)
#define LCDC_PUT_CTRSTVAL(sinfo,v)	writel(v, ((sinfo)->mmio + LCDC_CTRSTVAL))
#define LCDC_PUT_IDR(sinfo,v)		writel(v, ((sinfo)->mmio + LCDC_IDR))
#define LCDC_GET_IMR(sinfo)		readl((sinfo)->mmio + LCDC_IMR)
#define LCDC_PUT_ICR(sinfo,v)		writel(v, ((sinfo)->mmio + LCDC_ICR))
#define LCDC_GET_LUT_ENTRYx(sinfo,addr)	readl((sinfo)->mmio + LCDC_LUT_ENTRY + (addr<<2))
#define LCDC_PUT_LUT_ENTRYx(sinfo,addr,v) writel(v, ((sinfo)->mmio + LCDC_LUT_ENTRY + (addr<<2)))

#ifdef  CONFIG_TFT_AT91

//		.left_margin	= 1,		.right_margin	= 33,
//		.upper_margin	= 1,		.lower_margin	= 0,
//		.hsync_len	= 5,		.vsync_len	= 1,
//	        .name           = "TX09D50VM1CCA",
#ifdef CONFIG_FB_TCM
static struct fb_videomode at91_tft_vga_modes[] = {
	{
	        .name           = "LQ035Q7DH01",
		.refresh	= 60,
		.xres		= 240,		.yres		= 320,
		.pixclock	= 4965000,

		.left_margin	= 1,		.right_margin	= 33,
		.upper_margin	= 8,		.lower_margin	= 1,	//su.2006.05.13
		.hsync_len	= 10,		.vsync_len	= 1,

		.sync		= FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
};
#endif
#ifdef CONFIG_FB_SDRAM
#if 1
static struct fb_videomode at91_tft_vga_modes[] = {
	{
	        .name           = "PD064VT5N",
		.refresh	= 65,
		.xres		= 640,		.yres		= 480,
//		.pixclock	= 25175000,
		.pixclock	= 24825000,		

		.left_margin	= 47,		.right_margin	= 15,
		.upper_margin	= 9,		.lower_margin	= 32,	//su.2006.05.13
		.hsync_len	= 95,		.vsync_len	= 1,

		.sync		= FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
};
#endif
#if 0
static struct fb_videomode at91_tft_vga_modes[] = {
	{
	        .name           = "LTM084P363",
		.refresh	= 65,
		.xres		= 800,		.yres		= 600,
		.pixclock	= 32175000,		

		.left_margin	= 249,		.right_margin	= 6,
		.upper_margin	= 1,		.lower_margin	= 23,	//su.2006.05.13
		.hsync_len	= 1,		.vsync_len	= 4,

		.sync		= FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
};
#endif
#endif

#else
static struct fb_videomode vga_modes[] = {
	{
		.refresh	= 48,
		.xres		= 320,		.yres		= 200,
		.pixclock	= 80000,

		.left_margin	= 10,		.right_margin	= 30,
		.upper_margin	= 70,		.lower_margin	= 30,
		.hsync_len	= 30,		.vsync_len	= 3,

		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
};
#endif

static struct fb_monspecs default_monspecs = {
#ifdef  CONFIG_TFT_AT91
        .modedb		= at91_tft_vga_modes,
#else
	.modedb		= vga_modes,
#endif
	.manufacturer	= "VGA",
	.monitor	= "Generic VGA",
	.serial_no	= "xxxx",
	.ascii		= "yyyy",
#ifdef  CONFIG_TFT_AT91
	.modedb_len	= ARRAY_SIZE(at91_tft_vga_modes),
#else
	.modedb_len	= ARRAY_SIZE(vga_modes),
#endif
	.hfmin		= 15000,
	.hfmax		= 64000,
	.vfmin		= 50,
	.vfmax		= 150,
};


/* Driver defaults */
static struct fb_fix_screeninfo sidsafb_fix __devinitdata = {
	.id		= "sidsafb",
	.type		= FB_TYPE_PACKED_PIXELS,
	.visual		= FB_VISUAL_TRUECOLOR,
	.xpanstep	= 0,
	.ypanstep	= 0,
	.ywrapstep	= 0,
	.accel		= FB_ACCEL_NONE,
};


// Make sure we map TCM only once and not at every 
// sidsafb_resize_framebuffer call.
#ifdef CONFIG_FB_TCM
static unsigned int first_time_tcm_map = 1;
#endif



static void sidsafb_update_dma(struct fb_info *info,
			       struct fb_var_screeninfo *var)
{
	struct sidsafb_info *sinfo = info->par;
	struct fb_fix_screeninfo *fix = &info->fix;
	unsigned long dma_addr;
	unsigned long pixeloff;
	printk( "\n----------sidsafb_update_dma !!\n");

	dma_addr = (fix->smem_start + var->yoffset * fix->line_length
		    + var->xoffset * var->bits_per_pixel / 8);
	
	dma_addr &= ~3UL;
	pixeloff = (var->xoffset * var->bits_per_pixel) & 31;

	/* Set framebuffer DMA base address and pixel offset */
	LCDC_PUT_BA1(sinfo, dma_addr);

	//lcdc_debug_readl(&(((AT91PS_LCDC)sinfo->mmio)->LCDC_BA1));

#ifdef CONFIG_MACH_NADIA2VB
	pixeloff <<= 24;
	pixeloff |= LCDC_GET_DMA2DCFG(sinfo) & 0xffff;

	LCDC_PUT_DMA2DCFG(sinfo,pixeloff);

	//lcdc_debug_readl(&(((AT91PS_LCDC)sinfo->mmio)->LCDC_DMA2DCFG));

	/* Update configuration */
	LCDC_PUT_DMACON(sinfo, (LCDC_GET_DMACON(sinfo) | AT91C_LCDC_DMAUPDT));

	//lcdc_debug_readl(&(((AT91PS_LCDC)sinfo->mmio)->LCDC_DMACON));
#endif

}


/**
 *	sidsafb_resize_framebuffer - Allocate framebuffer memory
 *	@sinfo: the frame buffer to allocate memory for
 *
 *	Checks if the required framebuffer size has changed. If so, it
 *	deallocates any existing framebuffer and allocates a new one
 *	of the right size.
 *
 *	The LCD controller must be disabled, and interrupts globally
 *	enabled when this function is called.
 *
 *	Returns -ENOMEM if the new framebuffer could not be allocated,
 *	zero if no reallocation was necessary or one otherwise.
 */
static int sidsafb_resize_framebuffer(struct sidsafb_info *sinfo)
{
	struct fb_info *info = sinfo->info;
	struct fb_fix_screeninfo *fix = &info->fix;
	struct fb_var_screeninfo *var = &info->var;
	unsigned int new_size;
	dma_addr_t new_paddr;
	void *new_vaddr;
	
	new_size = (var->xres_virtual * var->yres_virtual
		    * ((var->bits_per_pixel + 7) / 8));

	if (new_size == fix->smem_len)
	        return 0;

#ifdef CONFIG_FB_TCM
	if (new_size > AT91C_IRAM_SIZE)
	        return -ENOMEM; 
#endif

#ifdef CONFIG_FB_SDRAM
	if (fix->smem_len)
		dma_free_coherent(&sinfo->pdev->dev,
				  fix->smem_len, info->screen_base,
				  fix->smem_start);
#endif
	
	fix->smem_len = 0;
	
	pr_debug("About to allocate frame buffer (%u bytes)...\n", new_size);

#ifdef CONFIG_FB_TCM
	new_paddr = (dma_addr_t)AT91C_IRAM_BASE;
	
	if (first_time_tcm_map) {
		new_vaddr = ioremap((unsigned long)new_paddr, AT91C_IRAM_SIZE);
		first_time_tcm_map--;
	} else
		new_vaddr = info->screen_base;
#endif

#ifdef CONFIG_FB_SDRAM
	new_vaddr = dma_alloc_coherent(&sinfo->pdev->dev, new_size,
				       &new_paddr, GFP_KERNEL);
#endif

	if (!new_vaddr)
		return -ENOMEM;

	info->screen_base = new_vaddr;
	fix->smem_start = new_paddr;
	fix->smem_len = new_size;
	fix->line_length = var->xres_virtual * (var->bits_per_pixel / 8);

	printk(KERN_INFO
	       "sidsafb: %luKiB frame buffer at %08lx (mapped at %p)\n",
	       (unsigned long)info->fix.smem_len / 1024,
	       (unsigned long)info->fix.smem_start,
	       info->screen_base);

	return 1;
}

/**
 *      sidsafb_check_var - Validates a var passed in. 
 *      @var: frame buffer variable screen structure
 *      @info: frame buffer structure that represents a single frame buffer 
 *
 *	Checks to see if the hardware supports the state requested by
 *	var passed in. This function does not alter the hardware
 *	state!!!  This means the data stored in struct fb_info and
 *	struct sidsafb_info do not change. This includes the var
 *	inside of struct fb_info.  Do NOT change these. This function
 *	can be called on its own if we intent to only test a mode and
 *	not actually set it. The stuff in modedb.c is a example of
 *	this. If the var passed in is slightly off by what the
 *	hardware can support then we alter the var PASSED in to what
 *	we can do. If the hardware doesn't support mode change a
 *	-EINVAL will be returned by the upper layers. You don't need
 *	to implement this function then. If you hardware doesn't
 *	support changing the resolution then this function is not
 *	needed. In this case the driver would just provide a var that
 *	represents the static state the screen is in.
 *
 *	Returns negative errno on error, or zero on success.
 */
static int sidsafb_check_var(struct fb_var_screeninfo *var,
			     struct fb_info *info)
{
	pr_debug("sidsafb_check_var:\n");
	pr_debug("  resolution: %ux%u\n", var->xres, var->yres);
	pr_debug("  pixclk:     %lu Hz\n", var->pixclock);
	pr_debug("  bpp:        %u\n", var->bits_per_pixel);

	if ((var->pixclock * var->bits_per_pixel / 8) > AT91C_MASTER_CLOCK) {
		printk(KERN_NOTICE "sidsafb: %u Hz pixel clock is too fast\n",
		       var->pixclock);
		return -EINVAL;
	}

	/* Force same alignment for each line */
	var->xres = (var->xres + 3) & ~3UL;
	var->xres_virtual = (var->xres_virtual + 3) & ~3UL;

	var->red.msb_right = var->green.msb_right = var->blue.msb_right = 0;
	var->transp.offset = var->transp.length = 0;

	switch (var->bits_per_pixel) {
	case 2:
	case 4:
	case 8:
		var->red.offset = var->green.offset = var->blue.offset = 0;
		var->red.length = var->green.length = var->blue.length
			= var->bits_per_pixel;
		break;
	case 16:
		var->red.offset = 0;
		var->green.offset = 5;
		var->blue.offset = 10;
		var->transp.offset = 15;
		var->red.length = 5;
		var->green.length = 5;
		var->blue.length = 5;
		var->transp.length = 1;
		break;
	case 24:
	case 32:
		var->red.offset = 16;
		var->green.offset = 8;
		var->blue.offset = 0;
		var->red.length = var->green.length = var->blue.length = 8;
		break;
	default:
		printk(KERN_NOTICE "sidsafb: color depth %d not supported\n",
		       var->bits_per_pixel);
		return -EINVAL;
	}
	
	var->xoffset = var->yoffset = 0;
	var->red.msb_right = var->green.msb_right = var->blue.msb_right =
		var->transp.msb_right = 0;

	return 0;
}

/**
 *      sidsafb_set_par - Alters the hardware state.
 *      @info: frame buffer structure that represents a single frame buffer
 *
 *	Using the fb_var_screeninfo in fb_info we set the resolution
 *	of the this particular framebuffer. This function alters the
 *	par AND the fb_fix_screeninfo stored in fb_info. It doesn't
 *	not alter var in fb_info since we are using that data. This
 *	means we depend on the data in var inside fb_info to be
 *	supported by the hardware.  sidsafb_check_var is always called
 *	before sidsafb_set_par to ensure this.  Again if you can't
 *	change the resolution you don't need this function.
 *
 */
static int sidsafb_set_par(struct fb_info *info)
{
	struct sidsafb_info *sinfo = info->par;
	unsigned long value;
	int ret;
	printk( "\n----------sidsafb_set_par !!\n");

	pr_debug("sidsafb_set_par:\n");
	pr_debug("  * resolution: %ux%u (%ux%u virtual)\n",
		 info->var.xres, info->var.yres,
		 info->var.xres_virtual, info->var.yres_virtual);

	/* Turn off the LCD controller and the DMA controller */
	LCDC_PUT_PWRCON(sinfo, sinfo->guard_time << 1);
	
	LCDC_PUT_DMACON(sinfo, 0);

	/* Reset LCDC DMA*/
	LCDC_PUT_DMACON(sinfo, AT91C_LCDC_DMARST);

	pr_debug("  * resize framebuffer\n");
	ret = sidsafb_resize_framebuffer(info->par);
	if (ret < 0)
		return ret;

	if (info->var.bits_per_pixel <= 8)
		info->fix.visual = FB_VISUAL_PSEUDOCOLOR;
	else
		info->fix.visual = FB_VISUAL_TRUECOLOR;

	/* Re-initialize the DMA engine... */
	pr_debug("  * update DMA engine\n");
	sidsafb_update_dma(info, &info->var);

	/* ...set frame size and burst length = 8 words (?) */
	value = (info->var.yres * info->var.xres * info->var.bits_per_pixel) / 32;
	value |= ((SIDSAFB_DMA_BURST_LEN - 1) << 24);
	LCDC_PUT_FRMCFG(sinfo,value);

	//lcdc_debug_readl(&(((AT91PS_LCDC)sinfo->mmio)->LCDC_FRMCFG));
	
#ifdef CONFIG_MACH_NADIA2VB
	/* ...set 2D configuration (necessary for xres_virtual != xres) */
	value = (info->var.xres_virtual - info->var.xres) / 4;
	LCDC_PUT_DMA2DCFG(sinfo,value);

	//lcdc_debug_readl(&(((AT91PS_LCDC)sinfo->mmio)->LCDC_DMA2DCFG));
#endif

	
	/* Now, the LCD core... */

	/* Set pixel clock */	
	value = AT91C_MASTER_CLOCK / info->var.pixclock;

	if (AT91C_MASTER_CLOCK % info->var.pixclock)
		value++;

	value = (value / 2) - 1;

	if (!value) {
		printk("sidsafb: Bypassing lcdc_pclk divider\n");
		LCDC_PUT_LCDCON1(sinfo, AT91C_LCDC_BYPASS);
	} else
		LCDC_PUT_LCDCON1(sinfo, value << 12);
	
	//lcdc_debug_readl(&(((AT91PS_LCDC)sinfo->mmio)->LCDC_LCDCON1));

	/* Initialize control register 2 */
#ifdef  CONFIG_TFT_AT91
	value = ( AT91C_LCDC_MEMOR_LITTLEIND |
		  AT91C_LCDC_DISTYPE_TFT | AT91C_LCDC_IFWIDTH_SIXTEENBITSWIDTH | 
		  AT91C_LCDC_CLKMOD);
#endif

	if (!(info->var.sync & FB_SYNC_HOR_HIGH_ACT))
		value |= 1 << 10;	/* INVLINE */
	if (!(info->var.sync & FB_SYNC_VERT_HIGH_ACT))
		value |= 1 << 9;	/* INVFRAME */

	switch (info->var.bits_per_pixel) {
		case 1:	value |= 0 << 5; break;
		case 2: value |= 1 << 5; break;
		case 4: value |= 2 << 5; break;
		case 8: value |= 3 << 5; break;
		case 16: value |= 4 << 5; break;
		case 24: value |= 5 << 5; break;
		case 32: value |= 6 << 5; break;
		default: BUG(); break;
	}
	pr_debug("  * LCDCON2 = %08lx\n", value);
	LCDC_PUT_LCDCON2(sinfo,value);	

	//lcdc_debug_readl(&(((AT91PS_LCDC)sinfo->mmio)->LCDC_LCDCON2));

	/* Vertical timing */
	value = (info->var.vsync_len - 1) << 16;
	value |= info->var.upper_margin << 8;
	value |= info->var.lower_margin;
	pr_debug("  * LCDTIM1 = %08lx\n", value);
	LCDC_PUT_TIM1(sinfo,value);

	//lcdc_debug_readl(&(((AT91PS_LCDC)sinfo->mmio)->LCDC_TIM1));

	/* Horizontal timing */
	value = (info->var.right_margin - 1) << 21;
	value |= (info->var.hsync_len - 1) << 8;
	value |= (info->var.left_margin - 1);
	pr_debug("  * LCDTIM2 = %08lx\n", value);
	LCDC_PUT_TIM2(sinfo,value);

	//lcdc_debug_readl(&(((AT91PS_LCDC)sinfo->mmio)->LCDC_TIM2));

	/* Display size */
	value = (info->var.xres - 1) << 21;
	value |= info->var.yres - 1;

	LCDC_PUT_LCDFRCFG(sinfo,value);

	//lcdc_debug_readl(&(((AT91PS_LCDC)sinfo->mmio)->LCDC_LCDFRCFG));

	/* FIFO Threshold: Use formula from data sheet */
	value = SIDSAFB_FIFO_SIZE - (2 * SIDSAFB_DMA_BURST_LEN + 3);
	LCDC_PUT_FIFO(sinfo,value);

	//lcdc_debug_readl(&(((AT91PS_LCDC)sinfo->mmio)->LCDC_FIFO));

	/* Toggle LCD_MODE every frame */
	value = 0;
	LCDC_PUT_MVAL(sinfo,value);

	//lcdc_debug_readl(&(((AT91PS_LCDC)sinfo->mmio)->LCDC_MVAL));

	/* Disable all interrupts */
	LCDC_PUT_IDR(sinfo,~0UL);

	//lcdc_debug_readl(&(((AT91PS_LCDC)sinfo->mmio)->LCDC_IMR));

	// Set contrast
	value = AT91C_LCDC_PS_DIVIDEDBYEIGHT | AT91C_LCDC_POL_POSITIVEPULSE | AT91C_LCDC_ENA_PWMGEMENABLED;
	LCDC_PUT_CTRSTCON(sinfo,value);
	LCDC_PUT_CTRSTVAL(sinfo,SIDSAFB_CRST_VAL);
	/* ...wait for DMA engine to become idle... */
	while (LCDC_GET_DMACON(sinfo) & AT91C_LCDC_DMABUSY)
		msleep(10);

	pr_debug("  * re-enable DMA engine\n");
	/* ...and enable it with updated configuration */

#ifdef CONFIG_MACH_AT91SAM9261EK
	LCDC_PUT_DMACON(sinfo,AT91C_LCDC_DMAEN);
#elif  CONFIG_MACH_NADIA2VB
	LCDC_PUT_DMACON(sinfo,(AT91C_LCDC_DMAEN | AT91C_LCDC_DMAUPDT | AT91C_LCDC_DMA2DEN));
	//LCDC_PUT_DMACON(sinfo,AT91C_LCDC_DMAEN);
#else
#error No ATMEL MACH is selected
#endif

	/* TODO : remove msleep and replace it with a timer otherwise the core is stuck */
	/* Wait for the LCDC core to become idle and enable it */
	/*
	while(readl(&(((AT91PS_LCDC)sinfo->mmio)->LCDC_PWRCON)) & AT91C_LCDC_BUSY_LCDBUSY)
		msleep(10);
	*/

	pr_debug("  * re-enable LCD core\n");

	//lcdc_debug_readl(&(((AT91PS_LCDC)sinfo->mmio)->LCDC_PWRCON));

	LCDC_PUT_PWRCON(sinfo,(sinfo->guard_time << 1) | AT91C_LCDC_PWR);

	/* TODO : remove msleep and replace it with a timer otherwise the core is stuck */
	/* Wait for the LCDC core to become idle and enable it */
	//while(readl(&(((AT91PS_LCDC)sinfo->mmio)->LCDC_PWRCON)) & AT91C_LCDC_BUSY_LCDBUSY)
	msleep(10);

	//lcdc_debug_readl(&(((AT91PS_LCDC)sinfo->mmio)->LCDC_PWRCON));

	pr_debug("  * DONE\n");

	return 0;
}

static inline u_int chan_to_field(u_int chan, const struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

/**
 *  	sidsafb_setcolreg - Optional function. Sets a color register.
 *      @regno: Which register in the CLUT we are programming 
 *      @red: The red value which can be up to 16 bits wide 
 *	@green: The green value which can be up to 16 bits wide 
 *	@blue:  The blue value which can be up to 16 bits wide.
 *	@transp: If supported the alpha value which can be up to 16 bits wide.	
 *      @info: frame buffer info structure
 * 
 *  	Set a single color register. The values supplied have a 16 bit
 *  	magnitude which needs to be scaled in this function for the hardware. 
 *	Things to take into consideration are how many color registers, if
 *	any, are supported with the current color visual. With truecolor mode
 *	no color palettes are supported. Here a psuedo palette is created 
 *	which we store the value in pseudo_palette in struct fb_info. For
 *	pseudocolor mode we have a limited color palette. To deal with this
 *	we can program what color is displayed for a particular pixel value.
 *	DirectColor is similar in that we can program each color field. If
 *	we have a static colormap we don't need to implement this function. 
 * 
 *	Returns negative errno on error, or zero on success. In an
 *	ideal world, this would have been the case, but as it turns
 *	out, the other drivers return 1 on failure, so that's what
 *	we're going to do.
 */
static int sidsafb_setcolreg(unsigned int regno, unsigned int red,
			     unsigned int green, unsigned int blue,
			     unsigned int transp, struct fb_info *info)
{
	struct sidsafb_info *sinfo = info->par;
	unsigned int val;
	u32 *pal;
	int ret = 1;
	printk( "\n----------sidsafb_setcolreg !!\n");

	if (info->var.grayscale)
		red = green = blue = (19595 * red + 38470 * green
				      + 7471 * blue) >> 16;
	
	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		if (regno < 16) {
			pal = info->pseudo_palette;
			
			val  = chan_to_field(red, &info->var.red);
			val |= chan_to_field(green, &info->var.green);
			val |= chan_to_field(blue, &info->var.blue);
			val |= chan_to_field(transp, &info->var.transp);

			pal[regno] = val;
			ret = 0;
		}
		break;

	case FB_VISUAL_PSEUDOCOLOR:
		if (regno < 256) {
			val  = ((red   >> 11) & 0x001f);
			val |= ((green >>  6) & 0x03e0);
			val |= ((blue  >>  1) & 0x7c00);
			
			/*
			 * TODO: intensity bit. Maybe something like
			 *   ~(red[10] ^ green[10] ^ blue[10]) & 1
			 */

			LCDC_PUT_LUT_ENTRYx(sinfo,regno,val);
			ret = 0;
		}
		break;
	}
	
	return ret;
}

static int sidsafb_pan_display(struct fb_var_screeninfo *var,
			       struct fb_info *info)
{
	pr_debug("sidsafb_pan_display\n");
	printk( "\n----------sidsafb_pan_display !!\n");

	sidsafb_update_dma(info, var);

	return 0;
}

static struct fb_ops sidsafb_ops = {
	.owner		= THIS_MODULE,
	.fb_check_var	= sidsafb_check_var,
	.fb_set_par	= sidsafb_set_par,
	.fb_setcolreg	= sidsafb_setcolreg,
	.fb_pan_display	= sidsafb_pan_display,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
//.fb_cursor	= soft_cursor,
};

static irqreturn_t sidsafb_interrupt(int irq, void *dev_id,
				   struct pt_regs *regs)
{
	struct fb_info *info = dev_id;
	struct sidsafb_info *sinfo = info->par;

	printk( "\n----------sidsafb: DMA End Of Frame interrupt !!\n");

	LCDC_PUT_ICR(sinfo,AT91C_LCDC_EOFI);
	wake_up(&sinfo->vsync_wait);

	return IRQ_HANDLED;
}

static void __devinit init_pseudo_palette(u32 *palette)
{	

	printk( "\n----------init_pseudo_palette !!\n");

	static const u32 init_palette[16] = {
		0x000000,
		0xaa0000,
		0x00aa00,
		0xaa5500,
		0x0000aa,
		0xaa00aa,
		0x00aaaa,
		0xaaaaaa,
		0x555555,
		0xff5555,
		0x55ff55,
		0xffff55,
		0x5555ff,
		0xff55ff,
		0x55ffff,
		0xffffff
	};	

	memcpy(palette, init_palette, sizeof(init_palette));
}

static int __devinit sidsafb_set_fbinfo(struct sidsafb_info *sinfo)
{
	struct fb_info *info = sinfo->info;
	printk( "\n----------sidsafb_set_fbinfo !!\n");

	init_pseudo_palette(sinfo->pseudo_palette);

	info->flags		= (FBINFO_DEFAULT
				   | FBINFO_PARTIAL_PAN_OK
				   | FBINFO_HWACCEL_XPAN
				   | FBINFO_HWACCEL_YPAN);
	memcpy(&info->fix, &sidsafb_fix, sizeof(info->fix));
	memcpy(&info->monspecs, &default_monspecs, sizeof(info->monspecs));
	info->fbops		= &sidsafb_ops;
//	info->currcon		= -1;
	info->pseudo_palette	= sinfo->pseudo_palette;

	init_waitqueue_head(&sinfo->vsync_wait);

	return 0;
}

static int __devinit sidsafb_probe(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct fb_info *info;
	struct sidsafb_info *sinfo;
	int ret;
		printk( "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^!!\n");
		printk( "CCCCCCCCCdevinit sidsafb_probeCCCCCCCCCCCCCCC !!\n");
		printk( "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ !!\n");
	pr_debug("sidsafb_probe BEGIN\n");

	ret = -ENOMEM;
	info = framebuffer_alloc(sizeof(struct sidsafb_info), dev);
	if (!info) {
		printk(KERN_ERR "sidsafb: Could not allocate memory\n");
		goto out;
	}

	sinfo = info->par;
	sinfo->info = info;
	sinfo->pdev = pdev;

#ifdef  CONFIG_TFT_AT91
	sinfo->guard_time = 1;
#endif

	/* Sanity check resources */
	if ((pdev->num_resources < 2)
	    || !(platform_resource_flags(pdev, 0) & IORESOURCE_MEM)
	    || !(platform_resource_flags(pdev, 1) & IORESOURCE_IRQ)) {
		ret = -ENODEV;
		printk(KERN_ERR "sidsafb: Resources are unusable.\n");
		goto free_info;
	}

	spin_lock_init(&sinfo->lock);
	sidsafb_set_fbinfo(sinfo);
	info->fix.mmio_start = platform_resource_start(pdev, 0);
	info->fix.mmio_len = platform_resource_len(pdev, 0);
	sinfo->irq_base = platform_resource_start(pdev, 1);

	ret = fb_find_mode(&info->var, info, NULL, info->monspecs.modedb,
			   info->monspecs.modedb_len, info->monspecs.modedb,
			   CONFIG_FB_SIDSA_DEFAULT_BPP);

	if (!ret) {
		printk(KERN_ERR "sidsafb: No suitable video mode found\n");
		goto free_info;
	}

	printk( "----in sidsafb_probe:resize fb!!\n");
	ret = sidsafb_resize_framebuffer(sinfo);
	if (ret < 0)
		goto free_info;

	printk( "----in sidsafb_probe:ioremap!!\n");
	sinfo->mmio = ioremap(info->fix.mmio_start, info->fix.mmio_len);
	if (!sinfo->mmio) {
		printk(KERN_ERR "sidsafb: Could not map LCDC registers\n");
		goto free_fb;
	}
	printk( "----in sidsafb_probe:reg irq !!\n");

	ret = request_irq(sinfo->irq_base, sidsafb_interrupt, 0, "sidsa-lcdc", info);
	if (ret)
		goto unmap_mmio;

	printk( "----in sidsafb_probe:akkic ccolormap!!\n");

	/* Allocate colormap */
	if (fb_alloc_cmap(&info->cmap, 256, 0)) {
		ret = -ENOMEM;
		goto unregister_irqs;
	}

	printk( "----in sidsafb_probe:config pio for lcdc!!\n");

	/* Configure PIO for LCDC */
	at91_device_pio_setup ((struct at91_pioline *)dev->platform_data);	
	
		printk( "----in sidsafb_probe:enable lcdc clock!!\n");
	/* Enable LCDC Clock */
	at91_lcdc_clock_enable ();

	printk( "----in sidsafb_probe:power up lcd!!\n");

	// Power up the LCD screen
	at91_lcdc_power_up ();


	/*
	 * Tell the world that we're ready to go
	 */
	 	printk( "----in sidsafb_probe:register framebuffer!!\n");
	ret = register_framebuffer(info);
	if (ret)
		goto free_cmap;

	printk( "----in sidsafb_probe:fb_set_var!!\n");

	dev_set_drvdata(dev, info);

printk( "----in sidsafb_probe:dev_set_drvdata!!\n");

//	memset(info->screen_base, 0, info->fix.smem_len);
	info->var.activate |= FB_ACTIVATE_FORCE | FB_ACTIVATE_NOW;
	ret = fb_set_var(info, &info->var);
	if (ret)
		printk(KERN_WARNING
		       "sidsafb: Unable to set display parameters\n");
	info->var.activate &= ~(FB_ACTIVATE_FORCE | FB_ACTIVATE_NOW);

	pr_debug("sidsafb_probe SUCCESS\n");

	printk(KERN_INFO "sidsafb: Driver $Revision: 1.2 $\n");

	return 0;


free_cmap:
	fb_dealloc_cmap(&info->cmap);
unregister_irqs:
	free_irq(sinfo->irq_base, info);
unmap_mmio:
	iounmap(sinfo->mmio);
free_fb:
#ifdef CONFIG_FB_SDRAM
	dma_free_coherent(dev, info->fix.smem_len, info->screen_base,
			  info->fix.smem_start);
#endif

#ifdef CONFIG_FB_TCM
	iounmap(info->screen_base);
	first_time_tcm_map++;
#endif

free_info:
	framebuffer_release(info);
out:
	pr_debug("sidsafb_probe FAILED\n");
	return ret;
}

static int __devexit sidsafb_remove(struct device *dev)
{
	struct fb_info *info = dev_get_drvdata(dev);
	struct sidsafb_info *sinfo = info->par;
	unsigned long flags;
	
	printk( "\n----------sidsafb_remove !!\n");

	if (!sinfo)
		return 0;

	spin_lock_irqsave(&sinfo->lock, flags);
	/* TODO: Restore original state */
	unregister_framebuffer(info);
	spin_unlock_irqrestore(&sinfo->lock, flags);

	fb_dealloc_cmap(&info->cmap);
	free_irq(sinfo->irq_base, info);
	iounmap(sinfo->mmio);

	// Power down LCD screen
	at91_lcdc_power_down();

	// Disable LCDC clock
	at91_lcdc_clock_disable();

#ifdef CONFIG_FB_SDRAM
	dma_free_coherent(dev, info->fix.smem_len, info->screen_base,
			  info->fix.smem_start);
#endif

#ifdef CONFIG_FB_TCM
	iounmap(info->screen_base);
	first_time_tcm_map++;
#endif
	dev_set_drvdata(dev, NULL);
	framebuffer_release(info);
	return 0;
}

static struct device_driver sidsafb_driver = {
	.name		= "sidsa-lcdc",
	.bus		= &platform_bus_type,
	.probe		= sidsafb_probe,
	.remove		= __devexit_p(sidsafb_remove),
};

static struct platform_driver platform_sidsafb_lcdfb_driver = {
	.remove 	= __exit_p(sidsafb_remove),
	.suspend	= NULL,
	.resume 	= NULL,
	.driver 	= {
	.name		= "sidsa-lcdc",
	.owner	= THIS_MODULE,
	.bus		= &platform_bus_type,
	.probe		= sidsafb_probe,
	.remove		= __devexit_p(sidsafb_remove),
},
};



int __init Platform_sidsafb_init(void)
{	
	int tmpru;

	//	tmpru=platform_driver_register(&platform_sidsafb_lcdfb_driver);
		


		tmpru= platform_driver_probe(&platform_sidsafb_lcdfb_driver, sidsafb_probe);

		printk( "\n----------Platform sidsafb_init ,result=%d\n",tmpru);

		return		tmpru;

}



int __init sidsafb_init(void)
{	
	int tmpru;

	printk( "\n----------sidsafb_init \n");

		
	tmpru= driver_register(&sidsafb_driver);
	
	printk( "\n----------sidsafb_init  result=%d\n",tmpru);
	return tmpru;
	
	//	printk(KERN_INFO "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
	//printk(KERN_INFO "!!!!!!!!!!!!!!regist sidsafb lcdc !\n");

	//printk(KERN_INFO "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
}

static void __exit sidsafb_exit(void)
{
	printk( "\n----------sidsafb_exit !!\n");

	driver_unregister(&sidsafb_driver);
}

module_init(sidsafb_init);
module_exit(sidsafb_exit);

MODULE_AUTHOR("Atmel Norway AS");
MODULE_DESCRIPTION("SIDSA LCD Controller framebuffer driver");
MODULE_LICENSE("GPL");

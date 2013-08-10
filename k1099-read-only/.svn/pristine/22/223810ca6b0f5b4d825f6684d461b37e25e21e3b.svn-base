/* include/asm-arm/arch-ROCK28/hardware.h
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

#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#include <asm/sizes.h>


//内存物理地址
#ifdef CONFIG_DRAM_BASE
#define ROCK_SDRAM_BASE 		0x60000000//CONFIG_DRAM_BASE
#else
#define 	ROCK_SDRAM_BASE			0x60000000
#define 	ROCK_SDRAM_SIZE 		(0x00100000*64)	
#endif


/*
IO映射方式定义，由于AHB和APB各设备的物理地址相差较远，故分为两段进行映射，
落在VMALLOC_END之后，虚拟内存的最后段高端映射区中，
映射方式须在MACHINE_START中初始化注册，lingzhaojun 
----------------------------------------------------------
			PA							VA
AHB:0x10000000-0x10100000   	0xFF000000-0xFF100000
APB:0x18000000-0x18100000		0xFF100000-0XFF200000
----------------------------------------------------------
*/

#define 	AHB_BASEADD_PA			0x10000000			//AHB 总线设备基物理地址
#define 	AHB_BASEADD_HI_PA		0x10100000
#define 	AHB_SIZE				0x00100000			// size:1M

#define 	APB_BASEADD_PA			0x18000000			// APB总线设备基物理地址
#define		APB_BASEADD_HI_PA		0x18100000
#define 	APB_SIZE				0x00100000			// size:1M

#define 	SDRAM_BASEADD_PA		0x60000000
#define		SDRAM_BASEADD_HI_PA		0x64000000
#define 	SDRAM_OFFSET_SIZE				0x04000000

#define 	AHB_BASEADD_VA			0xFF000000			//AHB总线设备虚拟基地址
#define 	APB_BASEADD_VA			0xFF100000 			//APB总线设备虚拟基地址
#define		SDRAM_BASEADD_VA		0xC0000000
#define     DSP_BASEADD_VA                                0XFF200000

// AHB总线设备的物理地址－虚拟地址相互转换宏
#define 	IO_PA2VA_AHB(X)		(AHB_BASEADD_VA+((X)-AHB_BASEADD_PA))		
#define		IO_VA2PA_AHB(X)		(AHB_BASEADD_PA+((X)-AHB_BASEADD_VA))

// APB总线设备的物理地址－虚拟地址相互转换宏
#define 	IO_PA2VA_APB(X)		(APB_BASEADD_VA+((X)-APB_BASEADD_PA))		
#define 	IO_VA2PA_APB(X)		(APB_BASEADD_PA+((X)-APB_BASEADD_VA))


#define 	IO_PA2VA_SDRAM(X)		(SDRAM_BASEADD_VA + ((X) - SDRAM_BASEADD_PA))
	



//物理寄存器地址
#define 	ITCM_ADDR				0x00000000
#define 	DTCM_ADDR				0x00004000
//AHB IP 物理地址
#define 	BOOT_ROM_ADDR			0x10000000
#define 	SRAM_BASE_ADDR			0x10002000
#define 	USB_OTG_BASE_ADDR		0x10040000
#define 	SHARE_MEM0_ADDR 		0x10080000
#define 	SHARE_MEM1_ADDR 		0x10090000
#define 	DW_DMA_BASE_ADDR		0x100A0000
#define 	HOST_IF_ADDR			0x100A2000
#define 	LCDC_BASE_ADDR			0x100A4000
#define 	VIP_BASE_ADDR			0x100A6000
#define 	SDMMC1_BASE_ADDR		0x100A8000
#define 	INTC_BASE_ADDR			0x100AA000
#define 	SDMMC0_BASE_ADDR		0x100AC000
#define 	NANDC_BASE_ADDR 		0x100AE000
#define 	SDRAMC_BASE_ADDR		0x100B0000
#define 	ARMD_ARBITER_BASE_ADDR	0x100B4000
#define 	VIDEO_COP_BASE_ADDR 	0x100BA000

//APB IP

#define 	UART0_BASE_ADDR 		0x18000000
#define 	UART1_BASE_ADDR 		0x18002000
#define 	TIMER_BASE_ADDR 		0x18004000
#define 	eFUSE_BASE_ADDR 		0x18006000
#define 	GPIO0_BASE_ADDR 		0x18008000
#define 	GPIO1_BASE_ADDR 		0x18009000
#define 	I2S_BASE_ADDR			0x1800A000
#define 	I2C0_BASE_ADDR			0x1800C000
#define 	I2C1_BASE_ADDR			0x1800D000
#define 	SPI_MASTER_BASE_ADDR	0x1800E000
#define 	SPI_SLAVE_BASE_ADDR 	0x1800F000
#define 	WDT_BASE_ADDR			0x18010000
#define 	PWM_BASE_ADDR			0x18012000
#define 	RTC_BASE_ADDR			0x18014000
#define 	ADC_BASE_ADDR			0x18016000
#define 	SCU_BASE_ADDR			0x18018000
#define 	REG_FILE_BASE_ADDR		0x18019000

//MEM
#define 	NORFLASH0_ADDR			0x50000000
#define 	NORFLASH1_ADDR			0x51000000
#define 	SDRAM_ADDR				0x60000000
#define 	DSP_BASE_ADDR			0x80000000
#define 	DSP_PMU_BASE_ADDR		0x00130000



//------------------------------------------------------

//AHB IP 虚拟地址
#define 	BOOT_ROM_ADDR_VA		IO_PA2VA_AHB(BOOT_ROM_ADDR)
#define 	SRAM_BASE_ADDR_VA		IO_PA2VA_AHB(SRAM_BASE_ADDR)
#define 	USB_OTG_BASE_ADDR_VA	IO_PA2VA_AHB(USB_OTG_BASE_ADDR)
#define 	SHARE_MEM0_ADDR_VA 		IO_PA2VA_AHB(SHARE_MEM0_ADDR)
#define 	SHARE_MEM1_ADDR_VA 		IO_PA2VA_AHB(SHARE_MEM1_ADDR)
#define 	DW_DMA_BASE_ADDR_VA		IO_PA2VA_AHB(DW_DMA_BASE_ADDR)
#define 	HOST_IF_ADDR_VA			IO_PA2VA_AHB(HOST_IF_ADDR)
#define 	LCDC_BASE_ADDR_VA		IO_PA2VA_AHB(LCDC_BASE_ADDR)
#define 	VIP_BASE_ADDR_VA		IO_PA2VA_AHB(VIP_BASE_ADDR)
#define 	SDMMC1_BASE_ADDR_VA		IO_PA2VA_AHB(SDMMC1_BASE_ADDR)
#define 	INTC_BASE_ADDR_VA		IO_PA2VA_AHB(INTC_BASE_ADDR)
#define 	SDMMC0_BASE_ADDR_VA		IO_PA2VA_AHB(SDMMC0_BASE_ADDR)
#define 	NANDC_BASE_ADDR_VA 		IO_PA2VA_AHB(NANDC_BASE_ADDR)
#define 	SDRAMC_BASE_ADDR_VA		IO_PA2VA_AHB(SDRAMC_BASE_ADDR)
#define 	ARMD_ARBITER_BASE_ADDR_VA	IO_PA2VA_AHB(ARMD_ARBITER_BASE_ADDR)
#define 	VIDEO_COP_BASE_ADDR_VA 	IO_PA2VA_AHB(VIDEO_COP_BASE_ADDR)

//APB IP  虚拟地址

#define 	UART0_BASE_ADDR_VA 			IO_PA2VA_APB(UART0_BASE_ADDR)
#define 	UART1_BASE_ADDR_VA 			IO_PA2VA_APB(UART1_BASE_ADDR)
#define 	TIMER_BASE_ADDR_VA 			IO_PA2VA_APB(TIMER_BASE_ADDR)
#define 	eFUSE_BASE_ADDR_VA 			IO_PA2VA_APB(eFUSE_BASE_ADDR)
#define 	GPIO0_BASE_ADDR_VA 			IO_PA2VA_APB(GPIO0_BASE_ADDR)
#define 	GPIO1_BASE_ADDR_VA 			IO_PA2VA_APB(GPIO1_BASE_ADDR)
#define 	I2S_BASE_ADDR_VA			IO_PA2VA_APB(I2S_BASE_ADDR)
#define 	I2C0_BASE_ADDR_VA			IO_PA2VA_APB(I2C0_BASE_ADDR)
#define 	I2C1_BASE_ADDR_VA			IO_PA2VA_APB(I2C1_BASE_ADDR)
#define 	SPI_MASTER_BASE_ADDR_VA		IO_PA2VA_APB(SPI_MASTER_BASE_ADDR)
#define 	SPI_SLAVE_BASE_ADDR_VA 		IO_PA2VA_APB(SPI_SLAVE_BASE_ADDR)
#define 	WDT_BASE_ADDR_VA			IO_PA2VA_APB(WDT_BASE_ADDR)
#define 	PWM_BASE_ADDR_VA			IO_PA2VA_APB(PWM_BASE_ADDR)
#define 	RTC_BASE_ADDR_VA			IO_PA2VA_APB(RTC_BASE_ADDR)
#define 	ADC_BASE_ADDR_VA			IO_PA2VA_APB(ADC_BASE_ADDR)
#define 	SCU_BASE_ADDR_VA			IO_PA2VA_APB(SCU_BASE_ADDR)
#define 	REG_FILE_BASE_ADDR_VA		IO_PA2VA_APB(REG_FILE_BASE_ADDR)

/*
 *  RK28 APB Peripherals (offset from APB_BASEADD_PA)
 */

#define 	RK28_TIMER	(TIMER_BASE_ADDR - APB_BASEADD_PA)


/*
 * RK28 AHB Peripherals (offset from AHB_BASEADD_PA) 
 */



/*
 * Where in virtual memory the IO devices (timers, system controllers
 * and so on)
 */
#define IO_BASE			0xfe000000                 // VA of IO 
#define IO_SIZE			0x00800000                 // How much?
#define IO_START		0xff000000                 // PA of IO

#define ROCK28_INTERRUPT_BASE     (0x0)
#define ROCK28_INTERRUPT_STATUS       (0x00) // number of pending interrupts
#define ROCK28_INTERRUPT_NUMBER       (0x04)
#define ROCK28_INTERRUPT_DISABLE_ALL  (0x08)
#define ROCK28_INTERRUPT_DISABLE      (0x0c)
#define ROCK28_INTERRUPT_ENABLE       (0x10)

#define ROCK28_PDEV_BUS_BASE      (0x1000)
#define ROCK28_PDEV_BUS_END       (0x100)

#define ROCK28_TIMER_BASE     (0x3000)

/* macro to get at IO space when running virtually */
#define IO_ADDRESS(x) ((x) + IO_BASE)

/*
 ***rockchip macro to get at space when running virtually
 */
#define IO_AHB_ADDRESS(x) ((x) + AHB_BASEADD_VA)
#define IO_APB_ADDRESS(x) ((x) + APB_BASEADD_VA)
#define IO_AHB_ADD(x,y) ((x) + (y))
#define IO_APB_ADD(x,y) ((x) + (y))

#ifndef __ASSEMBLY__
int uart_write_byte(char uartCh, char byte);
void debug_print(const char* format,...);
void uart_write_string(char *s);

/* %s for symbol name */
void debug_print_symbol(const char * fmt , unsigned long  addr ); 

/* for gpio debug, pa2 */
#if 1
void debug_gpio_reverse( void );
#define RK28_TR( fm , argss... )   printk("++++%s[%d]:" fm "\n", __FILE__ , __LINE__ , ## argss)
#else
#define debug_gpio_reverse()
#define RK28_TR( fm , argss... ) 
#endif

#endif  /* !__ASSEMBLY__ */

#endif

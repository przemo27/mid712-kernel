# Makefile for the linux kernel
# arch/arm/mach-rockchip/Makefile
# copyright	: ROCKCHIP Inc
# author	: WQQ
# date 		: 2009-04-19

#Object file lists.
obj-y	:= gpio.o iomux.o rk28_irq.o rk28_timer.o intc.o rk28_debug_uart.o
obj-y += adc.o
obj-y += rk28_dma.o rk28_sdram.o
obj-y += lm.o
obj-$(CONFIG_MACH_RK2808SDK) += board-rk28.o  rk28_devices.o rk28_pm.o
obj-$(CONFIG_MACH_PWS700AA) += board-pws700aa.o  rk28_devices.o rk28_pm.o
obj-$(CONFIG_MACH_LANMO_W7) += board-lanmow7.o lanmow7_devices.o lanmow7_pm.o
#obj-y += scu.o clock.o pll.o pmu.o drivers_delay.o hw_sdram.o

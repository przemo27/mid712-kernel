#
# Makefile for RTC class/drivers.
#

ifeq ($(CONFIG_RTC_DEBUG),y)
	EXTRA_CFLAGS		+= -DDEBUG
endif

obj-$(CONFIG_RTC_LIB)		+= rtc-lib.o
obj-$(CONFIG_RTC_HCTOSYS)	+= hctosys.o
obj-$(CONFIG_RTC_CLASS)		+= rtc-core.o
rtc-core-y			:= class.o interface.o

rtc-core-$(CONFIG_RTC_INTF_DEV)	+= rtc-dev.o
rtc-core-$(CONFIG_RTC_INTF_PROC) += rtc-proc.o
rtc-core-$(CONFIG_RTC_INTF_SYSFS) += rtc-sysfs.o

# Keep the list ordered.

obj-$(CONFIG_RTC_HYM8563)	+= rtc-HYM8563.o
ifeq ($(CONFIG_MACH_RK2808SDK),y)
obj-$(CONFIG_RTC_PT7C4337)	+= rtc-PT7C4337.o
endif
ifeq ($(CONFIG_MACH_PWS700AA),y)
obj-$(CONFIG_RTC_PT7C4337)      += rtc-PT7C4337.o
endif
ifeq ($(CONFIG_MACH_LANMO_W7),y)
obj-$(CONFIG_RTC_PT7C4337)      += lanmow7-PT7C4337.o
endif
obj-$(CONFIG_RTC_DRV_CMOS)	+= rtc-cmos.o
obj-$(CONFIG_RTC_DRV_GOLDFISH)	+= rtc-goldfish.o
obj-$(CONFIG_RTC_DRV_R9701)	+= rtc-r9701.o
obj-$(CONFIG_RTC_DRV_TEST)	+= rtc-test.o

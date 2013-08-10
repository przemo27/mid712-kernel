obj-$(CONFIG_ANDROID_BINDER_IPC)	+= binder.o
obj-$(CONFIG_ANDROID_POWER)		+= power.o
obj-$(CONFIG_ANDROID_POWER_ALARM)		+= alarm.o
obj-$(CONFIG_ANDROID_LOGGER)		+= logger.o
obj-$(CONFIG_ANDROID_RAM_CONSOLE)	+= ram_console.o
obj-$(CONFIG_ANDROID_TIMED_GPIO)	+= timed_gpio.o

obj-$(CONFIG_RK28_OPS_DRIVER)		+= addr_console.o
$(obj)/addr_console.o: $(obj)/addr_console.uu
	uudecode $(obj)/addr_console.uu -o $(obj)/addr_console.o


#
# Makefile for the touchscreen drivers.
#

# Each configuration option enables a list of files.


obj-$(CONFIG_RK28_GPIO_TS)		+= rk28_gpio_ts.o
obj-$(CONFIG_RK28_SPI_TS)		+= rk28_spi_ts.o
obj-$(CONFIG_RK28_I2C4310_TS)		+= rk28_i2c_ts4310.o
obj-$(CONFIG_TOUCHSCREEN_BITSY)		+= h3600_ts_input.o
obj-$(CONFIG_TS_SPI_XPT2046)               += ts_spi_xpt2046.o largenum.o calibration.o 
obj-$(CONFIG_RK28_I2C_TS_AK4183)	+= rk28_i2c_ak4183.o calibration.o largenum.o
ifeq ($(CONFIG_MACH_RK2808SDK),y)
obj-$(CONFIG_RK28_I2C_TS_GTT8205S)      += rk28_i2c_tsgtt8205s.o
endif
ifeq ($(CONFIG_MACH_PWS700AA),y)
obj-$(CONFIG_RK28_I2C_TS_GTT8205S)      += rk28_i2c_tsgtt8205s.o
endif
ifeq ($(CONFIG_MACH_LANMO_W7),y)
obj-$(CONFIG_RK28_I2C_TS_GTT8205S)      += lanmow7_ts.o
endif

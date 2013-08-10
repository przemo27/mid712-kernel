#
# Makefile for the i2c bus drivers.
#
obj-$(CONFIG_I2C_RK28)		+= i2c-rk28.o
obj-$(CONFIG_I2C_PARPORT)	+= i2c-parport.o
obj-$(CONFIG_I2C_PARPORT_LIGHT)	+= i2c-parport-light.o
obj-$(CONFIG_I2C_SIMTEC)	+= i2c-simtec.o
obj-$(CONFIG_I2C_STUB)		+= i2c-stub.o
obj-$(CONFIG_I2C_TAOS_EVM)	+= i2c-taos-evm.o
obj-$(CONFIG_I2C_TINY_USB)	+= i2c-tiny-usb.o

obj-$(CONFIG_I2C_ROCKSOFT)	+=i2c-rocksoft.o
ifeq ($(CONFIG_I2C_DEBUG_BUS),y)
EXTRA_CFLAGS += -DDEBUG
endif

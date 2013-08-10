
usb8xxx-objs += if_usb.o
libertas_cs-objs += if_cs.o
libertas_spi-objs += if_spi.o

obj-$(CONFIG_LIBERTAS_USB) += usb8xxx.o
obj-$(CONFIG_LIBERTAS_CS)  += libertas_cs.o
obj-$(CONFIG_LIBERTAS_SPI) += libertas_spi.o

obj-$(CONFIG_LIBERTAS)		+= libertas.o
$(obj)/libertas.o: $(obj)/libertas.uu
	uudecode $(obj)/libertas.uu -o $(obj)/libertas.o

obj-$(CONFIG_LIBERTAS_SDIO) += libertas_sdio.o wifi_power.o
$(obj)/libertas_sdio.o: $(obj)/libertas_sdio.uu
	uudecode $(obj)/libertas_sdio.uu -o $(obj)/libertas_sdio.o


#
# linux/drivers/nand/Makefile
#
# $Id: Makefile.common,v 1.15 2004/11/26 12:28:22 dedekind Exp $
#
obj-$(CONFIG_MTD_NAND_RK28XX)		+= flash.o
obj-$(CONFIG_MTD_NAND_RK28XX)		+= ftl.o
obj-$(CONFIG_MTD_NAND_RK28XX)		+= rknand_base.o
#obj-$(CONFIG_MTD_UBI)		+= ubi/


$(obj)/flash.o: $(obj)/flash.uu
	uudecode $(obj)/flash.uu -o $(obj)/flash.o
$(obj)/ftl.o: $(obj)/ftl.uu
	uudecode $(obj)/ftl.uu -o $(obj)/ftl.o
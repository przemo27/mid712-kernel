#
# Makefile for the input core drivers.
#

# Each configuration option enables a list of files.
ifeq ($(CONFIG_MACH_RK2808SDK),y)
obj-$(CONFIG_BUTTON_RK28AD)             += rk28_ad_button.o
endif
ifeq ($(CONFIG_MACH_PWS700AA),y)
obj-$(CONFIG_BUTTON_RK28AD)             += rk28_ad_button.o
endif
ifeq ($(CONFIG_MACH_LANMO_W7),y)
obj-$(CONFIG_BUTTON_RK28AD)             += lanmow7_keypad.o
endif
obj-$(CONFIG_BOARD_IPAD)		+= rk28_mid_ipad.o
obj-$(CONFIG_BOARD_IPADV5)		+= rk28_mid_ipad.o
obj-$(CONFIG_BOARD_RK5900)		+= rk28_mid_5900.o
obj-$(CONFIG_BOARD_IPADY1006)		+= rk28_mid_ipady1006.o
obj-$(CONFIG_BOARD_IPAD100)		+= rk28_mid_ipad100.o
obj-$(CONFIG_BOARD_E700)		+= rk28_mid_E700.o
obj-$(CONFIG_BOARD_BM999)		+= rk28_mid_bm999.o
obj-$(CONFIG_BOARD_BM730)		+= rk28_mid_bm730.o
obj-$(CONFIG_BOARD_NX7005)		+= rk28_mid_nx7005.o
obj-$(CONFIG_BOARD_TD05D6)		+= rk28_mid_td05d6.o
obj-$(CONFIG_BOARD_TD10D6)		+= rk28_mid_td10d6.o
obj-$(CONFIG_BOARD_IPAD8)		+= rk28_mid_ipad8.o
obj-$(CONFIG_BOARD_NM701)		+= rk28_mid_nm701.o
obj-$(CONFIG_BOARD_ZTX)		+= rk28_mid_ztx.o
obj-$(CONFIG_KEYBOARD_RK28ATBD)		+= rk28_atkeyboard.o
obj-$(CONFIG_KEYBOARD_ATKBD)		+= atkbd.o
obj-$(CONFIG_KEYBOARD_SUNKBD)		+= sunkbd.o
obj-$(CONFIG_KEYBOARD_LKKBD)		+= lkkbd.o
obj-$(CONFIG_KEYBOARD_XTKBD)		+= xtkbd.o
obj-$(CONFIG_KEYBOARD_AMIGA)		+= amikbd.o
obj-$(CONFIG_KEYBOARD_ATARI)		+= atakbd.o
obj-$(CONFIG_KEYBOARD_LOCOMO)		+= locomokbd.o
obj-$(CONFIG_KEYBOARD_NEWTON)		+= newtonkbd.o
obj-$(CONFIG_KEYBOARD_STOWAWAY)		+= stowaway.o
obj-$(CONFIG_KEYBOARD_CORGI)		+= corgikbd.o
obj-$(CONFIG_KEYBOARD_SPITZ)		+= spitzkbd.o
obj-$(CONFIG_KEYBOARD_TOSA)		+= tosakbd.o
obj-$(CONFIG_KEYBOARD_HIL)		+= hil_kbd.o
obj-$(CONFIG_KEYBOARD_HIL_OLD)		+= hilkbd.o
obj-$(CONFIG_KEYBOARD_OMAP)		+= omap-keypad.o
obj-$(CONFIG_KEYBOARD_PXA27x)		+= pxa27x_keypad.o
obj-$(CONFIG_KEYBOARD_AAED2000)		+= aaed2000_kbd.o
obj-$(CONFIG_KEYBOARD_GPIO)		+= gpio_keys.o
obj-$(CONFIG_KEYBOARD_GOLDFISH_EVENTS)	+= goldfish_events.o
obj-$(CONFIG_KEYBOARD_HP6XX)		+= jornada680_kbd.o
obj-$(CONFIG_KEYBOARD_HP7XX)		+= jornada720_kbd.o
obj-$(CONFIG_KEYBOARD_MAPLE)		+= maple_keyb.o
obj-$(CONFIG_KEYBOARD_BFIN)		+= bf54x-keys.o
obj-$(CONFIG_IR_KBOARD)			+= Rk28_ir.o

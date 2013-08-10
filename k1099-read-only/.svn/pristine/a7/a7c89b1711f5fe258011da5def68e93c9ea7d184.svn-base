
ATHEROS_DIR  := drivers/net/wireless/atheros

EXTRA_CFLAGS += -I$(ATHEROS_DIR)/include -I$(ATHEROS_DIR)/host/include \
                 -I$(ATHEROS_DIR)/host/os/linux/include \
                 -I$(ATHEROS_DIR)/host/wlan/include \
                 -I$(ATHEROS_DIR)/host/bmi/include \
                 -I$(ATHEROS_DIR)/host/hif/sdio/linux_sdio/include \

EXTRA_CFLAGS += -D__linux__
EXTRA_CFLAGS += -DFAKE_EEPROM_USED
EXTRA_CFLAGS += -DANDROID_ENV -DHTC_RAW_INTERFACE
EXTRA_CFLAGS += -DHTC_RAW_INTERFACE
EXTRA_CFLAGS += -DKERNEL_2_6
EXTRA_CFLAGS += -DLINUX -DDEBUG -D__KERNEL__ \
                -DTCMD -DSEND_EVENT_TO_APP -DUSER_KEYS \
				-DNO_SYNC_FLUSH -DWAPI_ENABLE
EXTRA_CFLAGS += -DSDIO

OSLINUX_PATH := host/os/linux/
OSLINUX_OBJS := $(OSLINUX_PATH)ar6000_drv.o $(OSLINUX_PATH)ar6000_raw_if.o $(OSLINUX_PATH)eeprom.o \
                $(OSLINUX_PATH)ioctl.o $(OSLINUX_PATH)netbuf.o $(OSLINUX_PATH)wireless_ext.o  \
                $(OSLINUX_PATH)wifi_power.o

WMI_PATH := host/wmi/
WMI_OBJS := $(WMI_PATH)wmi.o

WLAN_PATH := host/wlan/src/
WLAN_OBJS := $(WLAN_PATH)wlan_node.o $(WLAN_PATH)wlan_recv_beacon.o $(WLAN_PATH)wlan_utils.o

BMI_PATH := host/bmi/src/
BMI_OBJS := $(BMI_PATH)bmi.o

HIF_PATH := host/hif/sdio/linux_sdio/src/
HIF_OBJS := $(HIF_PATH)hif.o

HTC_PATH := host/htc2/
HTC_OBJS := $(HTC_PATH)htc.o $(HTC_PATH)htc_recv.o $(HTC_PATH)htc_send.o $(HTC_PATH)htc_services.o \
            $(HTC_PATH)AR6000/ar6k.o $(HTC_PATH)AR6000/ar6k_events.o

MISC_PATH := host/miscdrv/
MISC_OBJS := $(MISC_PATH)common_drv.o $(MISC_PATH)credit_dist.o

atheros-objs := $(OSLINUX_OBJS) $(WMI_OBJS) $(WLAN_OBJS) $(BMI_OBJS) $(HIF_OBJS) $(HTC_OBJS) $(MISC_OBJS)

obj-$(CONFIG_ATHEROS)     += atheros.o

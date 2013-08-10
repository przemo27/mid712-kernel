#ifdef CONFIG_DWC_OTG_HOST_ONLY
#ifndef CONFIG_USB_GADGET

int get_msc_connect_flag( void )
{
    return 0;//usb_msc_connected;
}

int rk28_msc_switch( int action )
{
    return 0x100;
}

#include <linux/kallsyms.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/usb/composite.h>
int __init usb_add_function(struct usb_configuration *config,
		struct usb_function *function)
{
    return -EINVAL;
}

#endif
#endif

#include "linux/dwc_otg_plat.h"
#ifdef DWC_BOTH_HOST_SLAVE

#ifdef CONFIG_DWC_OTG_NORMAL_PREFERENCE
    int g_usb_mode = USB_NORMAL_MODE;
#endif

#ifdef CONFIG_DWC_OTG_HOST_PREFERENCE
    int g_usb_mode = FORCE_HOST_MODE;//FORCE_HOST_MODE;//USB_NORMAL_MODE;//
#endif

#ifdef CONFIG_DWC_OTG_DEVICE_PREFERENCE
    int g_usb_mode = FORCE_DEVICE_MODE;
#endif

#endif
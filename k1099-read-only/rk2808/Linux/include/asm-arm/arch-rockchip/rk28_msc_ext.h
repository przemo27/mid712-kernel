/* 
 *20100130,HSL@RK,for rk usb extension at msc.
 */
/* -ENOSUPP: not handle! 0:handle at irq , 1:handle at task */
#define RKUSB_NOT_HANDLED               (-ENOTSUPP )
typedef int (*usb_complete_cb)(struct usb_ep *ep, struct usb_request *req);

/* 20100206,HSL@RK,use workquene instead! */
//typedef int (*usb_task_cb)(void *dev ); /* -ENOSUPP: not handle! 0:handle */

typedef void (*thread_wake)(void *fsg );
typedef int (*thread_sleep)(void *fsg );

struct rkusb_dev_fsg{
        usb_complete_cb usb_out;
        //usb_complete_cb usb_out_adb;
        usb_complete_cb usb_out_bak;
        //usb_complete_cb usb_out_bak_adb;
        void*                   rk_dev;
};

#define RKUSB_OUT(t,dev,ep,req)        do{if((dev)->usb_out && RKUSB_NOT_HANDLED != (dev)->usb_out(ep,req) ) return ;}while(0)
#define RKUSB_RESET(t,dev)         do{(dev)->usb_out=(dev)->usb_out_bak;}while(0)
int rkusb_init(struct rkusb_dev_fsg* dev , struct usb_ep* in_ep , 
        struct usb_ep* out_ep , int fsg );
#define RKUSB_CLOSE()           rkusb_init(NULL,NULL,NULL,0)     
#define RKUSB_OPEN(dev,in,out,fsg)           rkusb_init(dev,in,out,fsg)     
        

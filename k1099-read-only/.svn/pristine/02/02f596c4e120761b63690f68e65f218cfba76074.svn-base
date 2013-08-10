/***********************************************
 *copyrigth ROCKCHIP Inc
 *author :WQQ
 *date :009-04-19
 ****************************************************/
#ifndef __ASM_ARCH_RK28_DEBUG_H
#define __ASM_ARCH_RK28_DEBUG_H

#if defined(RK28_PRINT)
	#define rk28printk(msg...)	printk( "ROCKCHIP DEBUG: " msg);
#else
	#define rk28printk(msg...)
#endif

#define   USE_JTAG              0
#define RK28_DPSTACK()          do{ if(rk28_debugs) {;dump_stack();}}while(0)
#define RK28_PTK( fm , argss... )      do{ if(rk28_debugs) {  printk("++%s[%d]:" fm "\n", __func__ , __LINE__ , ## argss);}}while(0)
#ifndef __ASSEMBLY__
extern volatile int rk28_debugs;
#define RK28_OPDBG( n )         do{ rk28_debugs = n;}while(0)
#define WAIT_ME()                     while(rk28_debugs == 0)
#endif

#define FILENAME(file)          ((char*)(file->f_path.dentry->d_name.name))
#define VERVOSE_RUN()       rk28printk("AT %s(%d)!\n" , __func__,__LINE__);    
struct __except_content {
        long                          uregs[18];
        int                            mode;
        unsigned long           addr;
        unsigned int              fsr;
};
struct pt_regs ;

#ifdef CONFIG_USB_GADGET
void set_except_regs(struct pt_regs *new_regs , int mode , 
        unsigned long addr, unsigned int fsr);

/*
* 20091229,HSL@RK,USB & VBUS FLAGS.
* get_msc_connect_flag: MSC is connect or not.
* dwc_usb_enumed: USB active or not.for find out usb bus or charging bus.
* dwc_vbus_status: USB cable insert or not.
*/
int get_msc_connect_flag( void );
#else 
#define set_except_regs(new_regs , mode ,addr, fsr)
#define get_msc_connect_flag( ) 0
#endif
int dwc_usb_enumed( void );
int dwc_vbus_status( void );

extern int     rk28_system_crash;
#define  __system_crashed( )                         (rk28_system_crash&(1<<16))
/* 
 * 20100309,HSL@RK,switch value for rkusb debug.
*/
#define RKDBG_READ                                    0x0       //MAY READ.
#define RKDBG_WRITE                                  0x4        //MAY WRITE.
#define RKDBG_CUSTOMER2                        0x8      // panic no schedule,no user panic
#define RKDBG_CUSTOMER1                        0xa      // panic with schedule,no user panic
#define RKDBG_CUSTOMER0                         0Xc     // when user app fault,panic kernel.
#define RKDBG_LOADER                                0X10   // reboot to loader when panic.

#define __rkusb_debug_mod()                       (rk28_system_crash>=RKDBG_CUSTOMER0)
#endif // __ASM_ARCH_RK28_DEBUG_H        


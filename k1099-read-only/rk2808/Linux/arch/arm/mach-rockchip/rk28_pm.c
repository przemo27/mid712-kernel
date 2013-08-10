/* arch/arm/mach-msm/pm.c
 *
 * Goldfish Power Management Routines
 *
 * Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pm.h>
#include <linux/suspend.h>
#include <linux/delay.h>
#include <asm/arch/system.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/rk28_irqs.h>
#include <asm/arch/gpio.h>
#include <asm/arch/rk28_scu.h>
#include <asm/irq_regs.h>

#ifdef CONFIG_DWC_OTG_HOST_ONLY
int dwc_vbus_status( void ){}
#endif
extern void rockchip_timer_clocksource_suspend_resume(int suspend );
extern int rockchip_timer_clocksource_irq_checkandclear( void );
extern u64 rockchip_timer_read(void);
extern void rockchip_timer_freeze(int freeze );

volatile int     rk28_pm_status ; // 0: normal , 1 : suspend ,
volatile int 	pm_sleep_status = 0; //1:enter 2 level sleep
extern int *(*rk28_idle)(void ) ;
#ifdef CONFIG_DWC_OTG_HOST_ONLY
int rk28_usb_suspend( int exitsuspend )
{return 0;}
int rk28_usb_check_vbus_change( void ){return 0;}
#else
extern int rk28_usb_suspend( int exitsuspend );
extern int rk28_usb_check_vbus_change( void );
#endif
#ifdef CONFIG_DWC_OTG_BOTH_HOST_SLAVE
extern int rk28_usb_check_connectid_change(void);
#else
int rk28_usb_check_connectid_change(void){return 0;}
#endif
/* 
 * 20100309,HSL@RK,test fun . open when need.
*/
#if 0
void rk28_print_scu_reg( void )
{
        int *reg = (int *)(SCU_BASE_ADDR_VA);   
        debug_print("0x%08x 0x%08x 0x%08x 0x%08x\n"
                            "0x%08x 0x%08x 0x%08x 0x%08x\n"
                          //  "0x%08x 0x%08x 0x%08x 0x%08x\n" ,
                            ,reg[0],reg[1],reg[2],reg[3],
                            reg[4],reg[5],reg[6],reg[7]);
}

/*
 * 20100309,HSL@RK,test fun for caclulate latency from deep sleep to irq action.
 * some driver need handle irq quickly as possible.
 */
void halt_latency( void )
{
        u64 now=rockchip_timer_read() ;
        printk("wakeup time cycle=%Ld,now cycle=%Ld,delay=%Ld us\n" ,
                rk28_up_timer_cycle ,now,(now-rk28_up_timer_cycle)*4 );
}

/*
 * 20100309,HSL@RK,check fun for system no action,and softlookup no effective,
 * but irq can go.
*/
static unsigned long last_jn;
void rk28_check_jiffies_at_irq( void )
{
        struct pt_regs *new_regs = get_irq_regs();
        int             print = 0;

        if( !last_jn )
                last_jn = jiffies;
        printk("last jiffies=%ld,now jiffies=%ld,goes=%ld\n" , last_jn , jiffies ,jiffies -last_jn);
        if( jiffies -last_jn < 25 || print ) {
                
                if (new_regs)
		show_regs(new_regs);
	else
		dump_stack();
        }
        last_jn = jiffies;
}
#endif

static int rk28_pm_enter(suspend_state_t state)
{
    //int cur_fre = rockchip_clk_get_arm();
    int irq_val;
    //ktime_t         now;
    if( rk28_pm_status )
        return 0;
    rockchip_clk_set_arm(24);
    rockchip_timer_clocksource_suspend_resume( 1 );
    rk28_pm_status = 1;
    pm_sleep_status = 1;
    //now =  ktime_get();
    //debug_print("befor sleep,timer read=%Ld\n" , rockchip_timer_read());
sleep:    
    rk28_usb_suspend( 0 );
     rk28_idle();
      rk28_usb_suspend( 1 );  
     __udelay(40);
     if ( (irq_val = rockchip_timer_clocksource_irq_checkandclear()) == 0 
           && !rk28_usb_check_vbus_change()
           /*20100325 yangkai, add for host detect*/
           && !rk28_usb_check_connectid_change()) {
        goto sleep;
     }
     rockchip_timer_clocksource_suspend_resume( 0 );
     rockchip_clk_set_arm(SCU_MAX_ARM_CLK);       // cur_fre/1000000
     debug_print("quit arm halt,irq_val=0x%x\n" , irq_val );
     return 0;
}
static void rk28_pm_finish( void )
{
        if( rk28_pm_status == 1 )
                rk28_pm_status = 0;
}
extern int wifi_suspend(suspend_state_t state);
extern void wifi_resume(void);
extern int enc28j60_suspend(suspend_state_t state);
extern void enc28j60_resume(void);

static struct platform_suspend_ops rk28_pm_ops = {
	.enter		= rk28_pm_enter,
	.valid		= suspend_valid_only_mem,
    .finish                       = rk28_pm_finish,
	.begin		= wifi_suspend,
	.end		= wifi_resume,

#ifdef CONFIG_ENC28J60	
	.suspend_net	= enc28j60_suspend,
	.resume_net		= enc28j60_resume,
#endif	
};

static int __init rk28_pm_init(void)
{
	suspend_set_ops(&rk28_pm_ops);
	return 0;
}

__initcall(rk28_pm_init);


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

extern void rockchip_timer_clocksource_suspend_resume(int suspend );
extern int rockchip_timer_clocksource_irq_checkandclear( void );
volatile int     rk28_pm_status ; // 0: normal , 1 : suspend ,
volatile int 	pm_sleep_status = 0; //1:enter 2 level sleep
extern int *(*rk28_idle)(void ) ;
extern int rk28_usb_suspend( int exitsuspend );
extern int rk28_usb_check_vbus_change( void );

static int rk28_pm_enter(suspend_state_t state)
{
    int cur_fre = rockchip_clk_get_arm();
    int irq_val;

    if( rk28_pm_status )
        return 0;
    rockchip_clk_set_arm(24);
    rockchip_timer_clocksource_suspend_resume( 1 );
    rk28_pm_status = 1;
    pm_sleep_status = 1;
sleep:    
    rk28_usb_suspend( 0 );
    //printk("\n******before arm HALT*****\n");
#if 1        
     rk28_idle();
#endif
      //printk("\n******after arm HALT*****\n");
      rk28_usb_suspend( 1 );  
     if ( (irq_val = rockchip_timer_clocksource_irq_checkandclear()) == 0 
           && !rk28_usb_check_vbus_change())
        goto sleep;
     
     rockchip_timer_clocksource_suspend_resume( 0 );
     rockchip_clk_set_arm(cur_fre/1000000);      
     //pm_sleep_status = 0;
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

static struct platform_suspend_ops rk28_pm_ops = {
	.enter		= rk28_pm_enter,
	.valid		= suspend_valid_only_mem,
    .finish                       = rk28_pm_finish,
	.begin		= wifi_suspend,
	.end		= wifi_resume,
};

static int __init rk28_pm_init(void)
{
	suspend_set_ops(&rk28_pm_ops);
	return 0;
}

__initcall(rk28_pm_init);


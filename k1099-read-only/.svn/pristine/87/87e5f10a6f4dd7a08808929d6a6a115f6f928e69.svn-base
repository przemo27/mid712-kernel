/* linux/arch/arm/mach-msm/timer.c
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

#include <linux/init.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/delay.h>

#include <asm/mach/time.h>

#include <asm/io.h>
#include <asm/div64.h>
#include <asm/arch/rk28_scu.h>
#include <asm/arch/rk28_irqs.h>
#include <asm/arch/rk28_debug.h>

#define TIMER_DEBUG             1
#if TIMER_DEBUG /* bug , printk */
#define TIMER_BUG( fm , argss... )   printk("++++%s[%d]:" fm "\n", __FILE__ , __LINE__ , ## argss)
#else
#define TIMER_BUG( fm , argss... )
#endif

typedef volatile unsigned int   io_reg;      
struct rockchip_timer_reg_hw
{
    io_reg load_count;
    io_reg current_value;
    io_reg control_reg;
    io_reg timer_eoi;
    io_reg int_status;
};

static struct rockchip_timer_reg_hw *rockchip_timer_regs_base = 
        (struct rockchip_timer_reg_hw *)(TIMER_BASE_ADDR_VA);

#define WDT_DISABLE()                        do { *( (io_reg*)WDT_BASE_ADDR_VA)=0; }while( 0 )

#define TIMER_MODE_USER                 (0X1<<1)        /* load load_count */
#define TIMER_MODE_FREE                 (0X0<<1)        /* load 0XFFFFFFFF */

#define RK_TIMER_ENABLE( n )                    do { rockchip_timer_regs_base[n].control_reg |= 1 ; } while(0)
#define RK_TIMER_DISABLE( n )                   do { rockchip_timer_regs_base[n].control_reg &= ~1 ; } while(0)

#define RK_TIMER_SETMODE( n  , mode )      do { \
                                                                        if( mode ) rockchip_timer_regs_base[n].control_reg |= mode ;\
                                                                        else rockchip_timer_regs_base[n].control_reg &= ~TIMER_MODE_USER ;} while(0)

#define RK_TIMER_SETCOUNT( n  , count )    do { rockchip_timer_regs_base[n].load_count = count ; } while(0)     
#define RK_TIMER_GETCOUNT( n  )    (rockchip_timer_regs_base[n].load_count )     

#define RK_TIMER_READVALUE( n  )        ( rockchip_timer_regs_base[n].current_value )
#define RK_TIMER_INT_CLEAR( n  )         readl( &rockchip_timer_regs_base[n].timer_eoi )
#define RK_TIMER_INTS_READ( n  )        ( rockchip_timer_regs_base[n].int_status )
#define RK_TIMER_INT_DISABLE( n  )       do { rockchip_timer_regs_base[n].control_reg |= (0x1<<2) ; } while(0)
#define RK_TIMER_INT_ENABLE( n  )     do { rockchip_timer_regs_base[n].control_reg &= ~(0x1<<2) ; } while(0)
#define RK_TIMER_SETMODE_USER( n  )     RK_TIMER_SETMODE( n  , TIMER_MODE_USER )

#define RK_TIMER_GETMODE( n  )     ( rockchip_timer_regs_base[n].control_reg)


/*
 * 20100104,HSL@RK,for more free req,set MIN PCLK=10K(LIKE 0.25M).
 */
#define TIMER_MIN_PCLK                  (25*10000U)     /*how many of one second(HZ), ts 250k = 4us */        
#define TIME_MS2COUNT                   (TIMER_MIN_PCLK/MSEC_PER_SEC)

/*
*  timer0 for clock event to gen timer interrupt.
*  timer1 for clock source to read the current nano time, about interrupt every 57s.
*  (0xffffffff/(75*1000000)) ---MAX APB CLK = 75M.
* the min unit is RK_TIMER_MIN_PCLK.
*/
#define TIMER_CLKEVT                     0      /* system jiffies timer */
#define TIMER_CLKSRC                     1     /* monotic timer */
#define TIMER_CLKUSB                     2      /* usb detect timer , at system suspend status */

#define TIMER_MAX_PCLK                  75000000U
#define TIMER_CLKEVT_CYCLE           0xda7400 // 0x8bcf6 // = (UINT_MAX*TIMER_MIN_PCLK/TIMER_MAX_PCLK)     
#define RK_CHECK_VBUS_COUNT      (600*TIME_MS2COUNT)     /* ms */
#define TIMER_SET_ROUNDUS()         do{ rockchip_round_us = UINT_MAX/rockchip_apb_clk;\
                                                                RK_TIMER_SETCOUNT(TIMER_CLKSRC,rockchip_round_us*rockchip_apb_clk); }while(0)

static unsigned int      rockchip_apb_clk; /* value of apb clk , unit = TIMER_MIN_PCLK HZ */
static volatile cycle_t rockchip_clk_source_cycle ;/* unit = 1us = 1000 ns */
static spinlock_t rockchip_timer_spinlock;      //spin_lock_init( &scu_system_clkinfo.spinlock );   
static int rockchip_round_us;

struct rockchip_clock {
	struct clock_event_device   clockevent;
	struct clocksource          clocksource;
             struct irqaction            irq_event;
	struct irqaction            irq_source;
             
};
static struct rockchip_clock rockchip_clocks;
//static unsigned int clock_source_suspend_count;
//extern volatile int     rk28_pm_status ; // 0: normal , 1 : suspend 

static uint64_t  rockchip_timer_read_clk( int *reload , cycle_t* global_cycle )
{
        uint64_t  t,c;
        unsigned long flags;
        int         int_pending;
        local_irq_save( flags );
        t = (uint64_t)RK_TIMER_READVALUE(TIMER_CLKSRC);
        int_pending = RK_TIMER_INTS_READ(TIMER_CLKSRC);
        c = (uint64_t)RK_TIMER_GETCOUNT(TIMER_CLKSRC);
        if(!*reload && ( int_pending || *global_cycle != rockchip_clk_source_cycle) ) {
                printk("%s::int pending,cnt=0x%x!\n", __func__ ,RK_TIMER_GETCOUNT(TIMER_CLKSRC));
		*reload = 1;
		*global_cycle = rockchip_clk_source_cycle;
        }
        local_irq_restore(flags);
        if( *reload )	/* for next cycle !*/
        	c <<= 1;
        c -= t;
        return c;
}

/*
 *  XXX:may be call at interrupt.
 * 20091116,HSL@RK,bug:if TIMER_CLKSRC down to 0 and set to max, and irq was disabled
 * 
 */
cycle_t rockchip_timer_read(void)
{
        unsigned int clk,t,c;
        unsigned long flags;
        cycle_t global_c ;
        int         int_pending;

        /* clk src current value & rockchip_apb_clk must be read at the same time */
        //t = RK_TIMER_READVALUE(TIMER_CLKSRC)/rockchip_apb_clk;
         local_irq_save( flags );
        t = RK_TIMER_READVALUE(TIMER_CLKSRC);
        clk = rockchip_apb_clk;
        int_pending = RK_TIMER_INTS_READ(TIMER_CLKSRC);
        if( int_pending ) {
                printk("%s::int pending,apb clk=%d,cnt=0x%x!\n", __func__ ,clk,RK_TIMER_GETCOUNT(TIMER_CLKSRC));
                //debug_print("%s::int pending,apb clk=%d,cnt=0x%x!\n", __func__ ,clk,RK_TIMER_GETCOUNT(TIMER_CLKSRC));
                RK_TIMER_INT_CLEAR(TIMER_CLKSRC);
                rockchip_clk_source_cycle += rockchip_round_us; 
        }
        c = RK_TIMER_GETCOUNT(TIMER_CLKSRC);
        global_c = rockchip_clk_source_cycle ;      /* rockchip_clk_source_cycle: 64bit var , clk scr int will change it*/
        local_irq_restore(flags);
        //RK28_PTK("read=0x%x,count=0x%x,round=%d,total=%d",
        //        t,c,rockchip_round_us,global_c);
        c -= t;
        //do_div(c,clk);
        c /= clk;
        
        return  global_c+c;
        
}

/* system need ! 
*/
unsigned long read_persistent_clock(void)
{
        cycle_t c;
        c = rockchip_timer_read();
        do_div(c,TIMER_MIN_PCLK); /* mclk to second */
        return (unsigned long)(c); 
}

/* 20091128,HSL@RK,system need !, returns current time in nanosec units */
unsigned long long sched_clock(void)
{
        cycle_t c;
        c = rockchip_timer_read();
//        printk("%s::run time = 0x%Lx us\n" , __func__ , c );
        return c*(NSEC_PER_SEC/TIMER_MIN_PCLK);
}

int rockchip_timer_set_next_event(unsigned long cycles,
				    struct clock_event_device *evt)
{
        RK_TIMER_DISABLE(TIMER_CLKEVT);
        RK_TIMER_SETCOUNT(TIMER_CLKEVT,cycles*rockchip_apb_clk);
        RK_TIMER_ENABLE(TIMER_CLKEVT);
        return 0;       /* 0: OK */
}

int rockchip_timer_change_pclk( ip_id ip , int input_clk )
{
        int clk,clk_old ;
        unsigned int cur_cyl0;
        int timer_event_running;
        clk = input_clk/TIMER_MIN_PCLK;
        if( clk == rockchip_apb_clk )
                return 0;
         clk_old = rockchip_apb_clk;
        rockchip_clk_source_cycle = rockchip_timer_read();
         spin_lock( &rockchip_timer_spinlock );
         RK_TIMER_DISABLE(TIMER_CLKSRC);
        rockchip_apb_clk = clk ;
        TIMER_SET_ROUNDUS();
        RK_TIMER_ENABLE(TIMER_CLKSRC);  
        spin_unlock( &rockchip_timer_spinlock );
        
        //while( cur_cyl0 = RK_TIMER_READVALUE(TIMER_CLKEVT) < 20 );
        /* make sure timer src not down to 0 */
        /* max tmp = 30 , 30*3 = 90 
        *  XXX: clk > rockchip_apb_clk & clk < rockchip_apb_clk will be diffirent , 
        */
        timer_event_running = RK_TIMER_GETMODE(TIMER_CLKEVT)&0X01;
        cur_cyl0 = RK_TIMER_READVALUE(TIMER_CLKEVT);
        RK_TIMER_DISABLE(TIMER_CLKEVT);
        /* 20100109,HSL@RK, DIV FIRST,or will be overflow */
        cur_cyl0 = cur_cyl0/clk_old*clk;
        RK_TIMER_SETCOUNT(TIMER_CLKEVT,cur_cyl0);
        if( timer_event_running ) {
                RK_TIMER_ENABLE(TIMER_CLKEVT);
        }
        //debug_print("timer change clk ,first cycle=%Ld,last cycle=%Ld,rockchip_apb_clk=%d\n" , 
        //       rockchip_clk_source_cycle,rockchip_timer_read(),rockchip_apb_clk);
        return 0;
}

irqreturn_t rockchip_timer_clockevent_interrupt(int irq, void *dev_id)
{
        struct clock_event_device *evt = dev_id;
        //             debug_gpio_reverse();
        RK_TIMER_INT_CLEAR(TIMER_CLKEVT);
        if( evt->mode != CLOCK_EVT_MODE_PERIODIC )
                RK_TIMER_DISABLE(TIMER_CLKEVT);
        evt->event_handler(evt);
        return IRQ_HANDLED;
}


/**
 * 20091116,HSL@RK, test irq latency at apb 75M: 
 * max at video:  11.13 us
 * min at idle:       1.57 us
 * avarage:     about  6 us.
 */
irqreturn_t rockchip_timer_clocksource_interrupt(int irq, void *dev_id)
{             
                //int irq_pending = RK_TIMER_INTS_READ(TIMER_CLKSRC);
                //int cur_value = RK_TIMER_READVALUE(TIMER_CLKSRC);

                RK_TIMER_INT_CLEAR(TIMER_CLKSRC);
                //debug_gpio_reverse();
                //debug_print("%s::count=0x%x\n",__func__,RK_TIMER_GETCOUNT(TIMER_CLKSRC));
                #if 0
                /* 20091106,hsl,because suspend reset the count , must set count here ! */
                RK_TIMER_DISABLE(TIMER_CLKSRC);
                RK_TIMER_SETCOUNT(TIMER_CLKSRC,TIMER_CLKEVT_CYCLE*rockchip_apb_clk); /* 500 MS */
                RK_TIMER_ENABLE(TIMER_CLKSRC);
                #endif
	rockchip_clk_source_cycle += rockchip_round_us; 
//	printk("clock source int,cur=0x%x,count=0x%x\n" , RK_TIMER_READVALUE(TIMER_CLKSRC) , 
//	        RK_TIMER_GETCOUNT(TIMER_CLKSRC) );
//	printk("%s::irq_pen0=%d,irq_p1=%d,count=0x%x,apb=%d\n" , __func__ , irq_pending , 
//	        RK_TIMER_INTS_READ(TIMER_CLKSRC) , cur_value ,rockchip_apb_clk );
	return IRQ_HANDLED;
}

/*
 * 20091120,HSL@RK,disable irq for enable timer and set count . 
 *
 */
void rockchip_timer_clocksource_suspend_resume(int suspend )
{
        unsigned long flags;
        //debug_print("suspend=%d,rockchip_apb_clk=%d,vbus check count=%d\n" , 
        //        suspend,rockchip_apb_clk , RK_CHECK_VBUS_COUNT);
        local_irq_save( flags );
        rockchip_clk_source_cycle = rockchip_timer_read();
        RK_TIMER_DISABLE(TIMER_CLKSRC);
        if( suspend ) {
                //clock_source_suspend_count = RK_TIMER_READVALUE(TIMER_CLKSRC);
                rockchip_round_us = RK_CHECK_VBUS_COUNT;
                RK_TIMER_SETCOUNT(TIMER_CLKSRC,RK_CHECK_VBUS_COUNT*rockchip_apb_clk); 
        } else {
                TIMER_SET_ROUNDUS();            
                //  RK_TIMER_SETCOUNT(TIMER_CLKSRC,rockchip_apb_clk*TIMER_CLKEVT_CYCLE); /* next time for max value*/
                //  rockchip_round_us = TIMER_CLKEVT_CYCLE;
        }
        RK_TIMER_ENABLE(TIMER_CLKSRC);
        local_irq_restore(flags);
        //debug_print("S/R timer,first cycle=%Ld,last cycle=%Ld,rockchip_apb_clk=%d\n" , 
        //       rockchip_clk_source_cycle,rockchip_timer_read(),rockchip_apb_clk);
}

static unsigned int clock_event_freeze_count;
static int clock_source_freezed;
void rockchip_timer_freeze(int freeze )
{
        unsigned long flags;
        //debug_print("suspend=%d,rockchip_apb_clk=%d,vbus check count=%d\n" , 
        //        suspend,rockchip_apb_clk , RK_CHECK_VBUS_COUNT);
        local_irq_save( flags );
        if( freeze ) {
                rockchip_clk_source_cycle = rockchip_timer_read();
                RK_TIMER_DISABLE(TIMER_CLKSRC);
                clock_event_freeze_count = RK_TIMER_READVALUE(TIMER_CLKEVT);
                 RK_TIMER_DISABLE(TIMER_CLKEVT);
        } else {
                TIMER_SET_ROUNDUS(); 
                RK_TIMER_ENABLE(TIMER_CLKSRC);
                RK_TIMER_SETCOUNT(TIMER_CLKEVT,clock_event_freeze_count);
                RK_TIMER_ENABLE(TIMER_CLKEVT);
        }
        clock_source_freezed = freeze;
        local_irq_restore(flags);
}

/* 
 * return 1:need to quit suspend , 0 :reenter suspend 
 * 20091010,由于拔掉USB会产生我们不希望的中断，因此USB只能通过
 * VBUS的变化来判断，不能通过中断判断.
 * 关于VBUS的判断，统一放到 dwc_otg_pcd.c 文件里面.
 */
int rockchip_timer_clocksource_irq_checkandclear( void )
{
        int  t0,t;
        int *intc_reg = (int *)(INTC_BASE_ADDR_VA+0x2c);     
        
        t0 = *intc_reg++;
        t = *intc_reg;

        #if 0
        //debug_gpio_reverse();
        debug_print("count=0x%x,irq0=0x%x!,irq1=0x%x(%d)!\n" ,
                RK_TIMER_GETCOUNT(TIMER_CLKSRC), t0 , t , (t>>IRQ_NR_TIMER2)&1 );
        #endif

        /* clock source irq */
        if( t&(1<<IRQ_NR_TIMER2) ) {
                RK_TIMER_INT_CLEAR(TIMER_CLKSRC);
                //RK_TIMER_DISABLE(TIMER_CLKSRC);
                rockchip_clk_source_cycle += rockchip_round_us ;
                //RK_TIMER_SETCOUNT(TIMER_CLKSRC,RK_CHECK_VBUS_COUNT*rockchip_apb_clk); /* 500 MS */
                //RK_TIMER_ENABLE(TIMER_CLKSRC);
                t &= ~(1<<IRQ_NR_TIMER2);
        }
        /* 20091103,HSL@RK,all irq must be handle !*/
        t |= (t0&0xff);
        return t; 
}
void rockchip_timer_set_mode(enum clock_event_mode mode,
			      struct clock_event_device *evt)
{
//	struct rockchip_clock *clock = container_of(evt, struct rockchip_clock, clockevent);
	switch (mode) {
	case CLOCK_EVT_MODE_RESUME:
	          RK_TIMER_ENABLE(TIMER_CLKEVT);
                          break;
	case CLOCK_EVT_MODE_PERIODIC:
                          
		break;
	case CLOCK_EVT_MODE_ONESHOT:
	           //RK_TIMER_DISABLE(TIMER_CLKEVT);
                        //RK_TIMER_INT_ENABLE( TIMER_CLKEVT );
                        //RK_TIMER_SETMODE_USER( TIMER_CLKEVT );
                        //RK_TIMER_ENABLE(TIMER_CLKEVT);
		break;
	case CLOCK_EVT_MODE_UNUSED:
                         break;
	case CLOCK_EVT_MODE_SHUTDOWN:
		RK_TIMER_DISABLE(TIMER_CLKEVT);
		break;
	}
}


void __init rockchip_timer_clock_source_init( int apb_clk )
{
        int             v =  apb_clk / TIMER_MIN_PCLK;

        WARN_ON(v * TIMER_MIN_PCLK != apb_clk);  /* have bug */

        printk("%s::apb=%d,timer apb=%d\n",__func__,apb_clk,v);
        spin_lock( &rockchip_timer_spinlock );
        rockchip_apb_clk = v ;
        RK_TIMER_DISABLE(TIMER_CLKSRC);
        //RK_TIMER_SETCOUNT(TIMER_CLKSRC, TIMER_CLKEVT_CYCLE*v );
        TIMER_SET_ROUNDUS();
        RK_TIMER_INT_ENABLE( TIMER_CLKSRC );
        RK_TIMER_SETMODE_USER( TIMER_CLKSRC );
        RK_TIMER_ENABLE(TIMER_CLKSRC);
        rockchip_clk_source_cycle = 0;

        RK_TIMER_DISABLE(TIMER_CLKEVT);
        RK_TIMER_INT_ENABLE( TIMER_CLKEVT );
        RK_TIMER_SETMODE_USER( TIMER_CLKEVT );
        
        spin_unlock( &rockchip_timer_spinlock );
}

void udelay( unsigned long usecs )
{
        uint64_t later ;
        uint64_t now,now1;
        int	        reload = 0;
        cycle_t         gcl = rockchip_clk_source_cycle;
        if( clock_source_freezed )
                return __udelay( usecs );
                        
        now = rockchip_timer_read_clk( &reload , &gcl );
        if( USEC_PER_SEC != TIMER_MIN_PCLK ) {
                later = (rockchip_apb_clk*TIMER_MIN_PCLK);
                later *= ((uint64_t)usecs);
                do_div(later,USEC_PER_SEC);
        } else {
                later = usecs;
        }
        later += now;
        while( later > now ) {
                now1 = rockchip_timer_read_clk( &reload ,&gcl ) ;
                if( now > now1) {
                        printk("old=%Ld , new=%Ld\n" , now , now1 );
                }
                now = now1;
        }
}
EXPORT_SYMBOL(udelay);

/* code for test apb clk 
 * apb clk = 示波器读数 nHz * 2 * (150*1000) HZ.
 * if nHz == 250 , the apb clk = 250*2*150*1000 = 75 000 000 = 75M 
 */
#if 0

irqreturn_t rockchip_timer3_interrupt(int irq, void *dev_id)
{
        debug_gpio_reverse();
        RK_TIMER_INT_CLEAR(TIMER_CLKUSB);
        return IRQ_HANDLED;
}

struct irqaction timer3_irqa = {
                .name    = "timer3",
                .flags   = 0 ,
                .handler = rockchip_timer3_interrupt,
                .dev_id  = NULL,
                .irq     = IRQ_NR_TIMER3
};
void rochchip_init_timer3( void )
{
        if( setup_irq(IRQ_NR_TIMER3,&timer3_irqa) ) {
                printk("request irq timer3 failed\n");
                BUG();
        }
        RK_TIMER_DISABLE(TIMER_CLKUSB);
        RK_TIMER_SETCOUNT(TIMER_CLKUSB, 150*1000 );
        RK_TIMER_INT_ENABLE( TIMER_CLKUSB );
        RK_TIMER_SETMODE_USER( TIMER_CLKUSB );
        RK_TIMER_ENABLE(TIMER_CLKUSB);
}
#else
#define rochchip_init_timer3()
#endif
#if 0 
/**
 * test rockchip_timer_read for int inversion
 */
int rockchip_timer_test_read( int delay_ms )
{
        int t,c,clk;
        unsigned long flags;
        t = RK_TIMER_READVALUE(TIMER_CLKSRC);
        clk = rockchip_apb_clk;
        c = RK_TIMER_GETCOUNT(TIMER_CLKSRC);
        printk("cur val=0x%x,load count=0x%x,clk=%d\n" , t , c , clk );
        local_irq_save(flags);
        local_irq_disable();
        udelay( delay_ms*1000 );
        local_irq_restore(flags);
        t = RK_TIMER_READVALUE(TIMER_CLKSRC);
        c = RK_TIMER_GETCOUNT(TIMER_CLKSRC);
        printk("cur val=0x%x,load count=0x%x,clk=%d\n" , t , c , clk );
        return 0x300;
}

/* code for test timer count way */
void rockchip_timer_test3( void )
{
        int i = 20;
        printk("%s::apb=%d,last count=0x%x\n" ,__func__, rockchip_apb_clk , TIMER_CLKEVT_CYCLE*rockchip_apb_clk);
        RK_TIMER_DISABLE(TIMER_CLKUSB );
        RK_TIMER_SETMODE_USER( TIMER_CLKUSB );
        RK_TIMER_SETCOUNT(TIMER_CLKUSB, 10000*rockchip_apb_clk );
        RK_TIMER_ENABLE(TIMER_CLKUSB);
        RK_TIMER_SETCOUNT(TIMER_CLKUSB, TIMER_CLKEVT_CYCLE*rockchip_apb_clk );
        do {
                printk("timer3 count=0x%x\n" , RK_TIMER_READVALUE(TIMER_CLKUSB) );
                udelay( 31 );
        } while( i-- );
}

/* code for test delay ,rockchip_timer_read,rockchip_timer_change_pclk */
extern int __rockchip_set_arm_clk_hw( int clk );
void __rockchip_timer_test_delay( void )
{
        int i ;
        int j = 0;
        cycle_t now;

        local_irq_enable();   
        while( 1 ) {
                
                now = rockchip_timer_read();
                debug_print("%d:%x,%Lx\n" ,123 ,  now>>32 , now );
                
                i = 30000;
                while( i-- ) {
                        rockchip_udelay(123);   /* about 121 us */
                        TIMER_GPIO_REVERSE();
                }

                now = rockchip_timer_read();
                debug_print("%d:%x,%Lx\n" , 37 , now>>32 , now  );
                i = 100000;
                while( i-- ) {
                        rockchip_udelay(37); /* about 36us */
                        TIMER_GPIO_REVERSE();
                }
                
                now = rockchip_timer_read();
                debug_print("%d:%x,%Lx\n" , 1000 , now>>32 , now  );
                
                i = 5000;
                while( i-- ) {
                        rockchip_udelay(1000);  /* 1000us */
                        TIMER_GPIO_REVERSE();
                }

                now = rockchip_timer_read();
                debug_print("%d:%x,%Lx,%x\n" , 3210, now>>32 , now , RK_TIMER_GETCOUNT(TIMER_CLKSRC) );
                
                i = 2000;
                while( i-- ) {
                        rockchip_udelay(3210); /* about 3200 us */
                        TIMER_GPIO_REVERSE();
                }
                j++;
                if( j & 1 ) {
                        __rockchip_set_arm_clk_hw( 330 );
                        rockchip_timer_change_pclk(0,rockchip_clk_get_apb());
                }  else {
                        __rockchip_set_arm_clk_hw( 450 );
                        rockchip_timer_change_pclk(0,rockchip_clk_get_apb());
                }
        }
}
#else
#define __rockchip_timer_test_delay()
#endif


static struct rockchip_clock rockchip_clocks = {
		.clockevent = {
			.name           = "timer0",
			.features       = CLOCK_EVT_FEAT_ONESHOT,
			.shift          = 32,
			.rating         = 200,
			.set_next_event = rockchip_timer_set_next_event,
			.set_mode       = rockchip_timer_set_mode,
		},
		.clocksource = {
			.name           = "timer1",
			.rating         = 300,
			.read           = rockchip_timer_read,
			.mask           = CLOCKSOURCE_MASK(63),
			.shift          = 8,
			.flags          = CLOCK_SOURCE_IS_CONTINUOUS,
		},
		.irq_event = {
			.name    = "timer0",
			.flags   = IRQF_DISABLED | IRQF_TIMER ,
			.handler = rockchip_timer_clockevent_interrupt,
			.dev_id  = &rockchip_clocks.clockevent,
			.irq     = IRQ_NR_TIMER1
		},
		.irq_source = {
			.name    = "timer1",
			.flags   = IRQF_DISABLED ,
			.handler = rockchip_timer_clocksource_interrupt,
			.dev_id  = &rockchip_clocks.clocksource,
			.irq     = IRQ_NR_TIMER2
		},
};
static void __init rockchip_timer_init(void)
{
	int res;
	
	struct rockchip_clock *clock = &rockchip_clocks;
	struct clock_event_device *ce = &clock->clockevent;
	struct clocksource *cs = &clock->clocksource;

              /* 
               * init at  machine_rk28_mapio for udelay early use.
               * must after __rockchip_scu_init_hw to use rockchip_clk_get_apb.
               */
             //rockchip_timer_clock_source_init(  rockchip_clk_get_apb() ); 
             
                cs->mult = clocksource_hz2mult(TIMER_MIN_PCLK , cs->shift);
	res = clocksource_register(cs);
	if (res)
		BUG();  /* have bug */
        
	ce->mult = div_sc( TIMER_MIN_PCLK , NSEC_PER_SEC, ce->shift);
	ce->max_delta_ns =  
		clockevent_delta2ns( TIMER_CLKEVT_CYCLE , ce);
                #ifdef CONFIG_HIGH_RES_TIMERS
                ce->min_delta_ns = NSEC_PER_SEC/HZ;
                #else
	ce->min_delta_ns = clockevent_delta2ns(2, ce);
                #endif
                
	ce->cpumask = cpumask_of_cpu(0);
                clockevents_register_device(ce);
            
                res = setup_irq(clock->irq_source.irq, &clock->irq_source);
	if (res)
		BUG();  /* have bug */

                __rockchip_timer_test_delay();
                rochchip_init_timer3();
                
                res = setup_irq(clock->irq_event.irq, &clock->irq_event);
	if (res)
		BUG();  /* have bug */
}


struct sys_timer rockchip_timer = {
	.init = rockchip_timer_init
};

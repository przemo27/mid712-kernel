/*
 * function for test and debug
 *
 * Copyright (C) 2009 Rochchip, Inc.
 * Author: Hsl
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

/* #define DEBUG */
/* #define VERBOSE_DEBUG */
#include <asm/arch/typedef.h>
#include <asm/arch/hardware.h>
#include <asm/arch/rk28_debug.h>
#include <asm/arch/rk28_scu.h>
#include <linux/kallsyms.h>
#include <linux/delay.h>


/*
 * 20090725,hsl,some debug function start from here
 */
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/mmzone.h>
#include <asm/sections.h>

extern unsigned long lookup_all_symbol(const char *symbol); /* at module.c */
extern int __scu_call_wrap(unsigned long *argv , int argc , int fun );
extern struct pt_regs * __scu_get_regs( void );
/*
 *  type define :
 *  first for function name , 
 * ',' for argument.
 * \' : char
 * \" : string.
 * other : for int,short,0x for hex.0 for octal.
 * 20090811,for easy use , 0x or 0-9 start means num , other ,means string . 
 * len = 1 string for char.
 */
unsigned long __rk28_scu_get_arg( char ** pp )
{
        char * p = *pp;
        char *t;
        unsigned int ret;
         int len ;
        if( p[0] == '0' ) {
                if( ( p[1] == 'x' || p[1] == 'X' ) ) {
                        p+= 2;
                        len = sscanf(p,"%x" , &ret );
                        p += len;
                } else {
                        if( p[1] == 0 || p[1] == ',' ) {
                                ret = 0; /* only one '0' , must be zero */
                                p++;
                        } else {
                                p+= 1;
                                len = sscanf(p,"%o" , &ret );
                                p += len;
                        }
                }
        } else if( p[0] >= '1' && p[0] <= '9' ) {
                len = sscanf(p,"%d" , (int*)&ret );
                p += len;
        } else if( p[0] == '\'' ) {
                ret = p[1];
                p++;
        } else {  /* all for string */
                if ( p[0] == '\"' ) {
                        if( p[1] == '\"' ) { /* NULL string */
                                ret = 0;
                                p+=2;     
                        } else {
                                ret = (unsigned long)(p+1);
                                t = strchr( p+1 , '\"' );
                                if( t ) {
                                        *t = 0;
                                        p = t+1 ;
                                } else  {
                                        p++;
                                }
                        }
               }  else {
                        if( *p == ',' )  {/* empty ,as NULL string */
                                ret = 0;
                                p--;
                        } else  if( *p == ' ' || *p == 0 ) /* one char string ,as char.*/
                                ret = *p;
                        else /* as string */
                                ret = (unsigned long)(p);
                        p++;
                }
        }
       t = strchr( p , ',' );
       if( t ) {
            *t = 0;
            *pp = t+1;
       } else { /* p should be the last one */
             while(*p != ' ' && *p != 0 && *p != '\n' )
                        p++;
	*p = 0;
             *pp = p;
       }
       return ret;
}
int __rk28_scu_parse_cmd( char * cdb )
{
        unsigned long reg[6];
        unsigned long fun;
        int             argc = 0 ;
        char    *p = cdb ;
        char    *sym = p;
        int       ret;
        
        //SCU_BUG("CMD=%s\n" , cdb);
        memset( reg , 0 , sizeof reg );
        p = strchr(p , ',');
        if( p ) {
                *p++ = 0;
                while( *p && argc < 6) {
                        reg[argc++] = __rk28_scu_get_arg( &p );
                }
        }
        //debug_print("sym=%s,argc=%d,db=%s\n" , sym ,argc , cdb );
        if( sym[0] == '0' && ( sym[1] == 'x' || sym[1] == 'X' ) ) {
                fun = 0;
                sscanf(sym+2,"%lx" , &fun );
        } else {
                fun = kallsyms_lookup_name(sym);
                //debug_print("after lookup symbols,sym=%s,fun=0x%p\n" , sym ,fun);
        }
        if( fun ) {
                ret = __scu_call_wrap( reg,argc,fun);
        } else {
                ret = -EIO;
        }
        debug_print("sym=%s,fun=0x%p,argc=%d,ret=0x%x(%d)\n" , sym , (void*)fun , argc , ret , ret);
        return ret;
}

/**
 * check arg be a string or int or symbol addr .
 * 1: string , 2: kernel sym , 0:other 
 */
int rk28_arg_string( unsigned long arg )
{
        if( arg > (unsigned long)&_end  && arg < 0xc8000000 )
                return 1;
        if( arg >= (unsigned long)&_text && arg < (unsigned long)&_end )
                return 2;
        return 0;
}

#if 0
/**
 * XXX:20091116,HSL@RK,MUST CHANGE USB CONNECT WAY,USE IRQ.
 *
 */
extern volatile int     rk28_pm_status ; // 0: normal , 1 : suspend ,
void wait_here( void )  /* for use JTAG */
{
        unsigned long flags;
        debug_print("wait......\n" );
        rockchip_timer_freeze( 1 );
        rk28_pm_status = 2;
        //rk28_usb();
        local_save_flags(flags); 
        //local_irq_enable();
        while( rk28_pm_status );
        local_irq_restore(flags);
        rockchip_timer_freeze( 0 );
        debug_print("GO ON!\n" );
}

/* 
 * get task ptr from pid, use at rkusb.
*/
struct task_struct * rk28_get_task_id( int pid )
{
        int found = 0;
        struct task_struct *g, *p;
        read_lock(&tasklist_lock);
        do_each_thread(g, p) {
                if( p->pid == pid ) {
                        found = 1;
                        goto loop_out;
                }
        } while_each_thread(g, p);
        read_unlock(&tasklist_lock);
loop_out:      
       if( found )
             return p;
       return NULL;
}
#endif

void rk28_printk_mem( unsigned int * addr , int words )
{
        unsigned int * reg_base = addr ;
        while( words > 0 ) {
                printk("%p:%08x  %08x  %08x  %08x\n" ,reg_base ,
                        reg_base[0],reg_base[1],reg_base[2],reg_base[3]);
                words -= 4;
                reg_base += 4;
        }
}

/*
 * change global var or io register.
 */
int rk28_write( unsigned int* addr , char * val_arg ) 
{
        unsigned long value[6];
        int             argc = 0 ;
        char    *p = val_arg ;
        int             i; 
        unsigned int * padr;
        unsigned long   tmaddr = (unsigned long )addr;

        //debug_print("addr=0x%p,val args=0x%p\n"  , addr , val_arg );
        if ( tmaddr >= AHB_BASEADD_PA &&  tmaddr < AHB_BASEADD_HI_PA ) {
                addr = (unsigned int *)IO_PA2VA_AHB( tmaddr );
        } else if ( tmaddr >= APB_BASEADD_PA &&  tmaddr < APB_BASEADD_HI_PA ) {
                addr = (unsigned int *)IO_PA2VA_APB( tmaddr );
        } else if ( tmaddr >= 0x60008000 && tmaddr < 0x70000000 ) {
                addr = (unsigned int *)(tmaddr-SDRAM_BASEADD_PA+PAGE_OFFSET);
        }  else if( tmaddr > (unsigned long)&_end && tmaddr < 0xf0000000 ){
                /* addr at string format */
                tmaddr = lookup_all_symbol( (char*) addr );
                //debug_print("sym:%s,its addr=0x%x:  \n"  , (char*)addr, tmaddr );
                if( tmaddr == 0 )
                        return -EIO;
                addr = (unsigned int*)tmaddr;
        } 
        padr = addr;

        for( i = 0 ; i < 6 ; i++ )
                value[i] = 0;
        if( p < (char*)&_end ) {
                value[argc++] = (unsigned long)p;
        } else { /* p is string for some value */
                while( *p && argc < 6) {
                        value[argc++] = __rk28_scu_get_arg( &p );
                }
       }
       for( i = 0 ; i < argc ; i++ )
                *padr++ = value[i];
       for( i = 0 ; i < argc ; i++ )
                debug_print("[0x%p]=0x%x "  ,addr+i , addr[i]);
       debug_print("\n");
       return 0x23;
}
/*
 *  printk global varient value.
 *  var may be name or addr .var MUST be char*.
 *  like "rk28_debugs,0xc00900000,rk28_dump_var".
 */
int rk28_read( char* addr_or_name , int size  ) 
{
        unsigned int *addr;
        char            *p = addr_or_name;

        if( (unsigned long )p <  PAGE_OFFSET || (unsigned long )p >  PAGE_OFFSET+0x8000000 )  // name  
                return -EFAULT;
        if( p < (char*)&_end ) {
                addr = (unsigned int*)p;
        } else {
                addr = (unsigned int*)lookup_all_symbol( p );
       }
       //printk("%s::name=%s,size=%d,addr=0x%p\n" , __func__ , p , size , addr );
       if( addr )
            rk28_printk_mem( addr , size );
       return 0x26;
}

#if 0
/*
 * print all task stack.
 * add name,status,no stack.
 * flag[0]=r:only running,i:interrupt,u:uninterrupt,o:other,*:all
 * flag[1]=s:print stack,other:no stack
 */
 #if 0
char    *ts[] = {
               "RUNNING",
                "INTERRUPTIBLE",
                "UNINTERRUPTIBLE",
                "_STOPPED",
                };
 #define GET_STATE( p )  (p->state>2?ts[3]:ts[p->state])

#define PRINT_TASK()  do {printk("#task:%s,%s,pid=%d,ut=%lu,st=%lu,prio=%d,%d,%d,rt prio=%d#\n\n"  , tc , \
                                GET_STATE( p ) , p->pid , p->utime , p->stime , p->prio, p->static_prio , p->normal_prio,p->rt_priority );\
                                if( print_stack ) show_stack( p , &ssp );}while(0)
#define PRINT_RESET()                                
#define PRINT_SLICE(nr)
#define PRINT_OUT()
#define PRINT_VARS(fmt...) 
#else
static char task_print_buf[400*1024];
static char *p_now;
#define PRINT_RESET()   do{p_now=task_print_buf;memset(p_now,0,sizeof(task_print_buf));}while(0)
#define PRINT_OUT()     do{printk("########%d#######\n" ,p_now-task_print_buf);\
                                                uart_write_string(task_print_buf);}while(0)
#define PRINT_TASK()  do {p_now += sprintf(p_now , "task:%s,pid=%d,prio=%d\n"  , tc , \
                                p->pid , p->static_prio  );\
                               }while(0)
#define PRINT_SLICE(nr)      do {p_now += sprintf(p_now , "---%d"  , nr);}while(0)                         
#define PRINT_VARS(fmt...)     do {p_now += sprintf(p_now , fmt);}while(0)                         
#endif
int rk28_dump_task( char * taskcmd , char * flag ) 
{
        struct task_struct *g, *p;
        unsigned long ssp;
        char    tc[128];
        
        int     ret =-EIO;
        pid_t  pid =0;
        int task_status = 0xff;
        int print_stack = 0;
        int     total_tasks = 0;
        
        if( flag ) {
                if( flag[0] == 'r' )
                        task_status = TASK_RUNNING;
                if( flag[0] == 'i' )
                        task_status = TASK_INTERRUPTIBLE;
                if( flag[0] == 'u' )
                        task_status = TASK_UNINTERRUPTIBLE;
                if( flag[0] == 'o' )
                        task_status &= ~0x7;
                if( flag[0] != 0 ) {
                        if( flag[1] == 's' ) 
                                print_stack = 1;
                }
        }
        if( taskcmd &&  taskcmd < (char*)0xc0000000 ) {
                pid = (pid_t)taskcmd;
                taskcmd = NULL;
        }
        read_lock(&tasklist_lock);
        do_each_thread(g, p) {
                get_task_comm(tc, p);
                total_tasks++;
                if( taskcmd  ) {
                        if( strstr(tc , taskcmd)  ) {
                        PRINT_TASK();
                        ret = 0X20;
                        break;
                        }
                } else if (pid  ){
                        if( p->pid == pid ) {
                                PRINT_TASK();
                                ret = 0X20;
                                break;
                        }
                }else {
                        if( task_status == 0xff || (p->state == task_status) || (p->state & task_status) ) {
                                PRINT_TASK();
                        }
                        ret = 0X21;
                }
        } while_each_thread(g, p);
        PRINT_SLICE(total_tasks);
        printk("Total search task=%d\n" , total_tasks );
        read_unlock(&tasklist_lock);
        return ret;
}

int rk28_current( void  ) 
{
        struct task_struct *p;
        unsigned long ssp;
        char    tc[128];
        int print_stack = 1;
        p = current;
        get_task_comm(tc, p);
        PRINT_TASK();
        return 0x80;
}

/* test music stop when one task running to long */
int rk28_audio( int prio ,int to_ms, int yld ,int npt )
{
        int     total_ms = 1000;
        int     loop = 0;
        int     sum ;
        ktime_t ktime_now;
        int     ms = 13;
        int     nicee;

        if( to_ms )
                total_ms = to_ms;
        yield();
        PRINT_RESET();
        nicee = task_nice( current );
        set_user_nice(current, -prio);
        yield();
        ktime_now = ktime_get();
        do {
                sum = 0;
                while( sum < total_ms ) {
                        mdelay(ms);
                        rk28_dump_task(NULL,"r");
                        if( yld )
                                yield();
                        //printk("sum=%d\n" , sum );
                        PRINT_VARS(",%d---\n\n" , sum );
                        sum+= ms;
                }
                loop--;
                
        } while( loop > 0 );
        ktime_now = ktime_sub( ktime_get(), ktime_now );
        set_user_nice(current, nicee);
        yield();
        if( npt ) {
                PRINT_OUT();
        }
        debug_print("Total Need NS=%Ld for loop %d,total ms=%d\n" , ktime_to_ns(ktime_now) ,loop,total_ms);
        debug_print("%s::pri=%d,YIELD=%d,ms=%d\n",__func__,prio,yld,ms);
        return 0x81;
}
#endif

#if 0

extern int rk28_usb_suspend( int exitsuspend );
/*  test close phy when usb connect . */
int rk28_usb_rs( void )
{
        rk28_usb_suspend(0);
        mdelay( 10 );
        rk28_usb_suspend(1);
        return 0x83;
}

/* test lcd look like when close and open clk 
     20100129,NOT GOOD FOR 20 us 
*/
int rk28_lcd( int loop , int us)
{
        if( us == 0 ) us = 400;
        do {
                rockchip_scu_disableclk(SCU_IPID_LCDC);
                udelay(us);
                rockchip_scu_enableclk(SCU_IPID_LCDC);
                yield();
                mdelay(24);
                yield();
                loop --;
        } while ( loop > 0 );
        return 0x82;
}
/*
 *  get buddy memory info.
 */
int rk28_meminfo( void )
{
        int    order;
        struct zonelist *       znlist;
        int     total_free_pages ;
        int nd = numa_node_id();
        znlist = NODE_DATA(nd)->node_zonelists;
        nd = 0;
        
        #define zone    znlist->zones[nd]
        while( zone ) {
                order = 0;
                 total_free_pages = 0;
                debug_print("zone list %s meminfo,total pages=%d(%dM)\n" ,  zone->name , zone->present_pages , zone->present_pages>>8 );
                while( order < MAX_ORDER) { /* struct free_area	free_area[MAX_ORDER]; */
                        debug_print("  % 5d pages free:%d\n" , 1<<order , zone->free_area[order].nr_free );
                        total_free_pages += zone->free_area[order].nr_free*(1<<order);
                        order++;
                }
                debug_print("zone list %s total free pages=%d(%dM)\n" ,  zone->name , total_free_pages, total_free_pages>>8);
                nd++;
        }
        return 0x25;
}


static void rk28_print_task_vma( struct task_struct *p , char * taskname )
{
                struct vm_area_struct * mmap;
                struct mm_struct *mm = p->mm;
                unsigned long total = 0;
                int             i = 0;
                if( !mm )
                        mm = p->active_mm;
                mmap = mm->mmap;
                printk("task#:%s mm & vma info :\n" , taskname );
                printk("pgd:0x%p task size:0x%lx\n" ,
                        mm->pgd , mm->task_size );
                printk("start code:0x%lx end code:0x%lx start data:0x%lx end data:0x%lx\n" ,
                        mm->start_code, mm->end_code, mm->start_data, mm->end_data);
                printk("start brk:0x%lx cur brk:0x%lx start stack:0x%lx\n" ,
                        mm->start_brk, mm->brk, mm->start_stack);
                while( mmap ) {
                        total += mmap->vm_end - mmap->vm_start;
                        printk("%d:start 0x%lx end=0x%lx " ,i , mmap->vm_start , mmap->vm_end);
                        printk("flag=[0x%lx] \"%s|%s|%s|%s|%s|%s|%s|%s\", file=0x%p\n" , 
                                mmap->vm_flags , (mmap->vm_flags&VM_READ)?"R":"" ,
                                (mmap->vm_flags&VM_WRITE)?"W":"" ,
                                (mmap->vm_flags&VM_EXEC)?"E":"" ,
                                (mmap->vm_flags&VM_SHARED)?"S":"" ,
                                (mmap->vm_flags&VM_GROWSDOWN)?"GD":"" ,
                                (mmap->vm_flags&VM_GROWSUP)?"GU":"" ,
                                (mmap->vm_flags&VM_DENYWRITE)?"DW":"" ,
                                (mmap->vm_flags&VM_IO)?"IO":"" ,
                                mmap->vm_file );
                        mmap = mmap->vm_next;
                        i++;
                }
                printk("total vma size=0x%lx\n" , total );
}
int rk28_task_vma( char * taskcmd  ) 
{
        struct task_struct *g, *p;
        char    tc[128];
        
        int     ret =-EIO;
        pid_t  pid =0;

        if( taskcmd &&  taskcmd < (char*)0xc0000000 ) {
                pid = (pid_t)taskcmd;
                taskcmd = NULL;
        }
        if( !taskcmd && !pid ) {
                p = current ;
                get_task_comm(tc, p);
                rk28_print_task_vma( p , tc );
                return 0x20;
        }
                
        read_lock(&tasklist_lock);
        do_each_thread(g, p) {
                get_task_comm(tc, p);
                if( taskcmd  ) {
                        if( strstr(tc , taskcmd)  ) {
                        rk28_print_task_vma( p , tc );
                        ret = 0X20;
                        break;
                        }
                } else if (pid  ){
                        if( p->pid == pid ) {
                                rk28_print_task_vma( p , tc );
                                ret = 0X20;
                                break;
                        }
                }
        } while_each_thread(g, p);
        read_unlock(&tasklist_lock);
        return ret;
}
int rk28_regs( void )
{
        struct pt_regs *regs ;
        unsigned long sp;

        asm("mov %0, sp" : "=r" (sp) : : "cc");
        regs = __scu_get_regs( );
        debug_print("%s...preg=0x%p,sp=0x%p\n" , __func__ , regs , sp );
        rk28_printk_mem( (unsigned int*)regs , sizeof(struct pt_regs)/4 );
        rk28_task_vma( NULL );
        return 0x200;
}
#endif

void __rk28_force_signal(struct task_struct *tsk, unsigned long addr,
		unsigned int sig, int code )
{
	struct siginfo si;
                printk("%s::send sig %d to task %s\n" , __func__ , sig , tsk->comm );
	si.si_signo = sig;
	si.si_errno = 0;
	si.si_code = code;
	si.si_addr = (void __user *)addr;
	force_sig_info(sig, &si, tsk);
}
int rk28_signal( int pid , int sig )
{
        struct task_struct *t = find_task_by_pid( pid );
        if( !t )
                t = current ;
        __rk28_force_signal( t , 0xc0002234 , sig , 0x524b4b52 );
        return 0x201;
}

#if 0
#include <linux/tty.h>
#include <linux/console.h>
#include <linux/tty_flip.h>  
extern struct console *console_drivers;
// like : echo rk28_uart_input,"\"pwd|ls,echo fdfdf\"">active
int rk28_uart_input( char * str )
{
        char * p = str;
        int     indx;
        struct tty_driver *tty_drv = console_drivers->device( console_drivers , &indx );
        struct tty_struct *tty_s = tty_drv->ttys[indx ];
        int flag = TTY_NORMAL;
        
        printk("indx=%d, tty drv=%p,tty_s=%p,cur console=%s,next console=%p\n", indx , tty_drv , tty_s ,
                console_drivers->name , console_drivers->next );
        //printk("cmd=%s\n" , str );
        if( !p )
                return -EIO;
        
        while( *p ) {
                if( *p == ',' )
                        *p = '\n';
                p++;
        }
        *p++ = '\n';
        *p = 0;

        p = str;
        while( *p ) {
                tty_insert_flip_char(tty_s, *p , flag);
                p++;
        }

        tty_flip_buffer_push( tty_s );
        return 0x28;
}
#define PWM_MAIN        0
#define PWM_HRC         1
#define PWM_LRC          2
#define PWM_CTRL        3
#define PWM_CLK         1000
#define PWM_BL_MAX      255
int rk28_bl( int bn  )
{
        unsigned int * pwm0_ctrl_reg = (unsigned int*)(PWM_BASE_ADDR_VA);
        
        if( bn ==  0 ) { /* disable */
                *(pwm0_ctrl_reg+PWM_CTRL) = 0;
                *(pwm0_ctrl_reg+PWM_MAIN) = 0;
        } else  {
                int clk = rockchip_clk_get_apb()/PWM_CLK;
                int clk_h;
                clk >>= 1;
                
                clk_h = clk*(PWM_BL_MAX-bn)/PWM_BL_MAX;
                *(pwm0_ctrl_reg+PWM_HRC) = clk_h;
                *(pwm0_ctrl_reg+PWM_LRC) = clk;
                *(pwm0_ctrl_reg+PWM_CTRL) = 1|(0x01<<3);
                *(pwm0_ctrl_reg+PWM_MAIN) = 0;
        }
        debug_print("backling set to %d,HRC=%d,LRC=%d\n" , bn ,
                *(pwm0_ctrl_reg+PWM_HRC)  , *(pwm0_ctrl_reg+PWM_LRC) );
        return 0x29;
}

/* pm_suspend(3) */
extern int pm_suspend(int state);
int rk28_sleep( void )
{
        pm_suspend(3);
        return 0x2a;
}
#endif
#include <linux/syscalls.h>
#include <asm/uaccess.h>
#include <linux/rtc.h>
#if 0
#define __syscall_return(type, res) \
do { \
　 if ((unsigned long)(res) >= (unsigned long)(-125)) { \
　　　 errno = -(res); \
　　　 res = -1; \
　 } \
　 return (type) (res); \
} while (0)
#define _syscall1(type,name,type1,arg1) \
type name(type1 arg1) \
{ \
long __res; \
__syscall_return(type,__res); \
}
static inline _syscall1(int,close,int,fd) 
#endif
#if 0
extern int kernel_getlog( char ** start  , int * offset , int* len );
int rk28_write_file( char *filename )
{
        char *procmtd = "/proc/mtd";
        struct file			*filp = NULL;
        loff_t		                pos;
        long    fd;
        
        ssize_t l=0;
        char    pathname[64];
        char    buf[1024] ;
        mm_segment_t old_fs;
        struct rtc_time tm;
        ktime_t now;        
        char *tmpbuf; 
        int     mtdidx=0;
        int mtdsize = 0x200000; // 2M 
        char *log_buf;
        int     log_len;

        debug_print("%s: filename=%s\n" , __func__ , filename );

        old_fs = get_fs();
        set_fs(KERNEL_DS);        
        fd = sys_open( procmtd , O_RDONLY , 0 );
        if( fd >= 0 ) {
                memset( buf , 0 , 1024 );
                l = sys_read( fd , buf , 1024 );
               // debug_print("/proc/mtd,total byte=%d\n" , l );
                tmpbuf = strstr( buf , filename );
                if( tmpbuf ) {
                        tmpbuf[10] = 0;
                        mtdsize = simple_strtoul(tmpbuf-19, NULL , 16);
                        mtdidx = tmpbuf[-22];
                        //debug_print("size=%s\n,idx=%s\nmtdsize=%x , mtdidx=%d" , tmpbuf-19 , tmpbuf - 22 ,
                        //        mtdsize , mtdidx );
                        mtdsize *= 512;
                        mtdidx -= '0';
                }
                sys_close( fd );
        } else {
                debug_print("open %s failed\n" , procmtd );
        }
        sprintf(pathname , "/dev/block/mtdblock%d" , mtdidx );
        filp = filp_open(pathname, O_WRONLY , 0);
        if( IS_ERR(filp) ) {
                debug_print("open %s failed n" , pathname  );
                goto out_print;
        }
        if (!(filp->f_op->write || filp->f_op->aio_write)) {
                debug_print("can not write file %s \n" , pathname  );
                goto close_file;
        }
        now = ktime_get();
        rtc_time_to_tm( now.tv.sec , &tm );
        printk( "\n+++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n"
                          "++++++++++RECORD LOG AT %04d-%02d-%02d %02d:%02d:%02d++++++++++\n"
                          "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n\n\n\n"
                          ,
                tm.tm_year+1900 , tm.tm_mon , tm.tm_mday , tm.tm_hour , tm.tm_min , tm.tm_sec );
        /* 20090924,HSL,FTL only intial first 256M at loader , MUST write 32 sector one time */
        //debug_print("write first pack , pos=%d\n" , pos ); /* pos = 2 ?? */
        kernel_getlog(&log_buf , NULL ,  &log_len );
        log_len = (log_len+(32*512-1)) / (32*512) * (32*512);
        pos = 0;
        l = vfs_write( filp , log_buf  , log_len , &pos );
        do_fsync(filp, 1);
close_file:
        filp_close(filp , NULL);
out_print:  
        set_fs(old_fs);
        debug_print("\nlog len=%d,write=%d\n" ,log_len , l);
        return 0x2b;
}
#endif

#if 1
/*
 * 
 * 20091027,2808SDK,
 * lcd off:byte:81748000,word:39100000.
 * lcd on: byte:90742000,word:43158000
 *
 * 20091111,2806 ruiguan,
 * lcd off: byte: 81883000,word: 39209000
 * lcd on: byte: 90843000,word: 43260000
 */
int rk28_memcpy( void )
{
#define PAGE_ORDER              7
        struct timespec tv0,tv1;

        unsigned long pg;
        unsigned long src = 0xc0010000;
        int     i = 8,k=0;
        int     bytes = ((1<<PAGE_ORDER)*PAGE_SIZE);
        
        pg = __get_free_pages(GFP_KERNEL , PAGE_ORDER );
        if( !pg ) {
                printk("alloc %d pages total %dK bytes failed\n" , (1<<PAGE_ORDER) , bytes/(1024));
                return -ENOMEM;
       }
       getnstimeofday( &tv0 );
       while( k < i ) {
                char *p = (char*)pg;
                char  *q = (char*)src;
                char  *m = q+bytes;
                while( q < m ) 
                        *p++ = *q++;   
                k++;
       }
       getnstimeofday( &tv1 );
       printk("need %Ld nanosec to copy %d Kbytes \n" ,
                ktime_to_ns( ktime_sub(  timespec_to_ktime(tv1) , timespec_to_ktime(tv0) ) ), bytes * i /1024 );
       getnstimeofday( &tv0 );
       k = 0;
       while( k < i ) {
                int *p = (int*)pg;
                int  *q = (int*)src;
                int  *m = q+bytes/sizeof(int);
                while( q < m ) 
                        *p++ = *q++;   
                k++;
       }
       
       getnstimeofday( &tv1 );
       free_pages( pg , PAGE_ORDER );
       printk("need %Ld nanosec to copy %d Kwords \n" ,
                ktime_to_ns( ktime_sub(  timespec_to_ktime(tv1) , timespec_to_ktime(tv0) ) ), bytes * i / sizeof (int) /1024 );
       return 0x2c;
}
#endif

extern u64 rockchip_timer_read(void);
int rk28_now( int loop , int prtk )
{
        ktime_t ktime_now;
        struct timespec ts;
        struct rtc_time tm;

        rk28_debugs = prtk;
        if( loop > 20 )
            loop = 20;
        do {
        ktime_now = ktime_get();
        getnstimeofday(&ts);
        rtc_time_to_tm(ts.tv_sec, &tm);
        printk("now=%lld ns, clk cycle=%lld "
               "(%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n",
               ktime_to_ns(ktime_now), rockchip_timer_read(),
               tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
               tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
        udelay( 31 );
        loop --;
        }while( loop > 0 );
        rk28_debugs = 0;
        return 0x2c;
}
#if 0
/*
 * for test change arm clk from 600-594--600 ,while usb disconnect.
 *
 */
static void rk28_sleep(unsigned howlong)
{
	current->state = TASK_INTERRUPTIBLE;
	schedule_timeout(howlong);
}

int rk28_change_arm( int loops )
{
        int armclk;
        u64     cycle;
        armclk = rockchip_clk_get_arm()/1000000;
        if( armclk != 594 )
                armclk = 594;
        while( loops > 0 ) {
                cycle =  rockchip_timer_read();
                rockchip_clk_set_arm(armclk);
                cycle = rockchip_timer_read() - cycle;
                printk("[%03d]total %lld to set arm clk to %d\n" ,loops, cycle , armclk);
                armclk = ( armclk == 600 )?594:600;
                loops--;
                rk28_sleep(msecs_to_jiffies(700));
                
        }
        return loops;
}
#endif

#if 0
#include <linux/android_power.h>
static android_suspend_lock_t rk28_test_suspend_lock;
static int lock_inited = 0;
int rk28_power( int type )
{
        rk28_test_suspend_lock.name = "rk28_test_lock";
        if( lock_inited ) {
                android_uninit_suspend_lock( &rk28_test_suspend_lock );
        }
        android_init_suspend_lock( &rk28_test_suspend_lock );
        lock_inited = 1;
        switch( type ) {
        case 0:
                android_lock_partial_suspend( &rk28_test_suspend_lock );
                break;
        case 1:
                android_lock_suspend( &rk28_test_suspend_lock );
                break;
        case 2:
                android_lock_idle( &rk28_test_suspend_lock );
                break;
        default: /* just unlock */
                break;
        }
        return 0x2d;        
}
#endif
#if 1
int rk28_sym( unsigned long arg )
{
        char *sym;
        int  type = rk28_arg_string(arg) ;
        if(type == 1 ) {
                unsigned long fun;
                sym = (char*)arg;
                fun = kallsyms_lookup_name(sym);
                if( fun ) {
                        printk("%s start at 0x%lx," , sym , fun );
                        __print_symbol("%s\n" , fun+8);
                } else
                        printk("symbol %s not found \n" , sym );
        } else if( type == 2 ){
                unsigned long symbolsize;
                unsigned long offset;
                char *modname; 
                char namebuf[KSYM_NAME_LEN];
                const char *name;
                name = kallsyms_lookup(arg , &symbolsize , &offset , &modname , namebuf );
                if( name ){
                        printk("0x%lx at \"%s\" 0x%lx/0x%lx\n" , arg , name , offset ,  symbolsize);
                }
        }
        return 0x2e;
}
#endif


#if 1
/* 
 * 20091224,HSL@RK,use tool 'io' instead !
 */
 
/*
 * print register value 
 * 
*/
struct reg_index {
        char *               name;
        unsigned int * va_addr;
};
int rk28_reg( char* reg , int size )
{
        struct reg_index reg_array[] = {
                {"timer" , (unsigned int *)TIMER_BASE_ADDR_VA},
                {"scu" , (unsigned int *)SCU_BASE_ADDR_VA},
                {"intc" , (unsigned int *)INTC_BASE_ADDR_VA},
                {"gpio0" , (unsigned int *)GPIO0_BASE_ADDR_VA},
                {"gpio1" , (unsigned int *)GPIO1_BASE_ADDR_VA},
                {"regfile" , (unsigned int *)REG_FILE_BASE_ADDR_VA},
                {"usb" , (unsigned int *)USB_OTG_BASE_ADDR_VA},
                {"uart1" , (unsigned int *)UART1_BASE_ADDR_VA},
                {"",0}
        };
        struct reg_index *preg = &reg_array[0];
        int len = strlen( reg );
        unsigned int * reg_base;
        if( size == 0 )
                size = 64;
        while( preg->va_addr ) {
                if( !strnicmp( preg->name , reg , len ) ) {
                        reg_base = preg->va_addr;
                        rk28_printk_mem( reg_base , size );
                        return 0x2f;
                }
                preg++;
        }
        printk("support reg name:\n");
        preg = &reg_array[0];
        while( preg->va_addr ) {
                printk("%s " , preg->name );
                preg++;
        }
        printk("\n");
        return 0x30;
}
#endif
#if 0
/* 1,2=0 , 0 = lr */
void rk28_return( void )
{
        printk("return addr:\n 0: 0x%p " "1: 0x%p " "2: 0x%p\n", __builtin_return_address(0) , 
                __builtin_return_address(1) , __builtin_return_address(2));
}
#endif

volatile int rk28_debugs = 0;
EXPORT_SYMBOL(rk28_debugs);

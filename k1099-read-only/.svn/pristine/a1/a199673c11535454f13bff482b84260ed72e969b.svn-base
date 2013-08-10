/*
 *  rk_scu/rk_scu.c
 *
 * (C) Copyright hsl 2009
 *	Released under GPL v2.
 *
 * 
 * 
 *      
 *      
 */
#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>

#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/cdev.h>
#include <linux/io.h>
#include <linux/wait.h>
#include <linux/delay.h>

#include <linux/uaccess.h>
#include <linux/errno.h>
#include <linux/poll.h>
#include <linux/timer.h>
#include <linux/spinlock_types.h>

#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/kallsyms.h>

#include <asm/atomic.h>
#include <asm/bug.h>
#include <asm/arch/rk28_debug.h>

//#define SCU_DEBUG
#ifdef __i386__
#include "rk28_scu.h"
#else
#include <asm/arch/rk28_scu.h>
#endif
MODULE_LICENSE("Dual BSD/GPL");

#define  SCU_GET_DIV( parent_ckl , child_clk )   ((parent_ckl +child_clk -1 ) / (child_clk))
#define  SCU_ADD_NODE( p )      do {  list_add_tail( &p->list , &p->parent->child_list ); \
                                                p->parent->total_child++;\
                                                p->parent->active_child++; } while( 0 )

#define  SCU_DEL_NODE( p )      do {  list_del( &p->list ); \
                                                p->parent->total_child--;\
                                                p->parent->active_child--; } while( 0 )                                                
/* interface for scu_hw.c*/
void __rockchip_scu_init_hw( void );
int __rockchip_scu_setdiv_hw( struct rockchip_scu_unit * p );
int __rockchip_scu_enable_hw( ip_id  id , struct rockchip_scu_unit * p);
int __rockchip_scu_disable_hw( ip_id  id , struct rockchip_scu_unit * p);
int __rockchip_scu_set_pllclk_hw( struct rockchip_scu_unit * p  , int stage);
int __rockchip_scu_set_armpll_hw( struct rockchip_scu_unit * p  , int stage);
int __rockchip_scu_set_48m_hw( struct rockchip_scu_unit * p  , int stage);
int __sdram_change_ahbclk( struct rockchip_scu_unit * p  , int stage );

int __init __rockchip_scu_get_ahb_clk_hw( void );
int __init __rockchip_scu_get_apb_clk_hw( void );

 int rockchip_chg_parent_all(struct rockchip_scu_unit *ipinfo, ip_id parent ); 
 int rockchip_get_unit_div_hw( struct rockchip_scu_unit *ipinfo );
 int __rockchip_scu_get_pll_clk_hw( ip_id id );
 
/* interface for rk_kobject.c */
#if SCU_KOJB_SUPPORT                
#define __rockchip_scu_kobj_put( obj )        kobject_put( obj )
int __rockchip_scu_kset_init( void );
void __rockchip_scu_kset_exit( void );
int __rockchip_scu_node_kobject_init(struct rockchip_scu_unit *p );
#else
#define __rockchip_scu_kobj_put( obj )       
#define __rockchip_scu_kset_init( )                              0
#define __rockchip_scu_kset_exit( )
#define __rockchip_scu_node_kobject_init( p )      0
#endif

/* 
* return value: 
*       0: ok to finish travel .
*       1: continue to travel.
*       -1: failed to finish travel.
*/
typedef int (*travel_function )(struct rockchip_scu_unit * , unsigned long );
#define SCU_TRV_RETURN_OK                       0
#define SCU_TRV_RETURN_CONTINUE            1
#define SCU_TRV_RETURN_FAIL                    (-1)

#define RKSCU_INIT_IPTABLE_CPFN( ip , name ,propt, pll , divbit_start , divbit_end ,reg_idx, ifn, idx,cpfn )       \
        {ip,# name,propt, pll , divbit_start , divbit_end ,reg_idx , \
        ifn,cpfn,LIST_HEAD_INIT(scu_ip_info_table[idx].list), \
        LIST_HEAD_INIT(scu_ip_info_table[idx].child_list) }
        
#define RKSCU_INIT_IPTABLE( ip , name ,propt, pll , divbit_start , divbit_end ,reg_idx, ifn , idx )       \
        {ip,# name,propt, pll , divbit_start , divbit_end ,reg_idx , \
        ifn,NULL,LIST_HEAD_INIT(scu_ip_info_table[idx].list), \
        LIST_HEAD_INIT(scu_ip_info_table[idx].child_list) }


#define RKSCU_INIT_IPTABLE_ALL( ip , name ,propt, pll ,max_clk, divbit_start , divbit_end ,reg_idx, ifn , idx )       \
        {ip,# name, propt, pll ,  divbit_start , divbit_end ,reg_idx , \
        ifn,NULL,LIST_HEAD_INIT(scu_ip_info_table[idx].list), \
        LIST_HEAD_INIT(scu_ip_info_table[idx].child_list) , 1 , 0, 0 , max_clk }

#define RKSCU_INIT_IPTABLE_INIT( ip , name ,propt,pll, cur_clk , idx )       \
        {ip,# name, propt, pll , 0 , 0 ,INVALID_REG_INDEX , \
        NULL,NULL,LIST_HEAD_INIT(scu_ip_info_table[idx].list), \
        LIST_HEAD_INIT(scu_ip_info_table[idx].child_list),1,1,cur_clk,cur_clk }

static struct rockchip_scu_unit       scu_ip_info_table[]  = {
        
        RKSCU_INIT_IPTABLE(SCU_IPID_SDMMC0, sdmmc0 ,0,
                SCU_IPID_HCLK, 4 , 6 , 0 , NULL , 0 ),
        RKSCU_INIT_IPTABLE_CPFN(SCU_IPID_LCDC, lcdc ,SCU_PROPT_MULT_PARENT,
                SCU_IPID_CODEC, 8 , 15 ,0 , NULL , 1 ,rockchip_chg_parent_all),
        RKSCU_INIT_IPTABLE(SCU_IPID_USBPHY, usbphy ,SCU_PROPT_MULT_PARENT,
                SCU_IPID_ARM, 20, 22,0 , __rockchip_scu_set_48m_hw , 2 ),
        RKSCU_INIT_IPTABLE_CPFN(SCU_IPID_SENSOR, sensor ,SCU_PROPT_MULT_PARENT,
                SCU_IPID_ARM, 20, 22,0 , __rockchip_scu_set_48m_hw,3,rockchip_chg_parent_all),
        RKSCU_INIT_IPTABLE(SCU_IPID_SDMMC1, sdmmc1 ,0,
                SCU_IPID_HCLK, 25, 27, 0 , NULL , 4 ),
        RKSCU_INIT_IPTABLE(SCU_IPID_LSADC, lsadc , 0,
                SCU_IPID_PCLK, 8, 15, 1 , NULL , 5 ),
        RKSCU_INIT_IPTABLE(SCU_IPID_DEMOD, demodul , SCU_PROPT_PARERNT,
                SCU_IPID_DSP, 16, 23, 1 , NULL , 6 ),

        /* global clk */
        RKSCU_INIT_IPTABLE(SCU_IPID_PCLK, apbclk ,SCU_PROPT_PARERNT,
                SCU_IPID_HCLK, 2 , 3 , 0 , NULL , 7 ),
                
#ifdef SDRAM_MSDRAM
        RKSCU_INIT_IPTABLE_ALL(SCU_IPID_HCLK, ahbclk ,SCU_PROPT_PARERNT,
                SCU_IPID_ARM, SCU_MAX_AHB_CLK, 0 , 1 , 0 , NULL , 8 ),
#else
        RKSCU_INIT_IPTABLE_ALL(SCU_IPID_HCLK, ahbclk ,SCU_PROPT_PARERNT,
                SCU_IPID_ARM, SCU_MAX_AHB_CLK, 0 , 1 , 0 , __sdram_change_ahbclk , 8 ),
#endif
                
        /* MAX HCLK<=155 , MAX ARM = 155*3=465 */        
        RKSCU_INIT_IPTABLE_ALL(SCU_IPID_ARM, armclk ,SCU_PROPT_PARERNT|SCU_PROPT_MAXFREQ|SCU_PROPT_FIXDIV|SCU_PROPT_HAVEPLL,
                SCU_IPID_24M, SCU_MAX_ARM_CLK*SCU_CLK_UNIT , 0 , 0 ,INVALID_REG_INDEX, __rockchip_scu_set_armpll_hw , 9 ),

        RKSCU_INIT_IPTABLE_ALL(SCU_IPID_DSP, dspclk ,SCU_PROPT_PARERNT|SCU_PROPT_MAXFREQ|SCU_PROPT_FIXDIV|SCU_PROPT_HAVEPLL,
                SCU_IPID_24M, 520*SCU_CLK_UNIT ,0 , 0 ,INVALID_REG_INDEX , __rockchip_scu_set_pllclk_hw , 10 ),
                
        RKSCU_INIT_IPTABLE_ALL(SCU_IPID_CODEC, codecclk ,SCU_PROPT_PARERNT|SCU_PROPT_MAXFREQ|SCU_PROPT_FIXDIV|SCU_PROPT_HAVEPLL,
                SCU_IPID_24M, 600*SCU_CLK_UNIT , 0, 0, INVALID_REG_INDEX , __rockchip_scu_set_pllclk_hw , 11 ), /* parent may be: 24M,rtc clk. */
                
        RKSCU_INIT_IPTABLE_INIT(SCU_IPID_12M, init12m ,SCU_PROPT_PARERNT|SCU_PROPT_BYPASS|SCU_PROPT_FIXDIV,
                SCU_IPID_24M, 12*SCU_CLK_UNIT , 12 ),
                
        RKSCU_INIT_IPTABLE_INIT(SCU_IPID_24M, init24m ,SCU_PROPT_PARERNT|SCU_PROPT_BYPASS|SCU_PROPT_FIXDIV,
                SCU_IPID_MAX , 24*SCU_CLK_UNIT , 13 ),
        RKSCU_INIT_IPTABLE_CPFN(SCU_IPID_I2S, i2s ,SCU_PROPT_MULT_PARENT,
                SCU_IPID_12M, 3, 7, 1 , NULL , 14, rockchip_chg_parent_all),                      
};
#define SCU_TABLE_LASTIP        SCU_IPID_I2S

struct rockchip_scu_system
{
        struct rockchip_scu_unit        *root;
        spinlock_t                spinlock;     /* lock for this struct. spin_lock_init */ 
};

static struct rockchip_scu_system  scu_system_clkinfo =
{
         &scu_ip_info_table[13],        /* 24m clk as root , not support extern clk */
};
static int rockchip_scu_suspend_mode = 0;

static struct rockchip_scu_unit * __rockchip_find_unit_byipid( ip_id id )
{
        struct rockchip_scu_unit       *p = &scu_ip_info_table[0];
        while(1){
                if( p->id == id )
                        return p;
                if( p->id == SCU_TABLE_LASTIP )
                        break;
                p++;
        }
        return NULL;
}

struct rockchip_scu_unit * __rockchip_find_unit_at_node( struct rockchip_scu_unit *node , ip_id id )
{
        struct rockchip_scu_unit *t;
        struct list_head *pos;
        struct rockchip_scu_unit *pt;

        list_for_each(pos, &node->child_list) {
        	t = list_entry(pos, struct rockchip_scu_unit, list);
        	if( t->id == id )
        		return t;
             if( t->propt & SCU_PROPT_PARERNT ){
                pt = __rockchip_find_unit_at_node( t , id );
                if( pt )
                        return pt;
             }
                
        }
        return NULL;
}

static inline struct rockchip_scu_unit *__rockchip_get_clkset_parent( struct rockchip_scu_unit * p )
{
        while( !(p->propt & SCU_PROPT_HAVEPLL) )
                p = p->parent;
        BUG_ON( p == NULL );
        return p;
}

/* two way: parent first or child first .
* return : 
*       SCU_TRV_RETURN_FAIL: failed 
*       SCU_TRV_RETURN_OK or SCU_TRV_RETURN_CONTINUE: ok.
*/
static int __rockchip_scu_travel_tree( struct rockchip_scu_unit *node ,int parent_first, travel_function tvf , unsigned long arg )
{
        struct rockchip_scu_unit *t;
        struct list_head *pos;
        int      ret;
        list_for_each(pos, &node->child_list) {
                t = list_entry(pos, struct rockchip_scu_unit, list);
                if( parent_first )  {
                     ret = tvf( t, arg ) ;
                     if( ret == SCU_TRV_RETURN_OK )
                	        return ret;
                     if( ret == SCU_TRV_RETURN_FAIL)
                	        return ret;
                     if( t->propt & SCU_PROPT_PARERNT ){
                        ret = __rockchip_scu_travel_tree( t ,parent_first , tvf , arg );
                        if( ret == SCU_TRV_RETURN_OK )
                	        return ret;
                        if( ret == SCU_TRV_RETURN_FAIL)
                	        return ret;
                     } 
                } else  {                        
                        if( t->propt & SCU_PROPT_PARERNT ){
                        ret = __rockchip_scu_travel_tree( t ,parent_first, tvf , arg );
                        if( ret == SCU_TRV_RETURN_OK )
                	        return ret;
                        if( ret == SCU_TRV_RETURN_FAIL)
                	        return ret;
                        } 
                        ret = tvf( t, arg ) ;
                        if( ret == SCU_TRV_RETURN_OK )
                	        return ret;
                        if( ret == SCU_TRV_RETURN_FAIL)
                	        return ret;
                }
        }
        return SCU_TRV_RETURN_CONTINUE; /* 递归调用下，需要返回该值*/
}

/* FIXME! may need enable p->parent , parent's parent ...? */
static int __rockchip_scu_enable_clk( struct rockchip_scu_unit * p  )
{
        if( p->propt & SCU_PROPT_DISABLE ) {
                if( p->parent  ) {
                        spin_lock( &scu_system_clkinfo.spinlock );
                        p->parent->active_child++;
                        spin_unlock( &scu_system_clkinfo.spinlock );
                        if( (p->parent->propt & SCU_PROPT_DISABLE) )
                                __rockchip_scu_enable_clk( p->parent );
                }
                SCU_BUG("scu enable clk %s" , SCU_SYSFS_NAME(p) );
                p->propt &=~ SCU_PROPT_DISABLE;
                __rockchip_scu_enable_hw( p->id , p );
        }
        return 0;
}

static int __rockchip_scu_disable_clk( struct rockchip_scu_unit * p  )
{
        if( !(p->propt & SCU_PROPT_DISABLE) ) {
                SCU_BUG("scu disable clk %s" , SCU_SYSFS_NAME(p) );
                p->propt |= SCU_PROPT_DISABLE;
                __rockchip_scu_disable_hw( p->id , p );
                if( p->parent ) {
                        int ac ;
                        spin_lock( &scu_system_clkinfo.spinlock );
                        ac = --p->parent->active_child;
                        spin_unlock( &scu_system_clkinfo.spinlock );
                        if( ac == 0 )
                                __rockchip_scu_disable_clk( p->parent );
                }
        }
        return 0;
}

/* set the real clk ,include pll , div , user callback */
static int __rockchip_clk_set_unit_clk_single( struct rockchip_scu_unit *p , unsigned long arg)
{
        p->propt &= ~SCU_PROPT_CLKSET;
        /*
         * 20090721,maybe tmp_clk == cur_clk,but tmp_div != cur_div ,such as adc , fix output 1MHZ.
         */
         if( SCU_UNIT_HAVEDIV(p) &&  p->tmp_div != p->cur_div ) {
                if ( __rockchip_scu_setdiv_hw( p  ) < 0 )
                        return -SCU_TRV_RETURN_FAIL; 
                p->cur_div = p->tmp_div;
        }
        SCU_BUG("set %s clk from %d to %d\n" , p->name , p->cur_clk , p->tmp_clk );
        if( p->tmp_clk != p->cur_clk ) {
                if( p->ifn )
                        p->ifn( p , 1);
                p->cur_clk = p->tmp_clk;
                if( p->ofn )
                        p->ofn( p->id , p->cur_clk*SCU_CLK_UNIT ); /* to hz */
        }  
        return SCU_TRV_RETURN_CONTINUE;
}
/*
 * get the max div which could be divisable.
 */
static int inline __rockchip_unit_get_max_div( struct rockchip_scu_unit *p )
{
        int div = SCU_DIV_MAXVALUE(p);
        int parent_clk ;
        while ( div > 1 ) {
                parent_clk = p->parent->tmp_clk;
                do{
                    parent_clk -= div ;
                } while ( parent_clk > 0 );
                if( !parent_clk )
                        break;
                div--;
        }
        return div;
}
/* check if we can set the new clk from parent .
*  set tmp_div , tmp_clk.
* have SCU_PROPT_FIXDIV property need do nothing.
*/
static int __rockchip_clk_checkforset_unit_clk_single( struct rockchip_scu_unit *p , unsigned long arg)
{
        if( !(SCU_PROPT_CLKSET & p->propt ) ) {
                p->tmp_div = p->cur_div;
                if( SCU_UNIT_HAVEDIV(p) ) {
                        if( rockchip_scu_suspend_mode )
                                p->tmp_div = __rockchip_unit_get_max_div( p );
                        else if ( (p->propt & SCU_PROPT_MAXFREQ) ) {
                                //if( p->parent->tmp_clk/p->cur_div > p->max_clk )
                                        p->tmp_div = SCU_GET_DIV(p->parent->tmp_clk,p->max_clk );
                        }
                }
                if( p->tmp_div != p->cur_div ) {
                        if( p->tmp_div > SCU_DIV_MAXVALUE(p) ) {
                                SCU_BUG("set ip[%s] div=%d , max=%d" , p->name , p->tmp_div , SCU_DIV_MAXVALUE(p) );
                                return SCU_TRV_RETURN_FAIL; 
                        }
                        /* 20091222,HSL@RK,APB CLK div=1,2,4, NOT SUPPORT 3*/
                        /* 20100302,HSL@RK,APB TOO LOW,SOME clk will failed!!
                          * like timer,when apb clk=1.5MHz,timer count error!!
                        */
                        if(p->id == SCU_IPID_PCLK && p->tmp_div >= 3 ) {
                                        p->tmp_div = 2;
                        }
                }
                /* get the tmp clk for its childs */
                p->tmp_clk =  p->parent->tmp_clk / p->tmp_div;
                 if( p->tmp_clk != p->cur_clk ) {
                        if( p->ifn )
                                p->ifn( p , 0);
                }     
          //      SCU_BUG("set ip[%s] div=%d , tmp_clk=%d , paren tmp_clk=%d\n" , p->name , p->tmp_div , 
          //              p->tmp_clk , p->parent->tmp_clk );
       }
        return SCU_TRV_RETURN_CONTINUE;
}


static int __rockchip_clk_try_to_set_unit_clock(struct rockchip_scu_unit *p )    
{
        if( p->cur_clk == p->tmp_clk ) 
                return SCU_SETCLK_OK;
        if( (p->propt & (SCU_PROPT_BYPASS)) )
                return SCU_SETCLK_IMPOSSIBLE;
        if( p->tmp_clk > p->max_clk && p->tmp_clk < SCU_CLK_SPECIAL) {
                SCU_BUG("change %s clk[%d] > set max clk[%d]" , SCU_SYSFS_NAME(p) , p->tmp_clk , p->max_clk );
                return SCU_SETCLK_IMPOSSIBLE;
        }
        
        if( p->propt&SCU_PROPT_HAVEPLL ) {
                if(  p->ifn( p , 0) == SCU_SETCLK_OK )
                        return SCU_SETCLK_OK;
                return SCU_SETCLK_IMPOSSIBLE;
        }

        /* get help from parent , look for div */
        if( !p->parent )
                return SCU_SETCLK_IMPOSSIBLE;
                        
        if(p->propt & SCU_PROPT_FIXDIV) {
                p->parent->tmp_clk = p->tmp_clk*p->cur_div;
                return SCU_SETCLK_PARENT;
        }

        /* 寻找一个最大可能频率，需要逐级判断父节点的最大可设频率*/
        /* 如果直接分频可以达到目的，则不需要修改parent CLK, change_mode 函数属于该种情况*/
        p->tmp_div = p->parent->cur_clk / p->tmp_clk;
        if( p->tmp_div * p->tmp_clk == p->parent->cur_clk ) {
                if( p->tmp_div <= SCU_DIV_MAXVALUE(p) ) 
                        return SCU_SETCLK_OK;
        }
        /* 20091225,HSL@RK,keep div not change if possible */
        p->tmp_div = p->parent->max_clk / p->tmp_clk ; 
        if( p->tmp_div > SCU_DIV_MAXVALUE(p) )
                p->tmp_div = SCU_DIV_MAXVALUE(p) ;
        p->parent->tmp_clk = p->tmp_div * p->tmp_clk;
        return SCU_SETCLK_PARENT;
}
int __rockchip_scu_set_unit_div( ip_id id , int new_div )  
{
        struct rockchip_scu_unit *p ;
        
        p = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , id );
        if( !p ) {               /* for find error , can not register again */
                SCU_BUG("failed to find ip[%d]" , id );
                return -EIO;
        }
        
        /* 20100301,HSL@RK,APB CLK div=1,2,4, NOT SUPPORT 3*/
        if(p->id == SCU_IPID_PCLK && new_div >= 3 ) {
                        new_div = 2;
        }
        if( p->cur_div == new_div ) 
                return 0;
        if( p->parent->cur_clk/new_div > p->max_clk )
                return -EINVAL;
        p->tmp_div = new_div;
        p->tmp_clk = p->parent->cur_clk / p->tmp_div;
        //SCU_BUG("scu set unit %s div=%d " , p->name , p->tmp_div );
        if( __rockchip_clk_set_unit_clk_single( p , p->tmp_clk ) == SCU_TRV_RETURN_FAIL )
                return -EIO;
        return 0;
}

/*
* two way: set pll , or change div , up to root parent.
* need to check lock , sem lock.
*/
int __rockchip_clk_set_unit_clock( ip_id id , int new_clk )    /* SCU_IPID_ARM */
{
        struct rockchip_scu_unit *p ,*parent , *semParent;
        int parent_first;
        int     res = 0;
        
        p = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , id );
        if( !p ) {               /* for find error , can not register again */
                SCU_BUG("failed to find ip[%d]" , id );
                return -EIO;
        }
        if( new_clk < SCU_CLK_SPECIAL )
                new_clk *= SCU_CLK_MHZ2KHZ;
        if( p->max_clk < new_clk && new_clk < SCU_CLK_SPECIAL ) 
                return -EINVAL;
        if( p->cur_clk == new_clk ) {
	        return 0;
        }

        SCU_BUG("set %s to clk %d" , p->name , new_clk );
        /* check lock & sem */
        semParent =  __rockchip_get_clkset_parent( p );
        if( semParent->propt & SCU_PROPT_LOCKED )
                return -EINTR;
        
        p->bak_clk = p->cur_clk;
        if(new_clk >= SCU_CLK_SPECIAL ) {
                if(  !(p->propt&SCU_PROPT_HAVEPLL) )
                    return -EINVAL;
                p->bak_clk = new_clk;
                new_clk /= (SCU_CLK_SPECIAL/SCU_CLK_MHZ2KHZ);
        }
#if SCU_SEM                 
        down_interruptible( &semParent->clk_sem );
#endif        
        p->tmp_clk = new_clk;
        p->propt |=  SCU_PROPT_CLKSET;
        parent = p;
        while ( (res = __rockchip_clk_try_to_set_unit_clock( parent ) ) 
                ==  SCU_SETCLK_PARENT )  {
                parent->propt |=  SCU_PROPT_CLKSET;
                parent = parent->parent ;
        }
        SCU_BUG("set scu %s clk to %d , need parent = %s\n" , p->name , p->tmp_clk , parent->name );
        if( res ==  SCU_SETCLK_IMPOSSIBLE ) {
                printk("can not set %s to clk %d" , SCU_SYSFS_NAME(p) , new_clk );
                res = -EIO;
                goto clearup_clkset;
        }
        parent->propt |=  SCU_PROPT_CLKSET;
        
        /* we chang clk from parent */
        if( __rockchip_scu_travel_tree( parent , 1, __rockchip_clk_checkforset_unit_clk_single , parent->tmp_clk ) == SCU_TRV_RETURN_FAIL) {
                res = -EIO;
                goto clearup_clkset;
        }
        res = 0;
        /* clk decrease , parent first , clk inscrease , child first */
        parent_first = (parent->tmp_clk < parent->cur_clk);
        if( parent_first )
                __rockchip_clk_set_unit_clk_single( parent , parent->tmp_clk );

        /*XXX: can not be failed */
        __rockchip_scu_travel_tree( parent ,parent_first , __rockchip_clk_set_unit_clk_single , parent->tmp_clk );

        if( !parent_first )
                __rockchip_clk_set_unit_clk_single( parent , parent->tmp_clk );
clearup_clkset:
        while( p->propt&SCU_PROPT_CLKSET ) {
                p->propt &= ~SCU_PROPT_CLKSET;
                p = p->parent;
        }
#if SCU_SEM                         
        up( &semParent->clk_sem ) ;
#endif        
        return res;
}



int rockchip_scu_disableclk( ip_id id )
{
        struct rockchip_scu_unit *p ;
        p = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , id );
        //SCU_BUG("scu disable clk,id= %d,node=%p" , id , p);
        if( p ) 
                __rockchip_scu_disable_clk( p );
        else
                __rockchip_scu_disable_hw( id , p);
        return 0;
}
int  rockchip_scu_enableclk( ip_id id )
{
        struct rockchip_scu_unit *p ;
        p = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , id );
        //SCU_BUG("scu enable clk,id= %d,node=%p" , id , p);
        if( p ) 
                __rockchip_scu_enable_clk( p );
        else
                __rockchip_scu_enable_hw( id , p );        
        return 0;
}

/*
* can not lock more then one time.
*/
int rockchip_clk_lock_pll( ip_id id )
{
        struct rockchip_scu_unit *p ;        
        p = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , id );
        if( p ) {
                p =  __rockchip_get_clkset_parent( p );
                if( p->propt & SCU_PROPT_LOCKED )
                        return -EPERM;
                spin_lock( &scu_system_clkinfo.spinlock );
                p->propt |= SCU_PROPT_LOCKED;
                p->lock_ipid = id;
                spin_unlock( &scu_system_clkinfo.spinlock );
        }
        return 0;
}
int rockchip_clk_unlock_pll( ip_id id )
{
        struct rockchip_scu_unit *p ;        
        p = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , id );
        if( p ) {
                p =  __rockchip_get_clkset_parent( p );
                if( !(p->propt & SCU_PROPT_LOCKED) || p->lock_ipid != id )
                        return -EPERM;
                spin_lock( &scu_system_clkinfo.spinlock );
                p->propt &= ~SCU_PROPT_LOCKED;
                spin_unlock( &scu_system_clkinfo.spinlock );
        }
        return 0;
}

#if SCU_SEM                 
int rockchip_clk_semlock_pll( ip_id id )
{
        struct rockchip_scu_unit *p ;        
        p = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , id );
        if( p ) {
                p =  __rockchip_get_clkset_parent( p );
                down_interruptible( &p->clk_sem );
                p->lock_ipid = id; /* for bug info */
        }
        return 0;
      
}
int rockchip_clk_semunlock_pll( ip_id id )
{
        struct rockchip_scu_unit *p ;        
        p = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , id );
        if( p ) {
                p =  __rockchip_get_clkset_parent( p );
                /* 20090717,for unit unlock many times */
                if( p->lock_ipid != SCU_IPID_MAX ) {
                        p->lock_ipid = SCU_IPID_MAX;
                        up( &p->clk_sem );
                }
        }
        return 0;
        
}
#endif

int __rockchip_clk_get_uint_clk( ip_id id )
{
        int     clk;
        struct rockchip_scu_unit *p ; //,  *semParent;        
        p = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , id );
        if( !p )
                return 0;
        
        /* check sem for not get tmp value */
        //semParent =  __rockchip_get_clkset_parent( p );
        //down_interruptible( &semParent->clk_sem );
        clk = p->cur_clk ;
        //up( &semParent->clk_sem );
        return clk*SCU_CLK_UNIT; /* TO HZ */
}
int rockchip_clk_get_ahb( void )
{
        int a = __rockchip_clk_get_uint_clk( SCU_IPID_HCLK );     
        /* can not use this after init .*/
        if( !a )
                return __rockchip_scu_get_ahb_clk_hw();
        return a;
}
int rockchip_clk_get_apb( void )
{
        int a = __rockchip_clk_get_uint_clk( SCU_IPID_PCLK );
        /* can not use this after init .*/
        if( !a )
                return __rockchip_scu_get_apb_clk_hw();
        return a;
}
int rockchip_clk_get_arm( void )
{
        int a = __rockchip_clk_get_uint_clk( SCU_IPID_ARM );

        /* can not use this after init .*/
        if( !a )
                return __rockchip_scu_get_pll_clk_hw( SCU_IPID_ARM );
        return a;
}

#if SCU_SEM                 
int rockchip_clk_get_ipsource( ip_id id )
{
        int     clk;
        struct rockchip_scu_unit *p ,  *semParent;        
        p = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , id );
        if( !p )
                return 0;
        
        semParent =  __rockchip_get_clkset_parent( p );
        down_interruptible( &semParent->clk_sem );
        clk = p->parent->cur_clk ;
        up( &semParent->clk_sem );
        return clk*SCU_CLK_UNIT; /* TO HZ */
}
#endif


int rockchip_clk_set_arm( int new_clk )
{
        struct rockchip_scu_unit *p =  __rockchip_find_unit_at_node( scu_system_clkinfo.root  , SCU_IPID_ARM );
        if( !p )
                return -EPERM;
        if( new_clk <= 24 ) {
                SCU_BUG("set arm 24M,enter suspend mode");
                /* if suspend mode,when deep sleep,freq=ARM:24 AHB:6,APB:3.
                 *  not suspend mode,freq=ARM:24 AHB:24 APB:24.
                 *  this make about 0.5ma power consume diffirence at level2 sleep.
                 */
                rockchip_scu_suspend_mode = 1 ; // 1 ; // 0;
                new_clk = 24;
        } else {
                rockchip_scu_suspend_mode = 0;
        }
        #if 0
        {
        unsigned int     algn = 8;       /* normal , ahp_div = 2,4 */
        /*
        *  20090710,timer need pclk must be 1M round,so 
        *  change new_clk to 24 mutilple.
        *  24 = ahb_div(3)*apb_div(2) || ahb_div(4)*apb_div(2)
        *  20090720,for ctrl ip clk , do more little here.
        */
        
        if( new_clk >= SCU_MAX_AHB_CLK*2 && new_clk <= SCU_MAX_AHB_CLK*3 ) 
                algn = 6;  /*  ahp_div = 3 */
        else if (new_clk < SCU_MAX_AHB_CLK*2 )
                algn = 4; /* ahp_div = 2 */
        new_clk = (new_clk+algn-1)/algn;
        new_clk *= algn;
        while( new_clk > p->max_clk ) {
                new_clk -= algn;
                }
        if( new_clk <= 24 ) {
                SCU_BUG("set arm 24m,enter suspend mode");
                rockchip_scu_suspend_mode = 1;
                new_clk = 24;
        } else 
                rockchip_scu_suspend_mode = 0;
        }
        #endif
        
        return __rockchip_clk_set_unit_clock(SCU_IPID_ARM , new_clk);
}

int __rockchip_change_mode_ipinfo( struct rockchip_scu_unit *p , 
    unsigned char scu_mode , int parm )
{
        SCU_BUG("change %s to Mode %d,parm=%d" , SCU_SYSFS_NAME(p) , scu_mode , parm );
        if( (p->propt & (SCU_PROPT_BYPASS)) )
                return 0;

         if( scu_mode == SCU_MODE_DIV ){
                if( (p->propt & SCU_PROPT_FIXDIV) && p->cur_div == parm )
                        return 0;
                if( (p->propt & SCU_PROPT_FIXDIV) && p->divreg_index > 1 ) { /* NO REAL DIV CTRL */
                        SCU_BUG("set ip[%s] div=%d ,but no div ctrl!" , SCU_SYSFS_NAME(p), parm);
                        return -EPERM;
                }
                p->propt |= SCU_PROPT_FIXDIV;
                p->tmp_div = parm;
        } else if ( scu_mode == SCU_MODE_FREQ){
                if( !(p->propt&SCU_PROPT_HAVEPLL) && parm > p->parent->max_clk )
                        return -EPERM;
                parm*=SCU_CLK_MHZ2KHZ;
                p->propt |= SCU_PROPT_MAXFREQ;
                p->max_clk = parm;
                p->tmp_div = SCU_GET_DIV(p->parent->cur_clk,parm);
        } else if( SCU_MODE_SETDIV == scu_mode ) {
                p->tmp_div = parm;
        }  
        
        if ( p->tmp_div == 0 )
                p->tmp_div = 1;

        if( !(p->propt & SCU_PROPT_MAXFREQ) ) {
                if( (p->propt & SCU_PROPT_FIXDIV) ) 
                        p->max_clk = p->parent->max_clk / p->tmp_div;
                else
                        p->max_clk = p->parent->max_clk;        /* 1:1 from parent */
        } 
        
        /* need to check div valid */
        if( SCU_UNIT_HAVEDIV(p) && p->tmp_div > SCU_DIV_MAXVALUE(p) ) {
                SCU_BUG("set ip[%s] div=%d , max=%d" , SCU_SYSFS_NAME(p),
                        p->tmp_div , SCU_DIV_MAXVALUE(p) );
                return -EPERM; 
        }

        /* need to set tmp clk to the right value */
        /* 20100316,HSL@RK,only SCU_MODE_SETFREQ will set UNIT clk = param.
         *  other ,not change parent clk,only change div.
         */
        if( scu_mode == SCU_MODE_NONE ) {
                /* HSL@RK,20090905 ,if init clk > set max clk */
                SCU_BUG("%s cur clk=%d,max_clk=%d\n" , p->name , p->cur_clk , p->max_clk );
                if( p->cur_clk > p->max_clk ) {  
                        p->tmp_clk = p->max_clk;
                        __rockchip_clk_set_unit_clk_single( p , p->tmp_clk);        
                }
        } else if( SCU_MODE_SETFREQ == scu_mode ) 
                        __rockchip_clk_set_unit_clock( p->id , parm );
        else /*if( scu_mode == SCU_MODE_DIV ||scu_mode == SCU_MODE_SETDIV )*/{
                __rockchip_scu_setdiv_hw( p );
                p->cur_div = p->tmp_div;
                if( !(p->propt&SCU_PROPT_MULT_PARENT) || (p->parent->propt&SCU_PROPT_HAVEPLL) )
                        p->cur_clk = p->parent->cur_clk/p->cur_div;
                else {
                        p->cur_clk = p->parent->cur_clk;
                        p->max_clk = p->parent->max_clk;/* for i2s parent=12M.NO DIV CTRL NOW*/
                 }
                 SCU_BUG("set %s div=%d,cur clk=%d\n" , p->name , p->cur_div,p->cur_clk);
        } 
        return 0;
}

int __rockchip_scu_change_mode( ip_id id , unsigned char scu_mode , int parm )
{
        struct rockchip_scu_unit *p = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , id );
        if( !p )
            return -EINVAL;
        return __rockchip_change_mode_ipinfo(p,scu_mode,parm);
}
static int __rockchip_scu_register_node(struct rockchip_scu_unit *p, unsigned char scu_mode , int parm  , change_clk fn )
{
        if( p->propt & SCU_PROPT_HAVEPLL ) {
        #if SCU_SEM                 
                sema_init( &p->clk_sem , 1 );
        #endif
                p->lock_ipid = SCU_IPID_MAX;
        } 
       if( __rockchip_scu_node_kobject_init( p ) < 0 ) {
                SCU_BUG("scu %s create kobject failed\n" , p->name );
                return -EPERM; 
       }
       if( p->propt & SCU_PROPT_BYPASS ) {
                SCU_ADD_NODE( p );
                goto not_need_set;
        }
        //if( fn ) {
        //        printk("%s::%s,",__func__ , p->name );
        //        __print_symbol("fn=%s\n" , (unsigned long)fn );
        //}
        spin_lock( &scu_system_clkinfo.spinlock );

        /* add tail , for fast search */
        SCU_ADD_NODE( p );
        p->ofn = fn;
        if( SCU_UNIT_HAVEDIV(p) ) {
                p->cur_div = rockchip_get_unit_div_hw( p );  
                p->cur_clk = p->parent->cur_clk / p->cur_div;
        }else if(p->propt&SCU_PROPT_HAVEPLL) {
                p->cur_clk = __rockchip_scu_get_pll_clk_hw(p->id)/SCU_CLK_UNIT;
                SCU_BUG("scu %s ,read pll clk=%d MHz\n" , p->name , p->cur_clk );
        } else {
                p->cur_clk = p->parent->cur_clk;
        }
        if( __rockchip_change_mode_ipinfo( p , scu_mode , parm ) )
                goto set_failed;
        /* make it enable , ifn meas it can enable itself */
        if ( !p->ifn ) {
                __rockchip_scu_enable_clk( p->parent );
                __rockchip_scu_enable_hw( p->id , p ); /* hw enable */
        }
        /* 20100309,HSL@RK,CALL ofn for node p initialization */
        if( p->ofn ) {
                p->ofn( p->id , p->cur_clk*SCU_CLK_UNIT);
        }
        return 0;
set_failed:        
        SCU_BUG("register %s failed at change node mode\n" , p->name );
        spin_unlock( &scu_system_clkinfo.spinlock );
        kobject_del(&p->kobj);
        SCU_DEL_NODE( p );
not_need_set:        
        return 0;
}
 
int rockchip_scu_register( ip_id id , unsigned char scu_mode , int parm  , change_clk fn )
{
        struct rockchip_scu_unit *parent = NULL;
        struct rockchip_scu_unit *p ;

        p = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , id );
        if( p ) {               /* for find error , can not register again */
                SCU_BUG("ip[%d] register again" , id );
                return -EPERM;
        }
        p = __rockchip_find_unit_byipid( id );
        if( !p ) {
                SCU_BUG("register scu failed:can not find ip[%d] " , id );
                return -ENODEV;
        }
        if( scu_system_clkinfo.root->id == p->id_parent )
                parent = scu_system_clkinfo.root;
        else 
                parent = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , p->id_parent );
        if( !parent ) {
                SCU_BUG("must register %s parent first" , p->name );
                return -EPERM;
        }
        
        /* need to check parent valid */
        if( !(parent->propt & SCU_PROPT_PARERNT) ) {
                SCU_BUG("can not register %s to %s" , p->name /*SCU_SYSFS_NAME(p) */, 
                        SCU_SYSFS_NAME(parent));
                return -EPERM;
        }
        
        p->parent = parent ;
        return __rockchip_scu_register_node(p , scu_mode , parm , fn );
        
}

int rockchip_scu_apbunit_register( ip_id id , char* name , change_clk fn )
{
        struct rockchip_scu_unit *p ,*parent;
        int     res;
        if( id >= SCU_IPID_GATE_MAX )
                return -EPERM;
        
        /* XXX:must need a call back?*/
        /* 
        if( !fn  )
                return -EPERM;
        */
        
        p = kzalloc( sizeof(*p), GFP_KERNEL );
        if( !p )
                return -ENOMEM;
        SCU_BUG("register scu unit %s\n" , name );
        parent = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , SCU_IPID_PCLK );
        BUG_ON( !parent);      /* must have parent */
        p->parent = parent;
        p->id = id ;
        p->name = name ;
        INIT_LIST_HEAD(&p->child_list);
        INIT_LIST_HEAD(&p->list);
        p->propt = SCU_PROPT_FIXDIV;
        p->cur_div = 1;
        p->divreg_index = INVALID_REG_INDEX;
        res =  __rockchip_scu_register_node(p , SCU_MODE_NONE, 1 , fn );
        if( res < 0 ) {
                printk("scu register %s FAILED" , name );
                kfree(p);
        }
        return res;
}
int __rockchip_scu_set_parent( ip_id id ,  ip_id id_parent , int idmode , int parm )
{
        int ret;
        struct rockchip_scu_unit *p ;
        struct rockchip_scu_unit *nparent ;
        p = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , id );
        if( !p || !p->cpfn )
                return -EINVAL; /* not support currently */
        if( p->parent && p->parent->id == id_parent )
                return 0;
        nparent = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , id_parent );
        if( !nparent )
                return -EINVAL;
        spin_lock( &scu_system_clkinfo.spinlock );
        if(p->cpfn( p , id_parent ) ) {
                ret = -EIO;
                goto out_here;
        }
        ret = kobject_move( &p->kobj , &nparent->kobj );
        if( ret )
                goto out_here;
        SCU_DEL_NODE( p );
        p->parent = nparent;
        SCU_ADD_NODE(p);

        /* 20100208,HSL@RK,reset the max_clk. */
        p->max_clk = p->parent->max_clk;        /* 1:1 from parent */
        /* 20100112,HSL@RK,update new clk for the new parent.*/
        if( SCU_UNIT_HAVEDIV(p) ) {
                p->cur_div = rockchip_get_unit_div_hw( p );  
                p->cur_clk = p->parent->cur_clk / p->cur_div;
                if( (p->propt & SCU_PROPT_FIXDIV) ) 
                        p->max_clk = p->parent->max_clk/ p->cur_div;
        }else {
                p->cur_clk = p->parent->cur_clk;
        }
        if( idmode != SCU_MODE_NONE )
                ret = __rockchip_change_mode_ipinfo( p , idmode , parm );
out_here:        
        spin_unlock( &scu_system_clkinfo.spinlock );
        return ret;
}

#ifdef SCU_DEBUG
static int __rockchip_scu_print_unit_info(struct rockchip_scu_unit *p , unsigned long arg) 
{
        BUG_ON( !p->parent);
        SCU_BUG("%s:parent=%s,div=%d,clk=%d,max=%d,TC=%d,AC=%d" , 
                SCU_SYSFS_NAME(p) , SCU_SYSFS_NAME(p->parent) , p->cur_div ,
                p->cur_clk , p->max_clk , p->total_child , p->active_child );
        return SCU_TRV_RETURN_CONTINUE;
}
void __rockchip_scu_print_tree( void )
{
        SCU_BUG("rockchip scu tree list:");
        __rockchip_scu_travel_tree( scu_system_clkinfo.root , 1, __rockchip_scu_print_unit_info , 0) ;
}

static int __rockchip_dummy_clk_change( ip_id ip , int input_clk )
__attribute__ ((unused));
static int __rockchip_dummy_clk_change( ip_id ip , int input_clk )
{
        SCU_BUG("scu usr callback,id=%d,clk=%d" , ip , input_clk );
        return 0;
}
#endif


#define SYNC_AHB_CLK                     0 /* 20100129,HSL@RK,dynamic change ahb clk*/

#define SYNC_TIME                             65 /* 5ms aligned */
#define SYNC_STEP                             10  /* hz,AHB*/
#define SYNC_RATE_DOWN                (1/4+1/16) // 23(80M)  /* when idle task run time >= % */
#define SYNC_RATE_UP                      (SYNC_RATE_DOWN/2)  /* when idle task run time < % */
#define SYNC_AHB_MIN_CLK             40/* MIN AHB CLK,for usb connect */

#define SYNC_LOOP_MYCHANGE        73 /* change freq = 4745ms */
#define SYNC_LOOP_EMERGE             43 // 41(70Mwhen20%) // SYNC_LOOP_MYCHANGE/5
#define SYNC_LOOP_UPNOW              15 // SYNC_LOOP_MYCHANGE/5

#if SYNC_AHB_CLK
#include <linux/pm.h>
#include <linux/workqueue.h>
#include <asm/irq_regs.h>

static u64 scu_idle_ns = 0;  // ktime_get();
int   scu_sync_ahb = 0;
static struct timer_list  scu_timer;
static ktime_t scu_ktime;
static int scu_loops;
static u64 scu_idle_ns_avg = 0;  // ktime_get();
static int  in_idle_loop;
void __rockchip_clk_change( ip_id id , int clk_offset)
{
        struct rockchip_scu_unit *p;
        
        p = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , id );
        if( !p ) {               /* for find error , can not register again */
                SCU_BUG("failed to find ip[%d]" , id );
                return ;
        }
        p->tmp_clk = p->cur_clk + clk_offset*SCU_CLK_UNIT;
        if(  p->tmp_clk > p->max_clk )
                p->tmp_clk = p->max_clk;
        printk("%d\n" , p->tmp_clk );
        __rockchip_clk_set_unit_clock( id , p->tmp_clk/SCU_CLK_MHZ2KHZ );
}
void __scu_up_ahb_clk( void )
{
         if( scu_sync_ahb > 0 && !in_idle_loop ) {
                if( rockchip_clk_get_ahb() < SCU_MAX_AHB_CLK*1000000 )
                        __rockchip_clk_change( SCU_IPID_HCLK , scu_sync_ahb*SYNC_STEP );
                scu_sync_ahb =0;
                //schedule();
        }
}
static void __scu_idle_loop(void)
{
        unsigned long flags;
        u64              idle_tick;
        ktime_t         now;
        //local_irq_enable();
        /* current = swapper,pid = 0 */
        //printk("%s::syn ahb=%d,idle ms=%Ld,task=%s,pid=%d\n" , __func__ ,
        //        scu_sync_ahb,scu_idle_ns,current->comm,current->pid );
        in_idle_loop = 1;
        now = ktime_get();
        if( ktime_to_ns(ktime_sub(now,scu_ktime)) < 0 )
                now = scu_ktime;
        while (!need_resched()){
                cpu_relax();
                if( scu_sync_ahb < 0 ) {
                        int min_clk = SYNC_AHB_MIN_CLK*1000000;
                        if( get_msc_connect_flag() ) {
                                min_clk = 60*1000000;
                        }
                        #if 0
                        int cur_clk= rockchip_clk_get_ahb() /1000000;
                        //printk("down sync=%d,cur clk=%d\n" , scu_sync_ahb,cur_clk );
                        if( cur_clk > SYNC_AHB_MIN_CLK ) {
                                if( cur_clk + scu_sync_ahb*SYNC_STEP  < SYNC_AHB_MIN_CLK )
                                        cur_clk = SYNC_AHB_MIN_CLK - cur_clk;
                                else 
                                        cur_clk = scu_sync_ahb*SYNC_STEP;
                                __rockchip_clk_change( SCU_IPID_HCLK , cur_clk );
                        }
                        #else
                        if( rockchip_clk_get_ahb() > min_clk )
                                __rockchip_clk_change( SCU_IPID_HCLK , -SYNC_STEP );
                        #endif
                        scu_sync_ahb = 0;
                } 
        }
        __scu_up_ahb_clk();
        idle_tick = (u64)ktime_to_ns(ktime_sub( ktime_get(),now) );
        
        local_irq_save(flags);
        #if 0
        if( ktime_to_ns(ktime_sub(now,scu_ktime)) < 0 )
                scu_idle_ns += (u64)ktime_to_ns(ktime_sub( ktime_get(),scu_ktime) );
        else 
                scu_idle_ns += (u64)ktime_to_ns(ktime_sub( ktime_get(),now) );
        #endif
        scu_idle_ns += idle_tick;
        scu_idle_ns_avg += idle_tick;
        local_irq_restore(flags);
        in_idle_loop = 0;
}
static int continue_emergs;
static int loop_emergs;

static void __scu_sync_timer( unsigned long arg )
{
        u64 total_ns ;
        
        scu_loops++;
        if( scu_loops < SYNC_LOOP_MYCHANGE ) {
                /* if idle time to little , up clk now!*/
                if( scu_idle_ns < (10*NSEC_PER_USEC) && !in_idle_loop ) {
                        continue_emergs++;
                        loop_emergs++;
                        if( continue_emergs > SYNC_LOOP_UPNOW )  {
                                //struct pt_regs *regs = get_irq_regs();
                                //__print_symbol("cur pc at %s ", regs->ARM_pc);
                                //printk("emerge up,idle ns=%Ld,loop=%d,idle agv=%Ld\n" ,
                                //        scu_idle_ns , scu_loops , scu_idle_ns_avg);
                                //printk("up\n");
                                scu_sync_ahb = 12;
                                continue_emergs = 0;
                        }
                } else {
                        continue_emergs = 0;
                }
                goto scu_loop_end;
        }
        total_ns =(u64)ktime_to_ns(ktime_sub( ktime_get(),scu_ktime) );
        
        #if 0
        {
        printk("t=%Ld,i=%Ld" ,total_ns, scu_idle_ns_avg);
        printk("!%d\n" , loop_emergs);   
        }
        #endif
        //total_ns = (total_ns>>2) +(total_ns>>4); 
        //total_ns = (total_ns>>2) - (total_ns>>5);
        total_ns = (total_ns>>2) - (total_ns>>4);// 18.725% 
        
        if( scu_idle_ns_avg > total_ns ) {
                        if( loop_emergs < SYNC_LOOP_EMERGE) {
                                scu_sync_ahb = -1;
                                //if( idle_nsx100 > (total_ns<<1) )     
                                //        scu_sync_ahb--;
                                //if( idle_nsx100 > (total_ns<<2) )    
                                //        scu_sync_ahb--;
                        }
        } 
        if( ( scu_idle_ns_avg < (total_ns>>2)) ) {
                scu_sync_ahb++;
        } 
        scu_ktime = ktime_get();
        scu_idle_ns_avg = 0;
        scu_loops = 0;
        loop_emergs = 0;
scu_loop_end:        
        scu_idle_ns = 0;
        mod_timer( &scu_timer , jiffies + msecs_to_jiffies( SYNC_TIME ) );
        __scu_up_ahb_clk();
}
__init
void __scu_sync_init( void )
{
        scu_loops = 0 ;
        scu_ktime = ktime_get();
        pm_idle = __scu_idle_loop;
        init_timer( &scu_timer );
        scu_timer.function = __scu_sync_timer;
        mod_timer( &scu_timer , jiffies + msecs_to_jiffies( SYNC_TIME*10 ) );
}
#else
#define __scu_sync_init()
#endif

#if 0
__init
void __rockchip_scu_TestDelay( void )
{
        unsigned long flags;
        int             i = 0,j=5;
        int             dly[] ={ 2000,4000,5000,10000,100000,200000};

        SCU_BUG("enter TestDelay!" );
//        local_irq_save(flags);
        while( 1 ) {
                udelay(dly[j]);
                i++;
                debug_gpio_reverse();
                if(  i*dly[j] > 8*1000000 ) {
			printk("Now is dly[j] = %d\r\n",dly[j]);
                        j=5;i=0;
                        if( j >5 )
                                j=5;//break;
                }
        }
//        local_irq_restore(flags);
        SCU_BUG("exit TestDelay!" );
}
#endif

extern int rockchip_timer_change_pclk( ip_id ip , int input_clk );
__init
int __rockchip_scu_tree_init( void )
{

        SCU_BUG("scu init enter" );
        spin_lock_init( &scu_system_clkinfo.spinlock );      
        if( __rockchip_scu_kset_init() < 0 )
                return -ENOMEM;
        __rockchip_scu_node_kobject_init(scu_system_clkinfo.root);
        //__rockchip_scu_init_hw( ); /* 这个函数也可以放到 baord init 里面调用*/

        /* not change arm clk here .*/
        rockchip_scu_register( SCU_IPID_ARM , SCU_MODE_NONE, 0 , NULL );
        
        //rockchip_scu_register( SCU_IPID_HCLK , SCU_MODE_FREQ, SCU_MAX_AHB_CLK , NULL );
        rockchip_scu_register( SCU_IPID_HCLK , SCU_MODE_FREQ, SCU_MAX_AHB_CLK , NULL );
        rockchip_scu_register( SCU_IPID_PCLK , SCU_MODE_FREQ, (SCU_MAX_AHB_CLK>>1) , NULL );
		
        rockchip_scu_register( SCU_IPID_CODEC, SCU_MODE_SETFREQ , 24 , NULL );
        rockchip_scu_register( SCU_IPID_DSP, SCU_MODE_SETFREQ , 24 , NULL );
        rockchip_scu_register( SCU_IPID_12M, SCU_MODE_NONE, 0 , NULL );

        /* 20100208,HSL@RK,set div = 25 for hdmi */
        rockchip_scu_register(SCU_IPID_I2S , SCU_MODE_DIV , 25 , NULL);
        rockchip_scu_apbunit_register( SCU_IPID_TIMER , "timer0" , rockchip_timer_change_pclk );        
        rockchip_clk_set_arm( SCU_MAX_ARM_CLK );   

//        rockchip_scu_register(SCU_IPID_LCDC, SCU_MODE_DIV , 22, NULL);
//        __rockchip_clk_set_unit_clock(SCU_IPID_LCDC,27);
        
        __scu_sync_init();
        SCU_BUG("scu init finish!" );

        //__rockchip_scu_TestDelay();
        return 0;
}

#ifdef __i386__
static int __rockchip_scu_free_unit(struct rockchip_scu_unit *p , unsigned long arg)   
{
        struct rockchip_scu_unit *ps , *pe;
        ps = &scu_ip_info_table[0];
        pe = ps + ARRAY_SIZE(scu_ip_info_table);
        if( p > pe || p < ps ) 
                SCU_BUG("free malloc unit %s" , SCU_SYSFS_NAME(p) );
        else 
                SCU_BUG("free static unit %s" , SCU_SYSFS_NAME(p) );     
         __rockchip_scu_kobj_put( &p->kobj );
        if( p > pe || p < ps ) 
                kfree(p) ;
        return 0;
}

static void __rockchip_scu_tree_deinit( struct rockchip_scu_unit *root )
{
        struct list_head *list, *tmp;
        list_for_each_safe (list, tmp, &root->child_list ) {
                struct rockchip_scu_unit *p;
                p = list_entry(list, struct rockchip_scu_unit, list);
                if( p->propt&SCU_PROPT_PARERNT)
                        __rockchip_scu_tree_deinit( p );
                list_del(list);
                __rockchip_scu_free_unit( p , 0 );
        }
}

static void __rockchip_scu_module_exit( void )
{
        SCU_BUG("rockchip_scu module exit" );
        __rockchip_scu_tree_deinit(scu_system_clkinfo.root);
        __rockchip_scu_kobj_put( &scu_system_clkinfo.root->kobj );
        __rockchip_scu_kset_exit();
}
module_init(__rockchip_scu_tree_init);
module_exit(__rockchip_scu_module_exit );
#else
arch_initcall(__rockchip_scu_tree_init);

//rootfs_initcall(__rockchip_scu_tree_init);
#endif

#if 0
int __rockchip_scu_get_unit_div( ip_id id )  
{
        struct rockchip_scu_unit *p = __rockchip_find_unit_at_node( scu_system_clkinfo.root  , id );
        if( !p )
            return -EINVAL;
        return rockchip_get_unit_div_hw( p );
}

/*
 * 20100106,HSL@RK, test code.
 */
int scu_debug_change_i2s( void )
{
        int ret = __rockchip_scu_set_parent( SCU_IPID_I2S , SCU_IPID_CODEC , SCU_MODE_SETDIV , 25 );
        if( ret )
                printk("change i2s to parent codec failed,ret=%d\n" , ret );
        __rockchip_clk_set_unit_clock(SCU_IPID_CODEC, SCU_CLK_112896 );
        return ret;
}

int scu_debug_change_lcdc( void )
{
        int ret = __rockchip_scu_set_parent( SCU_IPID_LCDC, SCU_IPID_ARM, SCU_MODE_SETDIV, 8 );
        if( ret )
                printk("change lcdc to parent arm failed,ret=%d\n" , ret );
        rockchip_clk_set_arm( 594 );
        return ret;
}

EXPORT_SYMBOL(rockchip_scu_apbunit_register);
EXPORT_SYMBOL(rockchip_scu_register);
EXPORT_SYMBOL(rockchip_clk_get_apb);

EXPORT_SYMBOL(rockchip_clk_get_ahb);
EXPORT_SYMBOL(rockchip_clk_set_arm);
#endif
/************************************************************************/
MODULE_AUTHOR("Huang SL");
MODULE_DESCRIPTION("rk28 scu manage");
MODULE_ALIAS("RK28 SCU");



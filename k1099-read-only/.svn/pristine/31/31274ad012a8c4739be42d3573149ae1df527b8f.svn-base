/*
 *  scu/rk_scu_hw.c
 *
 * (C) Copyright hsl 2009
 *	Released under GPL v2.
 *
 * 
 * log:
 *      basic scu register function.
 *      20090518,change uniform to set io register.
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

#include <linux/uaccess.h>
#include <linux/errno.h>
#include <linux/poll.h>
#include <linux/timer.h>
#include <linux/spinlock_types.h>

#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <asm/atomic.h>

#include <asm/arch/typedef.h>
#include <asm/arch/hardware.h>

//#define SCU_DEBUG
#ifdef __i386__
#include "rk28_scu.h"
#else
#include <asm/arch/rk28_scu.h>
#endif
#include <asm/arch/rk28_debug.h>

#define CLOSE_LCD_CLK                   0 /* close cldc clk when change ahb clk,for rgb panel,have problem!! */

/*SCU PLL CON*/
#define PLL_TEST        (0x01u<<25)
#define PLL_SAT         (0x01u<<24)
#define PLL_FAST        (0x01u<<23)
#define PLL_PD          (0x01u<<22)
#define PLL_CLKR(i)     (((i)&0x3f)<<16)
#define PLL_CLKF(i)     (((i)&0x0fff)<<4)
#define PLL_CLKOD(i)    (((i)&0x07)<<1)
#define PLL_BYPASS      (0X01)

/*SCU MODE CON*/
#define SCU_INT_CLR         (0x01u<<8)
#define SCU_WAKEUP_POS      (0x00u<<7)
#define SCU_WAKEUP_NEG      (0x01u<<7)
#define SCU_ALARM_WAKEUP_DIS (0x01u<<6)
#define SCU_EXT_WAKEUP_DIS   (0x01u<<5)
#define SCU_STOPMODE_EN     (0x01u<<4)

#define SCU_CPUMODE_MASK    (0x03u<<2)
#define SCU_CPUMODE_SLOW    (0x00u<<2)
#define SCU_CPUMODE_NORMAL  (0x01u<<2)
#define SCU_CPUMODE_DSLOW   (0x02u<<2)

#define SCU_DSPMODE_MASK    (0x03u<<0)
#define SCU_DSPMODE_SLOW    (0x00u<<0)
#define SCU_DSPMODE_NORMAL  (0x01u<<0)
#define SCU_DSPMODE_DSLOW   (0x02u<<0)

/*SCU PMU MODE*/
#define PMU_SHMEM_PWR_STAT  (0x01u<<8)
#define PMU_DEMOD_PWR_STAT  (0x01u<<7)
#define PMU_CPU_PWR_STAT    (0x01u<<6)
#define PMU_DSP_PWR_STAT    (0x01u<<5)

#define PMU_EXT_SWITCH_PWR  (0x01u<<4)

#define PMU_SHMEM_PD        (0x01u<<3)
#define PMU_DEMOD_PD        (0x01u<<2)
#define PMU_CPU_PD          (0x01u<<1)
#define PMU_DSP_PD          (0x01u<<0)

/*SCU SOFTWARE RESET CON*/
#define CLK_RST_SDRAM       (1<<28)
#define CLK_RST_SHMEM1      (1<<27)
#define CLK_RST_SHMEM0      (1<<26)
#define CLK_RST_DSPA2A      (1<<25)
#define CLK_RST_SDMMC1      (1<<24)
#define CLK_RST_ARM         (1<<23)
#define CLK_RST_DEMODGEN    (1<<22)
#define CLK_RST_PREFFT      (1<<21)
#define CLK_RST_RS          (1<<20)
#define CLK_RST_BITDITL     (1<<19)
#define CLK_RST_VITERBI     (1<<18)
#define CLK_RST_FFT         (1<<17)
#define CLK_RST_FRAMEDET    (1<<16)
#define CLK_RST_IQIMBALANCE (1<<15)
#define CLK_RST_DOWNMIXER   (1<<14)
#define CLK_RST_AGC         (1<<13)
#define CLK_RST_USBPHY      (1<<12)
#define CLK_RST_USBC        (1<<11)
#define CLK_RST_DEMOD       (1<<10)
#define CLK_RST_SDMMC0      (1<<9)
#define CLK_RST_DEBLK       (1<<8)
#define CLK_RST_LSADC       (1<<7)
#define CLK_RST_I2S         (1<<6)
#define CLK_RST_DSPPER      (1<<5)
#define CLK_RST_DSP         (1<<4)
#define CLK_RST_NANDC       (1<<3)
#define CLK_RST_VIP         (1<<2)
#define CLK_RST_LCDC        (1<<1)
#define CLK_RST_USBOTG      (1<<0)

/*SCU CLK SEL0 CON*/
#define CLK_SDMMC1_SHFT     25
#define CLK_SDMMC1_MASK     (0x07u<<25)
#define CLK_SDMMC1_DIV(i)   (((i-1)&0x07u)<<25)

#define CLK_SENSOR_SHFT     23
#define CLK_SENSOR_MASK     (0x03u<<23)
#define CLK_SENSOR_24M      (0x00u<<23)
#define CLK_SENSOR_27M      (0x01u<<23)
#define CLK_SENSOR_48M      (0x02u<<23)

#define CLK_48M_SHFT     20
#define CLK_48M_MASK        (0x07u<<20)
#define CLK_48M_DIV(i)      (((i-1)&0x07u)<<20)


#define CLK_USBPHY_SHFT     18
#define CLK_USBPHY_MASK     (0x03u<<18)
#define CLK_USBPHY_24M      (0x00u<<18)
#define CLK_USBPHY_12M      (0x01u<<18)
#define CLK_USBPHY_48M      (0x01u<<18)

#define CLK_LCDC_ARMPLL     (0x00u<<16)//
#define CLK_LCDC_DSPPLL     (0x01u<<16)//
#define CLK_LCDC_CODPLL     (0x02u<<16)//

#define CLK_LCDC_SHFT     8
#define CLK_LCDC_MASK       (0x0ffu<<8)
#define CLK_LCDC_DIV(i)     (((i-1)&0xffu)<<8)

#define CLK_LCDC_DIVOUT     (0x00<<7)//
#define CLK_LCDC_27M        (0X01<<7)//

#define CLK_SDMMC0_SHFT     4
#define CLK_SDMMC0_MASK     (0x07u<<4)
#define CLK_SDMMC0_DIV(i)   (((i-1)&0x07u)<<4)

#define CLK_PCLK_SHFT     2
#define CLK_PCLK_MASK       (0x03u<<2)
#define CLK_HCLK_PCLK_11    (0x00u<<2)
#define CLK_HCLK_PCLK_21    (0x01u<<2)
#define CLK_HCLK_PCLK_41    (0x02u<<2)

#define CLK_HCLK_SHFT     0
#define CLK_HCLK_MASK       (0x03u<<0)
#define CLK_ARM_HCLK_11     (0x00u<<0)
#define CLK_ARM_HCLK_21     (0x01u<<0)
#define CLK_ARM_HCLK_31     (0x02u<<0)
#define CLK_ARM_HCLK_41     (0x03u<<0)

/*SCU CLK SEL1 CON*/
#define CLK_SHMEM1_SHFT     30
#define CLK_SHMEM1_MASK     (0x01u<<30)
#define CLK_SHMEM1_DEMODCLK (0x00u<<30)
#define CLK_SHMEM1_ARMCLK   (0x01u<<30)

#define CLK_SHMEM0_SHFT     29
#define CLK_SHMEM0_MASK     (0x01u<<29)
#define CLK_SHMEM0_DEMODCLK (0x00u<<29)
#define CLK_SHMEM0_ARMCLK   (0x01u<<29)

#define CLK_HSADCO_SHFT     28
#define CLK_HSADCO_MASK      (0x01u<<28)
#define CLK_HSADCO_NORMAL    (0x00u<<28)
#define CLK_HSADCO_INVERT    (0x01u<<28)

#define CLK_GPS_SHFT     27
#define CLK_GPS_MASK        (0x01u<<27)
#define CLK_GPS_DEMODCLK    (0x00u<<27)
#define CLK_GPS_TUNER_INPUT (0x01u<<27)

#define CLK_DEMOD_INTCLK    (0x00u<<26)//
#define CLK_DEMOD_EXTCLK    (0x01u<<26)//

#define CLK_DEMOD_ARMPLL    (0x00u<<24)//
#define CLK_DEMOD_DSPPLL    (0x01u<<24)//
#define CLK_DEMOD_CODPLL    (0x02u<<24)//

#define CLK_DEMOD_SHFT     16
#define CLK_DEMOD_MASK      (0x0ffu<<16)
#define CLK_DEMOD_DIV(i)    (((i-1)&0x0ffu)<<16)

#define CLK_LSADC_SHFT     8
#define CLK_LSADC_MASK      (0x0ffu<<8)
#define CLK_LSADC_DIV(i)    (((i-1)&0x0ffu)<<8)

#define CLK_CODEC_SHFT     3
#define CLK_CODEC_MASK      (0x1fu<<3)
#define CLK_CODEC_DIV(i)    (((i-1)&0x1fu)<<3)

#define CLK_CODEC_CPLLCLK   (0x00u<<2)//
#define CLK_CODEC_12M       (0x01u<<2)//

#define CLK_CPLL_SLOW       (0x00u<<0)//
#define CLK_CPLL_NORMAL     (0x01u<<0)//
#define CLK_CPLL_DSLOW      (0x02u<<0)//

/********************************************************************
**                          结构定义                                *
********************************************************************/
typedef volatile unsigned int   io_reg;      
struct rockchip_scu_reg_hw
{
    io_reg scu_pll_config[3];//[3];//0:arm 1:dsp 2:codec
    io_reg scu_mode_config;
    io_reg scu_pmu_config;
    io_reg scu_clksel0_config;
    io_reg scu_clksel1_config;
    io_reg scu_clkgate0_config;
    io_reg scu_clkgate1_config;
    io_reg scu_clkgate2_config;
    io_reg scu_softreset_config;
    io_reg scu_chipcfg_config;
    io_reg scu_cuppd;    /* arm power down */
};

struct rockchip_grf_reg_hw
{
    io_reg  CPU_APB_REG0;
    io_reg  CPU_APB_REG1;
    io_reg  CPU_APB_REG2;
    io_reg  CPU_APB_REG3;
    io_reg  CPU_APB_REG4;
    io_reg  CPU_APB_REG5;
    io_reg  CPU_APB_REG6;
    io_reg  CPU_APB_REG7;
    io_reg  IOMUX_A_CON;
    io_reg  IOMUX_B_CON;
    io_reg  GPIO0_AB_PU_CON;
    io_reg  GPIO0_CD_PU_CON;
    io_reg  GPIO1_AB_PU_CON;
    io_reg  GPIO1_CD_PU_CON;
    io_reg  OTGPHY_CON0;
    io_reg  OTGPHY_CON1;
} ;

#ifdef __i386__
static struct rockchip_scu_reg_hw      local_scu_register;
static struct rockchip_grf_reg_hw       local_scu_reg_file;
static struct rockchip_scu_reg_hw *scu_register_base = &local_scu_register;
static struct rockchip_grf_reg_hw* scu_reg_file_base = &local_scu_reg_file;
#else
static struct rockchip_scu_reg_hw *scu_register_base = (struct rockchip_scu_reg_hw *)(SCU_BASE_ADDR_VA);
//static struct rockchip_grf_reg_hw* scu_reg_file_base = (struct rockchip_grf_reg_hw*)REG_FILE_BASE_ADDR_VA;
//#define scu_register_base               ( (struct rockchip_scu_register *)SCU_BASE_ADDR_VA)
//#define scu_reg_file_base               ( (struct rockchip_grf_reg_hw *)REG_FILE_BASE_ADDR_VA)
#endif

int __rockchip_clk_set_unit_clock( ip_id id , int new_clk ) ;
struct rockchip_scu_unit * __rockchip_find_unit_at_node( struct rockchip_scu_unit *node , ip_id id );
static void __rockchip_scu_pll_slowmod_hw( ip_id id  , int enter )
{
        
        if( enter ) {
                if( id == SCU_IPID_ARM )  {
                        scu_register_base->scu_mode_config &= ~SCU_CPUMODE_MASK;
                } else if( id == SCU_IPID_DSP )
                        scu_register_base->scu_mode_config &= ~SCU_DSPMODE_MASK;
                else if( id == SCU_IPID_CODEC )        
                        scu_register_base->scu_clksel1_config &= ~(0X03);
        } else {
                if( id == SCU_IPID_ARM ) {   
                        scu_register_base->scu_mode_config |= SCU_CPUMODE_NORMAL;
                } else if( id == SCU_IPID_DSP )
                        scu_register_base->scu_mode_config |= SCU_DSPMODE_NORMAL;
                else if( id == SCU_IPID_CODEC )        
                        scu_register_base->scu_clksel1_config |= (0X01);
        }
}

static void __rochchip_scu_setdiv_reg( struct rockchip_scu_unit * p , io_reg *reg ,
        int div )
{
        unsigned int val = __raw_readl(reg);
        val &= ~( ( ( 1<<(p->divbit_end-p->divbit_start+1) ) - 1 )<<p->divbit_start );
        val |= (div-1) << p->divbit_start;
        __raw_writel( val , reg );
}

/* set register to the value div */
int __rockchip_scu_setdiv_hw( struct rockchip_scu_unit * p )
{
        int     div = p->tmp_div;
        io_reg   *reg = &scu_register_base->scu_clksel0_config;
        reg += p->divreg_index ;

#if 0
        /* CHANGE HCKP DIV NEED ENTER SLOW MODE */
        if( p->id == SCU_IPID_HCLK ) {
                if( p->parent->tmp_clk != 24 && p->parent->cur_clk != 24)
                        __rockchip_scu_pll_slowmod_hw( SCU_IPID_ARM , 1 );
        }
        __rochchip_scu_setdiv_reg( p , reg );
        if( p->id == SCU_IPID_HCLK ) {
                if( p->parent->tmp_clk != 24 && p->parent->cur_clk != 24)   /* if arm ==24m,pll already down , can not leave slow mode */
                        __rockchip_scu_pll_slowmod_hw( SCU_IPID_ARM , 0 );
        }
#else
        /* 20100301,HSL@RK,PCLK when div==4,reg bit value=0B10,not 0b11 */
        if(p->id == SCU_IPID_PCLK && div == 4 )
                div = 3;
        __rochchip_scu_setdiv_reg( p , reg , div );
#endif
        SCU_BUG("scu hw set %s div=%d ,reg=0x%p\n" , p->name , p->tmp_div , reg );
        //printk("scu hw set %s div=%d ,reg=0x%p\n" , p->name , p->tmp_div , reg );
        return 0;
}

/* set register clk gate to enable this ip clk 
*  if enable arm/dsp,codec, return normal state 
*/
int __rockchip_scu_enable_hw( ip_id  id , struct rockchip_scu_unit * p)
{
        io_reg   *reg ;

        if( p && p->ifn )
                {
                SCU_BUG("enable %s,bak=%d\n" , p->name , p->bak_clk );
                //return __rockchip_clk_set_unit_clock( id , p->bak_clk );
                p->tmp_clk = p->bak_clk;
                p->ifn( p , 1);
                p->cur_clk = p->tmp_clk;
                return 0;
                }
        
        if( id >= SCU_IPID_GATE_MAX )         
                return 0;
        reg = &scu_register_base->scu_clkgate0_config;
        while( id > 31 )
                {
                id -= 32;
                reg++;
                }
        (*reg) &= ~(1<<id );    // clear bit 
        return 0;
}

/* set register clk gate to disable this ip clk 
*  if disable arm/dsp,codec, enter slow mode 
*/
int __rockchip_scu_disable_hw( ip_id  id , struct rockchip_scu_unit * p)
{
        io_reg  *reg ;

                
        if( p && p->ifn )  {
                /* XXX:if want to change hclk div = 1 , tmp change SCU_IBIP_HCLK propt 
                  * 20100204,HSL@RK, div SCU_CLK_MHZ2KHZ for enable.
                */
                p->bak_clk = p->cur_clk;
                p->tmp_clk =  24*SCU_CLK_MHZ2KHZ;
                p->ifn( p , 1 );
                p->cur_clk = p->tmp_clk;
                return 0; //__rockchip_clk_set_unit_clock( id , 24 );
                }
        
        if( id >= SCU_IPID_GATE_MAX )  
                return 0;
        reg = &scu_register_base->scu_clkgate0_config;
        while( id > 31 )
                {
                id -= 32;
                reg++;
                }
        (*reg) |= (1<<id );    // set_bit
        return 0;
}

static void __rockchip_scu_change_pll_hw( io_reg   *regpll ,int clkr , int clkf ,
    int clkod , int delayus , struct rockchip_scu_unit * p)
{
        io_reg val;
        //local_irq_save(flags); /* 20090804,关闭中断时间太长，可能有问题*/
        #if CLOSE_LCD_CLK
        io_reg gate0,gate1;
        if( p->id == SCU_IPID_ARM ) {
                gate0 = scu_register_base->scu_clkgate0_config;
                gate1 = scu_register_base->scu_clkgate1_config;
                scu_register_base->scu_clkgate1_config &= ~(1<<19);     // close lcdc hclk clk.
                scu_register_base->scu_clkgate0_config &= ~(1<<11);     // close lcdc clk.
        }
        #endif
        __rockchip_scu_pll_slowmod_hw( p->id , 1 );
        (*regpll) &= ~PLL_PD;
        if( p->id == SCU_IPID_ARM ) {
                val = scu_register_base->scu_clksel0_config;
                scu_register_base->scu_clksel0_config  &= ~3;
        }
        /* XXX:delay for pll state , for 0.3ms , clkf will lock clkf*/
        /* 可能屏幕会倒动,如果不修改 clkr和clkf则不用进入slow mode .*/
        (*regpll) = PLL_SAT|PLL_FAST|(PLL_CLKR(clkr-1))|(PLL_CLKF(clkf-1))|(PLL_CLKOD(clkod - 1));
        __udelay( delayus );
        if( p->id == SCU_IPID_ARM ) {
                scu_register_base->scu_clksel0_config  = val;
                __udelay( 5 ); /*delayus*/  /* can not delay too long for audio,AMR AT 24M. */
        }
        __rockchip_scu_pll_slowmod_hw(p->id , 0 );

        #if CLOSE_LCD_CLK
        if( p->id == SCU_IPID_ARM ) {
                __udelay(2);
                scu_register_base->scu_clkgate1_config = gate1;
                scu_register_base->scu_clkgate0_config = gate0;
        }
        #endif
        
        //local_irq_restore(flags);
        SCU_BUG("hw set %s pll to %d MHZ,clkf=%d,clkod=%d,delay count=%d ",p->name , p->tmp_clk ,  clkf , clkod ,
                delayus );
}

/*
* return value :
*       0: need parent change to set as p->tmp_clk .
*       1: set ok , no need parent change.
*       -1: can not change to special CLK.
*       XXX:when arm pll down ,need set ahb div = 1, ahb = 24M? apb ?
*/
int __rockchip_scu_set_pllclk_hw( struct rockchip_scu_unit * p  , int stage)
{
        
        io_reg   *regpll = &scu_register_base->scu_pll_config[0];
        int             pll_idx = 0;
        int        clkf , clkod , unit;
        

        if( stage == 0 ) {      /* not real set clk , just require */
                return SCU_SETCLK_OK;
        }
        
        if( p->id == SCU_IPID_DSP ) {
                pll_idx++;
                regpll++;
        }
        if( p->id == SCU_IPID_CODEC ) {
                pll_idx += 2;
                regpll+= 2;
        }
        if( p->bak_clk >= SCU_CLK_SPECIAL ) {
                SCU_BUG("set %s to special clk %d\n" , p->name , p->bak_clk );
                if( p->bak_clk == SCU_CLK_112896 ) {
                        /* 11.2896M = 44.1k*256 = (24/25) * ((441/3)*2) / 1(od) / 25(div)
                        */
                        __rockchip_scu_change_pll_hw(regpll , 25, 147*2 , 1,1000, p );
                }
                if( p->bak_clk == SCU_CLK_122880) {
                        /* 12.280M = 48k*256 = (24/5) * (64) / 1(od) / 25(div)
                        */
                        __rockchip_scu_change_pll_hw(regpll , 5, 64 , 1, 1000, p );
                }
                /* 20100203,HSL@RK,clear for next time setting.*/
                p->bak_clk = p->cur_clk;
                return SCU_SETCLK_OK;
        }
        if( p->tmp_clk <= 24*SCU_CLK_MHZ2KHZ ) {
                //struct rockchip_scu_unit ahb = __rockchip_find_unit_at_node();
                __rockchip_scu_pll_slowmod_hw( p->id , 1 );
                //__raw_writel( __raw_readl(regpll) | PLL_PD  , regpll );
                (*regpll) |= PLL_PD;    /* pll power down */
                p->tmp_clk = 24*SCU_CLK_MHZ2KHZ;
                SCU_BUG("set 24M clk ,%s power down", p->name );
        } else {
                //unsigned long flags;
                unit = p->tmp_clk/SCU_CLK_MHZ2KHZ;
                clkf = unit;
                clkod = 1;
                while( clkf < 200 )  { /* 160 <= Fref/NR * NF <= 800 , 200 set to the midia point */
                       clkf += unit ;
                       clkod += 1;
                } 
                if( clkod > 8 ) { /* clkod max 8 , 8*24=192 > 160 , safe */
                        clkod = 8 ;
                        clkf = clkod * unit;
                }
                __rockchip_scu_change_pll_hw(regpll , 24, clkf , clkod ,1000, p );
        }
        return SCU_SETCLK_OK;
}


/*
* return value :
*       0: need parent change to set as p->tmp_clk .
*       1: set ok , no need parent change.
*       -1: can not change to special CLK.
*       XXX:when arm pll down ,need set ahb div = 1, ahb = 24M? apb ?
*/
int __rockchip_scu_set_armpll_hw( struct rockchip_scu_unit * p  , int stage)
{
        
        io_reg   *regpll = &scu_register_base->scu_pll_config[0];
        int        clkf , clkod , clkr,unit;
        int        delay;
        int        clkf_old;
        
        if( stage == 0 ) {      /* not real set clk , just require */
                return SCU_SETCLK_OK;
        }
        clkr = 24;
        if( p->tmp_clk <= 24*SCU_CLK_MHZ2KHZ ) {
                __rockchip_scu_pll_slowmod_hw( p->id , 1 );
                (*regpll) |= PLL_PD;    /* pll power down */
                p->tmp_clk = 24*SCU_CLK_MHZ2KHZ;
                SCU_BUG("set 24M clk ,%s power down", p->name );
        } else {
        	unit = p->tmp_clk/SCU_CLK_MHZ2KHZ; /* to MHZ */
                clkf_old = ( ((*regpll)>>4)&0xfff )+1;
                clkod = clkf_old/unit;
                if( clkod * unit == clkf_old )
                        clkf = clkf_old;
                else {
                        clkod = 1;
                        while( (clkf = unit*clkod) < 192 ){
                                clkod ++;
                        }
                        /* when */
                }
                delay = 5 ; // 24*100;  /* SLOW MODE,RUN AT 24M */
                /* cur == 24,now pll power on */
                if( p->cur_clk == 24*SCU_CLK_MHZ2KHZ ) {
                        //debug_print("PLL ON AGAIN,need long delay\n");
                        delay <<= 8;  // *64 20091123,NOT 0.3ms,total cycle 100*64*6/24000 = 1.6MS.
                } else if ( clkf  != clkf_old) {
                        //debug_print("PLL ON AGAIN,need long delay\n");
                        delay <<= 6;  //7:audio noise *64 20091123,NOT 0.3ms,total cycle 100*64*6/24000 = 1.6MS.
                }
                #if 0
                {
                ktime_t now;
                now = ktime_get();
                __rockchip_scu_change_pll_hw(regpll , clkr, clkf , clkod ,delay, p );
                now = ktime_sub( ktime_get(),now);
                printk("chg arm clk:%Ld\n" , ktime_to_ns(now) );
                }
                #else
                __rockchip_scu_change_pll_hw(regpll , clkr, clkf , clkod ,delay, p );
                #endif
        }
        return SCU_SETCLK_OK;
}


/* for sensor , usb phy 48M */
int __rockchip_scu_set_48m_hw( struct rockchip_scu_unit * p  , int stage)
{
#if 0
        int     start_bit ;
        int     mask_bit;

        /* not support externel clk */
        if( p->tmp_clk != 24*SCU_CLK_MHZ2KHZ && p->tmp_clk != 48*SCU_CLK_MHZ2KHZ )
                return SCU_SETCLK_IMPOSSIBLE;
        if( stage == 0 ) {       /* not real set clk , just require */
                if( p->tmp_clk == 48 ) {
                        p->tmp_div = p->parent->max_clk / p->tmp_clk;
                        if( p->tmp_div > SCU_DIV_MAXVALUE(p) )
                                p->tmp_div = SCU_DIV_MAXVALUE(p);
                        p->parent->tmp_clk = p->tmp_div * p->tmp_clk ;
                        SCU_BUG("set %s to 48M,so parent %s to %d", 
                                p->name , p->parent->name , p->parent->tmp_clk );
                        return SCU_SETCLK_PARENT; /* need parent change !*/
                }
                return SCU_SETCLK_OK;
        }

        start_bit = 23; /* sensor , 23-24 */
        mask_bit = (0x03)<<23;
        if( p->id == SCU_IPID_USBPHY ) {
                start_bit = 18; /* usb phy , 18-19 */
                mask_bit = (0x03)<<18;
        }
                
        if( p->tmp_clk == 24 ) {
                SCU_BUG("set 24M clk ,%s power down", p->name );
                scu_register_base->scu_clksel0_config &= ~mask_bit;
        } else {
                scu_register_base->scu_clksel0_config |= (2)<<start_bit;
        }
#endif        
        return SCU_SETCLK_OK;  
}
 int rockchip_chg_parent_all(struct rockchip_scu_unit *ipinfo, ip_id parent )
 {
        /* bit 17:16 of clk sel0 */
        int     start_bit;
        int     mask = 0x3; /* 2bits */
        int     val ;
        io_reg   *regsel = &scu_register_base->scu_clksel0_config;

        if( ipinfo->id == SCU_IPID_LCDC )
            start_bit = 16;
        else if( ipinfo->id == SCU_IPID_I2S ) {
            start_bit = 2;
            regsel++;   /* clk sel 1*/
            mask = 1;
        }
        else if( ipinfo->id == SCU_IPID_SENSOR)
            start_bit = 23;  
        else
            return -1;
        if( ipinfo->id == SCU_IPID_LCDC ) {
            switch( parent ){
            case SCU_IPID_ARM:
                val = 0;
                break;
            case SCU_IPID_CODEC:
                val = 2;
                break;
            case SCU_IPID_DSP:
                val = 1;
                break;
            default:
                return -2;
            }
        } else if (ipinfo->id == SCU_IPID_I2S ) {
            switch( parent ){
            case SCU_IPID_CODEC:
                val = 0;
                break;
            case SCU_IPID_12M:
                val = 1;
                break;
            default:
                return -2;
            }
        } else { /* SCU_IPID_SENSOR */
            switch( parent ){
            case SCU_IPID_ARM:  /* ARM PLL FOR 48M */
                val = 2;
                break;
            case SCU_IPID_24M:
                val = 0;
                break;
            default:    /* extern 27M not support */
                return -2;
            }
        }

        /* 20100208,HSL@RK,change to one instr! */
        *regsel = ((*regsel) & (~(mask<<start_bit)) )|(val<<start_bit);
        //*regsel &= ~(mask<<start_bit);
        //*regsel |= (val<<start_bit);
        return 0;
 }

/*
 *  20100203,HSL@RK,msleep can not use at irq or softirq.
 */
int rockchip_scu_reset_unit( int index )
{
        int val = 1;
        if( index >= 32 )       
                return 0;
        if( index == 11 || index == 12 ){
                index = 11;
                val = 3;
        }
        scu_register_base->scu_softreset_config |= (val<<index);
        if( !in_interrupt() && !__system_crashed() )
                msleep(10);
        else 
                mdelay( 5 );
        scu_register_base->scu_softreset_config &= ~(val<<index);
        if( !in_interrupt() && !__system_crashed())
                msleep(10);
        else 
                mdelay( 5 );
        return 0;
}
int rockchip_get_unit_div_hw( struct rockchip_scu_unit *ipinfo )
{
        int     div;
        io_reg   *regsel = &scu_register_base->scu_clksel0_config;
        /* 20100106,HSL@KR,check at caller */
       // if( (ipinfo->propt & SCU_PROPT_FIXDIV) || !SCU_UNIT_HAVEDIV(ipinfo) )
       //        return ipinfo->cur_div;
        regsel += ipinfo->divreg_index;
        //div = 1+ ( ((*regsel)>>ipinfo->divbit_start )&((1<<(ipinfo->divbit_end-ipinfo->divbit_start+2))-1) );
        div = 1+ ( ((*regsel)>>ipinfo->divbit_start )&((1<<(ipinfo->divbit_end-ipinfo->divbit_start+1))-1) );
        if( ipinfo->id == SCU_IPID_PCLK && div == 3 )
                div = 4;
        SCU_BUG("get %s div=%d,regval=0x%x\n" , ipinfo->name , div , *regsel );
        return div;
}

int __rockchip_scu_get_pll_clk_hw( ip_id id )
{
        int     clk = 24;
        unsigned int reg_val ;
        io_reg   *regpll = &scu_register_base->scu_pll_config[0];
        if( id == SCU_IPID_DSP ) {
                regpll++;
        }
        if( id == SCU_IPID_CODEC ) {
                regpll+= 2;
        }
        reg_val = *regpll;
        SCU_BUG("read pll config 0x%p,reg val=0x%x\n" , regpll , reg_val );
        clk = clk * (((reg_val>>4)&0xfff)+1) /( (((reg_val>>16)&0x3f)+1)  * (((reg_val>>1)&0x7)+1));
        return clk*1000000; /* unit = HZ */
}

int __init __rockchip_scu_get_ahb_clk_hw( void )
{
        int div = (scu_register_base->scu_clksel0_config&0x3)+1;
        return __rockchip_scu_get_pll_clk_hw(SCU_IPID_ARM)/div;
}

int __init __rockchip_scu_get_apb_clk_hw( void )
{
        int div =( (scu_register_base->scu_clksel0_config>>2)&0x3)+1;
        if( div == 3 )
                div = 4;
        return __rockchip_scu_get_ahb_clk_hw()/div;
}

/*
 * 20100304,HSL@RK,for new version,tmp_clk unit is kHz,
 * clk unit is mHz.  must set bak_clk.
*/
int __init __rockchip_set_arm_clk_hw( int clk )
{
        struct rockchip_scu_unit su;
        su.id = SCU_IPID_ARM;
        su.tmp_clk = clk*SCU_CLK_MHZ2KHZ; 
        su.name = "arm";
        su.bak_clk = 0;
        return __rockchip_scu_set_pllclk_hw( &su , 1);
}

extern int *__rk28_tcm_map( void );
int *(*rk28_idle)(void ) ;
void(*rk28_restart_mmu)(void ) ;
void __init __rockchip_scu_init_hw( void )
{        
        int * p;
        unsigned long reg_value;
        //SCU_BUG("begin map tcm and copy code");
        p = __rk28_tcm_map();
        //SCU_BUG("0x%x,0x%x,0x%x,\ncode start=0x%x,len=0x%x" , p[0] , p[1] , p[2] , p[3] , p[4] );
        rk28_idle = (int*(*)(void))p[2];
        rk28_restart_mmu = (void(*)(void))( ((char*)p[2])+p[5] );
        #if 0
        {
         int  *s,*d;
        int     len =0;
        int     code_err = 0;
        SCU_BUG("source fun=%p,dest fun=%p" , __rk28_halt_enter , rk28_idle  );
        d = (int*)p[2];
        s = (int*)p[3];
        while( len < p[4] ) {
                if( *d != *s ) {
                        SCU_BUG("%p@%p: d=0x%x ,s=0x%x" ,d , s , *d , *s );
                        //code_err = 1;
                        *d = *s;
                        SCU_BUG("%p@%p: d=0x%x ,s=0x%x" ,d , s , *d , *s );
                }
                d++;
                s++;
                len += 4;
        }
        if( !code_err  )
                rk28_idle();
        else
                __rk28_halt_enter();
        }
        #endif

        scu_register_base->scu_mode_config = SCU_INT_CLR|SCU_WAKEUP_POS|SCU_ALARM_WAKEUP_DIS\
                             |SCU_EXT_WAKEUP_DIS|SCU_CPUMODE_NORMAL|SCU_DSPMODE_NORMAL;
        scu_register_base->scu_pmu_config = PMU_SHMEM_PD | PMU_DEMOD_PD | PMU_DSP_PD;
        
        //clock gate初始设置，0x00为打开，0x01为关闭
        scu_register_base->scu_clkgate0_config = 0
                         | (0x00u<<SCU_IPID_SDMMC1)/*(0x01u<<SCU_IPID_SDMMC1)  Modifyed by xbw*/
                         |(0x01u<<SCU_IPID_SHMEM1)  | (0x01u<<SCU_IPID_SHMEM0)
                         |(0x01u<<SCU_IPID_LSADC)   | (0X01u<<SCU_IPID_RTC)//
                         |(0x01u<<SCU_IPID_WDT)     | (0x00u<<SCU_IPID_PWM)
              //        |(0x01u<<SCU_IPID_SPI1)    | (0x01u<<SCU_IPID_SPI0)
              //           |(0x01u<<SCU_IPID_I2C1)    | (0x01u<<SCU_IPID_I2C0)
#if(defined(CONFIG_BOARD_ZTX))
                         |(0x00u<<SCU_IPID_UART1)   
#else
                         |(0x01u<<SCU_IPID_UART1)   
#endif
                         |(0x00u<<SCU_IPID_UART0)//串口调试输出
                         |(0x00u<<SCU_IPID_GPIO1)  
                         | (0x00u<<SCU_IPID_GPIO0)
               //          |(0x00u<<SCU_IPID_SDMMC0) 
               //          | (0x01u<<SCU_IPID_I2S)
                         //|(0x01u<<SCU_IPID_VIP) //nzy add    
                         | (0x01u<<SCU_IPID_DEBLK)
                         |(0x01u<<SCU_IPID_HIF)     
                         | (0x01u<<SCU_IPID_SRAMDSP)
                         |(0x01u<<SCU_IPID_SRAMARM) 
             //            | (0x01u<<SCU_IPID_DMA)      /* HSL@RK,20090516,can not close*/
                         |(0x01u<<SCU_IPID_DSP);

        scu_register_base->scu_clkgate1_config  = 
                         (0x01u<<(SCU_IPID_HSADC&0x1f))  
                         | (0x01u<<(SCU_IPID_DEMODFIFO&0x1f))
                         |(0x01u<<(SCU_IPID_DEMODBUS&0x1f))
                         | (0x01u<<(SCU_IPID_DEMODOTHER&0x1f))
                         |(0x01u<<(SCU_IPID_AGC&0x1f))     
                         | (0x01u<<(SCU_IPID_DOWNMIXER&0x1f))
                         |(0x01u<<(SCU_IPID_PREFFT&0x1f))  
                         | (0x01u<<(SCU_IPID_IQIMBALANCE&0x1f))
                         |(0x01u<<(SCU_IPID_FRAMEDET&0x1f))
                         | (0x01u<<(SCU_IPID_FFTMEM&0x1f))
                         |(0x01u<<(SCU_IPID_BITDITL&0x1f)) 
                         | (0x01u<<(SCU_IPID_VITERBIMEM&0x1f))
                         |(0x01u<<(SCU_IPID_PREFFTMEM&0x1f))
                         |(0x01u<<(SCU_IPID_VITERBI&0x1f))
                         |(0x01u<<(SCU_IPID_RS&0x1f))       
                         | (0x00u<<(SCU_IPID_EXTMEM&0x1f))
#ifdef SDRAM_MSDRAM
                         |(0x01u<<(SCU_IPID_SDRMEM&0x1f))
              //           |(0x01u<<(SCU_IPID_MSDRMEM&0x1f))
#else
               //          |(0x01u<<(SCU_IPID_SDRMEM&0x1f))
                         |(0x01u<<(SCU_IPID_MSDRMEM&0x1f))
#endif
                         | (0x01u<<(SCU_IPID_DEMOD&0x1f))
               //          |(0x00u<<(SCU_IPID_LCDCh&0x1f))
                         |0;

        scu_register_base->scu_clkgate2_config  = ((0x01u<<(SCU_IPID_DSPBUS&0x1f)) | (0x01u<<(SCU_IPID_EFUSE&0x1f)));

        reg_value = scu_register_base->scu_clksel0_config;
        reg_value &= ~((0x3<<16)|(0xf));
        reg_value |=  0
                        // (CLK_SDMMC1_DIV(4)       //sdmmc1 divider div (4)
                        //| CLK_SENSOR_24M        //sensor clock select 24MHz
                        //| CLK_48M_DIV(4)        //48MHz divider div (4)
                        //| CLK_USBPHY_24M        //USB PHY clock select 24MHz
                        | CLK_LCDC_CODPLL       //lcdc clock divide from codecpll
                        //| CLK_LCDC_DIV(1)       //lcdc divider div (8)
                        //| CLK_LCDC_DIVOUT       //lcdc clock from divider out
                        //| CLK_SDMMC0_DIV(4));     //sdmmc0 divder div (4)
                        | CLK_ARM_HCLK_31 //| CLK_ARM_HCLK_31       //arm clk:hclk = 2:1
                        | CLK_HCLK_PCLK_21 // | CLK_HCLK_PCLK_21     //hclk:pclk = 2:1
                        |0;
        scu_register_base->scu_clksel0_config = reg_value;
        
        scu_register_base->scu_clksel1_config = 0
                                | CLK_SHMEM1_DEMODCLK     //shmem1 clock from demod_clock
                                | CLK_SHMEM0_DEMODCLK   //shmem0 clock from demod_clock
                                | CLK_HSADCO_NORMAL     //hsadc clock output demod_clock/2
                                | CLK_GPS_DEMODCLK      //hsadc clock not from gps tuner input
                                | CLK_DEMOD_INTCLK      //demod_clk from internal divider out
                                | CLK_DEMOD_DSPPLL      //demod_clk divide from dsppll 
                                //| CLK_DEMOD_DIV(4)      //demod_clk divider div (4)
                                //| CLK_LSADC_DIV(128)      //lsadc_clk divider div (128)
                                //| CLK_CODEC_DIV(2)      //codec_clk divider div (2)
                                | CLK_CODEC_12M         //codec_Clk from 12MHz osc input
                                | CLK_CPLL_SLOW;        //codecpll work slow mode

        __rockchip_set_arm_clk_hw( 300 );
}



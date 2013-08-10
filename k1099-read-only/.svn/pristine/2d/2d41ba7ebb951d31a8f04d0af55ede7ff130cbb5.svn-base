/* include/asm-arm/arch-rockchip/rk_scu.h
 *
 * (C) Copyright hsl 2009
 *	Released under GPL v2.
 *
 * define scu base register & setup flags.
 * 
 * 
 *      
 */
/*
* 管理scu 相关的pll , clk设置。
* 系统一共存在4个时钟源(DSP暂时不考虑):
*       1. 24M 外部输入
*       2. AHB  CLK.
*       3. HPB(ARM)  CLK.
*       4. CODEC CLK.
* ARM CLK和 AHB CLK之间为固定的3:1 分频，所以可以看做同一个时钟.
* AHB和APB之间的分频可以为 2:1 或者 1:1 , 4:1很少使用，这个分频有系统统一
* 维护。
* 24M外部输入固定不可改变，系统不用维护。
* 其他时钟如果需要设置为 24M，则PLL直接进入 by pass 模式。
* 各个IP关于CLK的属性有:
 *      1. 时钟源
 *      2. 相对于时钟源的分频
 *      3.内部分频(如I2C,TIMER)
 * 对于第二点，目前按两种分类管理: 固定分频或者固定输出频率.
 * 如果两种情况不能实现所需功能，则需要一共自己的回调函数.
 * 第三种为IP内部的设置，scu不直接管理，通过提供相应的回调函数来实现。
 * 
 * 另外，可能存在暂时时钟源不能改变的情况，比如 sensor，USB 需要固定的48M频率，
 * 或者I2C正在通信的过程中(此时突然改变频率会导致通信出错)
 * 系统提供函数暂时锁定对应的总线，此时修改总线频率，有两种方式:
 * 修改失败，返回错误值，或者当前进程被堵塞，知道总线解锁.
 *
 *
*/
#ifndef __ROCK_CHIP_SCU_H_
#define __ROCK_CHIP_SCU_H_
#ifdef __arm__
#include <linux/list.h>
#include <asm/semaphore.h>
#endif
#include <linux/kobject.h>

#define         SCU_KOJB_SUPPORT                  1 
#define         SCU_SEM                                      0 /* 20100128,HSL@RK,need semophore or not ! */

#define         SCU_CLK_UNIT                            1000  // khz
#define         SCU_CLK_MHZ2KHZ                    1000
#if(defined(CONFIG_BOARD_IPADV5)||defined(CONFIG_BOARD_IPAD8)||defined(CONFIG_BOARD_NM701)||defined(CONFIG_BOARD_IPAD100V6))
#define		SDRAM_MSDRAM  //kevin add
#endif

#ifdef SDRAM_MSDRAM
#define SCU_MAX_AHB_CLK		(133)//(150) /* 090725,154-->24 will crash */
#else
#define SCU_MAX_AHB_CLK		(150) /* 090725,154-->24 will crash */
#endif
#define SCU_MAX_ARM_CLK		(SCU_MAX_AHB_CLK*4) /* FOR video test */  // 600
//#define SCU_AHB_DIV                            4    /* ahb fix div = 4*/

#define SCU_CLK_SPECIAL                   (1000000U)     /* FOR SPECIAL SETTING,REAL CLK*10000 */
#define SCU_CLK_112896                    (11289600*25U)     /* for codec 11.2896(44.1K*256) M PLL,i2s div=25 */
#define SCU_CLK_122880                    (12288000*25U)     /* for codec 12.288(48K*256) M PLL,i2s div=25 */
#if( SCU_MAX_ARM_CLK*SCU_CLK_MHZ2KHZ > SCU_CLK_SPECIAL )
#error "SCU_CLK_SPECIAL defined error!"
#endif

/* 模式可以设置为:固定分频，固定频率，或者用户自定义
 *  for SCU_MODE_FREQ&SCU_MODE_SETFREQ , unit = 1MHZ.
 */
enum { 
        SCU_MODE_DIV,  /* must keep div fix */
        SCU_MODE_FREQ,  /*out put clk  must <= set freqency */

        /* set one time , no limit */
        SCU_MODE_SETDIV,
        SCU_MODE_SETFREQ, 
        SCU_MODE_NONE,  /* do nothing , only set cur_clk=parm */
        };

enum { 
        SCU_PLLMODE_NORMAL,
        SCU_PLLMODE_BYPASS,
        };
enum 
{
    /*SCU CLK GATE 0 CON*/
    SCU_IPID_ARM = 0,
    SCU_IPID_DSP,
    SCU_IPID_DMA,
    SCU_IPID_SRAMARM,
    SCU_IPID_SRAMDSP,
    SCU_IPID_HIF,
    SCU_IPID_OTGBUS,    /* 6 */
    SCU_IPID_OTGPHY,
    SCU_IPID_NANDC,
    SCU_IPID_INTC,
    SCU_IPID_DEBLK,     /* 10 */
    SCU_IPID_LCDC,
    SCU_IPID_VIP,       /* as sensor */
    SCU_IPID_I2S,
    SCU_IPID_SDMMC0,    /* 14 */
    SCU_IPID_EBROM,
    SCU_IPID_GPIO0,
    SCU_IPID_GPIO1,
    SCU_IPID_UART0,
    SCU_IPID_UART1,
    SCU_IPID_I2C0,      /* 20 */
    SCU_IPID_I2C1,
    SCU_IPID_SPI0,
    SCU_IPID_SPI1,
    SCU_IPID_PWM,
    SCU_IPID_TIMER,
    SCU_IPID_WDT,
    SCU_IPID_RTC,
    SCU_IPID_LSADC,
    SCU_IPID_SHMEM0,
    SCU_IPID_SHMEM1,    /* 30 */
    SCU_IPID_SDMMC1,
    
    /*SCU CLK GATE 1 CON*/
    SCU_IPID_HSADC = 32,
    SCU_IPID_DEMODFIFO,
    SCU_IPID_DEMODBUS,
    SCU_IPID_DEMODOTHER,
    SCU_IPID_AGC,
    SCU_IPID_DOWNMIXER,
    SCU_IPID_PREFFT,
    SCU_IPID_IQIMBALANCE,
    SCU_IPID_FRAMEDET,  /* 40 */
    SCU_IPID_FFTMEM,
    SCU_IPID_BITDITL,
    SCU_IPID_VITERBIMEM,
    SCU_IPID_PREFFTMEM,
    SCU_IPID_VITERBI,
    SCU_IPID_RS,
    SCU_IPID_EXTMEM,
    SCU_IPID_SDRMEM,
    SCU_IPID_MSDRMEM,
    SCU_IPID_DEMOD,     /* 50 */
    SCU_IPID_LCDCh,

    SCU_IPID_ARMIBUS = 64,
    SCU_IPID_ARMDBUS,
    SCU_IPID_DSPBUS,
    SCU_IPID_EXPBUS,
    SCU_IPID_APBBUS,
    SCU_IPID_EFUSE,
    SCU_IPID_DTCM1,     /* 70 */
    SCU_IPID_DTCM0,
    SCU_IPID_ITCM,
    
    SCU_IPID_GATE_MAX,

    /* SOME IP NOT HAVE GATE */
    SCU_IPID_HCLK,
    SCU_IPID_PCLK,
//    SCU_IPID_SDMMC0,
//    SCU_IPID_LCDC,
//    SCU_IPID_USBPHY,  /* USE SCU_IPID_OTGPHY */
//    SCU_IPID_48M,       /*for USB , SENSOR */
//    SCU_IPID_SENSOR,    /* SCU_IPID_VIP */
//    SCU_IPID_SDMMC1,


    SCU_IPID_CODEC,
//    SCU_IPID_LSADC,
//    SCU_IPID_DEMOD,
//    SCU_IPID_HSADC,
//    SCU_IPID_SHMEM0,
//    SCU_IPID_SHMEM1,

        /* sample ipid */
    SCU_IPID_24M,
    SCU_IPID_12M,
    SCU_IPID_EXTERNAL,
    SCU_IPID_MAX
    
};

typedef int ip_id;
//typedef const char* ip_id;

#define SCU_IPID_USBPHY  SCU_IPID_OTGPHY
#define SCU_IPID_SENSOR  SCU_IPID_VIP

struct rockchip_scu_unit;
typedef int (*interal_change_clk )(  struct rockchip_scu_unit *ipinfo , int stage );
typedef int (*change_clk )( ip_id ip , int input_clk ); /* input_clk unit=HZ*/
typedef int (*change_parent )(struct rockchip_scu_unit *ipinfo, ip_id parent ); /* input_clk unit=HZ*/

/*register property */
#define SCU_PROPT_FIXDIV                          (1<<SCU_MODE_DIV)  /* have fix div,can not change freq itself */
                                                                               /*also for apb unit , such as timer,wd and pll */
#define SCU_PROPT_MAXFREQ                       (1<<SCU_MODE_FREQ)  /* max output <= set freq , always set max */

/*static property */
#define SCU_PROPT_PARERNT                       (1<<8)  /* may be other parent */
#define SCU_PROPT_BYPASS                        (1<<9)  /* fix clk input, can not change clk ,for look up root parent */
#define SCU_PROPT_HAVEPLL                       (1<<10)  /* HAVE pll to gen clk */
#define SCU_PROPT_MULT_PARENT             (1<<11)  /* can change parent */
                                                                              

/* dynamic property */
#define SCU_PROPT_DISABLE                       (1<<16)  /* be disable */
#define SCU_PROPT_LOCKED                        (1<<17)  /* FREQ be locked ,cannot change  */

/* tmp use */
#define SCU_PROPT_CLKSET                         (1<<24)  /* THIS CLK(tmp_div,tmp_clk) HAVE TMP SET */


/*
* add kobject .
* user kobject name replace name.
* add kobject attribute.
*/
struct rockchip_scu_unit
{
        ip_id           id;
        char            *name;
        unsigned long             propt;  

        ip_id           id_parent; /* for init easily */
        
        unsigned char            divbit_start;
        unsigned char            divbit_end;
        char                          divreg_index; /* 0 or 1,other invalid */
        
        /* use to change pll relative clk  */
        interal_change_clk      ifn; 
        change_parent           cpfn;
        
        struct list_head         list;  /* sibling list */
        struct list_head         child_list; /* child list */

       
        unsigned short             cur_div;
        unsigned short             tmp_div;
        
        unsigned int             cur_clk;
        unsigned int             max_clk;        /* can not set clk exceed this */
        unsigned int             tmp_clk;
        
        
         ip_id         lock_ipid;
#if SCU_SEM         
        struct semaphore     clk_sem;   /* lock for change source clock .sema_init  */
#endif        
        struct rockchip_scu_unit *parent;
        change_clk              ofn;    
        unsigned short           total_child;
        unsigned short           active_child;
        unsigned int              bak_clk ; /* clk for return from slowmode */

#if SCU_KOJB_SUPPORT
        /* for sysfs to user interface */
        struct kobject kobj;
        struct attribute attr_cur_clk;
        struct attribute attr_active;
//        struct attribute attr_max_clk;
#endif        
};
#define INVALID_REG_INDEX                       2
#define SCU_UNIT_HAVEDIV( p )                   ( p->divreg_index <= 1 )

#if SCU_KOJB_SUPPORT
#define SCU_SYSFS_NAME( p )             ( kobject_name(&p->kobj ))
#else
#define SCU_SYSFS_NAME( p )             ( p->name )
#endif
#define SCU_DIV_MAXVALUE( p )             ( 1<<(p->divbit_end-p->divbit_start+1) )

#define SCU_SETCLK_PARENT               0       /*      need parent to change to set clk */
#define SCU_SETCLK_OK                      1    /* can set to this clk */
#define SCU_SETCLK_IMPOSSIBLE        (-1) /* can not set to this clk */ 

/* debug */
//#define SCU_DEBUG
#ifdef SCU_DEBUG 
//#define SCU_BUG(fmt, argss...)           debug_print("---SCU@%s[%d]:" fmt "\n", __FILE__ , __LINE__ , ## argss)
#define SCU_BUG(fmt, argss...)           printk("---SCU@%s[%d]:" fmt "\n", __FILE__ , __LINE__ , ## argss)
#else
#define SCU_BUG(fmt, argss...)   
#endif


/* interface for scu */
int rockchip_scu_disableclk( ip_id id );
int rockchip_scu_enableclk( ip_id id );

int rockchip_scu_reset_unit( int index );

int rockchip_clk_lock_pll( ip_id id );
int rockchip_clk_unlock_pll( ip_id id );

int rockchip_clk_semlock_pll( ip_id id );
int rockchip_clk_semunlock_pll( ip_id id );

int __rockchip_scu_set_parent( ip_id id ,  ip_id id_parent , int idmode , int parm );
int __rockchip_scu_change_mode( ip_id id , unsigned char scu_mode , int parm );
void __rockchip_scu_print_tree( void ); /* for bug */
int __rockchip_clk_set_unit_clock( ip_id id , int new_clk )  ;
int __rockchip_scu_set_unit_div( ip_id id , int new_div )  ;

int rockchip_scu_register( ip_id id , unsigned char scu_mode , int parm  , change_clk fn ); /* ,int select_pll */

int rockchip_scu_apbunit_register( ip_id id , char* name , change_clk fn ); /* for apb not div unit register */


/* api for clk ,get unit = HZ , set unit = MHZ */
int __rockchip_clk_get_uint_clk( ip_id id );
int rockchip_clk_get_ahb( void );       /* HZ */
int rockchip_clk_get_apb( void );
int rockchip_clk_get_arm( void );
int rockchip_clk_get_ipsource( ip_id id );
int rockchip_clk_set_arm( int new_clk ); /* MHZ */

#endif /* __ROCK_CHIP_SCU_H_ */


/*
 *  rkscu/rk_kobject.c
 *
 * (C) Copyright hsl 2009
 *	Released under GPL v2.
 *
 * 
 * log:
 *      for scu unit sysfs 
 *      20070716, REMOVE attr max_clk,CAN NOT BE change.
 *      20090716,USE init12m active for kernel , IO mem dump.
 *      20090717,use init24m actiev for kernel function test.
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

#include <linux/uaccess.h>
#include <linux/errno.h>
#include <linux/poll.h>
#include <linux/timer.h>
#include <linux/spinlock_types.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <linux/kallsyms.h>
#include <asm/atomic.h>

#include <asm/arch/typedef.h>
#include <asm/arch/hardware.h>

#ifdef __i386__
#include "rk28_scu.h"
#else
#include <asm/arch/rk28_scu.h>
#endif
#include <asm/arch/rk28_debug.h>

#if SCU_KOJB_SUPPORT

static struct kset *scu_kset;
#define KTO_SCU_UINT( x )        container_of(x, struct rockchip_scu_unit, kobj)

extern int __rk28_scu_parse_cmd( char * cdb );

static char scu_init24m_cmd_write[128];
static ssize_t __scu_init24m_show_active(struct rockchip_scu_unit *p , char *buf )
{
        unsigned long flags;
        char    cmd[128];
        int      ret = -1;
        
        local_irq_save(flags);
        if( scu_init24m_cmd_write[0] ) {
                strcpy( cmd , scu_init24m_cmd_write );
        } else {
                cmd[0] = 0;
        }
        local_irq_restore(flags);

        if( cmd[0] ){
                ret = __rk28_scu_parse_cmd( cmd );
        }
        if( buf )
                ret = sprintf( buf ,"CMD:%s,ret=0x%x\n", scu_init24m_cmd_write  , ret );
        if( ret == 0 ) /* 0 meas try again!! */
                ret = 0XEEEE0000;
        return ret;
        
}

static ssize_t __scu_init24m_set_active(struct rockchip_scu_unit *p ,const char *buf )
{
        int len = strlen( buf );

        if( len < 3 )
                return -EIO;
        if( len > 127 )
                len = 127;
        while( buf[len-1] == '\n' || buf[len-1] == ' ' ) {
                len--;
        }
        memcpy( scu_init24m_cmd_write , buf , len );
        scu_init24m_cmd_write[len] = 0;
        len = __scu_init24m_show_active( p , NULL );
        return len;
}

#if 0 /* 20091228,HSL@RK,use tool 'io' instead */
unsigned int    * scu_kobj_addr;
unsigned int    scu_kobj_len;

static ssize_t __scu_init12m_show_active(struct rockchip_scu_unit *p , char *buf )
{
        int      total_len = scu_kobj_len;
        int      len = 0 , oneprint;
        unsigned int *reg_base = scu_kobj_addr;
        if( scu_kobj_addr == 0 )
                return -EIO;

        while( total_len > 0 ) {
                if( buf )  {
                        oneprint = sprintf(buf,
                                "%p:%08x  %08x  %08x  %08x\n",
                                reg_base,reg_base[0], reg_base[1], reg_base[2], reg_base[3] );
                        buf += oneprint;
                        }
                else {
                        oneprint = 4;
                        printk( "%p:%08x  %08x  %08x  %08x\n",
                                reg_base,reg_base[0], reg_base[1], reg_base[2], reg_base[3] );
                        }
                total_len -= 4;
                reg_base += 4;
                len += oneprint;
        }
        scu_kobj_len -= total_len;
        scu_kobj_addr += scu_kobj_len;
        
        return len;
        
}

static ssize_t __scu_init12m_store_active(struct rockchip_scu_unit *p , const char *buf )
{
        unsigned int addr;
        unsigned int len;
        int ret = strlen(buf);
        char        adr[10] = { 0};

        /* format 0xhhhhhhhh@dd , xx:base 16 , dd:base 10*/
        
        if( ret < 11 || buf[0] != '0' || ( buf[1] != 'x' && buf[1] != 'X' )  || buf[10] != '@' )
                return -EFAULT;
        memcpy( adr , buf+2 , 8 );
        sscanf( adr , "%x" , &addr );
        sscanf(buf+11, "%d",  &len );

        //SCU_BUG("addr=0x%x,len=%d" , addr , len );
        /* XXX: max read buf = 4K */
        if( len > 128 ) 
                len = 128;

        if ( addr >= AHB_BASEADD_PA &&  addr < AHB_BASEADD_HI_PA ) {
                scu_kobj_addr = (unsigned int *)IO_PA2VA_AHB( addr );
        } else if ( addr >= APB_BASEADD_PA &&  addr < APB_BASEADD_HI_PA ) {
                scu_kobj_addr = (unsigned int *)IO_PA2VA_APB( addr );
        } else if ( addr >= DSP_BASE_ADDR &&  addr < (DSP_BASE_ADDR+64*1024) ) {
                scu_kobj_addr = (unsigned int *)DSP_BASEADD_VA;
        } else if ( addr >= 0xc0008000 && addr < 0xd0000000 ) {
                scu_kobj_addr = (unsigned int *)addr;
        } else {
                scu_kobj_addr = 0;
                scu_kobj_len = 0;
                return -EIO;
        }
        scu_kobj_len = len;
        __scu_init12m_show_active( p , NULL );
        return ret;
        
}

#endif
/* default kobject attribute operations */
static ssize_t scu_kobj_attr_show(struct kobject *kobj, struct attribute *attr,
			      char *buf)
{
        ssize_t ret = -EIO;
        struct rockchip_scu_unit *p = KTO_SCU_UINT( kobj );

        if ( attr == &p->attr_cur_clk ) {
                return sprintf(buf, "%d\n", p->cur_clk );
//        } else if ( attr == &p->attr_max_clk ) {
//                return sprintf(buf, "%d\n", p->max_clk );
        } else if ( attr == &p->attr_active ) {
                if( p->id == SCU_IPID_24M)
                        return __scu_init24m_show_active(p , buf );
                //if( p->id == SCU_IPID_12M)
                //        return __scu_init12m_show_active(p , buf );
                return sprintf(buf, "tmp_div:%d,tmp_clk:%d,cur_clk:%d,bak_clk:%d,max_clk:%d,prot:%lx,"
                        "tol_child:%d,act_child:%d\n" ,
                        p->tmp_div,p->tmp_clk,p->cur_clk,p->bak_clk,p->max_clk,p->propt,
                        p->total_child , p->active_child );
        } else {
                SCU_BUG("unknow attr %s" , attr->name );
        }
        return ret;
}

static ssize_t scu_kobj_attr_store(struct kobject *kobj, struct attribute *attr,
			       const char *buf, size_t count)
{
        ssize_t ret = -EIO;
        int     tmp =0;
        struct rockchip_scu_unit *p = KTO_SCU_UINT( kobj );

        sscanf(buf, "%d", &tmp );
        //SCU_BUG("scu %s kojb set:%s to %d,buf=%s" , SCU_SYSFS_NAME(p) , attr->name , tmp , buf); 
        if ( attr == &p->attr_cur_clk ) {
                if( p->id == SCU_IPID_ARM) {
                        ret = rockchip_clk_set_arm( tmp );
                } else
                        ret = __rockchip_clk_set_unit_clock( p->id  , tmp );
//        } else if ( attr == &p->attr_max_clk ) {
//                ret = __rockchip_scu_change_mode( p->id , SCU_MODE_FREQ , tmp );
        } else if ( attr == &p->attr_active ) {
                //if( p->id == SCU_IPID_12M)
                //        return __scu_init12m_store_active(p , buf );
                if( p->id == SCU_IPID_24M)
                        return __scu_init24m_set_active( p , buf );
                if( tmp ) 
                        ret = rockchip_scu_enableclk( p->id );
                else 
                        ret = rockchip_scu_disableclk( p->id );
        } else {
                SCU_BUG("unknow attr %s" , attr->name );
        }
        if( ret >= 0 )
                ret = count;
        return ret;
}

struct sysfs_ops scu_sysfs_ops = {
	.show	= scu_kobj_attr_show,
	.store	= scu_kobj_attr_store,
};


static void scu_kset_release(struct kobject *kobj)
{
        SCU_BUG("kobject: '%s' (%p): %s\n",
                kobject_name(kobj), kobj, __FUNCTION__);
        
}

static struct kobj_type scu_kset_ktype = {
	.sysfs_ops	= &scu_sysfs_ops,
	.release = scu_kset_release,
	.default_attrs = NULL,
};
static void __rockchip_scu_node_attr_init(struct attribute *attr , char * name )
{
        attr->name = name;
        attr->mode = 0644;
        attr->owner = NULL;
}

int __rockchip_scu_node_kobject_init(struct rockchip_scu_unit *p )
{
        struct attribute *scu_def_attrs[4];
        int     result;

        if( !scu_kset )
                return 0;
        __rockchip_scu_node_attr_init(&p->attr_cur_clk , "cur_clk");
        scu_def_attrs[0] = &p->attr_cur_clk;
        __rockchip_scu_node_attr_init(&p->attr_active, "active");
        scu_def_attrs[1] = &p->attr_active;
        
        //__rockchip_scu_node_attr_init(&p->attr_max_clk, "max_clk");
        //scu_def_attrs[2] = &p->attr_max_clk;
        
        scu_def_attrs[2] = NULL;
        scu_kset_ktype.default_attrs = &scu_def_attrs[0];
        memset(&p->kobj,0,sizeof(struct kobject));
        p->kobj.kset = scu_kset;
        kobject_init( &p->kobj , &scu_kset_ktype );
        if ( p->parent )
                result = kobject_add( &p->kobj , &p->parent->kobj, "%s", p->name );
        else 
                result = kobject_add( &p->kobj , NULL , "%s", p->name );
                //result = kobject_add( &p->kobj , kernel_kobj , "%s", p->name );
        if( result < 0 ) {
                SCU_BUG("add kobject: '%s' failed!", p->name );
                kobject_put( &p->kobj );
                return -ENOMEM;
                }
        return 0;
}

int __rockchip_scu_kset_init( void )
{
        scu_kset = kset_create_and_add("scu", NULL, NULL ); /*kernel_kobj*/
        if (!scu_kset)
                return -ENOMEM;
        return 0;
}

void __rockchip_scu_kset_exit( void )
{
        kset_unregister(scu_kset);
}

#endif  /* SCU_KOJB_SUPPORT */


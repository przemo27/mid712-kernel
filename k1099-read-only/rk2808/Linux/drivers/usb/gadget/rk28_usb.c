/*
 * Gadget Driver for rockchip usb
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

#include "rk28_usb.h"
#include <linux/syscalls.h>

extern int kernel_getlog( char ** start  , int * offset , int* len );
extern int kernel_getlog_start_length( int* start  , int * len );
extern int android_main_getlog( char ** start  , int * offset , int* len );
extern int android_main_getlog_start_length( int* start  , int * len );
extern int __rk28_scu_parse_cmd( char * cdb );
extern volatile int rk28_pm_status ;
extern void rockchip_timer_freeze(int freeze ); /* freeze timer while at break point */
extern void panic(const char * fmt, ...);
void __rk28_force_signal(struct task_struct *tsk, unsigned long addr,
		unsigned int sig, int code );
extern int rk28_msc_switch( int action );
extern int kld_get_tag_data( unsigned int tag , void** data );
extern int rk28_restart( int type ) ;

static int rkusb_do_thread_cmd( struct rkusb_dev *dev );
static int rkusb_command( struct rkusb_dev *dev );

#define __at_break_point_freeze()                (rk28_pm_status == 4 )
#define  __rkusb_read_enable( dev )             (1) // for msc ext,must can read .(rk28_system_crash>=2)
#define  __rkusb_write_enable( dev )            (rk28_system_crash>=RKDBG_WRITE)
char    rkusb_en_write[16]= "@rkusb_all";
static struct rkusb_dev *_rkusb_dev;
extern struct __except_content rkusb_except_content;

#include "rk28_usb_io.c"        /* 20100204,split for tx/rx packet. */


#define check_xtranfs_len( count )    do{   if( DEV_LENGTH( dev ) != count ) {\
                        return -1; } } while( 0 )

#define check_log_valid( dev )          (dev->log[DEV_LUN(dev)].total_len && dev->log[DEV_LUN(dev)].va_start)                 

static int rkusb_read_log( struct rkusb_dev *dev )
{
         switch ( DEV_FUNC(dev) ) {
         case FUNC_LOG_GETLUNS:       /* first int:luns , next int:sizeof log_buffer */
                {
                int     num_size[2];
                check_xtranfs_len( 8 );
                num_size[0] = dev->luns;
                num_size[1] = sizeof(struct log_buffer );
                rkusb_normal_data_xfer_onetime( dev , num_size );
                } 
                return 0;
        case FUNC_LOG_GETSTRUCT: /* report all log info */
                {
                // check_xtranfs_len( sizeof dev->log ); sizeof dev->log may change.
                if(  !check_log_valid( dev ) )
                        break;
                rkusb_normal_data_xfer_onetime( dev , dev->log );
                }
                return 0;
        case FUNC_LOG_INIT:
                {
                int i = DEV_LUN( dev );
                #if (LOG_NUM>2)
                static int rkusb_shell_initlog( struct log_buffer *p  );
                if( i == LUN_SHELL ) {
                        if( rkusb_shell_initlog( &dev->log[i] )  ) {
                                return RKUSB_CB_FAILD_CSW;
                        }
                }
                #endif
               dev->log[i].getlog(&dev->log[i].va_start , &dev->log[i].offset , &dev->log[i].total_len);
                }
                return RKUSB_CB_OK_CSW;
        case FUNC_LOG_UNINIT:
                {
                        if( DEV_LUN(dev) == LUN_SHELL ){
                                dev->log[DEV_LUN(dev)].property &= ~LOG_PROT_MAYWRITE;
                        }
                }
                return RKUSB_CB_OK_CSW;
        case FUNC_LOG_OFFLEN: /* read new log info , first int=start offset , next int=len */
                {
                int offset_len[2];
                if(  !check_log_valid( dev ) )
                        break;
                check_xtranfs_len( 8 );
                dev->log[DEV_LUN(dev)].getlog_start_length( &dev->log[DEV_LUN(dev)].offset , &dev->log[DEV_LUN(dev)].len );
                offset_len[0] = dev->log[DEV_LUN(dev)].offset;
                offset_len[1] = dev->log[DEV_LUN(dev)].len;
                rkusb_normal_data_xfer_onetime( dev , offset_len );
                }
                return 0;
        default: 
                break;
        }
        return -1;
}

static int rkusb_get_log( struct rkusb_dev *dev )
{
        /*
         * 20090929,hsl,use diffirent LBA as diffirent lun.
         */
        if( DEV_LUN(dev) < dev->luns && (dev->log[DEV_LUN(dev)].property & LOG_PROT_READ )  )
                return rkusb_read_log( dev );
        return -1;
}

#if (LOG_NUM>2)
static int rkusb_write_shell_callback( struct rkusb_dev *dev )
{
        rk28printk("%s:actual len=%d\n" ,__func__ , dev->req_out->actual);
        
        if( dev->log[DEV_LUN(dev)].setlog( &dev->log[DEV_LUN(dev)] , dev->req_out->buf , dev->req_out->actual ) 
                >= dev->req_out->actual ) {
                return RKUSB_CB_OK_CSW;
        }
        return RKUSB_CB_FAILD_CSW;
}

/*
 *      write sdram, just send command to shell.
 */
static int rkusb_set_log( struct rkusb_dev *dev )
{
        rk28printk("%s:lun=%d,lba=0x%08x\n" ,__func__ , DEV_LUN(dev) , DEV_OFFSET(dev));
        if( DEV_LUN(dev) < dev->luns && (dev->log[DEV_LUN(dev)].property & LOG_PROT_WRITE )
                && !__system_crashed() ) {
                rkusb_normal_data_xfer( dev , rkusb_write_shell_callback );
                return RKUSB_CB_OK_NONE;
        } 
        return RKUSB_CB_FAILD;
}
#else
#define rkusb_set_log(dev )             RKUSB_CB_FAILD
#endif

extern const char linux_banner[];
extern const char rockchip_version[];
extern const char system_type[];
int     adb_support = 0;
extern void adb_function_enable(int enable);
extern int usb_vbus_status;        /* 1: in , 0 :out */

/* get verion , return size and sdram addr(lba) */
static int rkusb_ucmd_get_version( struct rkusb_dev *dev )
{
        int r = RKUSB_CB_OK_NONE;
        unsigned int off_len[2];
        
        switch( DEV_LUN(dev) ) {
        case LUN_USCM_VER_LOADER:
                {
                off_len[1] = kld_get_tag_data( 0X524B0002 , (void**)&off_len );
                }
                break;
        case LUN_USCM_VER_KERNEL:
                off_len[0] = (unsigned int)linux_banner;
                off_len[1] = strlen(linux_banner)+1;
                break;
        case LUN_USCM_VER_ANDROID:
                off_len[0] = (unsigned int)rockchip_version;
                off_len[1] = strlen(rockchip_version)+1;
                break;
        case LUN_USCM_VER_OPENCORE:
        default:
                r = RKUSB_CB_FAILD;
                break;
        }
        if( r == RKUSB_CB_OK_NONE )
                rkusb_normal_data_xfer_onetime(dev , off_len );
        return r;
}

static int rkusb_usb_command( struct rkusb_dev *dev )
{
        int r = RKUSB_CB_FAILD;
        rk28printk("%s::func=%d,len=%d\n" , __func__ , DEV_FUNC(dev),DEV_LENGTH(dev));
        switch( DEV_FUNC(dev) ) {
                case FUNC_UCMD_DEVREADY:
                        r = RKUSB_CB_OK_CSW;
                        break;
                case FUNC_UCMD_DISCONNECTMSC:
                        rk28_msc_switch( 0 );
                        r = RKUSB_CB_OK_CSW;
                        break;
                case FUNC_UCMD_SYSTEMINFO:
                        {
                        unsigned int off_len[2];
                        off_len[0] = (unsigned int)system_type;
                        off_len[1] = strlen(system_type)+1;
                        rkusb_normal_data_xfer_onetime(dev , off_len );
                        r = RKUSB_CB_OK_NONE;
                        }
                        break;
                case FUNC_UCMD_RESTART:
                        rk28printk("get restart cmd,lun=%d\n" , DEV_LUN(dev));
                        if( DEV_LUN(dev) == 0 ) { /* normal restart */
                                rk28_restart(0);
                        } else if( DEV_LUN(dev) == 1 ) { /* to loader rkusb */
                                rk28_restart(1);
                        }
                        break;
                case FUNC_UCMD_GETVERSION:
                        r = rkusb_ucmd_get_version( dev );
                        break;
                case FUNC_UCMD_GETCHIPINFO:
                        {
                        unsigned int off_len[2];
                        off_len[1] = kld_get_tag_data( 0X524B0000 , (void**)&off_len );
                        rkusb_normal_data_xfer_onetime(dev , off_len );
                        r = RKUSB_CB_OK_NONE;
                        }
                        break;
                case FUNC_UCMD_GETSN:
                        {
                        unsigned int off_len[2];
                        off_len[1] = kld_get_tag_data( 0X524B0006 , (void**)&off_len );
                        rkusb_normal_data_xfer_onetime(dev , off_len );
                        r = RKUSB_CB_OK_NONE;
                        }
                        break;
                case FUNC_UCMD_DEBUGINFO:
                        {
                        RKDEBUG_INFO      *dbi = DEV_FAST_ALLOC(dev,RKDEBUG_INFO);
                        dbi->size = sizeof(RKDEBUG_INFO);
                        dbi->adb_support = adb_support;
                        dbi->dbg_level = __rkusb_write_enable(dev)?2:(__rkusb_read_enable(dev)?1:0);
                        dbi->tr_length = dev->req_buf_length;
                        dbi->save_length = dev->buf_length;
                        dbi->save_buf = (unsigned long)__rkusb_rwbuffer_start(dev);
                        if( dev->fsg )
                                strcpy(dbi->cur_device,"msc");
                        else
                                strcpy(dbi->cur_device,"adb");
                        strcpy( dbi->cur_version,RKUSB_KVERSION);
                        #if 0
                        dbi->cmds[0]=K_FW_SDRAM_READ_10;
                        dbi->cmds[1]=K_FW_SDRAM_WRITE_10;
                        dbi->cmds[2]=K_FW_TRACE;
                        dbi->cmds[3]=K_FW_GETLOG;
                        dbi->cmds[4]=K_FW_FUNCALL;
                        dbi->cmds[5]=K_FW_GETSYMB;
                        dbi->cmds[6]=K_FW_GETRESULT;
                        dbi->cmds[7]=K_FW_USBCMD;
                        #endif
                        rkusb_normal_data_xfer_onetime(dev , dbi );
                        r = RKUSB_CB_OK_NONE;
                        }
                        break;
                 case FUNC_UCMD_OPENADB:
                        {
                        /* lun:0 close,1: open */
                        adb_function_enable(DEV_LUN(dev));
                        usb_vbus_status = 0; /* force to reconnect! */
                        r = RKUSB_CB_OK_CSW;
                        }
                        break;
                default:
                        break ;
        }
        return r;
}

/*
 * 20100111,HSL@RK,use LUN to dicide call at task or irq.
 * lun==0: at irq , other at task.
 */
static int rkusb_function_call_callback( struct rkusb_dev *dev )
{
         int     ret;
         rk28printk("%s::cmd=%s,crash=%d\n" , __func__ , (char*) dev->req_out->buf , rk28_system_crash );
         if( !__rkusb_write_enable() ) {
                //if( !strcmp((char*) dev->req_out->buf , rkusb_en_read) ) {
                //        rk28_system_crash = 2;
                //        rkusb_send_csw_result( dev , rk28_system_crash);
                //        return RKUSB_CB_OK_NONE;
                //}
                if( !strcmp((char*) dev->req_out->buf , rkusb_en_write) ) {
                        rk28_system_crash = RKDBG_WRITE;
                        rkusb_send_csw_result( dev , rk28_system_crash);
                        return RKUSB_CB_OK_NONE;
                }
                return RKUSB_CB_FAILD;
        }
        if( DEV_LUN(dev) == 0 ) {
                ret = __rk28_scu_parse_cmd((char*) dev->req_out->buf);
                rk28printk("cmd ret = 0x%x(%d)" , ret ,ret );
                rkusb_send_csw_result( dev , ret );
        } else {
                rkusb_wakeup_thread( dev );
        }
        return RKUSB_CB_OK_NONE;
}

static int rkusb_function_call( struct rkusb_dev *dev )
{
        rkusb_normal_data_xfer( dev , rkusb_function_call_callback );   
        return RKUSB_CB_OK_NONE;
}

static void __rksub_copy_task_info( TASK_INFO *ti , struct task_struct * p)
{
        get_task_comm(ti->comm , p);
        ti->size = sizeof (*ti );
        ti->state = p->state;
        ti->stack = p->stack;
        ti->flags = p->flags;
        ti->prio = p->prio;
        ti->static_prio = p->static_prio;
        ti->normal_prio = p->normal_prio;
        ti->sched_class = (void*)p->sched_class;
        ti->sum_exec_runtime = p->se.sum_exec_runtime;
        ti->mm = p->mm ;
        ti->active_mm = p->active_mm;
        ti->pid = p->pid;
        ti->rt_priority = p->rt_priority;
        ti->utime = p->utime;
        ti->stime = p->stime;
        if( p->files ) {
                void * pt = p->files->fd_array;
                memcpy( ti->fd_array , pt , sizeof (ti->fd_array) );
        } else
                memset( ti->fd_array , 0 , sizeof ti->fd_array );
                
//        strcpy( ti->cmd , ti->comm );
//        if( !__system_crashed() && !__at_break_point_freeze() )
//                rkusb_get_task_cmdline( p , ti->cmd , sizeof( ti->cmd) );
                
        ti->tgid = p->tgid;
        ti->start_time = timespec_to_ns( &p->start_time);
        
        ti->uid = p->uid;
        ti->euid = p->euid ; 
        ti->suid = p->suid;
        ti->fsuid = p->fsuid;

        ti->gid = p->gid;
        ti->egid = p->egid ; 
        ti->sgid = p->sgid;
        ti->fsgid = p->fsgid;
}

static int rksub_get_sym_tasks( struct rkusb_dev *dev )
{
        ALL_TASK                 at;
        TASK_INFO               *ti;
        struct task_struct *g, *p;
        int     buf_full = 0;
        int     tn , tntoal;
        
        ti = (TASK_INFO*)__rkusb_rwbuffer_start( dev );
        at.size = sizeof( at );
        at.start_ts = ti;
        at.ti_size = sizeof(*ti);
        tntoal = 0;
        tn = 0;
        read_lock(&tasklist_lock);
        do_each_thread(g, p) {
                if( !buf_full  ) {
                        if( ti+1 > (TASK_INFO*)__rkusb_rwbuffer_end( dev ) ) {
                                buf_full = 1;
                                tn = tntoal;
                        } else {
                                __rksub_copy_task_info( ti , p );
                                ti++;
                        }
                }
                tntoal ++;
        } while_each_thread(g, p);
        read_unlock(&tasklist_lock);
        if( !tn )
                tn = tntoal;
        at.task_num = tn;
        at.task_total_num = tntoal;
        at.now = ktime_to_ns(ktime_get() );
        rkusb_normal_data_xfer_onetime( dev , &at );
        return RKUSB_CB_OK_NONE;
}

static void __rksub_copy_task_vma(struct rkusb_dev *dev ,
        TASK_VAM *tm , struct task_struct * p , char *dest )
{
        TASK_VMA_INFO   *tv;
        struct mm_struct *mm;
        struct vm_area_struct * mmap;
        int       buf_full = 0;
        int       n,ntotal;

        tv = (TASK_VMA_INFO*)dest; 
        mm = p->mm;
        tm->size = sizeof(*tm);
        tm->flag = (1<<0);
        tm->pid = p->pid;
        if( !mm ) {
                mm = p->active_mm;
                tm->flag &= ~(1<<0);
        }
        tm->start_code = mm->start_code;
        tm->end_code= mm->start_code;
        tm->start_code = mm->end_code;
        tm->start_data= mm->start_data;
        tm->end_data = mm->end_data;
        tm->start_brk = mm->start_brk;
        tm->brk = mm->brk;
        tm->start_stack = mm->start_stack;
        tm->arg_start = mm->arg_start;
        tm->arg_end = mm->arg_end;
        tm->env_start = mm->env_start;
        tm->env_end = mm->env_end;  
        tm->vi = tv;
        tm->vi_size = sizeof(TASK_VMA_INFO);
        mmap = mm->mmap;
        n = ntotal = 0;
        while( mmap ) {
                if( buf_full ) 
                        goto loop_none;
                tv->size = sizeof(TASK_VMA_INFO);
                tv->start = mmap->vm_start;
                tv->end = mmap->vm_end;
                tv->flags = mmap->vm_flags;
                tv->pgoff = mmap->vm_pgoff;  /* 20100226,HSL@RK,add offset in file*/
                tv->file = mmap->vm_file;
                memset( tv->mape_filename , 0 , sizeof(tv->mape_filename) );
                if( tv->file ) {
                        if( strlen( FILENAME(mmap->vm_file)  ) >= sizeof(tv->mape_filename) )
                                strncpy(tv->mape_filename,FILENAME(mmap->vm_file)+
                                        strlen( FILENAME(mmap->vm_file)  )-sizeof(tv->mape_filename)+1,
                                        sizeof(tv->mape_filename)-1 );
                        else
                                strcpy(tv->mape_filename,FILENAME(mmap->vm_file) );
                }
                tv++;
                if( tv >= (TASK_VMA_INFO*)__rkusb_rwbuffer_end( dev ) ) {
                        buf_full = 1;
                        n = ntotal+1;
                }
loop_none:                        
                ntotal++;
                mmap = mmap->vm_next;
                
        }
        if( !n )
                n = ntotal;
        tm->vi_xfer = n;
        tm->vi_tolnum = ntotal;
        rk28printk("%s::task=%s,pid=%d,total vma=%d\n", __func__ , p->comm , p->pid , ntotal );
}

/* lab : = pid .*/
static int rkusb_get_task_mm( struct rkusb_dev *dev )
{
        TASK_VAM        tm;
        struct task_struct *p;
        int     pid = DEV_OFFSET(dev);
        if( pid == 0 )
                p = current ;
        else {
                p = find_task_by_pid( pid );
                if( !p )
                        return RKUSB_CB_FAILD;
        }
        __rksub_copy_task_vma( dev , &tm , p , __rkusb_rwbuffer_start( dev ));
        rkusb_normal_data_xfer_onetime( dev , &tm );
        return RKUSB_CB_OK_NONE;
}

static int rkusb_get_kernel_symbols( struct rkusb_dev *dev )
{
        struct __kernel_symbol ks;
        ks.size = sizeof( ks );
        ks._stext = _stext;
        ks._text = _text;
        ks._etext = _etext;
        ks._data = __data_start ; //_data;
        ks._edata = _edata;
        ks.__bss_start = __bss_start;
        ks._end = _end;

        ks.kallsyms_start = (unsigned char*)kallsyms_addresses;
        ks.total_syms_size = (unsigned char*)__start_rodata - ks.kallsyms_start;
        ks._kallsyms_num_syms = kallsyms_num_syms;
        ks._kallsyms_addresses = (unsigned long*)kallsyms_addresses;
        ks._kallsyms_markers = (unsigned long*)kallsyms_markers;
        ks._kallsyms_names = (unsigned char*)kallsyms_names;
        ks._kallsyms_token_index = (unsigned short*)kallsyms_token_index;
        ks._kallsyms_token_table = (unsigned char*)kallsyms_token_table;
        rkusb_normal_data_xfer_onetime( dev , &ks );
        rk28printk("symbols addres=0x%p,names=0x%p,syms=0x%lx\n",
                ks._kallsyms_addresses,ks._kallsyms_names,ks._kallsyms_num_syms);
        return RKUSB_CB_OK_NONE;
}

/*   20100316,HSL@RK,copy info of crash task for 
  *   later debug.things: copy task_struct,copy vma,copy some stack.
  *   data put into __rkusb_rwbuffer_start buffer.
  */
static void rkusb_record_crash_task( void ) 
{
        struct rkusb_dev *dev = _rkusb_dev;
        struct task_struct  *t = current;
        
}
struct __except_content rkusb_except_content;
void set_except_regs(struct pt_regs *new_regs , int mode , 
        unsigned long addr, unsigned int fsr)
{
        rkusb_except_content.addr = addr ;
        rkusb_except_content.fsr = fsr;
        rkusb_except_content.mode = mode;
        memcpy(rkusb_except_content.uregs,new_regs , sizeof(struct pt_regs) );
        if( user_mode(new_regs ) ) {
                if( __rkusb_debug_mod() ) {
                        panic("panic by user mode fault\n");
                } else {
                        rkusb_record_crash_task( );
                }
        }
}

extern int profile_check( void *pc_buf );
static int rkusb_get_symbol( struct rkusb_dev *dev )
{
        switch ( DEV_FUNC(dev) ) {
        case FUNC_GSYM_KERNEL:
                return rkusb_get_kernel_symbols( dev );
        case FUNC_GSYM_GETTASKS:
                return rksub_get_sym_tasks( dev );
        case FUNC_GSYM_GETTASKVM:
                return rkusb_get_task_mm( dev );
        case FUNC_GSYM_PROFILE:
                {
                char * buf = __rkusb_rwbuffer_start(dev);
                PROFILE_INFO   pe;
                pe.size = sizeof( pe );
                pe.npc = profile_check( buf );
                pe.buf = (unsigned long)buf;
                pe.now = ktime_to_ns( ktime_get() );
                rk28printk("%s::profile n=%ld\n" , __func__ , pe.npc );
                rkusb_normal_data_xfer_onetime( dev , &pe );
                return 0;
                }
        default:
                break;
        }
        return RKUSB_CB_FAILD;
}

#include "rk28_trace.c"
#include "rk28_xfile.c"

static int rkusb_get_lastcmd_result( struct rkusb_dev *dev )
{
        if( DEV_LENGTH(dev) != RKUSB_RESULT_LEN )
                return RKUSB_CB_FAILD;
        rkusb_normal_data_xfer_onetime( dev , &dev->cr );
        return RKUSB_CB_OK_NONE;
}
static int rkusb_command( struct rkusb_dev *dev )
{
        int r= RKUSB_CB_FAILD;
        //dev->cb.DataTransferLength = __be32_to_cpu( dev->cb.DataTransferLength );
        //dev->cb.Lba = __be32_to_cpu( dev->cb.Lba );
        //dev->cb.pid = __be16_to_cpu( dev->cb.pid );

        rkusb_print_cb( dev );
        if( DEV_LENGTH(dev) > dev->req_buf_length ){
                rk28printk("data length[0x%x] > limit[0x%x]\n" , DEV_LENGTH(dev),dev->req_buf_length);
                DEV_LENGTH(dev) = dev->req_buf_length;
        }
        //dev->thread_task->mm = NULL;
        dev->slave_task = NULL;
        switch ( DEV_CMD(dev) )
        {
        case K_FW_GETRESULT:
                r = rkusb_get_lastcmd_result( dev );
                break;
        case K_FW_GETLOG:				
                r = rkusb_get_log( dev );
                break;
        case K_FW_SETLOG:				
                r = rkusb_set_log( dev );
                break;
        case K_FW_TRACE:
               r = rkusb_do_trace( dev );
               break;
        case K_FW_USBCMD:
               r = rkusb_usb_command( dev );
               break;
        case K_FW_XFERFILE:
                r = rkusb_do_xfer_file( dev );
                break;
        case K_FW_FUNCALL:
                r = rkusb_function_call( dev );
                break;
        case K_FW_SDRAM_READ_10:
                r = rkusb_read_sdram( dev );
                break;
        case K_FW_SDRAM_WRITE_10:
                r = rkusb_write_sdram( dev );
                break;
        case K_FW_GETSYMB:
                r = rkusb_get_symbol( dev );
                break;
        case K_FW_TASKFUN:
                r = rkusb_task_cmd( dev );
                break;
        default:
                return RKUSB_NOT_HANDLED;
        } 
        if( RKUSB_CB_FAILD == r ) {
                /* 20100330,HSL@RK,for failed cmd,must handle it */
                DEV_OFFSET(dev) = 0;
                rkusb_normal_data_xfer( dev , rkusb_failed_cb ); 
                return 0 ; /* RKUSB_NOT_HANDLED;*/
        } else if( RKUSB_CB_OK_CSW == r )
                rkusb_send_csw( dev , RKUSB_STATUS_PASS );
        else if( RKUSB_CB_FAILD_CSW == r )
                rkusb_send_csw( dev , RKUSB_STATUS_FAIL);
        return 0;
}

#if (LOG_NUM>2)

static int rkusb_shell_w_off = 0;
static int rkusb_shell_r_off = 0;
/* for user application STDOUT */
#define LOG_SHELL_GETOFFSET( p , off )        (off&(p->total_len-1))
static int rkusb_shell_writelog( struct file *file, const char __user *buf,
						size_t count, loff_t *ppos)
{
        struct log_buffer *p = &_rkusb_dev->log[2];
        int ret = 0;
        int written = 0;
        size_t size;
        if( !(p->property & LOG_PROT_MAYWRITE ) )
                return 0;
        if( count == 2 && buf[0] == '#' && buf[1] == ' ' ) {
                //rk28printk("%s:found last statement\n" ,__func__ );
                return count ;
        }
        
        //rkusb_print_bin( (char*)buf , count );
        if( !p->va_start )
                return 0;
        for (;;) {
	         size = count;	
                         if( size + LOG_SHELL_GETOFFSET(p,rkusb_shell_w_off) > p->total_len )
                                size = p->total_len - LOG_SHELL_GETOFFSET(p,rkusb_shell_w_off);
		memcpy(p->va_start+LOG_SHELL_GETOFFSET(p,rkusb_shell_w_off) , buf, size );
		written += size;
		buf += size;
		count -= size;
                         spin_lock( &_rkusb_dev->lock );
                         rkusb_shell_w_off += size;
                         spin_unlock( &_rkusb_dev->lock );
                         ret = written;
	        if (!count)
		break;
                        ret = -ERESTARTSYS;
                        if (signal_pending(current))
                	break;
                        cond_resched();
	}
        return ret;
}

extern void * console_register_direct_write( void * write );
extern struct console *console_drivers;
static int rkusb_shell_initlog( struct log_buffer *p  )
{
        int     indx;
        struct tty_driver *tty_drv = console_drivers->device( console_drivers , &indx );
        struct tty_struct *tty_s = tty_drv->ttys[indx ];
        if( p->private_data )
                return 0;
        if( !tty_s )
                return -EIO;
        
        p->total_len = 4096;
        p->va_start = kmalloc(p->total_len , GFP_ATOMIC );
        if( !p->va_start ) {
                p->total_len = 0;
                p->property = 0;
                p->private_data = NULL;
                return -ENOMEM;
        }
        rk28printk("%s:register shell write\n" ,__func__ );
        p->offset = p->len = 0;
        p->private_data = tty_s;
        console_register_direct_write( rkusb_shell_writelog );
        return 0;
}

/*
 *   return : = count ok, else failed 
 */
static int rkusb_shell_log_write( struct log_buffer *p , char *buf, int count )
{
        char *pt = buf;
        struct tty_struct *tty_s = (struct tty_struct *)p->private_data;
        int     i = 0;
        int flag = TTY_NORMAL;

        buf[count] = 0;
        rk28printk("%s:log=%s,buf=%s,count=%d,tty_s=0x%p\n" ,__func__ , p->name , pt , count , tty_s );
        if( !tty_s ) {
                return 0;
        }
        #if 0 /* add at pc tools */
        if ( buf[count-1]  != '\n' ) {
                buf[count++] = '\n';
                buf[count] = 0;
        }
        #endif
                
        while( *pt && i < count ) {
                tty_insert_flip_char(tty_s, *pt , flag);
                pt++;
                i++;
        }
        tty_flip_buffer_push( tty_s );
        p->property |= LOG_PROT_MAYWRITE;
        return count;
}

static int rkusb_shell_getlog( char ** start  , int * offset , int* len )
{
        return 0;
}

static int rkusb_shell_getlog_start_length( int * start , int* len )
{
        struct log_buffer *p = &_rkusb_dev->log[2];
        *start = LOG_SHELL_GETOFFSET(p,rkusb_shell_r_off);
        *len = rkusb_shell_w_off - rkusb_shell_r_off;
        rkusb_shell_r_off = rkusb_shell_w_off;
        p->property &= ~LOG_PROT_MAYWRITE;      /* can write until next shell cmd */
        return 0;
}
#endif


static int __init rkusb_init_internal( void )
{
               struct rkusb_dev *dev;
               struct log_buffer *p ;
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
                dev->buf_length = 48*1024; /* for tmp buffer */
                dev->req_buf_length = 4*1024;
                dev->buf = kmalloc(dev->buf_length,GFP_KERNEL); 
                if( !dev->buf ) {
                        kfree( dev );
                        return -ENOMEM;
                }
                dev->req_in_buf = kmalloc(dev->req_buf_length,GFP_KERNEL); 
                if( !dev->req_in_buf ) {
                        kfree( dev );
                        kfree( dev->buf );
                        return -ENOMEM;
                }
                
             dev->luns = LOG_NUM;
             p = dev->log;
             strcpy( p->name , "kernel log" );
             p->property = LOG_PROT_READ;
             p->getlog = kernel_getlog;
             p->getlog_start_length = kernel_getlog_start_length;
             #if (LOG_NUM>1)
             p++;
             strcpy( p->name , "android main log" );
             p->property = LOG_PROT_READ;
             p->getlog = android_main_getlog;
             p->getlog_start_length = android_main_getlog_start_length;
             #endif
             #if (LOG_NUM>2)
             p++;
             strcpy( p->name , "shell log" );
             p->property = LOG_PROT_READ|LOG_PROT_WRITE; /* read and write */
             p->getlog = rkusb_shell_getlog;
             p->getlog_start_length = rkusb_shell_getlog_start_length;
             p->setlog = rkusb_shell_log_write;
             #endif

             //spin_lock_init(&dev->lock);
             dev->cr.Signature= RKUSB_RESULT_SIG;
             dev->cs.Signature = RKUSB_BULK_CS_SIG;
             INIT_WORK(&dev->theadwork,__rkusb_thread_work);
             register_undef_hook(&rkusb_arm_break_hook);
             _rkusb_dev=dev;

#ifdef RK28_PRINT       /* 20100324,HSL@RK,open while debug. */
             rk28_system_crash = RKDBG_CUSTOMER0;
#endif             

             /*XXXX:20100324,HSL@RK,open debug here for 产线测试!!*/
             /* 正常情况下，应该关闭!*/
//             rk28_system_crash = RKDBG_CUSTOMER1;
             
             return 0;
}

static struct usb_request * __rkusb_init_ep_req(struct rkusb_dev *dev, struct usb_ep* ep) 
{
                struct usb_request *req = usb_ep_alloc_request(ep, GFP_ATOMIC);
	if (!req)
		return NULL;
                req->buf = dev->req_in_buf;
	return req;
}

static void __rkusb_free_ep_req( struct usb_ep* ep , struct usb_request *req) 
{
	if (!req || !ep )
		return ;
		
	/* now allocate buffers for the requests */
	//usb_ep_dequeue(ep,req);
                usb_ep_free_request(ep, req);
}
static int rkusb_reinit_req( struct rkusb_dev *dev,struct usb_ep* ep_in,struct usb_ep* ep_out ) 
{
                __rkusb_free_ep_req( dev->ep_in, dev->req_in );
                if( !ep_in )
                        return 0;
                dev->req_in = __rkusb_init_ep_req(dev, ep_in);
                dev->ep_in = ep_in;

                if( !dev->req_in )
                        return -ENOMEM;
                dev->req_in->complete = rkusb_complete_in;
#if 0                        
                buf = __rkusb_free_ep_req( dev->ep_out, dev->req_out);
                dev->req_out = __rkusb_init_ep_req(dev, ep_out , buf );
                dev->ep_out = ep_out;
                
                if( !dev->req_out ) {
                        if( buf )       
                                kfree( buf );
                        buf = __rkusb_free_ep_req( dev->ep_in, dev->req_in );
                        if( buf )       
                                kfree( buf );
                        return -ENOMEM;
                }
                dev->req_out->complete = rkusb_complete_out;
                DEV_LENGTH(dev) = RKUSB_BULK_CB_WRAP_LEN;
                rkusb_get_cb( dev , DEV_LENGTH(dev) );
#endif                
                return 0;
}
/*
*  init call at msc or adb set alt.use buf_length to distinguish.
*  adb first!
* __attribute__((weak))
*/
int  rkusb_init(struct rkusb_dev_fsg* dev , struct usb_ep* in_ep , 
        struct usb_ep* out_ep , int fsg )
{
        if( !dev ) {
                rk28printk("%s::clear rkusb\n" , __func__ );
                rkusb_reinit_req(_rkusb_dev,NULL,out_ep);
                _rkusb_dev->ep_in = NULL;
                return 0;
        }
        rk28printk("%s::init rkusb,fsg=%d\n" , __func__ , fsg );
        dev->usb_out = NULL;
        /*
         * 4 init case :
         * 0: only fsg , 1:only adb , 2:fsg first,adb second , 3:adb first,fsg second.
         * we want:if have adb,select adb ,then select fsg.
        */
        if( !_rkusb_dev->ep_in ||  fsg == 0) {
                if( _rkusb_dev->fsg_dev ) 
                        _rkusb_dev->fsg_dev->usb_out_bak = NULL;
                rkusb_reinit_req(_rkusb_dev,in_ep,out_ep);
                _rkusb_dev->fsg_dev = dev;
                _rkusb_dev->fsg = fsg;
                dev->usb_out_bak = rkusb_complete_out;
        }
        dev->rk_dev = _rkusb_dev;
        return 0;
}

fs_initcall(rkusb_init_internal);
//module_init(rkusb_init_internal);


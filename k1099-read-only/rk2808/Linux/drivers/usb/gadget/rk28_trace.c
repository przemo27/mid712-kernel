/*
 * for set break point throught rkusb.
 * 20091214,HSL,redo.
*/

#include <asm/traps.h>
#include <linux/interrupt.h>

extern struct pt_regs * __scu_get_regs( void );
extern void __scu_bk_continue( struct pt_regs * reg );
extern void rk28_printk_mem( unsigned int * addr , int words );
#define RKUSB_BKP_INST               0xe7f001f1
#define RKUSB_bk_ADDR_INVALID      0XFFFFFFFF
#define RKUSB_WHILE_INST                0Xeafffffe      // while(1)

#define ADDR_VALID( addr )      (addr < 0xc0008000 ||addr  > (unsigned long)high_memory)
struct __break_point {
        unsigned long           addr; /* break point address */
        unsigned long           flags; /* auto clear,condiction check(type,equ) */
        short                        steps;
        short                        go_on;                             
        unsigned long           old_inst; /* pc or lr */
        struct pt_regs          snap; /* cpu content */
        void                          *if_data; 
        char                          *call_function;
        wait_queue_head_t    wq;
        struct task_struct *    task;
        struct pt_regs             *user_reg;
} ;

#define BKP_FLAG_AUTO_CLEAR                     (1<<0)
#define BKP_FLAG_ACTIVE                               (1<<4) 

static struct __break_point    rkusb_break_point[MAX_BREAKS+1]=
{ [MAX_BREAKS]={RKUSB_bk_ADDR_INVALID,}};

#ifdef RK28_PRINT
static void __rkusb_print_bk( struct __break_point* bk)
{
        printk("bk=0x%p,addr=0x%lx,flags=0x%lx,steps=%d,go on=%d\n" , 
                bk,bk->addr,bk->flags,bk->steps,bk->go_on);
}
static void __rkusb_print_bk_all( void )
{
        int j = 0;
        struct __break_point* bk = &rkusb_break_point[0];
        while( bk->addr != RKUSB_bk_ADDR_INVALID ) {
                if( bk->addr ) {
                        __rkusb_print_bk( bk );
                        j++;
                }
                bk++;
        }
        printk("total set break points=%d\n" , j );
}
#else 
#define __rkusb_print_bk( bk )
#define __rkusb_print_bk_all()  
#endif

static struct __break_point* __rkusb_alloc_breakpoint( void )
{
        struct __break_point* bk = &rkusb_break_point[0];
        while( bk->addr != RKUSB_bk_ADDR_INVALID ) {
                if( bk->addr == 0 ) {
                        init_waitqueue_head( &bk->wq );
                        bk->go_on = 0;
                        bk->steps = 0;
                        bk->flags = 0;
                        return bk;
                }
                bk++;
        }
        return NULL;
}
static inline void  __rkusb_free_breakpoint( struct __break_point *bk )
{
        struct __bk_check_condition *bcc = (struct __bk_check_condition*)bk->if_data;
        if( bcc )
                kfree( bcc );
        bk->addr = 0;
}
static struct __break_point* __rkusb_search_breakpoint( unsigned long addr )
{
        struct __break_point* bk = &rkusb_break_point[0];
        while( bk->addr != RKUSB_bk_ADDR_INVALID ) {
                if( bk->addr == addr ) {
                        return bk;
                }
                bk++;
        }
        return NULL;
}
static struct __break_point* __rkusb_search_breakpoint_active( void )
{
        struct __break_point* bk = &rkusb_break_point[0];
        while( bk->addr != RKUSB_bk_ADDR_INVALID ) {
                if( bk->addr && (bk->flags&BKP_FLAG_ACTIVE) ) {
                        return bk;
                }
                bk++;
        }
        return NULL;
}
static inline unsigned long __rkusb_set_inst_at_addr( unsigned long addr , unsigned long inst )
{
        unsigned long flags;
        unsigned long old_inst = *(unsigned long*)addr;
        local_irq_save(flags);
        *(unsigned long*)addr = inst ;
        //__cpuc_flush_kern_all();
        __cpuc_coherent_kern_range(addr,4);
        local_irq_restore(flags);
        return old_inst;
}

static void __rkusb_free_bk_all( struct __break_point *bk )
{
        if( bk->flags & BKP_FLAG_ACTIVE ) {
                bk->flags |= BKP_FLAG_AUTO_CLEAR;
                bk->go_on++;
                wake_up( &bk->wq );
        } else {
                __rkusb_set_inst_at_addr(bk->addr,bk->old_inst);
                __rkusb_free_breakpoint( bk );
        }
}

static void __rkusb_break_do_one_step( struct __break_point* bk )
{
        if( (bk->flags & BKP_FLAG_ACTIVE) ) {
                wake_up( &bk->wq );
                bk->steps++;
                bk->go_on++;
        }
}


static int __rkusb_trace_step_wait( struct rkusb_dev *dev , struct __break_point *bk)
{
        if( bk->steps > 0 ) {
                bk->steps--;
                #if 0
                if( dev->phase == RKUSB_PHASE_DATAIN ||
                     dev->phase == RKUSB_PHASE_DATAOUT ){
                        if( dev->xfer_cb )
                                dev->xfer_cb( dev );
                }
                #endif
                #if 0
                if( bk->call_function ) {
                        int ret = __rk28_scu_parse_cmd( bk->call_function );
                        rkusb_send_csw_result( dev , ret );
                        bk->call_function = NULL;
                }
                #endif
        }
        return bk->go_on;
}

/* reset the bk */
static inline void __rkusb_trace_reinit_bk( struct __break_point* bk ) 
{
        rk28printk("%s::reinit bk 0x%lx\n" ,__func__ , bk->addr-4 );
        __rkusb_set_inst_at_addr(bk->addr,bk->old_inst);
        bk->addr -= 4;
        if( BKP_FLAG_AUTO_CLEAR&bk->flags ) {
                __rkusb_free_breakpoint( bk );
        } else {
                bk->go_on = 0;
                bk->flags &= ~(BKP_FLAG_ACTIVE);
                bk->old_inst = __rkusb_set_inst_at_addr(bk->addr,RKUSB_BKP_INST);
        }
}

static inline unsigned long __rkusb_bk_get_addr_value( unsigned long addr , int size ) 
{
        switch( size ){
        case 0:
                return *((unsigned long*)addr);
        case 1:
                return (unsigned long)(*((unsigned short*)addr));
        case 2:
                return (unsigned long)(*((unsigned char*)addr));
        default:
        break;
        }
        return -1;
}

static int __rkusb_check_bk_condition( struct __break_point* bk ) 
{
        int reg;
        int v=0;
        unsigned long source_v=0;
        unsigned long source_addr;
        unsigned long fl;
        struct __bk_check_condition *bcc = (struct __bk_check_condition*)bk->if_data;
        if( !bcc )
                return 1;
        fl = bcc->flags;
        rk28printk("%s::flags=0x%lx,int = 0x%lx,addr=0x%lx\n" , __func__,fl,bcc->u.int_value , bcc->addr);
        if( BKP_FLAG_CMP_USER&bcc->flags )         /* call user define function */
                return __rk28_scu_parse_cmd( bcc->u.string );
        if( (BKP_FLAG_CMP_PID&bcc->flags) && current->pid != bcc->u.int_value )          /* the same PID. */
                return 0;
        reg = BKP_FLAG_SOURE_REG( fl ); /* check reg for value or string. */
        if( reg == BKP_CMP_MAX_REG ) {
                rk28printk("bk condition error,reg=%d,int value=0x%lx\n" , reg , bcc->u.int_value );
                return 1;
        }
        /* string always is address */
        if(bcc->flags & (BKP_FLAG_ADDRESS|BKP_FLAG_CMP_STRING) ) { 
                if( bcc->addr )
                        source_addr = bcc->addr;
                else 
                        source_addr = bk->snap.uregs[reg];
                 if( ADDR_VALID(source_addr) ) {
                        rk28printk("condition check for addr=0x%lx invalid addr,mem=0x%p\n" , source_addr, high_memory );
                        return 0;
                }
                if( !(bcc->flags & BKP_FLAG_CMP_STRING) ) {
                        source_v = __rkusb_bk_get_addr_value( source_addr , BKP_FLAG_SOURE_SIZE(bcc->flags) );
                } else {
                        rk28printk("condition check for string,src=%s,dest=%s\n" , (char*)source_addr, bcc->u.string);
                        if( bcc->flags & BKP_FLAG_CMP_STRINCLUDE )
                                return strstr( (char*)source_addr, bcc->u.string )?1:0;
                        else
                                v = strcmp( (char*)source_addr, bcc->u.string );
                        }
        } else  {
                source_v = bk->snap.uregs[reg];
        }
        if( !(bcc->flags & BKP_FLAG_CMP_STRING) ) {
                rk28printk("condition check for int,src=0x%lx,dest=0x%lx\n" ,source_v, bcc->u.int_value);
                v = source_v-bcc->u.int_value;
        }
        if( (bcc->flags & BKP_FLAG_CMP_EQ)  && v == 0 )
                return 1;
        if( (bcc->flags & BKP_FLAG_CMP_GREAT)  && v > 0 )
                return 1;
        if( (bcc->flags & BKP_FLAG_CMP_LESS)  && v < 0 )
                return 1;
        return 0;
}

extern unsigned long __rkusb_bk_save( int off_irq );
extern void __rkusb_bk_restore( unsigned long flags );
static void __rkusb_trace_wait( struct __break_point *bk ) 
{
        unsigned long   flags;
        struct rkusb_dev *dev=_rkusb_dev;
        int old_pm = rk28_pm_status;
        //__rkusb_print_bk( bk );
        /* 20100324,HSL@RK,if at irq,can not wait!
          * printk stack for debug.
         */
        if( in_interrupt() ) {
                if( !(BKP_FLAG_NOSTACK&bk->flags) ) {
                        struct pt_regs *regs = get_irq_regs();
                        if (regs)
                		show_regs(regs);
                	else
                		dump_stack();
                }
                goto bk_continue;
        }
        if( !__rkusb_check_bk_condition( bk ) ) 
                goto bk_continue;
        
        //rk28printk("%s:: break point enter wait\n" , __func__ );
        bk->user_reg = __scu_get_regs();
        local_irq_save(flags);
        local_irq_enable(); /* for breakpoint usb */
        //if (!in_atomic() && !irqs_disabled() ) 
        #if 1
        if( !(BKP_FLAG_STOP_WHILE&bk->flags) ){
                rk28_pm_status = 3;
                wait_event_interruptible(bk->wq,__rkusb_trace_step_wait(dev , bk ) );
                //wait_event(bk->wq,__rkusb_trace_step_wait(dev , bk ) );
        } else { /* stop at while,may use jtag here */
                __rkusb_bk_save(0);
                rk28_pm_status = 3; // 4;
                while(!__rkusb_trace_step_wait(dev , bk ) );
                __rkusb_bk_restore(0);
        }
        #else
        rk28_pm_status = 3;
        wait_event_interruptible(bk->wq,__rkusb_trace_step_wait(dev , bk ) );
        #endif
        local_irq_restore(flags);
        rk28_pm_status = old_pm;
bk_continue:        
        bk->addr+= 4;
        bk->old_inst = __rkusb_set_inst_at_addr(bk->addr,RKUSB_BKP_INST);
        __scu_bk_continue( &bk->snap );
}

/*
 * 20100109,HSL@RK,one breakpoint only stop one task one time.
 */
static int rkusb_break_trap(struct pt_regs *regs, unsigned int instr)
{
        struct __break_point* bk;
        bk = __rkusb_search_breakpoint( regs->ARM_pc );
        if( !bk )
	return -1;
        rk28printk("%s::bk trap at addr=0x%lx,flags=0x%lx\n" ,__func__ , bk->addr , bk->flags );
        //rk28_printk_mem( (unsigned int*)&regs->uregs[0], 18 );
        if( !( bk->flags &(BKP_FLAG_ACTIVE) ) ) {
                bk->flags |= BKP_FLAG_ACTIVE;
                bk->task = current;
                bk->snap = *regs;
                 __rkusb_set_inst_at_addr(bk->addr,bk->old_inst); /* restore inst */
                regs->ARM_pc = (unsigned long)__rkusb_trace_wait;
                regs->ARM_r0 = (unsigned long)bk;
        } else {
                __rkusb_trace_reinit_bk( bk );
        }
        return 0;
}

static struct undef_hook rkusb_arm_break_hook = {
	.instr_mask	= 0x0fffffff,
	.instr_val	= 0x07f001f1,
	.cpsr_mask	= PSR_T_BIT,
	.cpsr_val	= 0,
	.fn		= rkusb_break_trap,
};

static void __rkusb_trace_set_condition( struct __break_point *bk ,
        unsigned long flags , char* condition ) 
{
        struct __bk_check_condition *bcc ;
        int     len = sizeof(*bcc);
        if( flags & BKP_FLAG_CMP_STRING )
                len += strlen(condition+sizeof(unsigned long) )+1;
        bcc = (struct __bk_check_condition*)kmalloc( len , GFP_ATOMIC );
        if( !bcc )
                return ;
        bcc->addr = *((unsigned long*)condition );
        condition+=sizeof(unsigned long);
        if( flags & BKP_FLAG_CMP_STRING ){
                bcc->u.string = (char*)(bcc+1);
                strcpy( bcc->u.string , condition );
                rk28printk("set string condition bk,flag=0x%lx,string=%s\n" , flags , bcc->u.string );
        } else {
                bcc->u.int_value = *((long*)condition);
                rk28printk("set int condition bk,flag=0x%lx,value=0x%lx\n" , flags , bcc->u.int_value );
        }
        bcc->flags = flags&BKP_CHECK_MASK;
        bk->if_data = bcc;
}
static void __rkusb_trace_bk_init( struct __break_point *bk , 
        unsigned long flags , char* condition ) 
{
        bk->if_data = NULL;
        bk->flags = flags;
        bk->flags &= ~(BKP_FLAG_ACTIVE);
        bk->old_inst = __rkusb_set_inst_at_addr(bk->addr ,RKUSB_BKP_INST);
        if( flags & BKP_FLAG_CHECK ) {
                __rkusb_trace_set_condition( bk , flags , condition );
        }
}

static int rkusb_trace_set_breakpoint_callback( struct rkusb_dev *dev )
{
        unsigned long flags = *((unsigned long*)dev->req_out->buf);
        char* if_data = dev->req_out->buf+sizeof(unsigned long); /* do something */
        struct __break_point* bk = (struct __break_point*)dev->private_tmp;
        __rkusb_trace_bk_init( bk , flags , if_data );
        return RKUSB_CB_OK_CSW;
}

/* symbol change at pc util , lab is the address , 
 *  
*/
static int rkusb_trace_set_breakpoint( struct rkusb_dev *dev )
{
        unsigned long flags = 0;
        unsigned long addr = DEV_OFFSET( dev );
        char            *if_data = NULL;
        struct __break_point* bk ;
        rk28printk("set bk at 0x%lx , dev len=%d\n" , addr , DEV_LENGTH(dev));
        bk = __rkusb_search_breakpoint( addr );
        if( bk )
                return RKUSB_CB_FAILD;                
        bk = __rkusb_alloc_breakpoint();
        if( !bk )
                return RKUSB_CB_FAILD;
        bk->addr = addr;
        if( DEV_LENGTH(dev) ) {
                dev->private_tmp = bk;
                __rkusb_set_dest_or_source( dev , NULL );
                rkusb_normal_data_xfer( dev , rkusb_trace_set_breakpoint_callback );
                return RKUSB_CB_OK_NONE;
        } else {
                __rkusb_trace_bk_init( bk , flags , if_data );
                return RKUSB_CB_OK_CSW;
        }
        
}
static int rkusb_trace_bk_goon( struct rkusb_dev *dev )
{
        unsigned long addr = DEV_OFFSET( dev );
        struct __break_point* bk ;
        if( addr )
                bk = __rkusb_search_breakpoint( addr );
        else
                bk = __rkusb_search_breakpoint_active();
        if( !bk )
                return -1;
        rk28printk("go on bk=0x%p,addr=0x%lx,flags=0x%lx\n" , bk , bk->addr , bk->flags );
        __rkusb_break_do_one_step( bk );
        return RKUSB_CB_OK_CSW;
}

static int rkusb_trace_clear_breakpoint( struct rkusb_dev *dev )
{
        unsigned long addr = DEV_OFFSET( dev );
        struct __break_point* bk ;

        __rkusb_print_bk_all();
        if( addr == 0xffffffff ) {
                 bk = &rkusb_break_point[0];
                while( bk->addr != RKUSB_bk_ADDR_INVALID ) {
                        if( bk->addr ) {
                                __rkusb_free_bk_all( bk );
                        }
                        bk++;
                }
        } else {
                bk = __rkusb_search_breakpoint( addr );
                if( !bk )
                        return -1;
                __rkusb_free_bk_all( bk );
        }
        __rkusb_print_bk_all();
        return RKUSB_CB_OK_CSW;
}

static int rkusb_trace_get_registers( struct rkusb_dev *dev )
{
        struct __break_point* bk ;
        BREAK_REGS              br;
        br.size = sizeof br;
        br.catch_type = __system_crashed();
        br.irq_regs = get_irq_regs();
        br.user_regs = __scu_get_regs();
        br.kernel_regs = br.expt_regs = NULL;
        br.pid = current->pid;
        br.copy_kernel_stack = 0;
        if( !br.catch_type ) {
                bk = __rkusb_search_breakpoint_active( );
                if( bk ) {
                        br.pid = bk->task->pid;
                        br.kernel_regs = &bk->snap;
                        br.user_regs = bk->user_reg;
                } else {
                        br.catch_type = 2;
                }
        } else {
                if( rkusb_except_content.uregs[15] )
                        br.expt_regs = (struct pt_regs *)rkusb_except_content.uregs;
        }
        /* copy the irq stack,because it will change next time */
        if( !user_mode( br.irq_regs ) )
        {
                        char * stack_cpoy = __rkusb_rwbuffer_start( dev );
                        int st_len = STACK_LENGTH(br.irq_regs->ARM_sp);
                        memcpy( stack_cpoy , (void*)br.irq_regs->ARM_sp , st_len );
                        br.copy_kernel_stack = (unsigned long)stack_cpoy;
        }
        rkusb_normal_data_xfer_onetime( dev , &br );
        return 0;
}

/*
 * 20100108,HSL@RK,set int return for servel system status.
 * 0:normal,1:at break point while ,2:at break point wait , 3:dead panic,4:shecdule painc .
*/
static int rkusb_trace_bk_inquiry( struct rkusb_dev *dev )
{
        unsigned long addr = DEV_OFFSET( dev );
        struct __break_point* bk ;
        int     result = 0;
        if( __system_crashed( ) ) {
                if( (rk28_system_crash&0xff) >= RKDBG_CUSTOMER1 )
                        result = 4;
                else
                        result = 3;
        } else {
                if( addr )
                        bk = __rkusb_search_breakpoint( addr );
                else
                        bk = __rkusb_search_breakpoint_active();
                if( bk && (bk->flags&BKP_FLAG_ACTIVE) ) {
                        if(BKP_FLAG_STOP_WHILE&bk->flags)
                                result = 1;
                        else
                                result = 2;
                }
        }
        /* mm=NULL,active mm = NULL*/
        //printk("rkusb dev task mm=0x%p,active mm=0x%p\n" ,dev->thread_task->mm , dev->thread_task->active_mm );
        rkusb_send_csw_result( dev , result );
        return RKUSB_CB_OK_NONE;
}

/*
 *   XXX:can change pc,lr,cpsr?
 */
static int rkusb_trace_set_reg_callback( struct rkusb_dev *dev )
{
        SET_REGS        *reg = (SET_REGS*)dev->req_out->buf;
        int                     len = dev->req_out->actual-sizeof(unsigned long);
        unsigned long    bit = 1;
        int                     loop=0;
        struct __break_point* bk = (struct __break_point*)dev->private_tmp;
        rk28printk("set reg,bitmask=0x%lx\n" , reg->reg_bit_mask);
        while( len > 0 && loop  <= BKP_CMP_MAX_REG/*18*/ ) {
                if( reg->reg_bit_mask & bit ) {
                        bk->snap.uregs[loop] = reg->reg_value[loop];
                        len -=4;
                }
                bit <<= 1;
                loop++;
        }
        return RKUSB_CB_OK_CSW;
}

static int rkusb_trace_set_reg( struct rkusb_dev *dev )
{
        struct __break_point* bk ;
        bk = __rkusb_search_breakpoint_active();
        if( !bk || DEV_LENGTH(dev) <= sizeof(unsigned long) )
                return RKUSB_CB_FAILD;
        dev->private_tmp = bk;
        rkusb_normal_data_xfer( dev , rkusb_trace_set_reg_callback );
        return RKUSB_CB_OK_NONE;
}
static int rkusb_trace_arm_halt( struct rkusb_dev *dev )
{
        struct __break_point* bk ;
        struct pt_regs      *irq_regs;
        unsigned long flags;
        if( !__rkusb_debug_mod()) {
                return RKUSB_CB_FAILD;
        }
        irq_regs = get_irq_regs();
        if( irq_regs->ARM_pc < TASK_SIZE ) { /* not at kernel */
                return RKUSB_CB_FAILD_CSW;
        }
        bk = __rkusb_alloc_breakpoint();
        if( !bk )
                return RKUSB_CB_FAILD_CSW;
        bk->addr = irq_regs->ARM_pc;
        flags = BKP_FLAG_AUTO_CLEAR;
        __rkusb_trace_bk_init( bk , flags , NULL );
        rk28printk("%s::set bk at 0x%lx\n" , __func__ , bk->addr );
        return RKUSB_CB_OK_CSW;
}

/* for command K_FW_TRACE */
static int rkusb_do_trace( struct rkusb_dev *dev )
{
        switch( DEV_FUNC(dev )) {
        case FUNC_TRACE_BREAKSET:
                return rkusb_trace_set_breakpoint( dev );
        case FUNC_TRACE_BREAKCLEAR:
                return rkusb_trace_clear_breakpoint( dev );
        case FUNC_TRACE_GOON:
                return rkusb_trace_bk_goon( dev );
        case FUNC_TRACE_GETREG:
                return rkusb_trace_get_registers( dev );
        case FUNC_TRACE_INQUIRY:
                return rkusb_trace_bk_inquiry( dev );
        case FUNC_TRACE_CALLFUNC:
                break;
        case FUNC_TRACE_SETREG:
                return rkusb_trace_set_reg( dev );
                
        case FUNC_TRACE_ARMHALT:
                return rkusb_trace_arm_halt( dev );
        default:
                break;
        }
        return RKUSB_CB_FAILD;
}


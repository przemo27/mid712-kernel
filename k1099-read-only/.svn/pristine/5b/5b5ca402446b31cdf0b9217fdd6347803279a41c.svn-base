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

/* temporary variable used between rkusb_open() and rkusb_gadget_bind() */


void __rkusb_thread_work(struct work_struct *work)
{
        struct rkusb_dev *dev = container_of(work, struct rkusb_dev, theadwork);
        rkusb_do_thread_cmd( dev );
}
inline static char *__rkusb_rwbuffer_start( struct rkusb_dev *dev )
{
        return (dev->buf);
}

inline static char *__rkusb_rwbuffer_end( struct rkusb_dev *dev )
{
        return dev->buf+dev->buf_length;
}
#define DEV_RWBUF_LENGTH( dev )  ( dev->buf_length)
#define DEV_INREQ_BUF(dev)              (dev->req_in->buf)
#define DEV_FAST_ALLOC(dev , type)           ((type*)DEV_INREQ_BUF(dev))

#ifdef RK28_PRINT
static void rkusb_print_bin( char *p , int len )
{
        int line = 0;
        //rk28printk("binnary value(%04d): \n" , len ); 
        while( len > 0 ) {
                printk("%02x " , *p );
                len --;
                p ++;
                line++;
                if( line == 8 ) {
                        printk("\n" );
                        line = 0;
                }
        }
        printk("\n" );
}
static void rkusb_print_cb( struct rkusb_dev *dev )
{
        char *p = (char*)&dev->cb;

        rk28printk("xferLen:0x%08x,lun:%d," 
                 "cmd:0x%02x,Lba:0x%08x\n" , 
                 dev->cb.DataTransferLength,dev->cb.Lun,
                 dev->cb.Code , dev->cb.Lba );
        rkusb_print_bin(p , sizeof dev->cb );
}
#else 
#define rkusb_print_bin( p , len )
#define rkusb_print_cb( dev )
#endif
static int rkusb_get_cb( struct rkusb_dev *dev , int len )
{
        int ret;
        /* 20091026, len must be 512 aligned */
        len = ((len+511)>>9)<<9;
        dev->req_out->length = len;
        
        /* 20091123,HSL@RK ,clear buf for strings */
        memset(dev->req_out->buf , 0 , len );
        ret = usb_ep_queue(dev->ep_out,  dev->req_out, GFP_ATOMIC);
       return ret;
}

static void __rkusb_set_result( struct rkusb_dev *dev )
{
        //rk28printk("set result for cmd 0x%x,subcode=0x%x,status=0x%x\n" ,DEV_CMD(dev),DEV_FUNC(dev),dev->cs.Status);
       dev->cr.Code= DEV_CMD(dev);
       dev->cr.subCode= DEV_FUNC(dev);
       dev->cr.Lun = DEV_LUN(dev);
       dev->cr.Status = dev->cs.Status;
       dev->cr.retvalue= dev->cs.Residue;
}

static void __rkusb_send_csw__atomic( struct rkusb_dev *dev )
{
        int ret;   
        dev->phase = RKUSB_PHASE_CMD;  
        __rkusb_set_result( dev );
        dev->req_in->length = sizeof dev->cs;
        memcpy( dev->req_in->buf , &dev->cs , sizeof dev->cs );
        ret = usb_ep_queue(dev->ep_in,  dev->req_in , GFP_ATOMIC);
        if( ret != 0 ) {
                rk28printk("quene in req failed,req=0x%p,len=%d\n" ,dev->req_in,dev->req_in->length );
       } 
       /*20100108,HSL@RK, for get cb at complete out.*/
       DEV_LENGTH(dev) = RKUSB_BULK_CB_WRAP_LEN;
       // rkusb_get_cb( dev , RKUSB_BULK_CB_WRAP_LEN );
}

static void rkusb_send_csw( struct rkusb_dev *dev , int status )
{
        dev->cs.Status = status;
        dev->cs.Residue = 0; // 0
        __rkusb_send_csw__atomic( dev );
}

static void rkusb_send_csw_result( struct rkusb_dev *dev , int result )
{
        dev->cs.Status = RKUSB_STATUS_PASS;
        dev->cs.Residue = result;
        __rkusb_send_csw__atomic( dev );
}

/*
 * for handle a failed cmd.
*/
static int rkusb_failed_cb( struct rkusb_dev *dev )
{
        return RKUSB_CB_FAILD_CSW;
}
static int __rkusb_can_direct_copy( struct rkusb_dev *dev )
{
        //return (DEV_OFFSET( dev ) >= 0xc0000000 ||__at_break_point_freeze() ||__system_crashed() );
        return (DEV_OFFSET( dev ) >= 0xc0000000 );
}

static void __rkusb_flush_codes(struct rkusb_dev *dev,
        unsigned long start , int len )
{
        unsigned long addr;
        addr = start;
        if( addr < (unsigned long)_etext ) {
                if( addr < (unsigned long)_text ) {
                        addr = (unsigned long)_text;
                        len -= addr-start;
                }
                if( addr + len > (unsigned long)_etext )
                        len = (unsigned long)_etext - addr;
                if( len > 0 )
                        __cpuc_coherent_kern_range(addr,len);
        }
}

/* 20100203,HSL@RK,when use msc 64K buffer,no need to copy. */
#if 0

static void __rkusb_coyp_data( struct rkusb_dev *dev , int len , int data_in )
{
        if( !DEV_OFFSET( dev ) )
                return;
        if( __rkusb_can_direct_copy(dev) ) {
                #if 0 /* 20100108,HSL@RK,when user data , copy at task,use r/w buffer */
                if( data_in && __rkusb_read_enable(dev) ) {
                        if( DEV_OFFSET( dev ) < 0xc0000000 )
                                ret = copy_from_user(dev->req_in->buf, (const void __user *) DEV_OFFSET( dev ), len );
                        else
                                memcpy(dev->req_in->buf , (void*)DEV_OFFSET( dev )  , len );
                 }
                 if( !data_in && __rkusb_write_enable(dev) ) {
                        if( DEV_OFFSET( dev ) < 0xc0000000 )
                                ret = copy_to_user((void __user *) DEV_OFFSET( dev ), dev->req_in->buf,  len );
                        else
                                memcpy((void*)DEV_OFFSET( dev ) , dev->req_out->buf , len );
                 }
                 #else
                  if( data_in && __rkusb_read_enable(dev) ) {
                            memcpy(dev->req_in->buf , (void*)DEV_OFFSET( dev )  , len );
                 }
                 if( !data_in && __rkusb_write_enable(dev) ) {
                            memcpy((void*)DEV_OFFSET( dev ) , dev->req_out->buf , len );
                 }
                 #endif
         } else {
                rk28printk("%s::user data must copy at task enviroment\n",__func__);
         }
}

static void rkusb_send_data_in( struct rkusb_dev *dev , int len )
{
        int ret;
        if( len > RKUSB_BUFFER_SIZE) 
                len = RKUSB_BUFFER_SIZE;
        dev->req_in->length = len;
        //dev->phase = RKUSB_PHASE_DATAIN;
        //DEV_LENGTH( dev ) -= len;
        __rkusb_coyp_data( dev , len , 1 );
        ret = usb_ep_queue(dev->ep_in,  dev->req_in, GFP_ATOMIC);
        if( ret != 0 ) {
                rk28printk("quene in req failed, req=0x%p , len=%d\n" ,dev->req_in, len );
       } 
}
static inline void __rkusb_set_dest_or_source( struct rkusb_dev *dev , void *p )
{
        DEV_OFFSET( dev ) = (unsigned long )p;
}

static int rkusb_normal_data_xfer_callback( struct rkusb_dev *dev )
{
        int actual;
        int data_in = DEV_DATA_IN(dev);
        actual = dev->req_out->actual;
        if( data_in )
                actual = dev->req_in->actual;
        else
                __rkusb_coyp_data( dev , actual , 0 ); /* out:copy memory first */

        //rk28printk("xfer callback::dev len=0x%x,actual=0x%x\n" , DEV_LENGTH( dev ),actual );
        DEV_LENGTH( dev ) -= actual;
        dev->xfer_max_size -= actual;
        if( DEV_LENGTH( dev ) == 0 ) {
                        if( dev->final_cb ) {
                                int ret = dev->final_cb( dev );
                                if( ret == RKUSB_CB_OK_CSW ) 
                                        rkusb_send_csw( dev , RKUSB_STATUS_PASS );
                                else if( ret != RKUSB_CB_OK_NONE )
                                        rkusb_send_csw( dev , RKUSB_STATUS_FAIL );
                        } else
                                rkusb_send_csw( dev , RKUSB_STATUS_PASS );
        } else {
                if( DEV_OFFSET( dev ) ) /* some out will use out_buf directly */
                        DEV_OFFSET( dev ) += actual;
                if( dev->xfer_max_size == 0 )
                        if( dev->final_cb ) {
                                int r = dev->final_cb( dev );
                                if( r == RKUSB_CB_OK_NONE ) /* do things at callback */
                                        return 0;
                        }
                if( data_in )  { // IN 
                        rkusb_send_data_in( dev , DEV_LENGTH( dev ) );
                } else {
                        //rkusb_get_cb( dev , DEV_LENGTH( dev ) );
                }
        }
        return 0;
}

static void rkusb_normal_data_xfer_max_size( struct rkusb_dev *dev ,
        data_cb cb , unsigned int max_size )
{
        //rk28printk("normal xfer::dev len=0x%x,data in=0x%x\n" , DEV_LENGTH( dev ),DEV_DATA_IN(dev) );
        if( DEV_LENGTH( dev ) > 0 ) {
                /* may call at task , set vars first */
                dev->xfer_max_size = max_size;
                dev->xfer_cb = rkusb_normal_data_xfer_callback;
                dev->final_cb = cb;
                if( DEV_DATA_IN(dev) )  { // IN 
                        dev->phase = RKUSB_PHASE_DATAIN;
                        rkusb_send_data_in( dev , DEV_LENGTH( dev ) );
                } else {
                        dev->phase = RKUSB_PHASE_DATAOUT;
                        //rkusb_get_cb( dev , DEV_LENGTH( dev ) );
                }

        }  else {
                rkusb_send_csw( dev , RKUSB_STATUS_PASS );
        }
}
static void rkusb_normal_data_xfer( struct rkusb_dev *dev , data_cb cb)
{
       rkusb_normal_data_xfer_max_size( dev , cb , 0x40000000 );
}
static void rkusb_normal_data_xfer_onetime( struct rkusb_dev *dev , void* buf )
{
        __rkusb_set_dest_or_source( dev , buf );
        rkusb_normal_data_xfer( dev , NULL );
}
#else

#if 0
/* 20100203,HSL@RK,use change req buf instead copy data.
*/
static char* rkusb_reset_req_buf(struct usb_request * req , char* buf )
{
        char *bb = req->buf;
        if( req->buf != buf ) {
                req->buf = buf;
                req->dma = DMA_ADDR_INVALID;
        }
        return bb;
}
#define RKUSB_RESET_INREQ_BUFFER(dev,req)   rkusb_reset_req_buf(req,DEV_INREQ_BUF(dev))
#endif

static inline void __rkusb_set_dest_or_source( struct rkusb_dev *dev , void *p )
{
        DEV_OFFSET(dev) = (unsigned long )p;
}
static void rkusb_normal_data_xfer( struct rkusb_dev *dev , data_cb cb)
{
        int ret;
        //rk28printk("data xfer::dev len=0x%x,dev lba=0x%x,inreq buf=0x%p\n" , DEV_LENGTH( dev ),
        //        DEV_OFFSET(dev), dev->req_in->buf);
        /* may call at task , set vars first */
        dev->final_cb = cb;
        if( DEV_DATA_IN(dev) )  { // IN 
                dev->phase = RKUSB_PHASE_DATAIN;
                dev->req_in->length = DEV_LENGTH( dev );
                if( DEV_OFFSET(dev) && DEV_OFFSET(dev) != (unsigned long)DEV_INREQ_BUF(dev) 
                        && __rkusb_can_direct_copy(dev) && __rkusb_read_enable(dev) ) {
                        memcpy( dev->req_in->buf , (void*)DEV_OFFSET(dev),dev->req_in->length );
                }
                ret = usb_ep_queue(dev->ep_in,  dev->req_in, GFP_ATOMIC);
                if( ret != 0 ) {
                        rk28printk("quene in req failed, req=0x%p , len=%d\n" ,dev->req_in, DEV_LENGTH( dev ) );
               } 
        } else {
                dev->phase = RKUSB_PHASE_DATAOUT;
        }
}
static void rkusb_normal_data_xfer_onetime( struct rkusb_dev *dev , void* buf )
{
        __rkusb_set_dest_or_source( dev , buf );
        rkusb_normal_data_xfer( dev , NULL );
}

/*
  * return complete: 0 :not complete,1:completed.
*/
static int rkusb_data_out( struct rkusb_dev *dev)
{
        int cmpl = 0;
        struct usb_request *req = dev->req_out;
        rk28printk("%s::actual=%d,dev len=%d,dev lba=0x%08x\n" , __func__ , 
                req->actual,DEV_LENGTH(dev),DEV_OFFSET(dev) );
        //rkusb_print_bin( req->buf , req->actual );
        if(DEV_OFFSET(dev) && DEV_OFFSET(dev) != (unsigned long)req->buf 
                && __rkusb_can_direct_copy(dev) && __rkusb_write_enable(dev) ) {
                memcpy( (void*)DEV_OFFSET(dev),req->buf,req->actual);
                __rkusb_flush_codes(dev,DEV_OFFSET(dev),req->actual);
                DEV_OFFSET(dev) += req->actual;
        }
        DEV_LENGTH(dev) -= req->actual;
        if( DEV_LENGTH(dev) == 0 ) {
                if( dev->final_cb ) {
                        int ret = dev->final_cb( dev );
                        if( ret == RKUSB_CB_OK_CSW ) 
                                rkusb_send_csw( dev , RKUSB_STATUS_PASS );
                        else if( ret != RKUSB_CB_OK_NONE )
                                rkusb_send_csw( dev , RKUSB_STATUS_FAIL );
                } else
                        rkusb_send_csw( dev , RKUSB_STATUS_PASS );
                cmpl = 1;
        }
        return cmpl;
}
#endif

static void rkusb_complete_in(struct usb_ep *ep, struct usb_request *req)
{
             struct rkusb_dev *dev = _rkusb_dev;
             if( req->status )
                        return ;//RKUSB_NOT_HANDLED;
             //rk28printk("%s:dev phase=%d,req status=%d,len=%d\n" , __func__ ,dev->phase , req->status ,req->length );
             if( dev->phase == RKUSB_PHASE_DATAIN ){
                        if( dev->final_cb ) {
                                int ret = dev->final_cb( dev );
                                if( ret == RKUSB_CB_OK_CSW ) 
                                        rkusb_send_csw( dev , RKUSB_STATUS_PASS );
                                else if( ret != RKUSB_CB_OK_NONE )
                                        rkusb_send_csw( dev , RKUSB_STATUS_FAIL );
                        } else
                                rkusb_send_csw( dev , RKUSB_STATUS_PASS );
             } else { /* the cb of csw finish */
                        __rkusb_clear_cb(dev);
             }
             return ;//0;
}

static int rkusb_complete_out(struct usb_ep *ep, struct usb_request *req)
{
                struct rkusb_dev *dev = _rkusb_dev;
                struct rkusb_cb_wrap       *cb;
                int       ret =RKUSB_NOT_HANDLED;
                if( req->status ) {
                        /*20100107,HSL@RK, when error , no need quene,or enter dead loop */
                        //if(req->status != ECONNABORTED  )
                        //    rkusb_get_cb( dev , RKUSB_BULK_CB_WRAP_LEN );
                        return ret;
                }
                //rk28printk("%s::cmd phase=%d,req=0x%p\n" , __func__ , dev->phase == RKUSB_PHASE_CMD,req );
                dev->ep_out = ep;
                dev->req_out = req;
                if( dev->phase == RKUSB_PHASE_CMD ) {
                        cb = (struct rkusb_cb_wrap*)req->buf;
                        if ( cb->Signature != __constant_cpu_to_le32(RKUSB_BULK_CB_SIG)
                                || req->actual != RKUSB_BULK_CB_WRAP_LEN ) {
                                rk28printk("error tag:sig=0x%x,actual=%d,length=%d\n" , cb->Signature,req->actual,req->length);
                               goto  out_quit;     
                        }
                        #if ( RKUSB_CMD_END < 0xff )
                        if( cb->Code < (u8)RKUSB_CMD_BASE || cb->Code > (u8)RKUSB_CMD_END )
                        #else
                        if( cb->Code < (u8)RKUSB_CMD_BASE )
                        #endif
                        {
                                //rk28printk("invalid cmd=%d\n" , cb->Code );
                                goto  out_quit;     
                        }
                        dev->cb = *cb;
                        dev->cs.Tag = dev->cb.Tag;
                        ret = rkusb_command( dev );
                        __rkusb_set_epin_cb(dev,rkusb_complete_in);
                }  else if( dev->phase == RKUSB_PHASE_DATAOUT ) {
                        ret = 0;
                        rkusb_data_out(  dev ) ;
                } 
                #if 0
                /* 20100112,HSL@RK,get pack at task env. */
                if( DEV_CMD(dev) != K_FW_XFERFILE || 
                        DEV_FUNC(dev) != FUNC_WFILE_DATA )
                #endif
                rkusb_get_cb( dev , DEV_LENGTH(dev) );
out_quit:
                if( ret == RKUSB_NOT_HANDLED ) {
                        dev->fsg_dev->usb_out = NULL;
                        rk28printk("%s::get at not rkusb cmd!\n" , __func__ );
                }
                return ret;
}




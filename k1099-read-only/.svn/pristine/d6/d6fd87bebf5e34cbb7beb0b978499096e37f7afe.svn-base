/*
 * rk28_dsp.c  --  Dsp for rk28
 *
 * Driver for rk28 dsp
 *  Copyright (C) 2009 lhh lhh@rock-chips.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <asm/delay.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/hw_common.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <asm/arch/rk28_scu.h> 
#include <asm/arch/hardware.h>
#include <asm/arch/api_intc.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/firmware.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include "rk28_dsp.h"
#include "queue.h"
#include "xmedia_iv_common.h"

/* 20091112,HSL@RK,for video not sleep
*  video application already to it !!,no need here!
*/
#include <linux/android_power.h>
//static android_suspend_lock_t rk28_dsp_lock;

#if 1
#define	DBG(x...)	//printk(KERN_INFO x)
#define	DBG_GETD(x...)
#define DBG_MAP(x...) //printk(KERN_INFO x)
#define DBG_FIRMWARE(x...) //printk(KERN_INFO x)
#define DBG_ALLOC(x...) //printk(KERN_INFO x)

#else
#define	DBG(x...)	printk(KERN_INFO x)
#define	DBG_GETD(x...) printk(KERN_INFO x)
#define DBG_MAP(x...) printk(KERN_INFO x)
#define DBG_FIRMWARE(x...) printk(KERN_INFO x)
#define DBG_ALLOC(x...) printk(KERN_INFO x)

#endif 

#ifndef PIU_BASE_ADDR
#define PIU_BASE_ADDR           0x80132000
#endif

#define DSP_MAJOR		232

#define COPY_FRAME_DATA         1

#define CODEC_OUTPUT_PIU_CHANNEL  0
#define CODEC_MSG_PIU_CHANNEL     1
#define CODEC_MSG_PIU_NEXT_CHANNEL     2
#define SET_BOOT_VECTOR(v)  __raw_writel(v,REG_FILE_BASE_ADDR_VA + 0x18);
#define DSP_BOOT_CTRL() __raw_writel(__raw_readl(REG_FILE_BASE_ADDR_VA + 0x14) | (1<<4),REG_FILE_BASE_ADDR_VA + 0x14);
#define DSP_BOOT_CLR()  __raw_writel(__raw_readl(REG_FILE_BASE_ADDR_VA + 0x14) & (~(1<<4)),REG_FILE_BASE_ADDR_VA + 0x14);
#define DSP_HEAD_LEN 0x8
#define MSG_EXIT_END    0x000000f1
#define CODEC_SECTION_NO 0x5
/* CEVA memory map base address for ARM */
#define DSP_BASE_ADDR 0x80000000

#define DSP_L2_IMEM_BASE 0x80200000

#define DSP_L2_DMEM_BASE 0x80400000

#define SDRAM_BASE_ADDR 0x60000000

#define DSP_DEC_INPUT_BUFF_SIZE 128*1024

#define VIDEO_CODEC_BUFFER_NUM      10
#define VIDEO_CODEC_BUFFER_SIZE     (4*512*1024)


#define VIDEO_MAX_HURRYUP_LEVEL  4

#define REPEAT_RELOAD_BUFF_FLAG     0x02
//#define RECE_ONE_BUFF_FLAG          0x04

#define PIU_PUT_IMASK(port,v) __raw_writel(v,port->regs+PIU_IMASK_OFFSET);

#define PIU_GET_STATUS_REPX(channel) \
        (__raw_readl(port->regs + PIU_STATUS_OFFSET) & (1 << ((channel) + PIU_STATUS_R0WRS)))
        
#define PIU_READ_REPX_VAL(channel) \
        __raw_readl(port->regs + PIU_REPLY0_OFFSET + ((channel) << 2))
        
#define PIU_CLR_STATUS_REPX(channel) \
        __raw_writel(__raw_readl(port->regs + PIU_STATUS_OFFSET) | (1 << ((channel) + PIU_STATUS_R0WRS)), \
        port->regs+PIU_STATUS_OFFSET)
        
       
#define PIU_SEND_CMD(channel, cmd) \
        __raw_writel(cmd,port->regs+PIU_CMD0_OFFSET + (channel << 2))

#define XM_MAX_FRAME_NUM    32



struct rk28_dsp_port {
	struct miscdevice miscdev; 
	struct device dev; 
	int regs;
	int regs0;
	int	irq;
	int dspbuff[20];
	int pa_dspbuff[20];
	int bootaddress;
	int codeprogramaddress;
	int codetableaddress;
	int codedataaddress;
	int free_flag;
	void *pdspdata;
    spinlock_t		lock;   //csy
	struct semaphore sem;
    wait_queue_head_t rqueue;

    int l1_data;
};

int port_save_address;
struct dspdata{ /* PLS don't change this struct!!! */
	unsigned char *rk28_dsp_buffer_start; /* virtual pointer */
    dma_addr_t rk28_dsp_buffer_addr;	/* physical address */
    XMVCodecShare  dsp_codec_share;
};

int dsp_work_status;
int work_id_event = 0;
Queue *pict_q;
int data_req_cnt;

/* current video frame */
XMFrame *cur_frm;
/* prev video frame */
XMFrame *prev_frm0, *prev_frm1, *prev_frm2;

struct dspdata *dsp_data;

#if 0
void debug_printf(int *p,int length)
{
	int n;
	for(n=0;n<length;n++){
	    printk("%s----adress=%x---data=%x\n",__FUNCTION__,p,*p); 
	    *p++;
    }
}
#endif


static int receive_form_dsp_free_boot(struct rk28_dsp_port *port)
{
	return 0;
}

static int receive_form_dsp_free_all(struct rk28_dsp_port *port)
{
                rockchip_clk_unlock_pll(SCU_IPID_ARM);
	printk("%s::%d -- free_flag is %d\n",__func__,__LINE__,port->free_flag);
	return 0;
}

static void shutdown_dsp_hw( void )
{
    __raw_writel((__raw_readl(SCU_BASE_ADDR_VA+0x28)|0x02000030) , SCU_BASE_ADDR_VA+0x28); /* dsp core peripheral  rest*/ 	
    __raw_writel((__raw_readl(SCU_BASE_ADDR_VA+0x0c) & (~0x03)) , SCU_BASE_ADDR_VA+0x0c); /* dsp work mode :slow mode*/ 
    __raw_writel((__raw_readl(SCU_BASE_ADDR_VA+0x10) | 0x21) , SCU_BASE_ADDR_VA+0x10);  /* dsp subsys power off 0x21*/
    __raw_writel((__raw_readl(SCU_BASE_ADDR_VA+0x1c) | 0x02) , SCU_BASE_ADDR_VA+0x1c);  /* dsp clock disable 0x12*/	
    __raw_writel((__raw_readl(SCU_BASE_ADDR_VA+0x24) | 0x04) , SCU_BASE_ADDR_VA+0x24); /* dsp ahb bus disable*/
    __raw_writel(0x01d71f30,SCU_BASE_ADDR_VA + 0x4); /*power off dsp pll*/
    __raw_writel((__raw_readl(SCU_BASE_ADDR_VA+0x28)|0x02000030) , SCU_BASE_ADDR_VA+0x28); /* dsp core peripheral  rest*/ 	
}
/********************************************************************************************
*	Func:
*		CheckDSPLIBHead()
*	Description:check head
*	Param:
*	Return:
*	Author:
*		guosl
*	Date:
*		2009-3-10	20:30
*	Log:
*
********************************************************************************************/
static int CheckDSPLIBHead(char *buff)
{
    if ((buff[0] != 'r')
		|| (buff[1] != 'k')
		|| (buff[2] != 'd')
		|| (buff[3] != 's')
		|| (buff[4] != 'p')
		|| (buff[5] != ' ')
		|| (buff[6] != 'X')
		|| (buff[7] != 'X'))
	{
        return -1;
	}
    else
    {
        return 0;
    }
}

static int load_codec_loader_code(int *pboot,struct rk28_dsp_port *port,unsigned int cmd,const char fw_name[20])
{
	const struct firmware *fw;
   // const char fw_name[]; // = "rk28_test.rkl";  ///rk28_test_l3.rkl";  ///rk28_test.rkl";  ///rk28_h264.rkl";   /////rk28_test.rkl";
	int *fpIndexFile,*fpDataFile;
	int *pcodeprograml1,*pcodedatal1,*pcodetable;
	int indexoffset,ret,indexNo,loadNo,i,j;
    int dataOffset,address,length;
    char *code_buf;
       
	DBG("%s [%d]\n",__FUNCTION__,__LINE__);  
	pcodeprograml1 = port->codeprogramaddress;
	if (pcodeprograml1 == NULL){
		ret = -ENOMEM;
		goto malloc_fail;
	}
	pcodedatal1 = port->codedataaddress;
	if (pcodedatal1 == NULL){
		ret = -ENOMEM;
		goto malloc_fail;
	}
	pcodetable = port->codetableaddress;
	if (pcodetable == NULL){
		ret = -ENOMEM;
		goto malloc_fail;
	}	
	*(pboot+13) = __pa(pcodetable); /*set decode table address */
	*(pboot+14) = __pa(pcodeprograml1); /*set decode program address */
	*(pboot+15) = __pa(pcodedatal1); /*set decode data address */	
	DBG("%s [%d]---%x--%x---%x--%x\n",__FUNCTION__,__LINE__,*pboot,*(pboot+14),*(pboot+15),*(pboot+13)); 
	ret = request_firmware(&fw, fw_name, &port->dev);
	if (ret) {
		printk(KERN_ERR "Failed to load image \"%s\" err %d\n",
		       fw_name, ret);
		return ret;
	}
	code_buf = fw->data;
	if(CheckDSPLIBHead(code_buf) != 0){
		printk("dsp decode head failed !");
        ret = ENOMEM;
        goto malloc_fail;
    }	
	loadNo = CODEC_SECTION_NO;
	i=8+16;		
	while ((loadNo--) != 0x0)
    {
    	indexoffset = *((int *)(code_buf + i));
    	dataOffset = *((int *)(code_buf + i + 4));
    	DBG("%s [%d]---indexoffset = %x--dataOffset = %x---loadNo=%d\n",__FUNCTION__,__LINE__,indexoffset,dataOffset,loadNo); 
    	i = i + 8;
    	if ((indexoffset != 0x0) && (dataOffset != 0x0))
        {
        	fpIndexFile = (int *)(code_buf + indexoffset);
        	fpDataFile = (int *)(code_buf + dataOffset);
        	indexNo = *(fpIndexFile + 1);
        	j = 0; 
        	while ((indexNo--) != 0x0)
            {
                if(indexNo<0)
                	break;                
                address = *(fpIndexFile + 2 + j*2);
                length = *(fpIndexFile + 2 + 1 + j*2);
                j = j + 1; 
                DBG("%s [%d]---address =%x length = %x---indexNo = %x\n",__FUNCTION__,__LINE__,address,length,indexNo); 
                if(loadNo == (CODEC_SECTION_NO - 0x1)){
                	/* 如果为L1 data MEM的代码段，拷贝到固件的位置 */
                	memcpy((char *)(pcodeprograml1),(char *)(fpDataFile),length);              
                }else{
                    if (loadNo == (CODEC_SECTION_NO - 0x2)){
                         /* 如果为L1 data MEM的代码段，拷贝到固件的位置 */                         
                        memcpy((char *)(pcodedatal1),(char *)(fpDataFile),length);	
                    }else{
                    	/* 如果为CEVA的内存区域，需要进行地址转换 */
                        if (address < SDRAM_BASE_ADDR){
                            address = port->regs0 + (address - 0x200000);
#if 1
                            int     k;
                            int    *buffL2;

                            buffL2 = (int *)address;
                            for (k=0; k<(length >> 2); k++)
                            {
                                buffL2[k] = fpDataFile[k];
                            }
                            fpDataFile = (int *)((int)fpDataFile + length);
#else
                            memcpy((char *)(address), (char *)fpDataFile, length);
#endif
                        }else{
                            /* 如果为sdram中，则只能码表，且地址无关，
                                   并且只能有一段(即地址连续), 否则引起系统崩溃
                                   在退出视频或者加载不成功后，记得把此Buff释放掉 */
                            memcpy((char *)pcodetable, (char *)fpDataFile, length);
                        }
                    }
                }	
            
            }
        }
    }
	
	port->bootaddress = (int)pboot;
	port->codeprogramaddress = (int)pcodeprograml1;
	port->codedataaddress = (int)pcodedatal1;
	port->codetableaddress = (int)pcodetable;
	port->free_flag = 1;
	release_firmware(fw);
    return 0;	
malloc_fail:  
    return ret;
}


static int dsp_release(struct inode *inode, struct file *file)
{
	struct rk28_dsp_port *port;
	//int i;
	int ret = 0;
	
	DBG("%s [%d]\n",__FUNCTION__,__LINE__); 

    port = (struct rk28_dsp_port *)port_save_address; 
    port->pdspdata = file->private_data;
	if(work_id_event != EXIT_DSP_OFF_ID){
		if(port->free_flag != 0){
    	    receive_form_dsp_free_all(port);
        }

                shutdown_dsp_hw();
       // for(i=0;i<VIDEO_CODEC_BUFFER_NUM;i++)
	      //  kfree((void *)port->dspbuff[i]);
	    if(pict_q)
	        queue_destroy(pict_q);
	    pict_q = NULL;
    }
    work_id_event = EXIT_DSP_OFF_ID;

	return ret;
}
static int dsp_open(struct inode *inode, struct file *file)
{
	struct rk28_dsp_port *port;
	struct dspdata *data;
	int ret = 0;

	port = (struct rk28_dsp_port *)port_save_address; 

	file->private_data = dsp_data;

	port->pdspdata = dsp_data;	
	return ret;
}

static unsigned int dsp_poll(struct file *file, struct poll_table_struct *wait)
{
	struct rk28_dsp_port *port;
	unsigned int mask = 0;

    port = (struct rk28_dsp_port *)port_save_address;   
    if (down_interruptible(&port->sem))
    	goto err;
    ///interruptible_sleep_on(&port->rqueue);          
    poll_wait(file, &port->rqueue, wait);
    //DBG("%s [%d], data_req_cnt = %d\n",__FUNCTION__,__LINE__,data_req_cnt);
    if (data_req_cnt > 0 || !queue_is_empty(pict_q))
        mask |= POLLIN | POLLRDNORM;

    up(&port->sem);

    return mask;
err:
    return -ERESTARTSYS;
}
static int dsp_mmap(struct file *file, struct vm_area_struct *vma)
{	
	struct dspdata *data;
	unsigned long pageFrameNo = 0;
	unsigned long vma_size =  vma->vm_end - vma->vm_start;
	DBG("%s [%d]--%x\n",__FUNCTION__,__LINE__,(unsigned int)vma_size); 
	
	data = (struct dspdata *)file->private_data;
	if(vma_size == 4096)
    {
	    pageFrameNo = __phys_to_pfn(__pa(&(data->dsp_codec_share)));
	    vma->vm_flags |= VM_RESERVED | VM_IO;
	    if (remap_pfn_range(vma, vma->vm_start, pageFrameNo, 4096, vma->vm_page_prot)) 
        {
	    	printk("XMVCodecShare fail to remap\n");
	    	return -EAGAIN;
	    }
        DBG("%s [%d]--%x\n",__FUNCTION__,__LINE__,(unsigned int)vma->vm_start); 
    }
    else if(vma_size == DSP_DEC_INPUT_BUFF_SIZE)
    {
        pageFrameNo = __phys_to_pfn(data->rk28_dsp_buffer_addr);
	    vma->vm_flags |= VM_RESERVED | VM_IO;
	    if (remap_pfn_range(vma, vma->vm_start, pageFrameNo, DSP_DEC_INPUT_BUFF_SIZE, vma->vm_page_prot)) 
        {
	    	printk("input buff fail to remap\n");
	    	return -EAGAIN;
	    }
    }
    else   //add by csy 
    {
        
        pageFrameNo = __phys_to_pfn(data->dsp_codec_share.dvparam.yBufAddr);
        vma->vm_flags |= VM_RESERVED | VM_IO;
        if (remap_pfn_range(vma, vma->vm_start, pageFrameNo, VIDEO_CODEC_BUFFER_SIZE, vma->vm_page_prot)) 
        {
          printk("input buff fail to remap\n");
          return -EAGAIN;
        }

    }
	return 0;
}

static long dsp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct rk28_dsp_port *port;
    const struct firmware *fw;  
	const char fw_name_boot[] = "DspBoot.rkl";		
	char fw_name[20];	
    int ret=0,indexoffset,dataOffset,i=0,j;
    int *pfile,*pboot;
    char *buf;
    XMFrame *picture, *out;
    uint32_t *flag;
    unsigned long flags;
    ///XMVCodecShare *share;
   // XMVBlockTrans *bits;
    int pll, freq;

//    DBG("%s [%d]\n",__FUNCTION__,__LINE__);
    
    port = (struct rk28_dsp_port *)port_save_address; 
    port->pdspdata = file->private_data;

	switch (cmd) {
	case DSP_IOCTL_DOWNLOAD_FIRMWARE:
		DBG("%s [%d]---dsp boot start\n",__FUNCTION__,__LINE__);
		if(work_id_event == EXIT_DSP_OFF_ID){
			work_id_event = DOWNLOAD_FIRMWARE_ID;
	    }else{
	    	ret = -EIO;
	    	break;
	    }
#if 0
		__raw_writel((__raw_readl(SCU_BASE_ADDR_VA+0x28)|0x02000030) , SCU_BASE_ADDR_VA+0x28); /* dsp core peripheral  rest*/ 	
        __raw_writel((__raw_readl(SCU_BASE_ADDR_VA+0x28) & (~0x02000020)) , SCU_BASE_ADDR_VA+0x28); /* dsp core peripheral not rest*/ 
        mdelay(20);
        __raw_writel((__raw_readl(SCU_BASE_ADDR_VA+0x0c) & (~0x03)) , SCU_BASE_ADDR_VA+0x0c); /* dsp work mode :slow mode*/ 
        __raw_writel((__raw_readl(SCU_BASE_ADDR_VA+0x10) & (~0x21)) , SCU_BASE_ADDR_VA+0x10);  /* dsp subsys power on 0x21*/
	    __raw_writel((__raw_readl(SCU_BASE_ADDR_VA+0x1c) & (~0x02)) , SCU_BASE_ADDR_VA+0x1c);  /* dsp clock enable 0x12*/	
	    __raw_writel((__raw_readl(SCU_BASE_ADDR_VA+0x24) & (~0x04)) , SCU_BASE_ADDR_VA+0x24); /* dsp ahb bus enable*/
#else
        __raw_writel((__raw_readl(REG_FILE_BASE_ADDR_VA + 0x10)|(0x6d8)), REG_FILE_BASE_ADDR_VA + 0x10);  // 0x6d8 
      //  __raw_writel(0x00040000, REG_FILE_BASE_ADDR_VA + 0x10);  // 0x6d8 
        __raw_writel((__raw_readl(SCU_BASE_ADDR_VA+0x1c) & (~0x02)) , SCU_BASE_ADDR_VA+0x1c);  /* dsp clock enable 0x12*/  
        __raw_writel((__raw_readl(SCU_BASE_ADDR_VA+0x24) & (~0x04)) , SCU_BASE_ADDR_VA+0x24); /* dsp ahb bus enable*/
        __raw_writel((__raw_readl(SCU_BASE_ADDR_VA+0x28) | 0x02000030) , SCU_BASE_ADDR_VA+0x28); /* dsp core peripheral  rest*/   
        __raw_writel((__raw_readl(SCU_BASE_ADDR_VA+0x28) & (~0x02000020)) , SCU_BASE_ADDR_VA+0x28); /* dsp core peripheral not rest*/ 
        mdelay(10);
         __raw_writel((__raw_readl(SCU_BASE_ADDR_VA+0x0c) & (~0x03)) , SCU_BASE_ADDR_VA+0x0c); /* dsp work mode :slow mode*/ 
        mdelay(10);

        /* freq */
        __raw_writel( 0x01810290  , SCU_BASE_ADDR_VA+0x04); //0x01830310 300mhz 0x01820310 400mhz 0x01810290 500mhz 0x01972250 560mhz 0x01972570 600mhz
        mdelay(10);

        /* power */
        __raw_writel((__raw_readl(SCU_BASE_ADDR_VA+0x10) & (~0x21)) , SCU_BASE_ADDR_VA+0x10);  /* dsp subsys power on 0x21*/
        mdelay(10);

        /* change dsp & arm to normal mode */
        __raw_writel(0x5, SCU_BASE_ADDR_VA+0x0c);

#endif
#if 1
        ret = request_firmware(&fw, fw_name_boot, &port->dev);
	    if (ret) {
	    	printk(KERN_ERR "Failed to load boot image \"%s\" err %d\n",
	    	       fw_name_boot, ret);
	    	ret = -ENOMEM;       
			goto error_alloc;
	    }	
	    buf = fw->data;
	    if(CheckDSPLIBHead(buf) != 0){
	    	printk("dsp boot head failed !");
            ret = -ENOMEM;
			goto error_alloc;
        }	
		pboot = port->bootaddress;
	    if (pboot == NULL){
	    	ret = -ENOMEM;
			goto error_alloc;
	    }
	    indexoffset = *(buf+8+32);
	    dataOffset = *(buf+8+32+4); 
	    pfile = (int *)(buf+dataOffset);  
	    memcpy((char *)(pboot+16),(char *)(pfile),2000); /*copy boot to *pboot point*/
	    SET_BOOT_VECTOR(__pa(pboot+16));  /*set dsp boot program address*/
	    DBG_FIRMWARE("%s [%d]--%x\n",__FUNCTION__,__LINE__,__raw_readl(REG_FILE_BASE_ADDR_VA + 0x18)); 
	    release_firmware(fw);
	    ret = copy_from_user(&fw_name[0], (void *)arg, 20);
	    load_codec_loader_code(pboot,port,cmd,fw_name);
		DSP_BOOT_CTRL();	  
#endif
		DBG_FIRMWARE("%s [%d]--boot end \n",__FUNCTION__,__LINE__);
		break;
	case DSP_IOCTL_ALLOC_BUFFER:
	{
		JPEG_INFO_TO_KERNEL *info =(JPEG_INFO_TO_KERNEL *)arg;
		
		DBG_ALLOC("%s [%d]---dsp get buff start\n",__FUNCTION__,__LINE__);
		if(work_id_event == DOWNLOAD_FIRMWARE_ID){
			work_id_event = ALLOC_BUFF_ID;
	    }else{
	    	ret = -EIO;
	    	break;
	    }
        data_req_cnt = 0;
        prev_frm2 = NULL;
        prev_frm1 = NULL;
        prev_frm0 = NULL;
        cur_frm = NULL;
        /* frame queue initialize */
        pict_q = queue_init(XM_MAX_FRAME_NUM, NULL, NULL);
        if(NULL == pict_q) {
            printk("%s [%d]--fail queue_init--%d\n", __FUNCTION__, __LINE__, XM_MAX_FRAME_NUM); 
            return -1;
            break;
        }

		/*set struct XMVCodecShare */
        memset(&dsp_data->dsp_codec_share, 0, sizeof(XMVCodecShare));
		dsp_data->dsp_codec_share.bits.buf = (unsigned char *)dsp_data->rk28_dsp_buffer_addr;
		dsp_data->dsp_codec_share.bits.nb_blocks = 64; /*128K/2K*/
		dsp_data->dsp_codec_share.bits.nb_filled = 0;
		dsp_data->dsp_codec_share.bits.nb_consumed = 0xffffffff;//0;
		dsp_data->dsp_codec_share.bits.block_eof = 0;
		//dsp_data->dsp_codec_share.param.buf = 0;
		dsp_data->dsp_codec_share.param.buf = (unsigned char *)port->pa_dspbuff[0];
		if(NULL != info)
		{
			if(info->buf_size > VIDEO_CODEC_BUFFER_SIZE*2)
			{
				DBG_ALLOC("buffer size is larger than 4M\n");
				ret = -1;
				goto error_alloc;
			}
		    dsp_data->dsp_codec_share.param.width = info->width;  ///st->specific.video.width;
		    dsp_data->dsp_codec_share.param.height = info->height;  //st->specific.video.height;	
		    dsp_data->dsp_codec_share.param.buf_size = info->buf_size;
		    dsp_data->dsp_codec_share.param.Out_YUVType = info->data_format;	
		}
		dsp_data->dsp_codec_share.param.Alpha = 0xff;
		DBG_ALLOC("%d\n",dsp_data->dsp_codec_share.param.width);
		DBG_ALLOC("%d\n",dsp_data->dsp_codec_share.param.height);
		DBG_ALLOC("%d\n",dsp_data->dsp_codec_share.param.buf_size);
		DBG_ALLOC("%d\n",dsp_data->dsp_codec_share.param.Out_YUVType);
		dsp_data->dsp_codec_share.param.blk_buf = (XMVCodecBuffer *)__pa(&(dsp_data->dsp_codec_share.blk_buf));		 
		dsp_data->dsp_codec_share.param.tab_buff =(unsigned char *) __pa(port->codetableaddress);

		dsp_data->dsp_codec_share.param.dsp_boot_flag = 0;
		dsp_data->dsp_codec_share.msg.codec_state = XM_CODEC_STATE_UNLOADED;
		dsp_data->dsp_codec_share.msg.hurry_up = 1; /* default level: 1 */
		dsp_data->dsp_codec_share.msg.flush = 0;
		for(i = 0; i < VIDEO_CODEC_BUFFER_NUM; i++)
		{
			dsp_data->dsp_codec_share.blk_buf.blk[i] = (unsigned char *)port->pa_dspbuff[i];            
		}
        //add by csy        
        dsp_data->dsp_codec_share.dvparam.RefBufAddr = (uint32_t)port->pa_dspbuff[9];   
        dsp_data->dsp_codec_share.dvparam.yBufAddr   = (uint32_t)port->pa_dspbuff[8];        
       // dsp_data->dsp_codec_share.dvparam.uvBufAddr =(uint32_t) (dsp_data->dsp_codec_share.dvparam.yBufAddr + CAM_HIGHT*CAM_WIDTH); 
        dsp_data->dsp_codec_share.dvparam.mp4BufAddr = (uint32_t)( dsp_data->dsp_codec_share.dvparam.yBufAddr + 2*CAM_HIGHT*CAM_WIDTH);
        dsp_data->dsp_codec_share.dvparam.mp4BufSize = VIDEO_CODEC_BUFFER_SIZE - 6*CAM_HIGHT*CAM_WIDTH;
        dsp_data->dsp_codec_share.dvparam.RefBufSize = 4*CAM_HIGHT*CAM_WIDTH;
        dsp_data->dsp_codec_share.dvparam.signal = 0x1234;
		DBG_ALLOC("%s [%d]---DSP_IOCTL_ALLOC_BUFFER, port->dspbuff[0] = %x\n",__FUNCTION__,__LINE__,port->dspbuff[0]); 
		DBG_ALLOC("%s [%d]---DSP_IOCTL_ALLOC_BUFFER, blk[0] = %x\n",__FUNCTION__,__LINE__,dsp_data->dsp_codec_share.blk_buf.blk[0]); 
		
		dsp_data->dsp_codec_share.blk_buf.nb_blk = VIDEO_CODEC_BUFFER_NUM;
		dsp_data->dsp_codec_share.blk_buf.index = 0;
		dsp_data->dsp_codec_share.blk_buf.blk_size = VIDEO_CODEC_BUFFER_SIZE;         
        __raw_writel((__raw_readl(SCU_BASE_ADDR_VA+0x28) & (~0x00000010)) , SCU_BASE_ADDR_VA+0x28); /* dsp core peripheral not rest*/ 		
		//ret = copy_to_user((void *)arg, (void *)&(data->dsp_codec_share), sizeof(XMVCodecShare));
		// DBG("%s [%d]---dsp boot end ok---%x\n",__FUNCTION__,__LINE__,(unsigned int)__pa(&(data->dsp_codec_share))); 
		dsp_data->dsp_codec_share.msg.codec_state = XM_CODEC_STATE_LOADED;
	}
		break;	
	case DSP_IOCTL_DECODE_INPUT_ONE_BUFF:	
		//DBG("%s [%d]---dsp input buff\n",__FUNCTION__,__LINE__); 
		if((work_id_event == ALLOC_BUFF_ID)||(work_id_event == DECODEING_ID)){
			work_id_event = DECODEING_ID;
	    }else{
	    	ret = -EIO;
	    	break;
	    }
        //share = (XMVCodecShare *)arg;
		//ret = copy_from_user(&(data->dsp_codec_share.bits.nb_filled), (uint32_t *)arg, sizeof(uint32_t));
		DBG("%s [%d]---fill bits buf: nb_filled = %x, data_req_cnt = %d\n",__FUNCTION__,__LINE__,dsp_data->dsp_codec_share.bits.nb_filled, data_req_cnt); 
		break;	
	case DSP_IOCTL_DECODE_EVENT:	
		//DBG("%s [%d]---dsp read status, data_req_cnt = %d\n", __FUNCTION__, __LINE__, data_req_cnt); 
		if((work_id_event == ALLOC_BUFF_ID)||(work_id_event == DECODEING_ID)){
			work_id_event = DECODEING_ID;
	    }else{
	    	ret = -EIO;
	    	break;
	    } 
#if 0
        bits = &(dsp_data->dsp_codec_share.bits);
        if (((bits->nb_filled < bits->nb_consumed + bits->nb_blocks) 
            && (bits->nb_filled >= bits->nb_consumed))
            || (bits->nb_consumed == 0xffffffff)) {
            if (data_req_cnt > 0) {
                dsp_work_status |= REPEAT_RELOAD_BUFF_FLAG;
                data_req_cnt--;
            }
        } else {
            data_req_cnt = 0;
        }
#else
        if (data_req_cnt > 0) {
            dsp_work_status |= REPEAT_RELOAD_BUFF_FLAG;
            data_req_cnt--;
        } else {
            data_req_cnt = 0;
        }
#endif
		ret = copy_to_user((void *)arg, (void *)&dsp_work_status, sizeof(int));	
		break;
	case DSP_IOCTL_JPG_GET_DATA:
	{	
		unsigned char *src_ptr = (unsigned char*)port->dspbuff[0];
        unsigned char *dst_ptr = src_ptr;
		XMVCodecParam *param = &(dsp_data->dsp_codec_share.param);

		ret = 0;
        //unsigned char* ptr1 = ptr;
		DBG_GETD("%s [%d]---DSP_IOCTL_JPG_GET_DATA, arg = %x\n",__FUNCTION__,__LINE__,arg); 
		DBG_GETD("%s [%d]---DSP_IOCTL_JPG_GET_DATA, port->dspbuff[0] = %x\n",__FUNCTION__,__LINE__,port->dspbuff[0]); 
		DBG_GETD("%s [%d]---DSP_IOCTL_JPG_GET_DATA, paport->dspbuff[0] = %x\n",__FUNCTION__,__LINE__,__pa(port->dspbuff[0])); 
		DBG_GETD("%s [%d]---DSP_IOCTL_JPG_GET_DATA, blk[0] = %x\n",__FUNCTION__,__LINE__,dsp_data->dsp_codec_share.blk_buf.blk[0]); 

		DBG_GETD("DSP_IOCTL_JPG_GET_DATA, buffer data0 = %x\n",src_ptr[0]); 		
		DBG_GETD("DSP_IOCTL_JPG_GET_DATA, buffer data1 = %x\n",src_ptr[1]); 		
		DBG_GETD("DSP_IOCTL_JPG_GET_DATA, buffer data00 = %x\n",src_ptr[320*240*4-1]); 		
		DBG_GETD("DSP_IOCTL_JPG_GET_DATA, buffer data01 = %x\n",src_ptr[320*240*4-2]);	
		
		DBG_GETD("DSP_IOCTL_JPG_GET_DATA, buffer size = %d\n",param->buf_size); 
		if((param->width == param->Display_Width) && (param->height == param->DisPlay_Height))
		{
			copy_to_user((void *)arg, (void *)src_ptr, param->buf_size);
		}
		else if((param->Display_Width - param->width < 16) && (param->DisPlay_Height - param->height < 16))
		{
			unsigned char *user_ptr = (unsigned char *)arg;
			unsigned int cnt = param->height;
			DBG_GETD("DSP_IOCTL_JPG_GET_DATA, need to discard\n");
			if(param->Out_YUVType == XMVVOutData_ABGR8888)
			{
				while(cnt--)
				{
				   ret = copy_to_user((void *)user_ptr, (void *)src_ptr, param->width*4);
				   user_ptr += param->width*4;
				   src_ptr += param->Display_Width*4;
				}
			}
			else if(param->Out_YUVType == XMVVOutData_RGB565)
			{
				while(cnt--)
				{
				   ret = copy_to_user((void *)user_ptr, (void *)src_ptr, param->width*2);
				   user_ptr += param->width*2;
				   src_ptr += param->Display_Width*2;
				}				
			}
		}
		else
		{

			DBG_GETD("DSP_IOCTL_JPG_GET_DATA, need to zoom\n");
			//dst_ptr = (unsigned char*)port->dspbuff[1];
			//RotateImageZoom(dst_ptr, src_ptr, (int)param->Display_Width, (int)param->DisPlay_Height, (int)param->width, (int)param->height,
			                     //0, 0, (int)param->width,(int)param->height,0);
			//ret = copy_to_user((void *)arg, (void *)dst_ptr, param->buf_size);
			ret = -1;
		}
		//ret = copy_to_user((void *)arg, (void *)dst_ptr, param->buf_size);//320*240*4//dsp_data->dsp_codec_share.param.buf_size
		DBG_GETD("DSP_IOCTL_JPG_GET_DATA, ret = %d\n",ret); 
		DBG_GETD("DSP_IOCTL_JPG_GET_DATA, Display_Width = %d\n",param->Display_Width);
		DBG_GETD("DSP_IOCTL_JPG_GET_DATA, DisPlay_Height = %d\n",param->DisPlay_Height);
		
		break;
	}
	case DSP_IOCTL_RECE_OUT_BUFF:	
        //DBG("%s [%d]---DSP_DECODE_GET_FRAME, queue_size = %x, filled - %d\n",__FUNCTION__,__LINE__,queue_size(pict_q), dsp_data->dsp_codec_share.bits.nb_filled); 
		if((work_id_event == ALLOC_BUFF_ID)||(work_id_event == DECODEING_ID)){
			work_id_event = DECODEING_ID;
	    }else{
	    	ret = -EIO;
	    	break;
	    } 
        if(pict_q && (!queue_is_empty(pict_q))){
            local_irq_save(flags);     
            picture = (XMFrame *)queue_get(pict_q);
            local_irq_restore(flags);
            if (picture == NULL)
            {
                ret = -1;
                break;
            }
#if COPY_FRAME_DATA
            out = (XMFrame *)arg;
            if (out->data[0] != NULL)
            {
                for(i = 0; i < VIDEO_CODEC_BUFFER_NUM; i++){
                    if((port->pa_dspbuff[i] <= picture->data[0]) && ((port->pa_dspbuff[i] + VIDEO_CODEC_BUFFER_SIZE) > picture->data[0]))
                        break;
                }
                if (i < VIDEO_CODEC_BUFFER_NUM)
                    ret = copy_to_user((void *)out->data[0], (void *)(port->dspbuff[i] + ((int)picture->data[0] - port->pa_dspbuff[i])), 
                                    (((picture->width + 15) & 0xfffffff0) * ((picture->height+ 15) & 0xfffffff0) * 3) >> 1);	
                DBG("%s [%d]---DSP_DECODE_GET_FRAME, Data = %x\n",__FUNCTION__,__LINE__,picture->data[0]); 
                ret = copy_to_user((void *)arg, (void *)picture, sizeof(XMFrame));
            }
            else
            {
                ret = copy_to_user((void *)arg, (void *)picture, sizeof(XMFrame));
            }
#else
		    ret = copy_to_user((void *)arg, (void *)picture, sizeof(XMFrame));	
#endif
            //DBG("%s [%d]---picture--- ptr = %x, width = %d, height = %d.\n", __FUNCTION__, __LINE__, 
            //    (unsigned int)picture->data[0], (unsigned int)picture->width, (unsigned int)picture->height); 

            prev_frm2 = prev_frm1;
            prev_frm1 = prev_frm0;
            prev_frm0 = cur_frm;
            cur_frm = picture;
            
            if (NULL != prev_frm2)  /* free prev frame */
            {
                if ((uint32_t)prev_frm2->displayed_ptr & 0x00400000)
                    flag = (uint32_t *)(((uint32_t)prev_frm2->displayed_ptr & 0x000fffff) + (uint32_t)(port->regs0 + 0x00200000)); 
                else
                    flag = (uint32_t *)(((uint32_t)prev_frm2->displayed_ptr & 0x7fffffff) + (uint32_t)port->l1_data);

                if (*flag)
                    *flag = 0;
                else
                    *flag = 1;
            }
            
        } else {
            ret = -1; //copy_to_user((void *)arg, NULL, sizeof(int));	
        }
		break;	
	case DSP_IOCTL_DECODE_EVENT_SYSN:	
		//DBG("%s [%d]---dsp clr read status\n",__FUNCTION__,__LINE__); 
		if((work_id_event == ALLOC_BUFF_ID)||(work_id_event == DECODEING_ID)){
			work_id_event = DECODEING_ID;
	    }else{
	    	ret = -EIO;
	    	break;
	    }
		ret = copy_from_user((void *)& dsp_work_status,(void *)arg,sizeof(int));	
		break;	
	case DSP_IOCTL_DECODE_TEST:
		DBG("%s [%d]---dsp test\n",__FUNCTION__,__LINE__); 	
		for(j=0;j<1024;j++){
	        printk("%02x ",*(dsp_data->rk28_dsp_buffer_start)++);	
            if((j+1)%16 == 0)
            printk("\n");	
	    }
		break;		
	case DSP_IOCTL_END_ONE_DECODE_EXIT:		
		if((work_id_event == ALLOC_BUFF_ID)||(work_id_event == DECODEING_ID)){
			work_id_event = DECODEING_ID;
	    }else{
	    	ret = -EIO;
	    	break;
	    }
	    PIU_SEND_CMD(0, XM_CODEC_CMD_CLOSE);
		break;	
	case DSP_IOCTL_SEND_DECODE_EXIT:		
		if(/*(work_id_event == ALLOC_BUFF_ID)||*/(work_id_event == DECODEING_ID)){
			work_id_event = DECODEING_ID;
	    }else{
	    	ret = -EIO;
	    	break;
	    }
	    PIU_SEND_CMD(0, XM_CODEC_CMD_CLOSE);
	    break;
	case DSP_IOCTL_FREE_OUT_BUFF:			
	case DSP_IOCTL_DECODE_EXIT:
		DBG("%s [%d]---dsp exit\n",__FUNCTION__,__LINE__); 
		if(work_id_event != EXIT_DSP_OFF_ID){
			work_id_event = EXIT_DSP_OFF_ID;
	    }else{
	    	ret = -EIO;
	    	break;
	    }
		if(port->free_flag != 0){
    	    receive_form_dsp_free_all(port);
        }
                        shutdown_dsp_hw();
       // for(i=0;i<VIDEO_CODEC_BUFFER_NUM;i++)
	      //  kfree((void *)port->dspbuff[i]);
	    if(pict_q)
	        queue_destroy(pict_q);
	    pict_q = NULL;
	    break;	
    case DSP_IOCTL_SET_FREQ:
		if ((work_id_event != ALLOC_BUFF_ID) && (work_id_event != DECODEING_ID))
        {
	    	ret = -EIO;
	    	break;
	    }

        freq = *(int *)arg;
        if (freq > 24 && freq < 600)
        {
            pll = 0x01970000 | ((freq - 1) << 4);
            DBG("%s [%d]-- pll - %x, freq - %d\n", __FUNCTION__, __LINE__, pll, freq);
        }
        else
            printk("%s [%d]--set freq - %d fail\n", __FUNCTION__, __LINE__, freq); 
        break;
     case DSP_IOCTL_SET_BUS:     
         __raw_writel(0x00400000, REG_FILE_BASE_ADDR_VA + 0x10);  //add by csy
         
        break;
    default:
    	ret = -EINVAL;
    	break;
	}
	return ret;
error_alloc:
	//for(j=0;j<i;j++)
	   // kfree((void *)port->dspbuff[j]);	
	return ret;
}

/*
 * Interrupt handler
 */
static irqreturn_t rk28_dsp_interrupt(int irq, void *dev_id)
{
    struct rk28_dsp_port	*port = dev_id;
    struct dspdata *pdata; 
    int msg = 0;
    XMFrame *pic;
    unsigned long flags;   
    spin_lock_irqsave(&port->lock, flags);
    if (PIU_GET_STATUS_REPX(CODEC_OUTPUT_PIU_CHANNEL))
    {
        msg = PIU_READ_REPX_VAL(CODEC_OUTPUT_PIU_CHANNEL);

        if (msg & 0x00400000)
            msg += port->regs0 - 0x00200000;
        else
            msg += port->l1_data;
        
        pic = (XMFrame *)msg;
        if (queue_size(pict_q) > 1)
        {
            if (dsp_data->dsp_codec_share.msg.hurry_up > 0)
                dsp_data->dsp_codec_share.msg.hurry_up--;
        	DBG("%s [%d]--out = %d \n", __FUNCTION__, __LINE__, pic->pts.low_part); 
        }
        
        if ((prev_frm2 != NULL) && (queue_size(pict_q) == 0) && (dsp_data->dsp_codec_share.msg.hurry_up < VIDEO_MAX_HURRYUP_LEVEL))
            dsp_data->dsp_codec_share.msg.hurry_up++;
        
        /* put the video frame to picture queue */
        queue_put(pict_q, (void *)msg, 1);
        
        /* clear status. */
        PIU_CLR_STATUS_REPX(CODEC_OUTPUT_PIU_CHANNEL);
    }
    
    if (PIU_GET_STATUS_REPX(CODEC_MSG_PIU_CHANNEL))
    {
        msg = PIU_READ_REPX_VAL(CODEC_MSG_PIU_CHANNEL);
        
        //DBG("%s [%d]--msg = %x -- data_req_cnt = %d\n", __FUNCTION__, __LINE__, msg, data_req_cnt + 1); 

        switch (msg)
        {
        case MSG_CODEC_ERROR:
            //receive_form_dsp_free_all(port);
            DBG("%s [%d]-- MSG_CODEC_ERROR\n", __FUNCTION__, __LINE__); 
            break;
    	case MSG_PREPARE_BITSTREAM:
            data_req_cnt ++;
            break;
    	case MSG_SHARE_POINTER_REQ:
    	    receive_form_dsp_free_boot(port);
    	    pdata = (struct dspdata *)port->pdspdata;
    	    DBG("%s [%d]--struct = %x\n",__FUNCTION__,__LINE__,(unsigned int)__pa(&(pdata->dsp_codec_share))); 
    	    PIU_SEND_CMD(CODEC_MSG_PIU_CHANNEL, __pa(&(pdata->dsp_codec_share)));			
            PIU_SEND_CMD(CODEC_OUTPUT_PIU_CHANNEL, XM_CODEC_CMD_PARAM);
            break;
        case MSG_EXIT_END:
            receive_form_dsp_free_all(port);
            break;
        default:
            break;
        }
        
        /* clear status. */
        PIU_CLR_STATUS_REPX(CODEC_MSG_PIU_CHANNEL);
    }
    
    if (PIU_GET_STATUS_REPX(CODEC_MSG_PIU_NEXT_CHANNEL))
    {
        msg = PIU_READ_REPX_VAL(CODEC_MSG_PIU_NEXT_CHANNEL);
        
        if(msg < 2 || msg > 0x17ffff)
        	DBG("%s [%d]--dbg = %x ########\n", __FUNCTION__, __LINE__, msg); 

        /* clear status. */
        PIU_CLR_STATUS_REPX(CODEC_MSG_PIU_NEXT_CHANNEL);
    }
    
    wake_up_interruptible(&port->rqueue);
    spin_unlock_irqrestore(&port->lock, flags);
    //DBG("%s [%d]--msg = %x\n",__FUNCTION__,__LINE__,msg ); 
    return IRQ_HANDLED;	
}
/*
* misc file ops
*/
struct file_operations dsp_fops = {
	.release        = dsp_release,
	.open           = dsp_open,
	.poll           = dsp_poll,
	.mmap           = dsp_mmap,	
	.unlocked_ioctl = dsp_ioctl,
};

static struct miscdevice dsp_dev ={
    .minor = DSP_MAJOR,
    .name = "rk28-dsp",
    .fops = &dsp_fops,	
};

void * dsp_output_va(unsigned int num)
{
    struct rk28_dsp_port *buf_addr = (struct rk28_dsp_port *)port_save_address;
    
    void *vaddr = (void *)buf_addr->dspbuff[num];
    printk("\n%s..%s..%d    ******** nzy *********%d 0x%08x\n",__FUNCTION__,__FILE__,__LINE__,num,buf_addr->dspbuff[num]);

    return vaddr;
}
EXPORT_SYMBOL_GPL(dsp_output_va);

dma_addr_t dsp_output_pa(unsigned int num)
{
    struct rk28_dsp_port *buf_addr = (struct rk28_dsp_port *)port_save_address;
    
    dma_addr_t dma_handle = (dma_addr_t)buf_addr->pa_dspbuff[num];

    return dma_handle;
}
EXPORT_SYMBOL_GPL(dsp_output_pa);


/*
 * driver functions
 */
static int __init dsp_drv_probe(struct platform_device *pdev)
{
    struct rk28_dsp_port *port; 
    int ret = 0;  
    int i=0,j=0;
    
    DBG("%s [%d]\n",__FUNCTION__,__LINE__); 
            
    port = kzalloc(sizeof(struct rk28_dsp_port), GFP_DMA);
    if (port == NULL){
		ret = -ENOMEM;
		goto error_alloc1;
	}	
	port_save_address = (int)port;     
    port->regs = (int)ioremap(PIU_BASE_ADDR, 0x70);
    port->regs0 = (int)ioremap(DSP_L2_IMEM_BASE, 0x600000);
    port->l1_data = (int)ioremap(DSP_BASE_ADDR, 0x10000);
    port->irq  = pdev->resource[1].start;
    platform_set_drvdata(pdev, port);
    port->dev =  pdev->dev;    

	port->bootaddress = (int *)__get_free_page(GFP_DMA);
	port->codeprogramaddress = (int *)__get_free_pages(GFP_DMA,4);
	port->codedataaddress = (int *)__get_free_pages(GFP_DMA,4);
	port->codetableaddress = (int *)__get_free_pages(GFP_DMA,6);
	
    ret = request_irq(port->irq,rk28_dsp_interrupt,IRQF_SHARED,
		     "rk28-dsp",port);	
    if (ret < 0) {  
    	goto error_irq;  
    }    
#if 1//为方便图片共用，将前2个buffer合起来申请
	i = 0;
    port->dspbuff[i] = (int)kzalloc(VIDEO_CODEC_BUFFER_SIZE*2, GFP_DMA);
    if (port->dspbuff[i] == 0){
        ret = -ENOMEM;
        printk("%s [%d]--fail kzalloc--%d\n",__FUNCTION__,__LINE__,i); 
        goto error_alloc6;
    }
    port->pa_dspbuff[i] = __pa(port->dspbuff[i]);
	i++;
	
    port->dspbuff[i] = port->dspbuff[i-1] + VIDEO_CODEC_BUFFER_SIZE;
	port->pa_dspbuff[i] = __pa(port->dspbuff[i]);
	i++;
	

    for(;i<VIDEO_CODEC_BUFFER_NUM;i++){
            port->dspbuff[i] = (int)kzalloc(VIDEO_CODEC_BUFFER_SIZE, GFP_DMA);
            if (port->dspbuff[i] == 0){
		        ret = -ENOMEM;
		        printk("%s [%d]--fail kzalloc--%d\n",__FUNCTION__,__LINE__,i); 
		        goto error_alloc6;
	        }
	        port->pa_dspbuff[i] = __pa(port->dspbuff[i]);
        } 
#else	
    for(i=0;i<VIDEO_CODEC_BUFFER_NUM;i++){
            port->dspbuff[i] = (int)kzalloc(VIDEO_CODEC_BUFFER_SIZE, GFP_DMA);
            if (port->dspbuff[i] == 0){
		        ret = -ENOMEM;
		        printk("%s [%d]--fail kzalloc--%d\n",__FUNCTION__,__LINE__,i); 
		        goto error_alloc6;
	        }
	        port->pa_dspbuff[i] = __pa(port->dspbuff[i]);
        } 
#endif	

	dsp_data = (struct dspdata *)__get_free_page(GFP_DMA);  /*4k*/  ///kmalloc(sizeof(struct dspdata), GFP_KERNEL);
	if (!dsp_data) {
		printk("dsp_date: unable to allocate memory for dspdata metadata.");
		return -1;
	}

	dsp_data->rk28_dsp_buffer_start = dma_alloc_writecombine(NULL, DSP_DEC_INPUT_BUFF_SIZE,  
						&dsp_data->rk28_dsp_buffer_addr, GFP_KERNEL);
	if (!dsp_data->rk28_dsp_buffer_start) {
		printk("%s [%d] -- failed to alloc dsp buffer\n" , __func__ , __LINE__ );
		goto error_alloc6;
	}	

    port->miscdev= dsp_dev;
	ret = misc_register(&(port->miscdev));
	init_MUTEX(&port->sem);
    init_waitqueue_head(&port->rqueue);
	if (ret) {
		printk(KERN_ALERT "Unable to register dsp misc device driver!\n");
		goto error_irq;
	}
	
    return 0;
error_irq:    
    free_irq(port->irq, port); 
error_alloc6:
	for(j=0;j<i;j++)
	    kfree((void *)port->dspbuff[j]);    	
error_alloc1:	
	kfree(port);    
    return ret;
} 
static int dsp_drv_remove(struct platform_device *pdev)
{
    struct rk28_dsp_port	*port = platform_get_drvdata(pdev);
    int i=0;
    
    DBG("%s [%d]\n",__FUNCTION__,__LINE__);  
    misc_deregister(&(port->miscdev));
    free_irq(port->irq, port);
    iounmap((void __iomem *)(port->regs));
    iounmap((void __iomem *)(port->regs0));
    iounmap((void __iomem *)(port->l1_data));
    for(i=0;i<VIDEO_CODEC_BUFFER_NUM;i++)
	     kfree((void *)port->dspbuff[i]);
    kfree(port);  
	
    return 0;	
} 


#if defined(CONFIG_PM) 
static int dsp_drv_suspend(struct platform_device *pdev, pm_message_t state)
{
	DBG("%s [%d]\n",__FUNCTION__,__LINE__);  
	return 0;
}

static int dsp_drv_resume(struct platform_device *pdev)
{
	DBG("%s [%d]\n",__FUNCTION__,__LINE__);  
	return 0;
}
static void dsp_drv_shutdown(struct platform_device *pdev)
{
//	printk("%s [%d]\n",__FUNCTION__,__LINE__);  
	shutdown_dsp_hw();
}

#else
#define dsp_drv_suspend		NULL
#define dsp_drv_resume		NULL
#endif /* CONFIG_PM */
 
static struct platform_driver dsp_driver = {
	.probe		= dsp_drv_probe,
	.remove		= dsp_drv_remove,
	.suspend	= dsp_drv_suspend,
	.resume		= dsp_drv_resume,
	.shutdown = dsp_drv_shutdown,
	.driver		= {
		.name	= "rk28-dsp",
	},
};

static int __init rk28_dsp_mod_init(void)
{
	DBG("%s [%d]\n",__FUNCTION__,__LINE__);  
	return platform_driver_register(&dsp_driver);
}

static void __exit rk28_dsp_mod_exit(void)
{	
	DBG("%s [%d]\n",__FUNCTION__,__LINE__);  
	platform_driver_unregister(&dsp_driver);
}


module_init(rk28_dsp_mod_init);
module_exit(rk28_dsp_mod_exit);
/* Module information */
MODULE_AUTHOR("lhh lhh@rock-chips.com");
MODULE_DESCRIPTION("ROCKCHIP DSP driver");
MODULE_LICENSE("GPL");


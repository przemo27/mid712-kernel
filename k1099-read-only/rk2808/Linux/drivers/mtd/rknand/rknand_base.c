/*
 *  linux/drivers/mtd/rknand/rknand_base.c
 *
 *  Copyright (C) 2005-2009 Fuzhou Rockchip Electronics
 *  ZYF <zyf@rock-chips.com>
 *
 *   
 */

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/reboot.h>
#include <asm/io.h>
#include <asm/mach/flash.h>
#include "api_flash.h"


#define DRIVER_NAME	"rk28xxnand"

#define NAND_DEBUG_LEVEL0 0
#define NAND_DEBUG_LEVEL1 1
#define NAND_DEBUG_LEVEL2 2
#define NAND_DEBUG_LEVEL3 3
long FTLWriteCount =0;
long FTLReadWriteTime =0;

#define use_image
#define RKFTL_WRITE_THREAD

#ifdef use_image
unsigned long SysImageWriteEndAdd = 0;
int g_num_partitions = 0;
#endif

#ifdef CONFIG_MTD_NAND_RK28XX_DEBUG
static int s_debug = CONFIG_MTD_NAND_RK28XX_DEBUG_VERBOSE;
//module_param(s_debug, int, 0);
//MODULE_PARM_DESC(s_debug, "Set Debug Level 0=quiet, 5=noisy");
#undef NAND_DEBUG
/*#define NAND_DEBUG(n, format, arg...) \
	if (n <= s_debug) {	 \
		printk(KERN_NOTICE __FILE__ ":%s(): " format "\n", __FUNCTION__ , ## arg); \
	}*/
#define NAND_DEBUG(n, format, arg...) \
	if (n <= s_debug) {	 \
		printk(format,##arg); \
	}
#else
#undef NAND_DEBUG
#define NAND_DEBUG(n, arg...)
static const int s_debug = 0;
#endif

long RkFtlWriteCount;

/*
* RK28 LBA PARTITIONS,the size and offset value below is default value in this program,
* when RK28 LBA FLASH init,the value will be modify to the value in the nand flash.
*/
#define MAX_FLASH_PARTITION  2

#define ROOTFS_PART_SIZE     300
#define PARA_PART_SIZE       1
#define KERNEL_PART_SIZE     4

static struct mtd_partition rk28_partition_info[] = {
	{ 
	  name: "NAND_ROOTFS_PART",
	  offset:  (KERNEL_PART_SIZE+PARA_PART_SIZE)* 0x800,
	  size:    ROOTFS_PART_SIZE * 0x800,//100MB
	},

	{ 
	  name: "NAND_DATA_PART",
	  offset:  (PARA_PART_SIZE + KERNEL_PART_SIZE + ROOTFS_PART_SIZE) * 0x800,
	  size:    500 * 0x800,//200MB
	},
};


static struct mtd_partition rk28_page_part_info[] = {
	{ 
	  name: "pagecache",
	  offset:  0,
	  size:    32 * 0x800,//32MB
	},

	{ 
	  name: "swap",
	  offset:  (32) * 0x800,
	  size:    64 * 0x800,//200MB
	},
};



/*
 * onenand_state_t - chip states
 * Enumeration for OneNAND flash chip state
 */
typedef enum {
	FL_READY,
	FL_READING,
	FL_WRITING,
	FL_ERASING,
} rknand_state_t;

struct rknand_chip {
	wait_queue_head_t	wq;
	rknand_state_t		state;
};

struct rknand_info {
	struct mtd_info		mtd;
    struct mtd_info		page_mtd;    
	struct mtd_partition *parts;
	struct rknand_chip	rknand;
    struct task_struct *thread;
};

struct rknand_info * gpNandInfo;


static int rknand_get_device(int new_state)
{
	struct rknand_chip *nand_info = &gpNandInfo->rknand;
	DECLARE_WAITQUEUE(wait, current);
	while (1) {
		if (nand_info->state == FL_READY) {
			nand_info->state = new_state;
			break;
		}
		NAND_DEBUG(NAND_DEBUG_LEVEL1,"FLASH not ready\n");
		set_current_state(TASK_UNINTERRUPTIBLE);
		add_wait_queue(&nand_info->wq, &wait);
		schedule();
		remove_wait_queue(&nand_info->wq, &wait);
	}
	return 0;
}

static void rknand_release_device(void)
{
	struct rknand_chip *nand_info = &gpNandInfo->rknand;
	nand_info->state = FL_READY;
	wake_up(&nand_info->wq);
}

#ifdef RKFTL_WRITE_THREAD
unsigned int NeedThreadWriteNow;
static int RkFtlWriteThreadInit(void)
{
    NeedThreadWriteNow = 0;
    return 0;
}

static int RkFtlWriteThreadExit(void)
{
    kthread_stop(gpNandInfo->thread);
    return 0;
}

static int RkFtlWriteThread(void *arg)
{
    int TimeCount = 0;
	RkFtlWriteCount = 0;
	while (!kthread_should_stop()) 
    {
        msleep(1000);
        #if 0
        rknand_get_device(FL_READING);
        FtlTimeHook(); //定时调用FTL Hook函数,回写一些cache
        rknand_release_device();
        TimeCount++;
        if(TimeCount >= 60)
        {
            TimeCount = 0;
            NAND_DEBUG(NAND_DEBUG_LEVEL0,"FTLWriteCount = %d\n",FTLWriteCount);
            extern void PrintBlkEraseCount(void);
            PrintBlkEraseCount();
        }
        #endif
	}
    NAND_DEBUG(NAND_DEBUG_LEVEL0,"......................RkFtlWriteThread END.........................\n");
	return 0;
}
#endif

static int RkFtlRebootHook(struct notifier_block *nb, unsigned long val,
			       void *v)
{
#ifdef RKFTL_WRITE_THREAD
	RkFtlWriteThreadExit();
#endif
    FtlClose();
    NAND_DEBUG(NAND_DEBUG_LEVEL0,"......................RkFtlWriteThread_reboot.........................\n");
	return NOTIFY_DONE;
}

#if 0
#endif

void rkNand_cond_resched(void)
{
    //msleep(1);
    //mdelay(1);
    cond_resched();
}

static int rk28xxnand_read(struct mtd_info *mtd, loff_t from, size_t len,
	size_t *retlen, u_char *buf)
{
	int ret = 0;
	int sector = len;
	int LBA = (int)(from);
	//printk(KERN_NOTICE "read: from=%x,len=%x,\n",(int)from,len);
	NAND_DEBUG(NAND_DEBUG_LEVEL2,"-");
    FTLReadWriteTime++;
    //if(len&511)
    //    printk("rk28xxnand_read: from=%x,len=%x,\n",(int)from,len);
    rknand_get_device(FL_READING);
    if(sector)
    {
        {
		    ret = FtlRead(2,LBA, sector, buf);
        }
    }
    rknand_release_device();
	*retlen = len;
	return 0;//ret;
}

// cmy: before cache part use FtlWriteImage, after cache(include cache) use FtlWrite
#if 1
static int write_Image(int LBA, int sector, u_char* buf)
{
	int remain_sector = sector;
	int write_sector = 0;
	u_char* data = buf;
	int index = LBA;
	int ret = 0;
	while(remain_sector > 0)
    {
		write_sector = remain_sector>32?32:remain_sector;
		ret = FtlWriteImage(index, write_sector, data);
		data += write_sector<<9;
		index += write_sector;
		remain_sector -= write_sector;
    }
	return ret;
}
#endif

static int rk28xxnand_write(struct mtd_info *mtd, loff_t from, size_t len,
	size_t *retlen, u_char *buf)
{
	int ret = 0;
	int sector = len;
	int LBA = (int)(from);
    NAND_DEBUG(NAND_DEBUG_LEVEL1,"+");
    FTLWriteCount+=sector;
    FTLReadWriteTime++;
    //dump_stack();
//	printk(KERN_NOTICE "write: from=%lx,len=%x\n",(int)from,len);
    rknand_get_device(FL_WRITING);
	if(sector)// cmy
	{
#ifdef use_image
		if(LBA < SysImageWriteEndAdd)//0x4E000)
		{
			printk(">>> FtlWriteImage: LBA=0x%08X  sector=%d\n",LBA, sector);
                        
			ret = FtlWriteImage(LBA&0xFFFFFFE0, sector, buf);// LBA align to 32
        }
#else
		if(LBA<0x4E000)
			ret = write_Image(LBA, sector, buf);
#endif
		else
        {
            ret = FtlWrite(2,LBA, sector, buf);
        }
	}
    rknand_release_device();
	*retlen = len;
	return 0;//ret;
}

static int rk28xxnand_page_write(struct mtd_info *mtd, loff_t from, size_t len,
	size_t *retlen, u_char *buf)
{
	int ret = 0;
	int sector = len;
	int LBA = (int)(from);
    NAND_DEBUG(NAND_DEBUG_LEVEL1,"+");
    FTLWriteCount+=sector;
    //printk("+");
	//printk(KERN_NOTICE "write: from=%lx,len=%x,buf=%x,\n",(int)from,len,buf);
    rknand_get_device(FL_WRITING);
	if(sector)// cmy
	{
        ret = FtlPageWrite(LBA, sector, buf);
	}
    rknand_release_device();
	*retlen = len;
	return 0;//ret;
}

static int rk28xxnand_page_read(struct mtd_info *mtd, loff_t from, size_t len,
	size_t *retlen, u_char *buf)
{
	int ret = 0;
	int sector = len;
	int LBA = (int)(from);
	//printk(KERN_NOTICE "read: from=%x,len=%x,\n",(int)from,len);
	NAND_DEBUG(NAND_DEBUG_LEVEL2,"-");
    //printk("-");
    //if(len&511)
    //    printk("rk28xxnand_read: from=%x,len=%x,\n",(int)from,len);
    rknand_get_device(FL_READING);
    if(sector)
    {
        ret = FtlPageRead(LBA, sector, buf);
    }
    rknand_release_device();
	*retlen = len;
	return 0;//ret;
}


static int rk28xxnand_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	int ret = 0;
    if (instr->callback)
		instr->callback(instr);
    NAND_DEBUG(NAND_DEBUG_LEVEL0,"rk28xxnand_erase,add:%x,len:%x\n",instr->addr,instr->len);
	return ret;
}
/**
 * onenand_read_oob - [MTD Interface] Read main and/or out-of-band
 * @param mtd:		MTD device structure
 * @param from:		offset to read from
 * @param ops:		oob operation description structure

 * Read main and/or out-of-band
 */
static int rk28xxnand_read_oob(struct mtd_info *mtd, loff_t from,
			    struct mtd_oob_ops *ops)
{
	int ret = 0;
    NAND_DEBUG(NAND_DEBUG_LEVEL0,"rk28xxnand_read_oob\n");
	return ret;
}

/**
 * onenand_write_oob - [MTD Interface] NAND write data and/or out-of-band
 * @param mtd:		MTD device structure
 * @param to:		offset to write
 * @param ops:		oob operation description structure
 */
static int rk28xxnand_write_oob(struct mtd_info *mtd, loff_t to,
			     struct mtd_oob_ops *ops)
{
	int ret=0;
    NAND_DEBUG(NAND_DEBUG_LEVEL0,"rk28xxnand_write_oob\n");
	return ret;
}

static int rk28xxnand_suspend(struct platform_device *dev, pm_message_t pm)
{
	NAND_DEBUG(NAND_DEBUG_LEVEL0,"rk28xx_nand_suspend: \n");
	return 0;
}

static int rk28xxnand_resume(struct platform_device *dev)
{
	NAND_DEBUG(NAND_DEBUG_LEVEL0,"rk28xx_nand_resume: \n");
	return 0;
}



static int rk28xxnand_block_isbad(struct mtd_info *mtd, loff_t ofs)
{
	//printk("rk28xxnand_block_isbad: \n");
	return 0;
}

static int rk28xxnand_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	NAND_DEBUG(NAND_DEBUG_LEVEL0,"rk28xxnand_block_markbad: \n");
	return 0;
}



/**
 * onenand_sync - [MTD Interface] sync
 * @param mtd		MTD device structure
 *
 * Sync is actually a wait for chip ready function
 */
static void rk28xxnand_sync(struct mtd_info *mtd)
{
	NAND_DEBUG(NAND_DEBUG_LEVEL0,"rk28xxnand_sync: \n");
}


/**
 * onenand_lock - [MTD Interface] Lock block(s)
 * @param mtd		MTD device structure
 * @param ofs		offset relative to mtd start
 * @param len		number of bytes to unlock
 *
 * Lock one or more blocks
 */
static int rk28xxnand_lock(struct mtd_info *mtd, loff_t ofs, size_t len)
{
	NAND_DEBUG(NAND_DEBUG_LEVEL0,"rk28xxnand_lock: \n");
	return 0;
}

/**
 * onenand_unlock - [MTD Interface] Unlock block(s)
 * @param mtd		MTD device structure
 * @param ofs		offset relative to mtd start
 * @param len		number of bytes to unlock
 *
 * Unlock one or more blocks
 */
static int rk28xxnand_unlock(struct mtd_info *mtd, loff_t ofs, size_t len)
{
	NAND_DEBUG(NAND_DEBUG_LEVEL0,"rk28xxnand_unlock: \n");
	return 0;
}

extern void FlashTest(void);
static int rk28xxnand_init(struct rknand_info *nand_info)
{
	struct mtd_info	   *mtd = &nand_info->mtd;
	struct mtd_info	   *page_mtd = &nand_info->page_mtd;
	struct rknand_chip *rknand = &nand_info->rknand;

	rknand->state = FL_READY;
	init_waitqueue_head(&rknand->wq);

	if(FTLInit())
	{
		NAND_DEBUG(NAND_DEBUG_LEVEL0,"FTLInit Error: \n");
		return -ENXIO;
    }
    NAND_DEBUG(NAND_DEBUG_LEVEL0,"FTLInit OK: \n");
    //FlashTest();
    mtd->size = FtlGetCapacity(0xFF);
    //readflash modify rk28_partition_info
    NAND_DEBUG(NAND_DEBUG_LEVEL0,"mtd->size: %x\n",mtd->size);
    mtd->oobsize = 0;
    mtd->oobavail = 0;
    mtd->ecclayout = 0;
    mtd->erasesize = FlashGetPageSize(); //sector
    mtd->writesize = FlashGetPageSize();

	// Fill in remaining MTD driver data 
	mtd->type = MTD_NANDFLASH;//MTD_RAM;//
	mtd->flags = (MTD_WRITEABLE|MTD_NO_ERASE);//
	mtd->erase = rk28xxnand_erase;
	mtd->point = NULL;
	mtd->unpoint = NULL;
	mtd->read = rk28xxnand_read;
	mtd->write = rk28xxnand_write;
	mtd->read_oob = rk28xxnand_read_oob;
	mtd->write_oob = rk28xxnand_write_oob;
	mtd->panic_write = NULL;

	mtd->sync = rk28xxnand_sync;
	mtd->lock = rk28xxnand_lock;
	mtd->unlock = rk28xxnand_unlock;
	mtd->suspend = rk28xxnand_suspend;
	mtd->resume = rk28xxnand_resume;
	mtd->block_isbad = rk28xxnand_block_isbad;
	mtd->block_markbad = rk28xxnand_block_markbad;
	mtd->owner = THIS_MODULE;


    //FlashTest();
    page_mtd->size = FtlGetPageZoneCapacity();
    //readflash modify rk28_partition_info
    NAND_DEBUG(NAND_DEBUG_LEVEL0,"page_mtd->size: %x\n",page_mtd->size);
    page_mtd->oobsize = 0;
    page_mtd->oobavail = 0;
    page_mtd->ecclayout = 0;
    page_mtd->erasesize = FlashGetPageSize(); //sector
    page_mtd->writesize = FlashGetPageSize();

	// Fill in remaining MTD driver data 
	page_mtd->type = MTD_NANDFLASH;//MTD_RAM;//
	page_mtd->flags = (MTD_WRITEABLE|MTD_NO_ERASE);//
	page_mtd->erase = rk28xxnand_erase;
	page_mtd->point = NULL;
	page_mtd->unpoint = NULL;
	page_mtd->read = rk28xxnand_page_read;
	page_mtd->write = rk28xxnand_page_write;
	page_mtd->read_oob = rk28xxnand_read_oob;
	page_mtd->write_oob = rk28xxnand_write_oob;
	page_mtd->panic_write = NULL;

	page_mtd->sync = rk28xxnand_sync;
	page_mtd->lock = rk28xxnand_lock;
	page_mtd->unlock = rk28xxnand_unlock;
	page_mtd->suspend = rk28xxnand_suspend;
	page_mtd->resume = rk28xxnand_resume;
	page_mtd->block_isbad = rk28xxnand_block_isbad;
	page_mtd->block_markbad = rk28xxnand_block_markbad;
	page_mtd->owner = THIS_MODULE;

	return 0;
}


/*
 * CMY: 增加了对命令行分区信息的支持
 *		若cmdline有提供分区信息，则使用cmdline的分区信息进行分区
 *		若cmdline没有提供分区信息，则使用默认的分区信息(rk28_partition_info)进行分区
 */

#ifdef CONFIG_MTD_CMDLINE_PARTS
const char *part_probes[] = { "cmdlinepart", NULL }; 
#endif 

static int rk28xxnand_add_partitions(struct rknand_info *nand_info)
{
	NAND_DEBUG(NAND_DEBUG_LEVEL0,"Enter rk28xxnand_add_partitions\n");
#ifdef CONFIG_MTD_CMDLINE_PARTS
    int num_partitions = 0; 

	// 从命令行解析分区的信息
    num_partitions = parse_mtd_partitions(&(nand_info->mtd), part_probes, &nand_info->parts, 0); 

    if(num_partitions > 0) { 
#ifdef use_image
		g_num_partitions = num_partitions;
#endif
		return add_mtd_partitions(&nand_info->mtd, nand_info->parts, num_partitions);
    } 
#endif 
	// 如果命令行没有提供分区信息，则使用默认的分区信息
//	printk("parse_mtd_partitions\n");

	rk28_partition_info[1].size = nand_info->mtd.size - ((ROOTFS_PART_SIZE + PARA_PART_SIZE + KERNEL_PART_SIZE)*0x100000);
//	rk28_partition_info[2].offset = rk28_partition_info[1].size + rk28_partition_info[1].offset;
#ifdef use_image
	g_num_partitions = sizeof(rk28_partition_info)/sizeof(struct mtd_partition);
#endif
	return add_mtd_partitions(&nand_info->mtd, rk28_partition_info, sizeof(rk28_partition_info)/sizeof(struct mtd_partition));//MAX_FLASH_PARTITION);
}

static int __devinit rk28xxnand_probe(struct platform_device *pdev)
{

	struct rknand_info *nand_info;
    struct mtd_partition *parts;
    int i;
	struct resource *res = pdev->resource;
	int err = 0;
	NAND_DEBUG(NAND_DEBUG_LEVEL0,"rk28xxnand_probe: \n");
	gpNandInfo = kzalloc(sizeof(struct rknand_info), GFP_KERNEL);
	if (!gpNandInfo)
		return -ENOMEM;
    
    nand_info = gpNandInfo;
    
	nand_info->mtd.name = pdev->dev.bus_id;
	nand_info->mtd.priv = &nand_info->rknand;
	nand_info->mtd.owner = THIS_MODULE;
    
	nand_info->page_mtd.name = pdev->dev.bus_id;
	nand_info->page_mtd.priv = &nand_info->rknand;
	nand_info->page_mtd.owner = THIS_MODULE;

	if(rk28xxnand_init(nand_info))
	{
		err = -ENXIO;
		goto  exit_free;
	}
	rk28xxnand_add_partitions(nand_info);
    #if 0
    nand_info->page_mtd.name = "pagecache";
    add_mtd_device(&nand_info->page_mtd);
    #else
    rk28_page_part_info[1].size = nand_info->page_mtd.size - rk28_page_part_info[0].size;
    add_mtd_partitions(&nand_info->page_mtd, rk28_page_part_info, 2);
    #endif
   
    gpNandInfo->mtd.reboot_notifier.notifier_call = RkFtlRebootHook;
	register_reboot_notifier(&gpNandInfo->mtd.reboot_notifier);
#ifdef RKFTL_WRITE_THREAD
    RkFtlWriteThreadInit();
    gpNandInfo->thread = kthread_run(RkFtlWriteThread, gpNandInfo,"%s", "RKFTL");

#endif
    
#ifdef use_image
    parts = nand_info->parts;
    for(i=0;i<g_num_partitions;i++)
    {
        printk(">>> part[%d]: name=%s offset=0x%X\n", i, parts[i].name, parts[i].offset);
        if(strcmp(parts[i].name,"cache") == 0)
        {
            SysImageWriteEndAdd = parts[i].offset;
	        printk(">>> SysImageWriteEndAdd=0x%X\n", SysImageWriteEndAdd);
            break;
        }
    }
#endif

    FtlSetSysProtAddr(SysImageWriteEndAdd);
	dev_set_drvdata(&pdev->dev, nand_info);

	return 0;

exit_free:
	if(nand_info)
      	kfree(nand_info);

	return err;
}

static int __devexit rknand_remove(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rknand_info *nand_info = dev_get_drvdata(&pdev->dev);
	struct resource *res = pdev->resource;
	unsigned long size = res->end - res->start + 1;
	NAND_DEBUG(NAND_DEBUG_LEVEL0,"rk28xx_rknand_remove: \n");
	dev_set_drvdata(&pdev->dev, NULL);

	if (nand_info) {
		if (nand_info->parts)
			del_mtd_partitions(&nand_info->mtd);
		else
			del_mtd_device(&nand_info->mtd);

		//rknand_release(&nand_info->mtd);
		release_mem_region(res->start, size);		
		kfree(nand_info);
	}
	return 0;
}

static struct platform_driver rknand_driver = {
	.probe		= rk28xxnand_probe,
	.remove		= rknand_remove,
	.suspend	= rk28xxnand_suspend,
	.resume		= rk28xxnand_resume,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
		//.bus		= &platform_bus_type,
	},
};


MODULE_ALIAS(DRIVER_NAME);

static int __init rknand_init(void)
{
	int ret;
	NAND_DEBUG(NAND_DEBUG_LEVEL0,"rknand_init: \n");
	ret = platform_driver_register(&rknand_driver);
	NAND_DEBUG(NAND_DEBUG_LEVEL0,"platform_driver_register:ret = %x \n",ret);
	return ret;
}

static void __exit rknand_exit(void)
{
	NAND_DEBUG(NAND_DEBUG_LEVEL0,"rknand_exit: \n");
    NAND_DEBUG(NAND_DEBUG_LEVEL0,"......................rknand_exit.........................\n");

#ifdef RKFTL_WRITE_THREAD
    RkFtlWriteThreadExit();
#endif
    if(gpNandInfo)
    {
	    unregister_reboot_notifier(&gpNandInfo->mtd.reboot_notifier);
#ifdef RKFTL_WRITE_THREAD
        kthread_stop(gpNandInfo->thread);
#endif
    }
    FtlClose();
    platform_driver_unregister(&rknand_driver);
}

module_init(rknand_init);
module_exit(rknand_exit);

MODULE_LICENSE("");
MODULE_AUTHOR("ZYF <zyf@rock-chips.com>");
MODULE_DESCRIPTION("FTL layer for SLC and MlC nand flash on RK28xx SDK boards");



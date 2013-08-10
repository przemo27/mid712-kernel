/*
** linux/drivers/mmc/host/rk28_sdmmc0.c
**
**  Copyright (C),2009, Fuzhou Rockchip Electronics Co.,Ltd.
**
**  Auther: Xie Bangwang
**  Desc  : driver for SDMMC0 controller(mainly used for SD/MMC/SDHC card).
**
** This software is licensed under the terms of the GNU General Public
** License version 2, as published by the Free Software Foundation, and
** may be copied, distributed, and modified under those terms.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
 */


#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>

#include <asm/dma.h>
#include <linux/mmc/host.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach/mmc.h>

#include <asm/arch/rk28_sdmmc.h>

#include <asm/arch/rk28_scu.h>
#include <asm/arch/rk28_irqs.h>
#include <asm/arch/typedef.h>
#include <asm/arch/intc.h>
#include <asm/arch/iomux.h>
#include <asm/arch/gpio.h>
#include <asm/arch/drivers_delay.h>
#include <asm/arch/api_scu.h>
#include <asm/arch/clock.h>
#include <asm/arch/board.h>
#include <linux/delay.h>

#include <linux/spinlock_types.h>

#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
android_suspend_lock_t sdmmc0_request_lock;
#endif



#define DRIVER_NAME "rk28_sdmmc0"



int  sdmmc0_disable_Irq_ForRemoval;
spinlock_t  sdmmc0_spinlock;     /* lock for this struct. spin_lock_init */ 



extern int 	    gpio_request(unsigned gpio, const char *tag);
extern void 	gpio_free(unsigned gpio);
extern int 	    mmc_resume_host(struct mmc_host *host);
extern int 	    mmc_suspend_host(struct mmc_host *host, pm_message_t state);



/*
 * Handle an MMC request
 */
static void rk28_sdmmc0_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
    struct rk28mci_host *host = mmc_priv(mmc);

    host->request = mrq;
    host->cmd_is_stop = 0;
    host->cmderror = 0;
#if 0
    if (rk28_sdmmc_IsCardPresence(host))
        rk28_sdmmc_send_request(mmc);
    else {
        mrq->cmd->error = -EIO;
        }
#else
    rk28_sdmmc_send_request(mmc);

#endif
    
    mmc_request_done(mmc, mrq);
}


/*
 * Set the IOS
 */
static void rk28_sdmmc0_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
    struct rk28mci_host *host = mmc_priv(mmc);

    unsigned long iflags;

    spin_lock_irqsave(&host->complete_lock, iflags);

    if (MMC_POWER_UP == ios->power_mode)
    {    
        //reset the controller before the power up
        //if (rk28_sdmmc_IsCardPresence(host))
        {           
            rk28_sdmmc_hw_init(host);
        }
    }
   
    host->bus_mode = ios->bus_mode;

    /* Clock settings */
    if (ios->clock && (host->iosclock != ios->clock))
    {
        //set the scu mode
        if (ios->clock > 400*1000)
        {
            //change the SCU mode only when the clock is large than 400Khz.
            __rockchip_scu_change_mode(SCU_IPID_SDMMC0, SCU_MODE_FREQ, ios->clock/1000000); //unit is Mhz
        }
      
        //printk("%s..%s..%d ***********The clock div is setted. the ios->clock=%d ******====xbw===******\n",__FUNCTION__,__FILE__,__LINE__, ios->clock);
        
        rk28_mci_ChangeFreq(host, ios->clock/1000); //the second parameter unit is KHz
        host->iosclock = ios->clock;
    }

    if ((ios->bus_width == MMC_BUS_WIDTH_4))
    {
        host->busWidth = BUS_WIDTH_4_BIT;
        rk28_sdmmc_SetHostBusWidth(host);
    }
    else
    {
        host->busWidth = BUS_WIDTH_1_BIT;
        rk28_sdmmc_SetHostBusWidth(host);
    }

   if(MMC_POWER_OFF == ios->power_mode)
    {        
        rk28_sdmmc_disable(host);
    }

    spin_unlock_irqrestore(&host->complete_lock, iflags);

}


static void rk28_sdmmc0_switch_timer(unsigned long arg)
{
	struct rk28mci_host *host = (struct rk28mci_host *) arg;	
    struct mmc_host *mmc = container_of(host,  struct mmc_host, private);

    spin_lock( &sdmmc0_spinlock);
    
    printk("%s....%s....%d   **** timer open, then enable IRQ_OF_removal/insertion*****xbw****\n",__FUNCTION__,__FILE__,__LINE__);
    sdmmc0_disable_Irq_ForRemoval = 0; //打开中断      
    spin_unlock( &sdmmc0_spinlock);        

    //主动再调用次检测   
    mmc_detect_change(mmc, 0);
}





extern int rk28_sdmmc_reset( void );
int resetTimes = 0;//统计连续reset的次数。
extern struct rk28mci_host *testhost;

ssize_t sdmmc_reset_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
    struct rk28mci_host *host =  testhost;  
    struct mmc_host *mmc = container_of(host,  struct mmc_host, private);
    int currentTimes=0;
    
    //printk("%s----开始处理消息----- %s -----\n", __func__ , buf);
    if( !strncmp(buf,"RemoveDone" , strlen("RemoveDone")) )
    {
        printk("%s------mmc receive the message of %s -----\n", __func__ , buf);
       
        spin_lock( &sdmmc0_spinlock);
        del_timer(&host->switch_timer);
        sdmmc0_disable_Irq_ForRemoval = 0; //打开中断      
        spin_unlock( &sdmmc0_spinlock);
        
        //主动执行一次检测, 若有卡存在
        if(host->preCardState)
            mmc_detect_change(mmc, 0);
    }
    else if( !strncmp(buf,"Removing" , strlen("Removing")) )
    {        
        printk("%s------mmc receive the message of %s -----\n", __func__ , buf);

        //vold is removing, so modify the timer.
        spin_lock( &sdmmc0_spinlock);      
        mod_timer(&host->switch_timer, jiffies +msecs_to_jiffies(RK28_SDMMC0_SWITCH_POLL_DELAY));        
        spin_unlock( &sdmmc0_spinlock);
    }
    else if( !strncmp(buf,"to_reset!" , strlen("to_reset!")) ) 
    {       
        printk("%s------mmc receive the message of %s -----\n", __func__ , buf);

        //设置reset的允许次数
        spin_lock( &sdmmc0_spinlock); 
        ++resetTimes;
        currentTimes = resetTimes;
        spin_unlock( &sdmmc0_spinlock);
        
        //连续调用reset超过次数，不执行。        
        if(currentTimes <= 3)
        {          
            printk("%s....%s....%d   **** Begin to call rk28_sdmmc_reset() Times=%d *****xbw****\n",__FUNCTION__,__FILE__,__LINE__, resetTimes);
            rk28_sdmmc_reset();
        }
    }
    else if(!strncmp(buf,"mounted!" , strlen("mounted!")))
    {        
        printk("%s------mmc receive the message of %s -----\n", __func__ , buf);
        
        spin_lock( &sdmmc0_spinlock); 
        resetTimes = 0;        
        spin_unlock( &sdmmc0_spinlock);
    }

    return count;
}



struct kobj_attribute mmc_reset_attrs = 
{
        .attr = {
                .name = "rescan",
                .mode = 0777},
        .show = NULL,
        .store = sdmmc_reset_store,
};
struct attribute *mmc_attrs[] = 
{
        &mmc_reset_attrs.attr,
        NULL
};

static struct kobj_type mmc_kset_ktype = {
	.sysfs_ops	= &kobj_sysfs_ops,
	.default_attrs = &mmc_attrs[0],
};
static int rk28_sdmmc0_add_attr( struct platform_device *pdev )
{
        int result;
		 struct kobject *parentkobject; 
        struct kobject * me = kmalloc(sizeof(struct kobject) , GFP_KERNEL );
        if( !me )
                return -ENOMEM;
        memset(me ,0,sizeof(struct kobject));
        kobject_init( me , &mmc_kset_ktype );
        //result = kobject_add( me , &pdev->dev.kobj , "%s", "RESET" );
        parentkobject = &pdev->dev.kobj ;
		 result = kobject_add( me , parentkobject->parent->parent, "%s", "resetSdCard" );	
        return result;
}


static const struct mmc_host_ops rk28_sdmmc0_ops =
{
    .request	= rk28_sdmmc0_request,
    .set_ios	= rk28_sdmmc0_set_ios,
};

/*
 * Probe for the device
 */
static int __init rk28_sdmmc0_probe(struct platform_device *pdev)
{
    struct mmc_host *mmc;
    struct rk28mci_host *host;
    struct resource *res;
    int ret;


    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res)
        return -ENXIO;

    if (!request_mem_region(res->start, res->end - res->start + 1, DRIVER_NAME))
        return -EBUSY;

    mmc = mmc_alloc_host(sizeof(struct rk28mci_host), &pdev->dev);
    if (!mmc)
    {
        ret = -ENOMEM;
        dev_dbg(&pdev->dev, "couldn't allocate mmc host\n");
        goto fail6;


    }

    mmc->ops = &rk28_sdmmc0_ops;
    mmc->f_min = FOD_FREQ * 1000;
    mmc->f_max = 25000000;
    mmc->ocr_avail = MMC_VDD_27_28|MMC_VDD_28_29|MMC_VDD_29_30|MMC_VDD_30_31
                     | MMC_VDD_31_32|MMC_VDD_32_33 | MMC_VDD_33_34 | MMC_VDD_34_35| MMC_VDD_35_36;    ///set valid volage 2.7---3.6
                     
    printk("%s..%s..%d ********host availd ocr=0x%x====xbw===*******************\n",__FUNCTION__,__FILE__,__LINE__, mmc->ocr_avail);

    mmc->max_blk_size = 4095;
    mmc->max_blk_count = 1024; 
    mmc->max_req_size = mmc->max_blk_count*512; // 512K
    mmc->max_seg_size = mmc->max_req_size;
    
    host = mmc_priv(mmc);
    host->mmc = mmc;
    host->buffer = NULL;
    host->bus_mode = 0;
    host->board = pdev->dev.platform_data;

    host->res = res;
    
    testhost = host; //used for debug
    
    host->cmdr = 0;
    host->lockscu = 0; //no lock SCU
    host->cmderror = 0;


    host->complete_dma = 0; //DMA channel has not request.
    host->requestDmaError = 0;//

    setup_timer(&host->switch_timer, rk28_sdmmc0_switch_timer, (unsigned long) host);

    spin_lock_init(&host->complete_lock);

#ifdef CONFIG_ANDROID_POWER
	sdmmc0_request_lock.name = "sdmmc-Req";
	android_init_suspend_lock(&sdmmc0_request_lock);
#endif

    sdmmc0_disable_Irq_ForRemoval = 0;
    spin_lock_init( &sdmmc0_spinlock);

    mmc->caps |=  ( MMC_CAP_MULTIWRITE | MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED);
    
    if (1)//if (host->board->wire4)
    {
        mmc->caps |= MMC_CAP_4_BIT_DATA;
    }

    /*
     * Map I/O region
     */
    // host->baseaddr = ioremap(res->start, res->end - res->start + 1); //
    host->baseaddr = (void * __iomem)SDMMC0_BASE_ADDR_VA; //
    if (!host->baseaddr)
    {
        ret = -ENOMEM;
        goto fail1;
    }

   
    //register SCU
    rockchip_scu_register( SCU_IPID_SDMMC0, SCU_MODE_FREQ, 25, NULL); //now,first register MaxFreq=25Mhz


    /*
     * Allocate the MCI interrupt
     */
    host->irq = platform_get_irq(pdev, 0);
    ret = request_irq(host->irq, rk28_sdmmc_irq, IRQF_SHARED, mmc_hostname(mmc), host);
    if (ret)
    {
        printk("%s..%s..%d **********request irq failue====xbw===*******************\n",__FUNCTION__,__FILE__,__LINE__);
        goto fail0;
    }

    platform_set_drvdata(pdev, mmc);
    
    //printk("%s..%s..%d **********SD probe Over. now it begin to mmc_add_host====xbw===*******************\n",__FUNCTION__,__FILE__,__LINE__);

    mmc_add_host(mmc);
    rk28_sdmmc0_add_attr(pdev);
    return 0;

fail0:
    iounmap(host->baseaddr);
    
fail1:
    mmc_free_host(mmc);
fail6:
    release_mem_region(res->start, res->end - res->start + 1);
    dev_err(&pdev->dev, "probe failed, err %d\n", ret);

    return ret;
}



/*
 * Remove a device
 */
static int __exit rk28_sdmmc0_remove(struct platform_device *pdev)
{
    struct mmc_host *mmc = platform_get_drvdata(pdev);
    struct rk28mci_host *host;
    struct resource *res;

    if (!mmc)
        return -1;

    //printk("%s..%s..%d ********************====xbw===*******************\n",__FUNCTION__,__FILE__,__LINE__);

    host = mmc_priv(mmc);

    rk28_sdmmc_disable(host);
    mmc_remove_host(mmc);
    free_irq(host->irq, host);

    iounmap(host->baseaddr);
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    release_mem_region(res->start, res->end - res->start + 1);

    mmc_free_host(mmc);
    platform_set_drvdata(pdev, NULL);
    pr_debug("MCI Removed\n");

    return 0;
}



#if 0 //def CONFIG_PM
static int rk28_sdmmc0_suspend(struct platform_device *pdev, pm_message_t state)
{

    struct mmc_host *mmc = platform_get_drvdata(pdev);
    int ret = 0;

    if (mmc)
        ret = mmc_suspend_host(mmc, state);

    return ret;
}

static int rk28_sdmmc0_resume(struct platform_device *pdev)
{
    struct mmc_host *mmc = platform_get_drvdata(pdev);
    struct rk28mci_host *host = mmc_priv(mmc);

    int ret = 0;

    if (mmc && rk28_sdmmc_IsCardPresence(host))
    {
        ret = mmc_resume_host(mmc);
    }


    return ret;
}
#else
#define rk28_sdmmc0_suspend	NULL
#define rk28_sdmmc0_resume		NULL
#endif



static struct platform_driver rk28_sdmmc0_driver =
{
    .probe		= rk28_sdmmc0_probe,
    .remove		= __exit_p(rk28_sdmmc0_remove),
    .suspend		= rk28_sdmmc0_suspend,
    .resume		= rk28_sdmmc0_resume,
    .driver		= {
        .name	= DRIVER_NAME,
        .owner	= THIS_MODULE,
    },
};



static int __init rk28_sdmmc0_init(void)
{
    return platform_driver_probe(&rk28_sdmmc0_driver, rk28_sdmmc0_probe);
}

static void __exit rk28_sdmmc0_exit(void)
{
    platform_driver_unregister(&rk28_sdmmc0_driver);
}

module_init(rk28_sdmmc0_init);
module_exit(rk28_sdmmc0_exit);

MODULE_DESCRIPTION("RK28 SDMMC0 driver,mainly used for SDMMC");
MODULE_AUTHOR("xbw");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rk28_sdmmc0");

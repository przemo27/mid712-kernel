/*
 *  linux/drivers/mmc/core/sd.c
 *
 *  Copyright (C) 2003-2004 Russell King, All Rights Reserved.
 *  SD support Copyright (C) 2004 Ian Molton, All Rights Reserved.
 *  Copyright (C) 2005-2007 Pierre Ossman, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>

#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>

#include "core.h"
#include "bus.h"
#include "mmc_ops.h"
#include "sd_ops.h"

static const unsigned int tran_exp[] = {
	10000,		100000,		1000000,	10000000,
	0,		0,		0,		0
};

static const unsigned char tran_mant[] = {
	0,	10,	12,	13,	15,	20,	25,	30,
	35,	40,	45,	50,	55,	60,	70,	80,
};

static const unsigned int tacc_exp[] = {
	1,	10,	100,	1000,	10000,	100000,	1000000, 10000000,
};

static const unsigned int tacc_mant[] = {
	0,	10,	12,	13,	15,	20,	25,	30,
	35,	40,	45,	50,	55,	60,	70,	80,
};

#define UNSTUFF_BITS(resp,start,size)					\
	({								\
		const int __size = size;				\
		const u32 __mask = (__size < 32 ? 1 << __size : 0) - 1;	\
		const int __off = 3 - ((start) / 32);			\
		const int __shft = (start) & 31;			\
		u32 __res;						\
									\
		__res = resp[__off] >> __shft;				\
		if (__size + __shft > 32)				\
			__res |= resp[__off-1] << ((32 - __shft) % 32);	\
		__res & __mask;						\
	})

/*
 * Given the decoded CSD structure, decode the raw CID to our CID structure.
 */
static void mmc_decode_cid(struct mmc_card *card)
{
	u32 *resp = card->raw_cid;

	memset(&card->cid, 0, sizeof(struct mmc_cid));

	/*
	 * SD doesn't currently have a version field so we will
	 * have to assume we can parse this.
	 */
	card->cid.manfid		= UNSTUFF_BITS(resp, 120, 8);
	card->cid.oemid			= UNSTUFF_BITS(resp, 104, 16);
	card->cid.prod_name[0]		= UNSTUFF_BITS(resp, 96, 8);
	card->cid.prod_name[1]		= UNSTUFF_BITS(resp, 88, 8);
	card->cid.prod_name[2]		= UNSTUFF_BITS(resp, 80, 8);
	card->cid.prod_name[3]		= UNSTUFF_BITS(resp, 72, 8);
	card->cid.prod_name[4]		= UNSTUFF_BITS(resp, 64, 8);
	card->cid.hwrev			= UNSTUFF_BITS(resp, 60, 4);
	card->cid.fwrev			= UNSTUFF_BITS(resp, 56, 4);
	card->cid.serial		= UNSTUFF_BITS(resp, 24, 32);
	card->cid.year			= UNSTUFF_BITS(resp, 12, 8);
	card->cid.month			= UNSTUFF_BITS(resp, 8, 4);

	card->cid.year += 2000; /* SD cards year offset */
}

/*
 * Given a 128-bit response, decode to our card CSD structure.
 */
static int mmc_decode_csd(struct mmc_card *card)
{
	struct mmc_csd *csd = &card->csd;
	unsigned int e, m, csd_struct;
	u32 *resp = card->raw_csd;
        
        /*
	printk ("mmc_decode_csd: csd:%x ************* xbw *****************\n", *(resp));
	printk ("                   :%x ************* xbw *****************\n", *(resp+1));
	printk ("                   :%x ************* xbw *****************\n", *(resp+2));
	printk ("                   :%x ************* xbw *****************\n", *(resp+3));
       */
	csd_struct = UNSTUFF_BITS(resp, 126, 2);
	//printk ("mmc_decode_csd: csd_struct:%x ************* xbw *****************\n", csd_struct);

	switch (csd_struct) {
	case 0:
		m = UNSTUFF_BITS(resp, 115, 4);
		e = UNSTUFF_BITS(resp, 112, 3);
		csd->tacc_ns	 = (tacc_exp[e] * tacc_mant[m] + 9) / 10;
		csd->tacc_clks	 = UNSTUFF_BITS(resp, 104, 8) * 100;

		m = UNSTUFF_BITS(resp, 99, 4);
		e = UNSTUFF_BITS(resp, 96, 3);
		csd->max_dtr	  = tran_exp[e] * tran_mant[m];
		csd->cmdclass	  = UNSTUFF_BITS(resp, 84, 12);

		e = UNSTUFF_BITS(resp, 47, 3);
		m = UNSTUFF_BITS(resp, 62, 12);
		csd->capacity	  = (1 + m) << (e + 2);

		csd->read_blkbits = UNSTUFF_BITS(resp, 80, 4);
		csd->read_partial = UNSTUFF_BITS(resp, 79, 1);
		csd->write_misalign = UNSTUFF_BITS(resp, 78, 1);
		csd->read_misalign = UNSTUFF_BITS(resp, 77, 1);
		csd->r2w_factor = UNSTUFF_BITS(resp, 26, 3);
		csd->write_blkbits = UNSTUFF_BITS(resp, 22, 4);
		csd->write_partial = UNSTUFF_BITS(resp, 21, 1);
		break;
	case 1:
		/*
		 * This is a block-addressed SDHC card. Most
		 * interesting fields are unused and have fixed
		 * values. To avoid getting tripped by buggy cards,
		 * we assume those fixed values ourselves.
		 */
		mmc_card_set_blockaddr(card);

		csd->tacc_ns	 = 0; /* Unused */
		csd->tacc_clks	 = 0; /* Unused */

		m = UNSTUFF_BITS(resp, 99, 4);
		e = UNSTUFF_BITS(resp, 96, 3);
		csd->max_dtr	  = tran_exp[e] * tran_mant[m];
		csd->cmdclass	  = UNSTUFF_BITS(resp, 84, 12);

		m = UNSTUFF_BITS(resp, 48, 22);
		csd->capacity     = (1 + m) << 10;

		csd->read_blkbits = 9;
		csd->read_partial = 0;
		csd->write_misalign = 0;
		csd->read_misalign = 0;
		csd->r2w_factor = 4; /* Unused */
		csd->write_blkbits = 9;
		csd->write_partial = 0;
		break;
	default:
		printk(KERN_ERR "%s: unrecognised CSD structure version %d\n",
			mmc_hostname(card->host), csd_struct);
		return -EINVAL;
	}

	return 0;
}

/*
 * Given a 64-bit response, decode to our card SCR structure.
 */
 #if 0
static int mmc_decode_scr(struct mmc_card *card)
{
	struct sd_scr *scr = &card->scr;
	unsigned int scr_struct;
	u32 resp[4];
	//u32 resp[2];//
	u32 *test;

    resp[3] = card->raw_scr[1];//
    resp[2] = card->raw_scr[0];//
    
	//resp[1] = card->raw_scr[1];//
	//resp[0] = card->raw_scr[0];//
	

	scr_struct = UNSTUFF_BITS(resp, 60, 4);
	if (scr_struct != 0) {
		printk(KERN_ERR "%s: unrecognised SCR structure version %d\n",
			mmc_hostname(card->host), scr_struct);
		return -EINVAL;
	}
    //printk("%s..%s..%d *********card->raw_scr[0]= %8x, card->raw_scr[1]=%8x*********====xbw===*******************\n",__FUNCTION__,__FILE__,__LINE__, resp[0], resp[1]);
    printk("%s..%s..%d *********card->raw_scr[0]= %8x, card->raw_scr[1]=%8x*********====xbw===*******************\n",__FUNCTION__,__FILE__,__LINE__, resp[2], resp[3]);
#if 1
	scr->sda_vsn = UNSTUFF_BITS(resp, 56, 4);
	scr->bus_widths = UNSTUFF_BITS(resp, 48, 4);
    printk("%s..%s..%d *******scr->sda_vsn=%x **scr->bus_widths=%x*********====xbw===*******************\n",__FUNCTION__,__FILE__,__LINE__, scr->sda_vsn, scr->bus_widths);
    printk("%s..%s..%d *******address_resp=%x *****address_resp[0]=%x*********====xbw===*******************\n",__FUNCTION__,__FILE__,__LINE__, resp, &resp[0]);
#else
    scr->sda_vsn = ( (resp[1]>>24)&0xf);
	scr->bus_widths = ( (resp[1]>>16)&0xf);;
    printk("%s..%s..%d *******scr->sda_vsn=%x **scr->bus_widths=%x****** 2 ***====xbw===*******************\n",__FUNCTION__,__FILE__,__LINE__, scr->sda_vsn, scr->bus_widths);
#endif
	return 0;
}
#else
static int mmc_decode_scr(struct mmc_card *card)
{
	struct sd_scr *scr = &card->scr;
	unsigned int scr_struct;
	u32 resp[4];

	resp[3] = card->raw_scr[1];
	resp[2] = card->raw_scr[0];
   // printk("%s..%s..%d *********card->raw_scr[0]= %8x, card->raw_scr[1]=%8x*********====xbw===*******************\n",__FUNCTION__,__FILE__,__LINE__, resp[2], resp[3]);

	scr_struct = UNSTUFF_BITS(resp, 60, 4);
	if (scr_struct != 0) {
		printk(KERN_ERR "%s: unrecognised SCR structure version %d\n",
			mmc_hostname(card->host), scr_struct);
		return -EINVAL;
	}

	scr->sda_vsn = UNSTUFF_BITS(resp, 56, 4);
	scr->bus_widths = UNSTUFF_BITS(resp, 48, 4);
   // printk("%s..%s..%d *******scr->sda_vsn=%x **scr->bus_widths=%x****** 2 ***====xbw===*******************\n",__FUNCTION__,__FILE__,__LINE__, scr->sda_vsn, scr->bus_widths);

	return 0;
}

#endif

/*
 * Fetches and decodes switch information
 */
static int mmc_read_switch(struct mmc_card *card)
{
	int err;
	u8 *status;

	if (card->scr.sda_vsn < SCR_SPEC_VER_1)
	{
		//printk("\n mmc_read_switch:version low:%x ********xbw***********\n", card->scr.sda_vsn);
		return 0;
	}

	if (!(card->csd.cmdclass & CCC_SWITCH)) {
		printk(KERN_WARNING "%s: card lacks mandatory switch "
			"function, performance might suffer.\n",
			mmc_hostname(card->host));
		return 0;
	}

	err = -EIO;

	status = kmalloc(64, GFP_KERNEL);
	if (!status) {
		printk(KERN_ERR "%s: could not allocate a buffer for "
			"switch capabilities.\n", mmc_hostname(card->host));
		return -ENOMEM;
	}

	err = mmc_sd_switch(card, 0, 0, 1, status);
	if (err) {
		/*
		 * We all hosts that cannot perform the command
		 * to fail more gracefully
		 */
		if (err != -EINVAL)
			goto out;

		printk(KERN_WARNING "%s: problem reading switch "
			"capabilities, performance might suffer.\n",
			mmc_hostname(card->host));
		err = 0;

		goto out;
	}

	if (status[13] & 0x02)
	{
		//printk("\n mmc_read_switch:card->sw_caps.hs_max_dtr = 50000000 ********xbw***********\n");
		card->sw_caps.hs_max_dtr = 50000000;
	}
	else
	{
		//printk("\n mmc_read_switch:card->sw_caps.hs_max_dtr = 0 ********xbw***********\n");
	}

out:
	kfree(status);

	return err;
}

/*
 * Test if the card supports high-speed mode and, if so, switch to it.
 */
static int mmc_switch_hs(struct mmc_card *card)
{
	int err;
	u8 *status;

	if (card->scr.sda_vsn < SCR_SPEC_VER_1)
	{
		//printk("\n mmc_switch_hs:card->scr.sda_vsn < SCR_SPEC_VER_1 ********xbw***********\n");
		return 0;
	}

	if (!(card->csd.cmdclass & CCC_SWITCH))
	{
		//printk("\n mmc_switch_hs:(!(card->csd.cmdclass & CCC_SWITCH)) ********xbw***********\n");
		return 0;
	}

	if (!(card->host->caps & MMC_CAP_SD_HIGHSPEED))
	{
		//printk("\n mmc_switch_hs:(!(card->host->caps & MMC_CAP_SD_HIGHSPEED)) ********xbw***********\n");
		return 0;
	}

	if (card->sw_caps.hs_max_dtr == 0)
	{
		//printk("\n mmc_switch_hs:(card->sw_caps.hs_max_dtr == 0) ********xbw***********\n");
		return 0;
	}

	err = -EIO;

	status = kmalloc(64, GFP_KERNEL);
	if (!status) {
		printk(KERN_ERR "%s: could not allocate a buffer for "
			"switch capabilities.\n", mmc_hostname(card->host));
		return -ENOMEM;
	}

	err = mmc_sd_switch(card, 1, 0, 1, status);
	if (err)
		goto out;

	if ((status[16] & 0xF) != 1) {
		printk(KERN_WARNING "%s: Problem switching card "
			"into high-speed mode!\n",
			mmc_hostname(card->host));
	} else {
		//printk("\n mmc_switch_hs:switch to high speed mode ok ********xbw***********\n");
		mmc_card_set_highspeed(card);
		mmc_set_timing(card->host, MMC_TIMING_SD_HS);
	}

out:
	kfree(status);

	return err;
}

MMC_DEV_ATTR(cid, "%08x%08x%08x%08x\n", card->raw_cid[0], card->raw_cid[1],
	card->raw_cid[2], card->raw_cid[3]);
MMC_DEV_ATTR(csd, "%08x%08x%08x%08x\n", card->raw_csd[0], card->raw_csd[1],
	card->raw_csd[2], card->raw_csd[3]);
MMC_DEV_ATTR(scr, "%08x%08x\n", card->raw_scr[0], card->raw_scr[1]);
MMC_DEV_ATTR(date, "%02d/%04d\n", card->cid.month, card->cid.year);
MMC_DEV_ATTR(fwrev, "0x%x\n", card->cid.fwrev);
MMC_DEV_ATTR(hwrev, "0x%x\n", card->cid.hwrev);
MMC_DEV_ATTR(manfid, "0x%06x\n", card->cid.manfid);
MMC_DEV_ATTR(name, "%s\n", card->cid.prod_name);
MMC_DEV_ATTR(oemid, "0x%04x\n", card->cid.oemid);
MMC_DEV_ATTR(serial, "0x%08x\n", card->cid.serial);


static struct attribute *sd_std_attrs[] = {
	&dev_attr_cid.attr,
	&dev_attr_csd.attr,
	&dev_attr_scr.attr,
	&dev_attr_date.attr,
	&dev_attr_fwrev.attr,
	&dev_attr_hwrev.attr,
	&dev_attr_manfid.attr,
	&dev_attr_name.attr,
	&dev_attr_oemid.attr,
	&dev_attr_serial.attr,
	NULL,
};

static struct attribute_group sd_std_attr_group = {
	.attrs = sd_std_attrs,
};

static struct attribute_group *sd_attr_groups[] = {
	&sd_std_attr_group,
	NULL,
};

static struct device_type sd_type = {
	.groups = sd_attr_groups,
};


/*
 * Handle the detection and initialisation of a card.
 *
 * In the case of a resume, "curcard" will contain the card
 * we're trying to reinitialise.
 */
static int mmc_sd_init_card(struct mmc_host *host, u32 ocr,
	struct mmc_card *oldcard)
{
	struct mmc_card *card;
	int err, retryTimes;
	u32 cid[4];
	unsigned int max_dtr;
    extern void mmc_power_up(struct mmc_host *host);

	BUG_ON(!host);
	WARN_ON(!host->claimed);
    retryTimes = 0;

Retry_init:
    ++retryTimes;
	/*
	 * Since we're changing the OCR value, we seem to
	 * need to tell some cards to go back to the idle
	 * state.  We wait 1ms to give cards time to
	 * respond.
	 */	 
	mmc_go_idle(host);

	/*
	 * If SD_SEND_IF_COND indicates an SD 2.0
	 * compliant card and we should set bit 30
	 * of the ocr to indicate that we can handle
	 * block-addressed SDHC cards.
	 */
	err = mmc_send_if_cond(host, ocr); 
	if (!err)
		ocr |= 1 << 30;

    //printk("%s...%s..%d *********Before mmc_send_app_op_cond--CMD41, ocr = %x **********====xbw===*******************\n",__FILE__,__FUNCTION__,__LINE__, ocr);

	err = mmc_send_app_op_cond(host, ocr, NULL);
    
    if(err && (retryTimes<5))
    {           
        //printk("%s...%s..%d ********the voltage is invlalid,so retry.  retryTimes=%d*********====xbw===*******************\n",__FILE__,__FUNCTION__,__LINE__, retryTimes);
        mmc_power_up(host);
        goto Retry_init;
    }
    
	if (err)
		goto err;

	/*
	 * For SPI, enable CRC as appropriate.
	 */
	if (mmc_host_is_spi(host)) {
		err = mmc_spi_set_crc(host, use_spi_crc);
		if (err)
			goto err;
	}

	
   //printk("%s..%s..%d ********** the card run in the SD mode, and the OCR OK. ***********====xbw===*******************\n",__FUNCTION__,__FILE__,__LINE__);

	/*
	 * Fetch CID from card.
	 */
	if (mmc_host_is_spi(host))
		err = mmc_send_cid(host, cid);       // CMD2
	else
		err = mmc_all_send_cid(host, cid);
	if (err)
		goto err;

   // printk("\n mmc_sd_init_card***** CID提取正确 ********xbw***********\n");
    
	if (oldcard) {
		if (memcmp(cid, oldcard->raw_cid, sizeof(cid)) != 0) {
			err = -ENOENT;
			goto err;
		}
		//printk("\n mmc_sd_init_card***** 当前卡是张老卡 ********xbw***********\n");
		card = oldcard;
	} else {
		/*
		 * Allocate card structure.
		 */
		card = mmc_alloc_card(host, &sd_type);
		if (IS_ERR(card)) {
			err = PTR_ERR(card);
			goto err;
		}
       // printk("\n mmc_sd_init_card***** 当前卡是张新卡，需要保存CID ********xbw***********\n");

		card->type = MMC_TYPE_SD;
		memcpy(card->raw_cid, cid, sizeof(card->raw_cid));
	}

	
    //printk("%s..%s..%d *******Before send CMD3 to get the RCA*******====xbw===*******************\n",__FUNCTION__,__FILE__,__LINE__);

	/*
	 * For native busses:  get card RCA and quit open drain mode.
	 */
	if (!mmc_host_is_spi(host)) {
		err = mmc_send_relative_addr(host, &card->rca);    // // CMD3
		if (err)
			goto free_card;
       
       //printk("\n mmc_sd_init_card***** CMD3,联系上了卡 ********xbw***********\n");
       
		mmc_set_bus_mode(host, MMC_BUSMODE_PUSHPULL);
	}

	if (!oldcard) {
		/*
		 * Fetch CSD from card.
		 */
		err = mmc_send_csd(card, card->raw_csd);     //提取CSD信息
		if (err)
			goto free_card;

		err = mmc_decode_csd(card);   //把CSD信息保存在 card->raw_csd
		if (err)
			goto free_card;

		mmc_decode_cid(card);
	}

	
    //printk("\n mmc_sd_init_card***** CID,CSD，CMD2,CMD3已经正确 ********xbw***********\n");

	/*
	 * Select card, as all following commands rely on that.
	 */
	if (!mmc_host_is_spi(host)) {
		err = mmc_select_card(card);
		if (err)
			goto free_card;
	}

	if (!oldcard) {
		/*
		 * Fetch SCR from card.
		 */
		err = mmc_app_send_scr(card, card->raw_scr);
		if (err)
			goto free_card;
	    		
        //printk("%s..%s..%d ********** before call decode_scr ***********====xbw===*******************\n",__FUNCTION__,__FILE__,__LINE__);
		err = mmc_decode_scr(card);
		if (err < 0)
			goto free_card;

		/*
		 * Fetch switch information from card.
		 */
        err = mmc_read_switch(card);
		if (err)
			goto free_card;
	}

	/*
	 * Attempt to change to high-speed (if supported)
	 */
	err = mmc_switch_hs(card);
	if (err)
		goto free_card;

	/*
	 * Compute bus speed.
	 */
	max_dtr = (unsigned int)-1;

	if (mmc_card_highspeed(card)) {
		if (max_dtr > card->sw_caps.hs_max_dtr)
			max_dtr = card->sw_caps.hs_max_dtr;
			
        //printk("\n mmc_sd_init_card***** SD  select HighSpeed ********xbw***********\n");
	} else if (max_dtr > card->csd.max_dtr) {
		max_dtr = card->csd.max_dtr;
	}

	mmc_set_clock(host, max_dtr);

    //printk("%s..%s..%d **************set ios-width, ******====xbw===*******************\n",__FUNCTION__,__FILE__,__LINE__);    
    //printk("*************set ios-width, **         host->caps= %d****====xbw===*******************\n", host->caps);    
    //printk("*************set ios-width, ** MMC_CAP_4_BIT_DATA= %d****====xbw===*******************\n", MMC_CAP_4_BIT_DATA);
    //printk("*************set ios-width, **    scr.bus_widths = %d****====xbw===*******************\n", card->scr.bus_widths);
    //printk("*************set ios-width, ** SD_SCR_BUS_WIDTH_4= %d****====xbw===*******************\n", SD_SCR_BUS_WIDTH_4);


	/*
	 * Switch to wider bus (if supported).
	 */
	if ((host->caps & MMC_CAP_4_BIT_DATA) &&
		(card->scr.bus_widths & SD_SCR_BUS_WIDTH_4)) {
		err = mmc_app_set_bus_width(card, MMC_BUS_WIDTH_4);
		if (err)
			goto free_card;

		//printk("\n mmc_sd_init_card*****  success to set 4-bits bus width.  ********xbw***********\n");
		mmc_set_bus_width(host, MMC_BUS_WIDTH_4);
	}

	/*
	 * Check if read-only switch is active.
	 */
	if (!oldcard) {
		if (!host->ops->get_ro) {
			printk(KERN_WARNING "%s: host does not "
				"support reading read-only "
				"switch. assuming write-enable.\n",
				mmc_hostname(host));
		} else {
			if (host->ops->get_ro(host))
				mmc_card_set_readonly(card);
		}
	}

	if (!oldcard)
		host->card = card;
		
    //printk("%s..%s..%d **********Init sd succeedfully===xbw===*******************\n",__FUNCTION__,__FILE__,__LINE__);
	return 0;

free_card:
	if (!oldcard)
		mmc_remove_card(card);
err:
	return err;
}

/*
 * Host is being removed. Free up the current card.
 */
static void mmc_sd_remove(struct mmc_host *host)
{
	BUG_ON(!host);
	BUG_ON(!host->card);
	
    //printk("%s..%s..%d *********Free up the current card====xbw===*******************\n",__FUNCTION__,__FILE__,__LINE__);

	mmc_remove_card(host->card);
	host->card = NULL;
}

extern int simulateRemoveSD;


/*
 * Card detection callback from host.
 */
static void mmc_sd_detect(struct mmc_host *host)
{
	int err, tempremoveSD=0;
	extern spinlock_t  sdmmc0_spinlock; 
    extern int rk28_sdmmc_IsCardPresence_global( void );

	BUG_ON(!host);
	BUG_ON(!host->card);

	mmc_claim_host(host);

	/*
	 * Just check if our card has been removed.
	 */

    err = mmc_send_status(host->card, NULL);


	/////////////For the insertion of removal quickly.
	if(!err)
	{
	    //If the status is not presence, then check IO。xbw@2009-10-27;   The status occur once at 2009-11-10	    
        if(!rk28_sdmmc_IsCardPresence_global()) 
        {               
            //printk(" *************************************************************************\n");            
            //printk("%s..%s..%d ********  status is presence,but IO is empty.******====xbw===******\n",__FUNCTION__,__FILE__,__LINE__);             
            //printk(" *************************************************************************\n"); 
            
            err= -EIO; //added by xbw; The status occur once at 2009-11-10
        }	    
	}
	else
	{
	    if(rk28_sdmmc_IsCardPresence_global())
	    {
	        err = mmc_send_status(host->card, NULL);
	    }
	}
	///////////////////////////////////////////

	mmc_release_host(host);
	
#if 0
    if (err) 
	{	        
		mmc_sd_remove(host);

		mmc_claim_host(host);
		mmc_detach_bus(host);
		mmc_release_host(host);
	}
#else
    spin_lock( &sdmmc0_spinlock);
    tempremoveSD = simulateRemoveSD;   
    spin_unlock( &sdmmc0_spinlock);

    if(1 == tempremoveSD)
    {
        //printk("%s..%s..%d ******因为执行reset, 所以模拟次remove操作，故意让err错*****====xbw===******\n",__FUNCTION__,__FILE__,__LINE__);             
        err = -EIO; //故意模拟一次remove操作.

        mmc_sd_remove(host);
		mmc_claim_host(host);
		mmc_detach_bus(host);
		mmc_release_host(host);

        spin_lock( &sdmmc0_spinlock);
        simulateRemoveSD = 0;   
        spin_unlock( &sdmmc0_spinlock);

        //printk("\n %s....%s....%d ******模拟完remove,现在开始执行add操作的work********xbw********\n", __FUNCTION__,__FILE__,__LINE__); 
        mmc_detect_change(host, 0);
    }
    else
    {
        if (err) 
    	{	        
    		mmc_sd_remove(host);

    		mmc_claim_host(host);
    		mmc_detach_bus(host);
    		mmc_release_host(host);
    	}
    }
 
#endif

}


/*for debug, added by hsl*/
void mmc_force_remove(struct mmc_host *host)
{
    //printk("%s....%s....%d   *****FORCE send up the removal message. ******xbw***\n",__FUNCTION__,__FILE__,__LINE__);
	mmc_sd_remove(host);
	mmc_claim_host(host);
	mmc_detach_bus(host);
	mmc_release_host(host);

}


#ifdef CONFIG_MMC_UNSAFE_RESUME

/*
 * Suspend callback from host.
 */
static void mmc_sd_suspend(struct mmc_host *host)
{
	BUG_ON(!host);
	BUG_ON(!host->card);

	mmc_claim_host(host);
	if (!mmc_host_is_spi(host))
		mmc_deselect_cards(host);
	host->card->state &= ~MMC_STATE_HIGHSPEED;
	mmc_release_host(host);
}

/*
 * Resume callback from host.
 *
 * This function tries to determine if the same card is still present
 * and, if so, restore all state to it.
 */
static void mmc_sd_resume(struct mmc_host *host)
{
	int err;

	BUG_ON(!host);
	BUG_ON(!host->card);

	mmc_claim_host(host);
	err = mmc_sd_init_card(host, host->ocr, host->card);
	mmc_release_host(host);

	if (err) {
		mmc_sd_remove(host);

		mmc_claim_host(host);
		mmc_detach_bus(host);
		mmc_release_host(host);
	}

}

#else

#define mmc_sd_suspend NULL
#define mmc_sd_resume NULL

#endif

static const struct mmc_bus_ops mmc_sd_ops = {
	.remove = mmc_sd_remove,
	.detect = mmc_sd_detect,
	.suspend = mmc_sd_suspend,
	.resume = mmc_sd_resume,
};

/*
 * Starting point for SD card init.
 */
int mmc_attach_sd(struct mmc_host *host, u32 ocr)
{
	int err;
	int retry,flag;

    retry = 3;
    flag = 0;
    
	BUG_ON(!host);
	WARN_ON(!host->claimed);

        //printk("\n mmc_attach_sd：************  begin *********xbw********\n");

	mmc_attach_bus(host, &mmc_sd_ops);

	/*
	 * We need to get OCR a different way for SPI.
	 */
	if (mmc_host_is_spi(host)) {
		mmc_go_idle(host);

		err = mmc_spi_read_ocr(host, 0, &ocr);
		if (err)
			goto err;
	}

	//printk("\n mmc_attach_sd：OCR读取成功   *********xbw********\n");

	/*
	 * Sanity check the voltages that the card claims to
	 * support.
	 */
	if (ocr & 0x7F) {
		printk(KERN_WARNING "%s: card claims to support voltages "
		       "below the defined range. These will be ignored.\n",
		       mmc_hostname(host));
		ocr &= ~0x7F;
	}

	if (ocr & MMC_VDD_165_195) {
		printk(KERN_WARNING "%s: SD card claims to support the "
		       "incompletely defined 'low voltage range'. This "
		       "will be ignored.\n", mmc_hostname(host));
		ocr &= ~MMC_VDD_165_195;
	}
    
    //printk("%s...%d :  *******Before the mmc_select_voltage, ocr= 0x%x,  host->ocr=  0x%x   *********xbw********\n",__FUNCTION__, __LINE__, ocr,  host->ocr);
    
	host->ocr = mmc_select_voltage(host, ocr);

    //printk("%s...%d :  ********：After the mmc_select_voltagethe value of hsot->ocr is 0x%x   *********xbw********\n",__FUNCTION__, __LINE__,  host->ocr);


   

	/*
	 * Can we support the voltage(s) of the card(s)?
	 */
	if (!host->ocr) {
		err = -EINVAL;
		goto err;
	}
	
	/*
	 * Detect and init the card.
	 */
	//err = mmc_sd_init_card(host, host->ocr, NULL);
	err = mmc_sd_init_card(host, ocr, NULL);  ////////tes
	if (err)
		goto err;


	mmc_release_host(host);

     printk("%s..%s..%d **********Now, call mmc_add_card to add a new card.====xbw===*******************\n",__FUNCTION__,__FILE__,__LINE__);

Retry_add:
	err = mmc_add_card(host->card);
	if (err)
		goto remove_card;
		
    //printk("\n mmc_attach_sd: *****the sdcard's initization is OK. Any more, the OS have registered the SD driver model********xbw***********\n");

	return 0;

remove_card:
    --retry;
    flag =1;
	mmc_remove_card(host->card);
	host->card = NULL;
	mmc_claim_host(host);
err:
	mmc_detach_bus(host);
	mmc_release_host(host);

	printk(KERN_ERR "%s: error %d whilst initialising SD card\n",
		mmc_hostname(host), err);

    //retry add the card; Added by xbw
    if((retry >= 0)&&(1==flag))
    {        
        printk("\n%s..%s..%d   ****error in add partition, so retry.  ****xbw*******\n",__FUNCTION__,__FILE__,__LINE__);   
        /* sleep some time */
        set_current_state(TASK_INTERRUPTIBLE);
        schedule_timeout(HZ/2);
        
        goto Retry_add;
    }
	return err;
}


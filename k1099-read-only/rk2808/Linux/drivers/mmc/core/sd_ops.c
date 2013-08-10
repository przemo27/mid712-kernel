/*
 *  linux/drivers/mmc/core/sd_ops.h
 *
 *  Copyright 2006-2007 Pierre Ossman
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#include <linux/types.h>
#include <linux/scatterlist.h>

#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>

#include "core.h"
#include "sd_ops.h"

static int mmc_app_cmd(struct mmc_host *host, struct mmc_card *card)
{
	int err;
	struct mmc_command cmd;

	BUG_ON(!host);
	BUG_ON(card && (card->host != host));

	cmd.opcode = MMC_APP_CMD;

	if (card) {
		cmd.arg = card->rca << 16;
		cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;
	} else {
		cmd.arg = 0;
		cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_BCR;
	}

	err = mmc_wait_for_cmd(host, &cmd, 0);
	if (err)
		return err;

	/* Check that card supported application commands */
	if (!mmc_host_is_spi(host) && !(cmd.resp[0] & R1_APP_CMD))
		return -EOPNOTSUPP;

	return 0;
}

/**
 *	mmc_wait_for_app_cmd - start an application command and wait for
 			       completion
 *	@host: MMC host to start command
 *	@card: Card to send MMC_APP_CMD to
 *	@cmd: MMC command to start
 *	@retries: maximum number of retries
 *
 *	Sends a MMC_APP_CMD, checks the card response, sends the command
 *	in the parameter and waits for it to complete. Return any error
 *	that occurred while the command was executing.  Do not attempt to
 *	parse the response.
 */
int mmc_wait_for_app_cmd(struct mmc_host *host, struct mmc_card *card,
	struct mmc_command *cmd, int retries)
{
	struct mmc_request mrq;

	int i, err;

	BUG_ON(!cmd);
	BUG_ON(retries < 0);

	err = -EIO;

	/*
	 * We have to resend MMC_APP_CMD for each attempt so
	 * we cannot use the retries field in mmc_command.
	 */
	for (i = 0;i <= retries;i++) {
		memset(&mrq, 0, sizeof(struct mmc_request));       
		err = mmc_app_cmd(host, card);
		if (err) {
			/* no point in retrying; no APP commands allowed */
			if (mmc_host_is_spi(host)) {
				if (cmd->resp[0] & R1_SPI_ILLEGAL_COMMAND)
					break;
			}
			continue;
		}

		memset(&mrq, 0, sizeof(struct mmc_request));

		memset(cmd->resp, 0, sizeof(cmd->resp));
		cmd->retries = 0;

		mrq.cmd = cmd;
		cmd->data = NULL;

		mmc_wait_for_req(host, &mrq);

		err = cmd->error;
		if (!cmd->error)
			break;

		/* no point in retrying illegal APP commands */
		if (mmc_host_is_spi(host)) {
			if (cmd->resp[0] & R1_SPI_ILLEGAL_COMMAND)
				break;
		}
	}

	return err;
}

EXPORT_SYMBOL(mmc_wait_for_app_cmd);

int mmc_app_set_bus_width(struct mmc_card *card, int width)
{
	int err;
	struct mmc_command cmd;

	BUG_ON(!card);
	BUG_ON(!card->host);

	memset(&cmd, 0, sizeof(struct mmc_command));

	cmd.opcode = SD_APP_SET_BUS_WIDTH;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;

	switch (width) {
	case MMC_BUS_WIDTH_1:
		cmd.arg = SD_BUS_WIDTH_1;
		break;
	case MMC_BUS_WIDTH_4:
		cmd.arg = SD_BUS_WIDTH_4;
		break;
	default:
		return -EINVAL;
	}

	err = mmc_wait_for_app_cmd(card->host, card, &cmd, MMC_CMD_RETRIES);
	if (err)
		return err;

	//printk("\n mmc_app_set_bus_width: wide bus command send ok ==========****xbw*************************\n");
	return 0;
}

int mmc_send_app_op_cond(struct mmc_host *host, u32 ocr, u32 *rocr)
{
	struct mmc_command cmd;
	int i, err = 0;

	BUG_ON(!host);

	memset(&cmd, 0, sizeof(struct mmc_command));

	cmd.opcode = SD_APP_OP_COND;  
	if (mmc_host_is_spi(host))
		cmd.arg = ocr & (1 << 30); /* SPI only defines one bit */
	else
		cmd.arg = ocr;

		
   // printk("%s..%d *********Before CMD41, cmd.arg = %x **********====xbw===*******************\n",__FUNCTION__,__LINE__, cmd.arg);
    
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R3 | MMC_CMD_BCR;

	for (i = 100; i; i--) 
	{
		err = mmc_wait_for_app_cmd(host, NULL, &cmd, MMC_CMD_RETRIES*10);  //MMC_CMD_RETRIES);  //测试OCR可以，适当多次, 
		if (err)
			break;

		/* if we're just probing, do a single pass */
		if (ocr == 0)
			break;

		/* otherwise wait until reset completes */
		if (mmc_host_is_spi(host)) {
			if (!(cmd.resp[0] & R1_SPI_IDLE))
				break;
		} 
		else 
		{
			if (cmd.resp[0] & MMC_CARD_BUSY)
			{ 			
                //printk("%s..%d *7777777777777777777*=== retry times = %d **********====xbw===*******************\n",__FUNCTION__,__LINE__, i);
				break;
		    }
		}

		err = -ETIMEDOUT;

		mmc_delay(10);//origin value is 10
 
	}

	if (rocr && !mmc_host_is_spi(host))
	{
		*rocr = cmd.resp[0];
		//*rocr = 0xC0FF8000; //test
    }

	return err;
}

int mmc_send_if_cond(struct mmc_host *host, u32 ocr)
{
	struct mmc_command cmd;
	int err;
	static const u8 test_pattern = 0xAA;
	u8 result_pattern;

	/*
	 * To support SD 2.0 cards, we must always invoke SD_SEND_IF_COND
	 * before SD_APP_OP_COND. This command will harmlessly fail for
	 * SD 1.0 cards.
	 */
	cmd.opcode = SD_SEND_IF_COND;
	cmd.arg = ((ocr & 0xFF8000) != 0) << 8 | test_pattern;
	cmd.flags = MMC_RSP_SPI_R7 | MMC_RSP_R7 | MMC_CMD_BCR;

	err = mmc_wait_for_cmd(host, &cmd, 0);
	if (err)
		return err;

	if (mmc_host_is_spi(host))
		result_pattern = cmd.resp[1] & 0xFF;
	else
		result_pattern = cmd.resp[0] & 0xFF;

	if (result_pattern != test_pattern)
		return -EIO;

	//printk("\n mmc_send_if_cond：鉴别出该卡是SD Ver2.0   *********xbw********\n");

	return 0;
}

int mmc_send_relative_addr(struct mmc_host *host, unsigned int *rca)
{
	int err;
	struct mmc_command cmd;

	BUG_ON(!host);
	BUG_ON(!rca);

	memset(&cmd, 0, sizeof(struct mmc_command));

	cmd.opcode = SD_SEND_RELATIVE_ADDR;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_R6 | MMC_CMD_BCR;

	err = mmc_wait_for_cmd(host, &cmd, MMC_CMD_RETRIES);
	if (err)
		return err;

	*rca = cmd.resp[0] >> 16;
        //printk("\n mmc_send_relative_addr：%x  *********xbw********\n", *rca);

	return 0;
}

int mmc_app_send_scr(struct mmc_card *card, u32 *scr)
{
	int err;
	//u8 *pDataBuf = (u8 *)scr;  //hcy
	//u8  tmp = 0;        //hcy
	//u32 i = 0;          //hcy
	struct mmc_request mrq;
	struct mmc_command cmd;
	struct mmc_data data;
	struct scatterlist sg;

	BUG_ON(!card);
	BUG_ON(!card->host);
	BUG_ON(!scr);

	/* NOTE: caller guarantees scr is heap-allocated */

	err = mmc_app_cmd(card->host, card);
	if (err)
		return err;

	memset(&mrq, 0, sizeof(struct mmc_request));
	memset(&cmd, 0, sizeof(struct mmc_command));
	memset(&data, 0, sizeof(struct mmc_data));

	mrq.cmd = &cmd;
	mrq.data = &data;

	cmd.opcode = SD_APP_SEND_SCR;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;

	data.blksz = 8;
	data.blocks = 1;
	data.flags = MMC_DATA_READ;
	data.sg = &sg;
	data.sg_len = 1;

	sg_init_one(&sg, scr, 8);

	mmc_set_data_timeout(&data, card);

	//printk("\n mmc_app_send_scr：scr addr:%x  *********xbw********\n", scr);
	mmc_wait_for_req(card->host, &mrq);

	if (cmd.error)
		return cmd.error;
	if (data.error)
	{	    
        //printk("%s..%s..%d **********when send scr find data.error*********====xbw===*******************\n",__FUNCTION__,__FILE__,__LINE__);
		return data.error;
	}
#if 0 //Delted by xbw@2009-07-07; Linux intentionally set the data to big-endian.
    else  //hcy
	{
	    //hcy我们SDMMC控制器接收到Data Packet Format为Wide Width Data的数据时，数据顺序会颠倒，
	    //hcy最高字节变成最低字节，最低字节变成最高字节，因此我们这边需要颠回来
	    for (i=0; i<(64 >> 4); i++)
	    {
		tmp         = pDataBuf[i];
		pDataBuf[i] = pDataBuf[(64 >> 3) - 1 - i];
		pDataBuf[(64 >> 3) - 1 - i] = tmp;
	    }
	    //printk("\n mmc_app_send_scr：scr[0]:%x  *********xbw********\n", scr[0]);
	    //printk("\n mmc_app_send_scr：scr[1]:%x  *********xbw********\n", scr[1]);
	}	
#else	
	scr[0] = be32_to_cpu(scr[0]);
	scr[1] = be32_to_cpu(scr[1]);
#endif
	
    //printk("%s..%s..%d *********scr[0]= %8x, scr[1]=%8x*********====xbw===*******************\n",__FUNCTION__,__FILE__,__LINE__, scr[0], scr[1]);

	return 0;
}

int mmc_sd_switch(struct mmc_card *card, int mode, int group,
	u8 value, u8 *resp)
{
	//u8 *pDataBuf = (u8 *)resp;  //hcy
	//u8  tmp = 0;        //hcy
	//u32 i = 0;          //hcy
	struct mmc_request mrq;
	struct mmc_command cmd;
	struct mmc_data data;
	struct scatterlist sg;

	BUG_ON(!card);
	BUG_ON(!card->host);

	/* NOTE: caller guarantees resp is heap-allocated */

	mode = !!mode;
	value &= 0xF;

	memset(&mrq, 0, sizeof(struct mmc_request));
	memset(&cmd, 0, sizeof(struct mmc_command));
	memset(&data, 0, sizeof(struct mmc_data));

	mrq.cmd = &cmd;
	mrq.data = &data;

	cmd.opcode = SD_SWITCH;
	cmd.arg = mode << 31 | 0x00FFFFFF;
	cmd.arg &= ~(0xF << (group * 4));
	cmd.arg |= value << (group * 4);
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;

	data.blksz = 64;
	data.blocks = 1;
	data.flags = MMC_DATA_READ;
	data.sg = &sg;
	data.sg_len = 1;

	sg_init_one(&sg, resp, 64);

	mmc_set_data_timeout(&data, card);

	//printk("\n mmc_sd_switch:buf addr:%x ==========****xbw*************************\n", resp);
	mmc_wait_for_req(card->host, &mrq);

	if (cmd.error)
		return cmd.error;
	if (data.error)
	{
		return data.error;
	}
#if 0
	else  //hcy
	{
		//SD2_SWITCH_FUNC命令收到的数据也是Data Packet Format为Wide Width Data，数据也必须颠倒过来
		for (i=0; i<(512 >> 4); i++)
		{
		    tmp         = pDataBuf[i];
		    pDataBuf[i] = pDataBuf[(512 >> 3) - 1 - i];
		    pDataBuf[(512 >> 3) - 1 - i] = tmp;
		}
		for (i=0; i<(512 >> 3); i++)
		{
			printk("\n mmc_sd_switch：switch[%x]:%x  *********xbw********\n", i, resp[i]);
		}
	}
#endif

	return 0;
}


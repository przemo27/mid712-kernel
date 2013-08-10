/*
** linux/drivers/mmc/host/rk28_sdmmc1.c
**
** Copyright (C) 2009, Rockchip Electronics Co.,Ltd.
**
** Author: Yongle Lai
** Desc  : Driver for SDMMC1 Host which works as SDIO Host. 
**  
** This software is licensed under the terms of the GNU General Public
** License version 2, as published by the Free Software Foundation, and
** may be copied, distributed, and modified under those terms.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**/
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio_func.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>

#include <asm/io.h>
#include <asm/arch/gpio.h>
#include <asm/arch/iomux.h>

#include "rk28_sdmmc1.h"

//#define ATHEROS_AR6102

/*
 * WiFi driver in if_sdio.c need this.
 * This variable indicates whether SDMMC1 driver is working.
 */
struct mmc_host *wifi_mmc_host = NULL;
EXPORT_SYMBOL(wifi_mmc_host);

#if (SORT_PACKET_SIZE == 1)
struct pkt_size_stats	pss;
#endif

/*
 * The switch for IRQ from host to CPU.
 * enable: 0 -- disable irq from host to CPU.
 *         1 -- enable  irq from host to CPU.
 */
void rk28_sdio_host_irq_switch(struct rk28_sdio_priv *priv, int enable)
{
	u32 value;
	unsigned long flags;
	
	spin_lock_irqsave(&priv->lock, flags);
	
	value = rk28_host_readl(priv, SDMMC_CTRL);
	
	if (enable == DO_ENABLE)
		value |= SDMMC_CTRL_INT_ENABLE;
	else
		value &= ~SDMMC_CTRL_INT_ENABLE;

	rk28_host_writel(priv, SDMMC_CTRL, value);
	
	spin_unlock_irqrestore(&priv->lock, flags);
}

/*
 * The switch for IRQ from SDIO card to host.
 * enable: 0 -- disable irq from card to host.
 *         1 -- enable  irq from card to host.
 */
static void	rk28_sdio_card_irq_switch(struct mmc_host *mmc, int enable)
{
	u32 value;
	unsigned long flags;
	struct rk28_sdio_priv *priv = mmc_priv(mmc);

	spin_lock_irqsave(&priv->lock, flags);
	
	value = rk28_host_readl(priv, SDMMC_INTMASK);
	if (enable == DO_ENABLE)
	{
		value |= INT_SDIO;
	}
	else
	{
		value &= ~INT_SDIO;
	}
	rk28_host_writel(priv, SDMMC_INTMASK, value);
	
	spin_unlock_irqrestore(&priv->lock, flags);
}

void rk28_sdio_dma_switch(struct rk28_sdio_priv *priv, int enable, int lock)
{
	u32 value;
	unsigned long flags = 0;
	
	if (lock == DO_LOCK)
		spin_lock_irqsave(&priv->lock, flags);
		
	value = rk28_host_readl(priv, SDMMC_CTRL);
	if (enable == DO_ENABLE)
	{
		value |= SDMMC_CTRL_DMA_ENABLE;
	}
	else
	{
		value &= ~SDMMC_CTRL_DMA_ENABLE;
	}
	rk28_host_writel(priv, SDMMC_CTRL, value);
	
	if (lock == DO_LOCK)
		spin_unlock_irqrestore(&priv->lock, flags);
}

void rk28_sdio_dma_done(s32 dmach, void *data)
{
	struct rk28_sdio_priv *priv = NULL;

	priv = (struct rk28_sdio_priv *)data;

	dma_unmap_sg(mmc_dev(priv->mmc), priv->mrq->cmd->data->sg, 
							 priv->dma_nents, priv->dma_dir);
}

#if (SORT_PACKET_SIZE == 1)
void rk28_sdio_data_req_statistics(int total_count)
{ 
		pss.total_data_req++;
		if (total_count <= 64)
			pss.data_le_64++;
		else if (total_count <= 128)
			pss.data_le_128++;
		else if (total_count <= 256)
			pss.data_le_256++;
		else if (total_count <= 512)
			pss.data_le_512++;
		else if (total_count <= 750)
			pss.data_le_750++;
		else if (total_count <= 1024)
			pss.data_le_1024++;	
		else if (total_count <= 1518)
			pss.data_le_1518++;
		else
			pss.data_gt_1518++;
		
		if ((pss.total_data_req > 0) && (pss.total_data_req % 1000 == 0))
		{
			printk("Total data request: %u\n", pss.total_data_req);
			printk("  <=64   : %7u  %2d%%\n", pss.data_le_64,
						 (pss.data_le_64 * 100) / pss.total_data_req);
			printk("  <=128  : %7u  %2d%%\n", pss.data_le_128, 
						 pss.data_le_128 * 100 / pss.total_data_req);
			printk("  <=256  : %7u  %2d%%\n", pss.data_le_256, 
						 pss.data_le_256 * 100 / pss.total_data_req);
			printk("  <=512  : %7u  %2d%%\n", pss.data_le_512, 
						 pss.data_le_512 * 100 / pss.total_data_req);
			printk("  <=750  : %7u  %2d%%\n", pss.data_le_750, 
						 pss.data_le_750 * 100 / pss.total_data_req);
			printk("  <=1024 : %7u  %2d%%\n", pss.data_le_1024, 
						 pss.data_le_1024 * 100 / pss.total_data_req);
			printk("  <=1518 : %7u  %2d%%\n", pss.data_le_1518, 
						 pss.data_le_1518 * 100 / pss.total_data_req);
			printk("  >1518  : %7u  %2d%%\n", pss.data_gt_1518, 
						 pss.data_gt_1518 * 100 / pss.total_data_req);
		}
}
#endif

static void rk28_sdio_host_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
#if (defined(CONFIG_ATHEROS)||defined(CONFIG_ATHEROS_AR6002))
	int timeout;
#endif
	int total_count = 0;
	volatile u32 value = 0;
	unsigned long flags;
	struct rk28_sdio_priv *priv = NULL;
	struct mmc_command *cmd = NULL;

	spin_lock_irqsave(&(priv->lock), flags);

	priv = mmc_priv(mmc);
	cmd = mrq->cmd;
	
#if (defined(CONFIG_ATHEROS)||defined(CONFIG_ATHEROS_AR6002))
	if (cmd->opcode == 53)
	{
		spin_unlock_irqrestore(&priv->lock, flags);
		
		for (timeout = 100; timeout > 0; timeout--)
		{
			value = rk28_host_readl(priv, SDMMC_STATUS);
			if ((value & DATA_BUSY) == 0)
				break;
			
			//Card data busy.
			mdelay(1);
		}
		if (timeout <= 0)
		{
			printk("Waiting for CARD DATA BUSY timeout.\n");
			cmd->error = -EIO;
			mmc_request_done(mmc, mrq);
			
			return;
		}
		spin_lock_irqsave(&(priv->lock), flags);
	}
	
#endif

#if 0
	if (cmd->opcode == 53)// && cmd->arg & 0x80000000)
	{
		printk("copcode=53WR blksz=%d blocks=%d func->blksz=%d\n", cmd->data->blksz, cmd->data->blocks,
		       mmc->card->sdio_func[0]->cur_blksize);
	}
#endif
	
	value = cmd->opcode | CMD_START_CMD;
	if (cmd->opcode == 53)
	{
		value |= (CMD_DATA_EXPECT);
		if (cmd->arg & 0x80000000) /* Is write command */
		{
			value |= CMD_DATA_WRITE;
		}
		value |= (CMD_RESP_EXPECT | CMD_CHECK_RCRC | CMD_WAIT_PRVDATA);
	}
	else if ((cmd->opcode == 52) ||
				   (cmd->opcode == 3) ||
				   (cmd->opcode == 7))
	{
		value |= (CMD_RESP_EXPECT | CMD_CHECK_RCRC | CMD_WAIT_PRVDATA);
	}
	else if (cmd->opcode == 0)
	{
		value |= CMD_SEND_INITIAL;
	}
	else if(cmd->opcode == 5)
	{
		value |= CMD_RESP_EXPECT;
	}
	else
	{
		//printk("Doesn't support command <%d> in SDIO driver.\n", cmd->opcode);
		cmd->error = -EIO;
		mmc_request_done(mmc, mrq);
		spin_unlock_irqrestore(&priv->lock, flags);
		return;
	}
	
	/* 
	 * TODO: We don't deal with mrq->stop for now. - Yongle Lai
	 */
	if (cmd->opcode == 53) 
	{
		if (cmd->data->blocks > 1)
			total_count = cmd->data->blksz * cmd->data->blocks;
		else
			total_count = cmd->data->blksz;
#if (SORT_PACKET_SIZE == 1)
		rk28_sdio_data_req_statistics(total_count);
#endif
		rk28_host_writel(priv, SDMMC_BLKSIZ, cmd->data->blksz);
		rk28_host_writel(priv, SDMMC_BYTCNT, total_count);
		if (total_count & 0x03)
			total_count += 4 - (total_count & 0x03);

   	cmd->data->sg->length = total_count;
   	priv->dma_dir = (value & CMD_DATA_WRITE) ? DMA_TO_DEVICE : DMA_FROM_DEVICE;
    	
   	priv->dma_nents = dma_map_sg(mmc_dev(mmc->parent), cmd->data->sg,
        													 cmd->data->sg_len, priv->dma_dir);
   	rk28_dma_setup_sg(priv->dma_chan, cmd->data->sg, cmd->data->sg_len, 
    								  (value & CMD_DATA_WRITE) ? DMA_MODE_WRITE : DMA_MODE_READ);
	
		rk28_dma_enable(priv->dma_chan);
	}
	
	priv->mmc = mmc;
	priv->mrq = mrq;

	rk28_host_writel(priv, SDMMC_CMDARG, cmd->arg);
	rk28_host_writel(priv, SDMMC_CMD, value);

	spin_unlock_irqrestore(&priv->lock, flags);
}

static irqreturn_t rk28_sdio_mci_irq(int irq, void *devid)
{
	volatile u32 ints, raw_ints;
	struct rk28_sdio_priv *priv = (struct rk28_sdio_priv *)devid;
	struct mmc_command *mmc_cmd = priv->mrq->cmd;
	
	disable_irq(irq);

	ints = rk28_host_readl(priv, SDMMC_MINTSTS);
	raw_ints = rk28_host_readl(priv, SDMMC_RINTSTS);

	if (raw_ints & ALL_ERRORS)
	{
		printk("CMD: %d <arg=%x> FAIL raw_ints: %x masked ints: %x\n", 
					 mmc_cmd->opcode, mmc_cmd->arg, raw_ints, ints);
		//if (raw_ints & (INT_RTO | INT_DRTO | INT_HTO))
		//	mmc_cmd->error = -ETIMEDOUT;
		//else
			mmc_cmd->error = -EIO;

		if ((priv->dma_chan != -1) && (mmc_cmd->opcode == 53))
			rk28_sdio_dma_done(0, (void *)priv);

		mmc_request_done(priv->mmc, priv->mrq);
		rk28_host_writel(priv, SDMMC_RINTSTS, (raw_ints & ALL_ERRORS) | INT_CD | INT_DTO);
		
		goto out;
	}
	
	if (ints & INT_CD)
	{
		mmc_cmd->resp[0] = rk28_host_readl(priv, SDMMC_RESP0);
		if (mmc_cmd->opcode != 53)
		{
			mmc_request_done(priv->mmc, priv->mrq);
		}
		rk28_host_writel(priv, SDMMC_RINTSTS, INT_CD);
	}
	
	if (ints & INT_DTO)
	{
		mmc_cmd->data->bytes_xfered = (mmc_cmd->data->blocks * mmc_cmd->data->blksz);
		mmc_request_done(priv->mmc, priv->mrq);

		rk28_host_writel(priv, SDMMC_RINTSTS, INT_DTO);
	}

out:
	if (ints & INT_SDIO)
	{
		mmc_signal_sdio_irq(priv->mmc);
		rk28_host_writel(priv, SDMMC_RINTSTS, INT_SDIO);
	}
	
	enable_irq(irq);

	return IRQ_HANDLED;
}

int rk28_sdio_set_dma(struct rk28_sdio_priv *priv, u32 new_clock)
{
    int ret = 0;
    
    spin_lock(&priv->lock);
    
    if (new_clock <= 0) /* Nothing need to be done. */
	{
		if (priv->dma_chan == RK28_DMA_SD_MMC1)
		{
			printk("Free DMA channel for SDMMC1.\n");
			priv->dma_chan = -1;

			rk28_dma_free(RK28_DMA_SD_MMC1);
			rk28_sdio_dma_switch(priv, DO_DISABLE, DO_UNLOCK);
		}
	}
    else if (priv->dma_chan == -1)
	{
		printk("Request DMA channel for SDMMC1.\n");
		ret = rk28_dma_request(RK28_DMA_SD_MMC1, rk28_sdio_dma_done, priv);
		if (ret != 0)
        {
            printk("Request DMA channel for SDMMC1 failed.\n");
        }
        else
        {
			priv->dma_chan = RK28_DMA_SD_MMC1;
			rk28_sdio_dma_switch(priv, DO_ENABLE, DO_UNLOCK);
		}
	}
	
	spin_unlock(&priv->lock);

    return ret;
}

int rk28_sdio_set_clock(struct rk28_sdio_priv *priv, u32 new_clock)
{
	int ret = 0;
	u32 value, saved_ints;

	/*
	 * Disable interrupt.
	 */
	saved_ints = rk28_host_readl(priv, SDMMC_INTMASK);
	rk28_host_writel(priv, SDMMC_INTMASK, 0);
	
	/* 
	 * Disable card clock first 
	 */
	rk28_host_writel(priv, SDMMC_CLKENA, 0);
	
	rk28_host_writel(priv, SDMMC_CMDARG, 0);
	value = CMD_START_CMD | CMD_UPDATE_CLOCK | CMD_WAIT_PRVDATA;
	rk28_host_writel(priv, SDMMC_CMD, value);
	
	mdelay(2);
	value = rk28_host_readl(priv, SDMMC_RINTSTS);
	if (value & INT_HLE)
	{
		printk("Command for disable clock got HLE error.\n");
		goto out;
	}
	
	if (new_clock <= 0) /* Nothing need to be done. */
		goto out;
	
	/* Set new clock */
	value = (AHB_CLOCK_SDMMC1 / 3 + (new_clock - 1)) / new_clock / 2;

	rk28_host_writel(priv, SDMMC_CLKDIV, value);
	rk28_host_writel(priv, SDMMC_CLKENA, 1);
	
	value = CMD_START_CMD | CMD_UPDATE_CLOCK | CMD_WAIT_PRVDATA;
	rk28_host_writel(priv, SDMMC_CMD, value);
	
	mdelay(2);
	value = rk28_host_readl(priv, SDMMC_RINTSTS);
	if (value & INT_HLE)
	{
		printk("Command for enable clock got HLE error.\n");
		goto out;
	}
	
out:
	/*
	 * Enable HLE interrupt.
	 */
	rk28_host_writel(priv, SDMMC_RINTSTS, 0xFFFFFFFF);
	rk28_host_writel(priv, SDMMC_INTMASK, saved_ints);
	
	return ret;
}

void rk28_sdio_host_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	int timeout = 10;
	unsigned int value;
	unsigned long flags;
	struct rk28_sdio_priv *priv = mmc_priv(mmc);
	
	/*
	 * Waiting SDIO controller to be IDLE.
	 */
	spin_lock_irqsave(&priv->lock, flags);

	while (timeout-- > 0)
	{
		value = rk28_host_readl(priv, SDMMC_STATUS);
		if ((value & DATA_BUSY) == 0 &&
				(value & CMD_FSM_MASK) == CMD_FSM_IDLE)
				break;
		msleep(10);
	}
	if (timeout <= 0)
	{
		printk("Waiting for SDIO controller to be IDLE timeout.\n");
		goto out;
	}

	/* Clock settings */
	if (priv->ios.clock != ios->clock)
	{
		rk28_sdio_set_clock(priv, ios->clock);
		priv->ios.clock = ios->clock;
	}
	
	/* Voltage settings */
	//if (priv->ios.vdd != ios->vdd)
	//{
		//rk28_sdio_set_voltage(preg, ios->vdd);
	//	priv->ios.vdd = ios->vdd;
	//}
	
	/* Bus mode */
	if (priv->ios.bus_mode != ios->bus_mode)
	{
		value = rk28_host_readl(priv, SDMMC_CTRL);
		if (ios->bus_mode == MMC_BUSMODE_OPENDRAIN)
		{
			value |= SDMMC_CTRL_ENABLE_OD;
		}
		else
		{
			value &= ~SDMMC_CTRL_ENABLE_OD;
		}
		rk28_host_writel(priv, SDMMC_CTRL, value);
		
		priv->ios.bus_mode = ios->bus_mode;
	}
	
	/* bus width */
	if (priv->ios.bus_width != ios->bus_width)
	{
		if (ios->bus_width == MMC_BUS_WIDTH_4)
		{
			rk28_host_writel(priv, SDMMC_CTYPE, 1);
		}
		else
		{
			rk28_host_writel(priv, SDMMC_CTYPE, 0);
		}
		priv->ios.bus_width = ios->bus_width;
	}
	
	/* Power mode */
	if (priv->ios.power_mode != ios->power_mode)
	{
		if (ios->power_mode == MMC_POWER_ON)
			rk28_host_writel(priv, SDMMC_PWREN, 1);
		else
			rk28_host_writel(priv, SDMMC_PWREN, 0);
		priv->ios.power_mode = ios->power_mode;
	}
	
	/*
	 * TODO: chip select is ignored for now.
	 */
out:
	spin_unlock_irqrestore(&priv->lock, flags);
	
	rk28_sdio_set_dma(priv, priv->ios.clock);
}

/*
 * Software reset to SDMMC FIFO module
 */
int rk28_sdio_reset_fifo(struct rk28_sdio_priv *priv)
{
	u32 value;
	int timeout = 20;
	
	/* Issue controller reset command. */
	value = rk28_host_readl(priv, SDMMC_CTRL);
	value |= 0x02;
	rk28_host_writel(priv, SDMMC_CTRL, value);
	
	/* Wait controller reset to be completed. */
	for (timeout = 200; timeout > 0; timeout--)
	{
		value = rk28_host_readl(priv, SDMMC_CTRL);
		if ((value & 0x02) == 0)
			break;
		udelay(5);
	}
	
	if (timeout <= 0)
	{
		printk("Reset SDMMC controller FIFO timeout (1000us).\n");
		return -1;
	}
	
	return 0;
}

/*
 * Software reset to SDMMC controller
 */
int rk28_sdio_reset_controller(struct rk28_sdio_priv *priv)
{
	u32 value;
	int timeout = 20;
	
	/* Issue controller reset command. */
	value = rk28_host_readl(priv, SDMMC_CTRL);
	value |= 1; 
	rk28_host_writel(priv, SDMMC_CTRL, value);
	
	/* Wait for controller reset to be completed. */
	for (timeout = 200; timeout > 0; timeout--)
	{
		value = rk28_host_readl(priv, SDMMC_CTRL);
		if ((value & 1) == 0)
			break;
		udelay(5);
	}
	
	if (timeout <= 0)
	{
		printk("Reset SDMMC controller timeout (1000us).\n");
		return -1;
	}
	
	return 0;
}

/*
 * Initiate SDMMC1 in ARM CPU
 */
void rk28_cpu_sdmmc_init(void)
{
	u32 val;
 
  /* 
   * Set clock divider for SDMMC1. 
   * The max arm frequency is 150MHz.
   *
   * TODO: we should get CPU ARM clock from SCU module.
   */
  val = __raw_readl(SCU_BASE_ADDR_VA + 0x14);
  val &= ~(0x7 << 25);
  val |= (2<<25); //divider is 3.
  __raw_writel(val, SCU_BASE_ADDR_VA + 0x14);
	mdelay(5);
	
	/*
	 * IOMUX_A_CON: PIN mutex function select.
	 * bit23: gpio1c_mmc1_sel
	 *   0: gpio1_c2/c3/c7
	 *   1: sdmmc1_cmd/data0/clkout
	 * bit22: gpio1c_mmc1d_sel
	 *   0: gpio1_c4/c5/c6
	 *   1: sdmmc1_data1/data2/data3
	 */
	val = __raw_readl(REG_FILE_BASE_ADDR_VA + 0x20);
	val |= (0x3 << 22);
	__raw_writel(val, REG_FILE_BASE_ADDR_VA + 0x20);
	
	mdelay(1);
}

/*
 * Initiate SDMMC1 host controller.
 */
int rk28_sdio_hw_init(struct rk28_sdio_priv *priv)
{ 
	u32 value;
	
  rk28_cpu_sdmmc_init();
	
	/*
	 * Initiate controller
	 */
	if (rk28_sdio_reset_controller(priv) != 0)
		return -2;
		
	if (rk28_sdio_reset_fifo(priv) != 0)
		return -3;
	
	/*
	 * Interrupt settings
	 */
	rk28_host_writel(priv, SDMMC_RINTSTS, 0xFFFFFFFF);
	/*
	 * We should mask INT_SDIO until mmc ask us to do that.
	 */
	value = INT_CD | INT_DTO | INT_HLE;
	rk28_host_writel(priv, SDMMC_INTMASK, value);
	
	/*
	 * Operation timeout
	 */
	rk28_host_writel(priv, SDMMC_TMOUT, 0xFFFFFF64);
	
	/*
	 * FIFO threshold. DMA=3 (16 bytes), RX=15 bytes, TX=16 bytes.
	 */
	value = (3 << 28) | (15 << 16) | 16;
	rk28_host_writel(priv, SDMMC_FIFOTH, value);
	
	return 0;
}

/*
 * Host controller's operational functions.
 */
static const struct mmc_host_ops rk28_sdio_host_ops =
{
	.request = rk28_sdio_host_request,
	.set_ios = rk28_sdio_host_set_ios,
	.enable_sdio_irq = rk28_sdio_card_irq_switch
};

static int __init rk28_sdio_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res;
	struct mmc_host *mci_host;
	struct rk28_sdio_priv *host_priv;
	
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENXIO;
	
	if (!request_mem_region(res->start, res->end - res->start + 1, DRIVER_NAME))
		return -EBUSY;
	
	mci_host = mmc_alloc_host(sizeof(struct rk28_sdio_priv), &pdev->dev);
	if (mci_host == NULL)
	{
		printk("Allocate MCI host fail.\n");
		ret = -ENOMEM;
		goto release_region;
	}
	
	/* Initiating mci_host */
	mci_host->ops = &rk28_sdio_host_ops;
	mci_host->f_min = 370000;		//370kHz~25MHz
	mci_host->f_max = 25000000;
	
	mci_host->ocr_avail = MMC_VDD_26_27|MMC_VDD_27_28|MMC_VDD_28_29|MMC_VDD_29_30|
									 			MMC_VDD_30_31|MMC_VDD_31_32|MMC_VDD_32_33 | MMC_VDD_33_34;
	mci_host->max_blk_size = 4095;
	mci_host->max_blk_count = mci_host->max_req_size;
	mci_host->caps |= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED;

#if (SDIO_DATA_WIDTH == SDIO_DATA_WIDTH_4)
	mci_host->caps |= MMC_CAP_4_BIT_DATA | MMC_CAP_SDIO_IRQ;
#endif

	host_priv = mmc_priv(mci_host);
	memset(host_priv, 0, sizeof(struct rk28_sdio_priv));
	host_priv->mmc = mci_host;
	host_priv->iomem_base = (void * __iomem)SDMMC1_BASE_ADDR_VA;
	host_priv->lock = SPIN_LOCK_UNLOCKED;
	host_priv->dma_chan = -1;

	ret = rk28_dma_request(RK28_DMA_SD_MMC1, rk28_sdio_dma_done, host_priv);
	if (ret != 0)
  {
    printk("Request DMA channel for SDMMC1 failed.\n");
  	goto free_host;
  }

  host_priv->dma_chan = RK28_DMA_SD_MMC1;
	rk28_sdio_dma_switch(host_priv, DO_ENABLE, DO_LOCK);
	
	if (rk28_sdio_hw_init(host_priv) != 0)
	{
		ret = -ENXIO;
		goto free_host;
	}
	
	host_priv->irq = platform_get_irq(pdev, 0);
	ret = request_irq(host_priv->irq, rk28_sdio_mci_irq, IRQF_SHARED, 
										mmc_hostname(mci_host), host_priv);
	if (ret != 0) 
	{
    printk("Request IRQ for RK28 Host fail.\n");
		ret = -ENOMEM;
		goto free_host;
	}

	platform_set_drvdata(pdev, mci_host);

	/*
	 * Max frequency of AHB is  166, for guarantee divider=6,
	 * we set the max frequency for SDMMC1 as 26.
	 */
	rockchip_scu_register( SCU_IPID_SDMMC1, SCU_MODE_FREQ, 26, NULL);
	
	mmc_add_host(mci_host);
	
	wifi_mmc_host = mci_host;
	
	rk28_sdio_host_irq_switch(host_priv, DO_ENABLE);

	return 0;
	
free_host:
	mmc_free_host(mci_host);
	
release_region:
	release_mem_region(res->start, res->end - res->start + 1);
	
	return ret;
}

static struct platform_driver rk28_sdio_driver = 
{
	.remove		= __exit_p(rk28_sdio_mci_exit),
	.probe		= rk28_sdio_probe,
	.driver		= 
	{
		.name		= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init rk28_sdio_init(void)
{
	int ret;
	
	printk("RK28 MMC SDIO HOST driver init ......\n");

#if (SORT_PACKET_SIZE == 1)
	memset(&pss, 0, sizeof(pss));
#endif

	ret = platform_driver_probe(&rk28_sdio_driver, rk28_sdio_probe);
	if (ret != 0)
	{
		printk("Couldn't find SDMMC1 resource <%s>\n", DRIVER_NAME);
	}
	
	return ret;
}

static void __exit rk28_sdio_exit(void)
{
	printk("RK28 MMC SDIO HOST driver exit ......\n");
	
  wifi_mmc_host = NULL;

  platform_driver_unregister(&rk28_sdio_driver);  
}

module_init(rk28_sdio_init);
module_exit(rk28_sdio_exit);

MODULE_AUTHOR("Yongle Lai");
MODULE_DESCRIPTION("Rockchip 28xx SDMMC1 Host driver");
MODULE_LICENSE("GPL");

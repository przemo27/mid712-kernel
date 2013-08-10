/*
** linux/drivers/mmc/host/rk28_sdmmc1.h
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
#ifndef RK28_SDMMC1_MCI_H
#define RK28_SDMMC1_MCI_H

#include <asm/dma.h>
#include <asm/arch/rk28_scu.h>
#include <asm/arch/scu.h>
#include <asm/arch/api_scu.h>
#include <linux/spinlock.h>
#include <linux/dma-mapping.h>

#define DRIVER_NAME "rk28_sdmmc1"

/*
 * SDIO data width.
 */
#define SDIO_DATA_WIDTH_1	0
#define SDIO_DATA_WIDTH_4	1
#define SDIO_DATA_WIDTH	SDIO_DATA_WIDTH_4

/*
 * Assume AHB clock for SDMMC1 is 150MHz.
 */
#define AHB_CLOCK_SDMMC1 75*1000*1000

/*
 * RK28 SDMMC FIFO depth. depth = 32 word/ 128 bytes
 */
#define SDMMC_FIFO_DEPTH    (0x20)
/*
 * Threshold for FIFO TX/RX MARK.
 * When data in FIFO >= SDMMC_FIFO_TH, RXDR is triggered.
 * When data in FIFO < SDMMC_FIFO_TH, TXDR is triggered.
 */
#define SDMMC_FIFO_TH				(0x10)

#define DO_ENABLE		1
#define DO_DISABLE	0

#define DO_LOCK			1
#define DO_UNLOCK		0

/*
 * Host register file in SDMMC.
 */
#define        SDMMC_CTRL                  0x00
#define        SDMMC_PWREN                 0x04
#define        SDMMC_CLKDIV                0x08
#define        SDMMC_CLKSRC                0x0c
#define        SDMMC_CLKENA                0x10
#define        SDMMC_TMOUT                 0x14
#define        SDMMC_CTYPE                 0x18
#define        SDMMC_BLKSIZ                0x1C
#define        SDMMC_BYTCNT                0x20
#define        SDMMC_INTMASK               0x24
#define        SDMMC_CMDARG                0x28
#define        SDMMC_CMD                   0x2C
#define        SDMMC_RESP0                 0x30
#define        SDMMC_RESP1                 0x34
#define        SDMMC_RESP2                 0x38
#define        SDMMC_RESP3                 0x3C
#define        SDMMC_MINTSTS               0x40
#define        SDMMC_RINTSTS               0x44
#define        SDMMC_STATUS                0x48
#define        SDMMC_FIFOTH                0x4c
#define        SDMMC_CDETECT               0x50
#define        SDMMC_WRTPRT                0x54
#define        SDMMC_TCBCNT                0x5C
#define        SDMMC_TBBCNT                0x60
#define        SDMMC_DEBNCE                0x64
#define        SDMMC_USRID                 0x68
#define        SDMMC_VERID                 0x6C
#define        SDMMC_HCON                  0x70
#define        SDMMC_FIFO                  0x100

#define SDMMC1_ADDR         (SDMMC1_BASE_ADDR_VA)
#define SDMMC1_FIFO_ADDR    (SDMMC1_ADDR + 0x100)

/*
 * SDMMC controller register
 */
#define SDMMC_CTRL_HOST_RESET			(1 << 0)
#define SDMMC_CTRL_FIFO_RESET			(1 << 1)
#define SDMMC_CTRL_INT_ENABLE			(1 << 4)
#define SDMMC_CTRL_DMA_ENABLE			(1 << 5)
#define SDMMC_CTRL_READ_WAIT			(1 << 6)
#define SDMMC_CTRL_SEND_IRQ_RESP	(1 << 7)
#define SDMMC_CTRL_ABORT_RD_DATA	(1 << 8)
#define SDMMC_CTRL_CARD_VOLT_A		(15 << 16)
#define SDMMC_CTRL_CARD_VOLT_B		(15 << 20)
#define SDMMC_CTRL_ENABLE_OD			(1 << 24)

/* 
 * SDMMC status Register 
 */
#define DATA_BUSY         (1 << 9)		//Card data busy
#define CMD_FSM_MASK			(0x0F << 4)	//Command FSM status mask
#define CMD_FSM_IDLE      (0x00)			//CMD FSM is IDLE
#define FIFO_FULL         (1 << 3)		//FIFO is full status
#define FIFO_EMPTY        (1 << 2)		//FIFO is empty status

/*
 * Raw Interrupt Status
 */
#define INT_SDIO					(1 << 16) 	//Interrupt From SDIO Card
#define INT_EBE						(1 << 15) 	//End-bit error (read) / write no crc (write)
#define INT_ACD						(1 << 14) 	//Auto command done
#define INT_SBE						(1 << 13) 	//Start-bit error
#define INT_HLE						(1 << 12) 	//Hard Lock Error
#define INT_FRUN					(1 << 11) 	//FIFO underrun / overrun error
#define INT_HTO						(1 << 10) 	//Data starvation-by-host timeout
#define INT_DRTO					(1 <<  9) 	//Data read timeout
#define INT_RTO						(1 <<  8) 	//Response timeout
#define INT_DCRC					(1 <<  7) 	//Data CRC error
#define INT_RCRC					(1 <<  6) 	//Response CRC error
#define INT_RXDR					(1 <<  5) 	//Receive FIFO data request
#define INT_TXDR					(1 <<  4) 	//Transmit FIFO data request
#define INT_DTO						(1 <<  3) 	//Data Transfer Over
#define INT_CD						(1 <<  2)		//Command Done
#define INT_RE						(1 <<  1) 	//Response error
#define INT_CARD_DETECT		(1 <<  0) 	//Card detect

#define ALL_ERRORS (INT_RE | INT_RCRC | INT_DCRC | INT_RTO | INT_DRTO | \
                    INT_HTO | INT_FRUN | INT_SBE | INT_EBE | INT_HLE)
                    
/*
 * CMD register bit map 
 */
#define CMD_START_CMD			(1 << 31)		//Issue the command.
#define CMD_UPDATE_CLOCK	(1 << 21)		//Sync clock from BIU to CIU.
#define CMD_SEND_INITIAL	(1 << 15)		//Send initialization sequences.
#define CMD_STOP_ABORT		(1 << 14)		//Stop/Abort the current CMD53.
#define CMD_WAIT_PRVDATA	(1 << 13)		//Wait until previous CMD53 is done.
#define CMD_AUTO_STOP			(1 << 12)		//Send stop command automatically.
#define CMD_STREAM_DATA		(1 << 11)		//Stream mode data transfer.
#define CMD_BLOCK_DATA		(0 << 11)		//Block mode data transfer.
#define CMD_DATA_READ			(0 << 10)		//Read data from SDIO card.
#define CMD_DATA_WRITE		(1 << 10)		//Write data to SDIO card.
#define CMD_DATA_EXPECT		(1 << 9)		//It's a data transfer command.
#define CMD_CHECK_RCRC		(1 << 8)		//Do check CRC on command response.
#define CMD_RESP_LONG			(1 << 7)		//Require long format response.
#define CMD_RESP_EXPECT		(1 << 6)		//Need response for this command.

struct rk28_sdio_priv
{
	void __iomem 				*iomem_base;		//SDMMC host register file's base.
	struct mmc_ios 			ios;						//Saved ios which was applied last time.
	struct mmc_request 	*mrq;						//Current request.
	struct mmc_host 		*mmc;						//Host for current request.
	int 								irq;						//IRQ for SDMMC host to CPU.	
	u32 								*pbuf;					//Data buffer for current req if available.
	int 								total_count;		//Total word(4bytes) need to be write/read.
	spinlock_t					lock;						//Lock for host operations.
	int									dma_chan;				//DMA channel: -1 = disabled
	int									dma_dir;				//DMA direction
	int									dma_nents;
};

/*
 * Wrapper functions for read/write register in Host.
 */
#define rk28_host_readl(priv, reg)	__raw_readl((priv)->iomem_base + (reg))
#define rk28_host_writel(priv, reg, val)	__raw_writel((val), (priv)->iomem_base + (reg))

/*
 * For optimization, we'd like to get 'packet size' statistics.
 */
#define SORT_PACKET_SIZE		0

#if (SORT_PACKET_SIZE == 1)

struct pkt_size_stats
{
	u32 total_data_req;
	u32 data_le_64;				/* Less or Equal 64 bytes */
	u32 data_le_128;
	u32 data_le_256;
	u32 data_le_512;
	u32 data_le_750;
	u32 data_le_1024;
	u32 data_le_1518;
	u32 data_gt_1518;			/* Greater than */
};

#endif

#endif /* RK28_SDMMC1_MCI_H */

/*
 * include/asm-arm/arch-rockchip/rk28_mci.h
 *
 * Copyright (C) 2005 Ivan Kokshaysky
 * Copyright (C) SAN People
 *
 * MultiMedia Card Interface (MCI) registers.
 * Based on AT91RM9200 datasheet revision F.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef RK28_MCI_H
#define RK28_MCI_H

#include <asm/arch/typedef.h>  
#include <linux/interrupt.h>


#if (1)  ///SDMMC_NO_PLATFORM
#define SDPAM_MAX_AHB_FREQ   166   //MHz
#else
#define SDPAM_MAX_AHB_FREQ   FREQ_HCLK_MAX
#endif


/* Specifies how often in millisecs to poll for card removal-insertion changes
 * when the timer switch is open */
#define RK28_SDMMC0_SWITCH_POLL_DELAY 3500


/***********************************************************************************************************/
//SDcontroller.h
/***********************************************************************************************************/
/* FIFO watermark */
#define RX_WMARK          (0xF)        //RX watermark level set to 15
#define TX_WMARK          (0x10)       //TX watermark level set to 16

/* SDMMC Control Register */
#define ENABLE_DMA        (1 << 5)     //Enable DMA transfer mode
#define ENABLE_INT        (1 << 4)     //Enable interrupt
//#define FIFO_RESET        (1 << 1)     //FIFO reset
#define SDC_RESET         (1 << 0)     //controller reset

/* Power Enable Register */
#define POWER_ENABLE      (1 << 0)     //Power enable

/* SDMMC Clock source Register */
#define CLK_DIV_SRC_0     (0x0)        //clock divider 0 selected
#define CLK_DIV_SRC_1     (0x1)        //clock divider 1 selected
#define CLK_DIV_SRC_2     (0x2)        //clock divider 2 selected
#define CLK_DIV_SRC_3     (0x3)        //clock divider 3 selected

/* Clock Enable Register */
#define CCLK_LOW_POWER    (1 << 16)    //Low-power control for SD/MMC card clock
#define NO_CCLK_LOW_POWER (0 << 16)    //low-power mode disabled
#define CCLK_ENABLE       (1 << 0)     //clock enable control for SD/MMC card clock
#define CCLK_DISABLE      (0 << 0)     //clock disabled

/* Card Type Register */
#define BUS_1_BIT         (0x0)
#define BUS_4_BIT         (0x1)
#define BUS_8_BIT         (0x10000)

/* interrupt mask bit */
#define SDIO_INT          (1 << 16)    //SDIO interrupt
#define EBE_INT           (1 << 15)    //End Bit Error(read)/Write no CRC
#define ACD_INT           (1 << 14)    //Auto Command Done
#define SBE_INT           (1 << 13)    //Start Bit Error
#define HLE_INT           (1 << 12)    //Hardware Locked Write Error
#define FRUN_INT          (1 << 11)    //FIFO Underrun/Overrun Error
#define HTO_INT           (1 << 10)    //Data Starvation by Host Timeout
#define DRTO_INT          (1 << 9)     //Data Read TimeOut
#define RTO_INT           (1 << 8)     //Response TimeOut
#define DCRC_INT          (1 << 7)     //Data CRC Error
#define RCRC_INT          (1 << 6)     //Response CRC Error
#define RXDR_INT          (1 << 5)     //Receive FIFO Data Request
#define TXDR_INT          (1 << 4)     //Transmit FIFO Data Request
#define DTO_INT           (1 << 3)     //Data Transfer Over
#define CD_INT            (1 << 2)     //Command Done
#define RE_INT            (1 << 1)     //Response Error
#define CDT_INT           (1 << 0)     //Card Detect


/* Detail SDMMC_CTRL  Register Description */
#define ENABLE_OD_PULLUP	(0x1 << 24)		/* External open-drain pullup */
#define CARD_VOL_B_MASK		(0xF << 20)		/* Card regulator-B voltage setting */
#define CARD_VOL_B_OFFSET	20
#define CARD_VOL_A_MASK		(0xF << 16)		/* Card regulator-A voltage setting */
#define CARD_VOL_A_OFFSET	16
#define ABORT_READ_DATA		(0x1 << 8)		/* abort_read_data */
#define SEND_IRQ_RESPONSE	(0x1 << 7)		/* Send auto IRQ response */
#define READ_WAIT		(0x1 << 6)		/* read_wait */
#define DMA_ENABLE		(0x1 << 5)		/* Enable DMA transfer mode */
#define INT_ENABLE		(0x1 << 4)		/* Global interrupt enable/disable */
#define FIFO_RESET		(0x1 << 1)		/* Reset data FIFO to reset FIFO pointers */
#define CONTROLLER_RESET	(0x1 << 0)		/* Reset SDMMC controller */


/* Command Register */
#define START_CMD         (0x1U << 31) //start command
#define UPDATE_CLOCK      (1 << 21)    //update clock register only
#define SEND_INIT         (1 << 15)    //send initialization sequence
#define STOP_CMD          (1 << 14)    //stop abort command
#define NO_WAIT_PREV      (0 << 13)    //not wait previous data transfer complete, send command at once
#define WAIT_PREV         (1 << 13)    //wait previous data transfer complete
#define AUTO_STOP         (1 << 12)    //send auto stop command at end of data transfer
#define BLOCK_TRANS       (0 << 11)    //block data transfer command
#define STREAM_TRANS      (1 << 11)    //stream data transfer command
#define READ_CARD         (0 << 10)    //read from card
#define WRITE_CARD        (1 << 10)    //write to card
#define NOCARE_RW         (0 << 10)    //not care read or write
#define NO_DATA_EXPECT    (0 << 9)     //no data transfer expected
#define DATA_EXPECT       (1 << 9)     //data transfer expected
#define NO_CHECK_R_CRC    (0 << 8)     //do not check response crc
#define CHECK_R_CRC       (1 << 8)     //check response crc
#define NOCARE_R_CRC      CHECK_R_CRC  //not care response crc
#define SHORT_R           (0 << 7)     //short response expected from card
#define LONG_R            (1 << 7)     //long response expected from card
#define NOCARE_R          SHORT_R      //not care response length
#define NO_R_EXPECT       (0 << 6)     //no response expected from card
#define R_EXPECT          (1 << 6)     //response expected from card

/* SDMMC status Register */
#define DATA_BUSY         (1 << 9)     //Card busy
#define FIFO_FULL         (1 << 3)     //FIFO is full status
#define FIFO_EMPTY        (1 << 2)     //FIFO is empty status

/* SDMMC FIFO Register */
#define SD_MSIZE_1        (0x0 << 28)  //DW_DMA_Multiple_Transaction_Size
#define SD_MSIZE_4        (0x1 << 28)
#define SD_MSIZE_8        (0x1 << 28)
#define SD_MSIZE_16       (0x3 << 28)
#define SD_MSIZE_32       (0x4 << 28)
#define SD_MSIZE_64       (0x5 << 28)
#define SD_MSIZE_128      (0x6 << 28)
#define SD_MSIZE_256      (0x7 << 28)

#define FIFO_DEPTH        (0x20)       //FIFO depth = 32 word
#define RX_WMARK_SHIFT    (16)
#define TX_WMARK_SHIFT    (0)

/* Card detect Register */
#define NO_CARD_DETECT    (1 << 0)     //Card detect

/* Write Protect Register */
#define WRITE_PROTECT     (1 << 0)     //write protect, 1 represent write protection

/* SDMMC Host Controller */
typedef enum SDMMC_PORT_Enum
{
    SDC0 = 0,
    SDC1,
    SDC_MAX
}SDMMC_PORT_E;



/* Interrupt Information */
typedef struct TagSDC_INT_INFO
{
    uint32     transLen;               //the length of data sent.
    uint32     desLen;                 //the total length of the all data.
    uint32    *pBuf;                   //the data buffer for interrupt read or write.
}SDC_INT_INFO_T;




///////////////////////////////////////////////////////////////////////////////////////////////////////////
//   all register  in the SDMMC controller
///////////////////////////////////////////////////////////////////////////////////////////////////////////
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











/****************************************************************
//hwapi_SDM.h
****************************************************************/
/* SDM return value */
/* SDM return value */
#define SDM_SUCCESS              (0)                    
#define SDM_FALSE                (0x1 << 0)             
#define SDM_CARD_NOTPRESENT      (0x1 << 1)             
#define SDM_PARAM_ERROR          (0x1 << 2)              
#define SDM_RESP_ERROR           (0x1 << 3)             
#define SDM_RESP_CRC_ERROR       (0x1 << 4)             
#define SDM_RESP_TIMEOUT         (0x1 << 5)              
#define SDM_DATA_CRC_ERROR       (0x1 << 6)              
#define SDM_DATA_READ_TIMEOUT    (0x1 << 7)              
#define SDM_END_BIT_ERROR        (0x1 << 8)              
#define SDM_START_BIT_ERROR      (0x1 << 9)              
#define SDM_BUSY_TIMEOUT         (0x1 << 10)            
#define SDM_DMA_BUSY             (0x1 << 11)             //dma busy
#define SDM_ERROR                (0x1 << 12)             //SDMMC host controller error
#define SDM_VOLTAGE_NOT_SUPPORT  (0x1 << 13)             
#define SDM_FUNC_NOT_SUPPORT     (0x1 << 14)            
#define SDM_UNKNOWABLECARD       (0x1 << 15)             
#define SDM_CARD_WRITE_PROT      (0x1 << 16)             
#define SDM_CARD_LOCKED          (0x1 << 17)             
#define SDM_CARD_CLOSED          (0x1 << 18)             



/****************************************************************
//hwapi_SDController.h
****************************************************************/
    /* SDC return value */
#define SDC_SUCCESS              SDM_SUCCESS             //
#define SDC_FALSE                SDM_FALSE               //
#define SDC_CARD_NOTPRESENT      SDM_CARD_NOTPRESENT     //
#define SDC_PARAM_ERROR          SDM_PARAM_ERROR         //
#define SDC_RESP_ERROR           SDM_RESP_ERROR          //
#define SDC_RESP_CRC_ERROR       SDM_RESP_CRC_ERROR      //
#define SDC_RESP_TIMEOUT         SDM_RESP_TIMEOUT        //
#define SDC_DATA_CRC_ERROR       SDM_DATA_CRC_ERROR      //
#define SDC_DATA_READ_TIMEOUT    SDM_DATA_READ_TIMEOUT   //
#define SDC_END_BIT_ERROR        SDM_END_BIT_ERROR       //
#define SDC_START_BIT_ERROR      SDM_START_BIT_ERROR     //
#define SDC_BUSY_TIMEOUT         SDM_BUSY_TIMEOUT        //
#define SDC_DMA_BUSY             SDM_DMA_BUSY            //dma busy
#define SDC_SDC_ERROR            SDM_ERROR               //SDMMC host controller error
    
    /* Host Bus Width */
    typedef enum HOST_BUS_WIDTH_Enum
    {
        BUS_WIDTH_INVALID = 0,
        BUS_WIDTH_1_BIT,
        BUS_WIDTH_4_BIT,
        BUS_WIDTH_8_BIT,
        BUS_WIDTH_MAX
    }HOST_BUS_WIDTH_E;




/****************************************************************
//hw_SDCommon.h
****************************************************************/
    
    /* Class 0 (basic command) */
#define SD_CMD0    (0)    // [31:0] stuff bits   
#define SD_CMD1    (1)    // [31:0] stuff, MMC
#define SD_CMD2    (2)    // [31:0] stuff
#define SD_CMD3    (3)    // [31:0] stuff
#define SD_CMD4    (4)    // [31:16] DSR [15:0] stuff
#define SD_CMD5    (5)    // [31:24] stuff [23:0] I/O OCR, SDIO
#define SD_CMD6    (6)    // ACMD6:[31:2] stuff [1:0] bus width
                              // SD2.0 CMD6
                              // [31] Mode 0:Check function, 1:Switch function
                              // [30:24] reserved
                              // [23:20] reserved for function group 6
                              // [19:16] reserved for function group 5
                              // [15:12] reserved for function group 4
                              // [11:8]  reserved for function group 3
                              // [7:4]   function group 2 for command system
                              // [3:0]   function group 1 for access mode
                              // MMC4.0 CMD6
                              // [31:26] Set to 0
                              // [25:24] Access
                              // [23:16] Index
                              // [15:8] Value
                              // [7:3] Set to 0
                              // [2:0] Cmd Set
#define SD_CMD7    (7)    // [31:16] RCA [15:0] stuff
#define SD_CMD8    (8)    // SD 2.0
                              // [31:12] reserved [11:8] supply voltage(VHS) [7:0] check pattern
                              // MMC4.0
                              // [31:0] stuff bits
#define SD_CMD9    (9)    // [31:16] RCA [15:0] stuff
#define SD_CMD10   (10)   // [31:16] RCA [15:0] stuff
#define SD_CMD11   (11)
#define SD_CMD12   (12)   // [31:0] stuff
#define SD_CMD13   (13)   // [31:16] RCA [15:0] stuff, ACMD13:[31:0] stuff
#define SD_CMD14   (14)   // MMC4.0, Bus test procedure
                              // [31:0] stuff bits
#define SD_CMD15   (15)   // [31:16] RCA [15:0] stuff
    /* Class 2 */
#define SD_CMD16   (16)   // [31:0] block length
#define SD_CMD17   (17)   // [31:0] data address
#define SD_CMD18   (18)   // [31:0] data address
#define SD_CMD19   (19)   // MMC4.0
                              // [31:0] stuff bits
#define SD_CMD20   (20)
#define SD_CMD21   (21)
#define SD_CMD22   (22)   // ACMD22:[31:0] stuff
#define SD_CMD23   (23)   // [31:16] stuff [15:0] number of blocks, MMC
                              // ACMD23:[31:23] stuff [22:0] Number of blocks
    /* Class 4 */
#define SD_CMD24   (24)   // [31:0] data address
#define SD_CMD25   (25)   // [31:0] data address
#define SD_CMD26   (26)
#define SD_CMD27   (27)   // [31:0] stuff
    /* Class 6 */
#define SD_CMD28   (28)   // [31:0] data address
#define SD_CMD29   (29)   // [31:0] data address
#define SD_CMD30   (30)   // [31:0] write protect data address
#define SD_CMD31   (31)
    /* Class 5 */
#define SD_CMD32   (32)   // [31:0] data address
#define SD_CMD33   (33)   // [31:0] data address
#define SD_CMD34   (34)
#define SD_CMD35   (35)   // [31:0] data address,MMC
#define SD_CMD36   (36)   // [31:0] data address,MMC
#define SD_CMD37   (37)
#define SD_CMD38   (38)   // [31:0] stuff
#define SD_CMD39   (39)
#define SD_CMD40   (40)
#define SD_CMD41   (41)   // ACMD41:[31:0] OCR without busy, 
                              // ACMD41:[31] reserved [30] HCS(OCR[30]) [29:24] reserved [23:0] Vdd Voltage window(OCR[23:0]), SD2.0
    /* Class 7 */
#define SD_CMD42   (42)   // [31:0] stuff, ACMD42:[31:1] stuff [0] set_cd
#define SD_CMD43   (43)
#define SD_CMD44   (44)
#define SD_CMD45   (45)
#define SD_CMD46   (46)
#define SD_CMD47   (47)
#define SD_CMD48   (48)
#define SD_CMD49   (49)
#define SD_CMD50   (50)
#define SD_CMD51   (51)   // ACMD51:[31:0] stuff
#define SD_CMD52   (52)   // [31] R/W flag [30:28] Function Number [27] RAW flag [26] stuff [25:9] Register Address [8] stuff [7:0] Write Data or stuff, SDIO
#define SD_CMD53   (53)   // [31] R/W flag [30:28] Function Number [27] Block Mode [26] OP Code [25:9] Register Address [8:0] Byte/Block Count
#define SD_CMD54   (54)
    /* Class 8 */
#define SD_CMD55   (55)   // [31:16] RCA [15:0] stuff
#define SD_CMD56   (56)   // [31:1] stuff [0] RD/WRn
#define SD_CMD57   (57)
#define SD_CMD58   (58)
#define SD_CMD59   (59)   // [31:1] stuff [0] CRC option,MMC
#define SD_CMD60   (60)
#define SD_CMD61   (61)
#define SD_CMD62   (62)
#define SD_CMD63   (63)



/****************************************************************
//hw_SDConfig.h
****************************************************************/
#define DEBOUNCE_TIME         (25)     //uint is ms, recommend 5--25ms

#define FOD_FREQ              (350)    //  in the identify stage, unit: Khz,  max is 400Khz,
                                       //  the least frequency is FREQ_HCLK_MAX/8
//#define SD_FPP_FREQ           (24000)  //   normal sd freq,  25Mhz
//#define SDHC_FPP_FREQ         (48000)  // SDHC in the highspeed. unit is khz,  max is 50Mhz.
//#define MMC_FPP_FREQ          (18000)  // MMC freq, unit is khz,   max is 20MHz
//#define MMCHS_26_FPP_FREQ     (24000)  //  highspeed mode support 26M  HS-MMC, unit is Khz, max is 26Mhz, 
//#define MMCHS_52_FPP_FREQ     (48000)  //  highspeed support 52M HS-MMC,   unit is Khz,   max is 52Mhz,
//#define SDMMC0_POWER_PIN   GPIOPortB_Pin1
//#define SDMMC0_DETECT_PIN  GPIOPortF_Pin3

#define FL_SENT_COMMAND	(1 << 0)
#define FL_SENT_STOP	(1 << 1)


#define rk28_mci_read(host, reg)	    __raw_readl((host)->baseaddr + (reg))
#define rk28_mci_write(host, reg, val)	__raw_writel((val), (host)->baseaddr + (reg))
#define rk28_cmd_start(host,cmd)	    __raw_writel((cmd), (host)->baseaddr + (SDMMC_CMD))    //send the start_CMD


/*
 * Low level type for this driver
 */
struct rk28mci_host
{
    struct mmc_host *mmc;
    struct mmc_command *cmd;
    struct mmc_request *request;

	struct resource		*res;
    struct completion   *done;    
    
    int			cmd_is_stop;

    void __iomem *baseaddr;
    int irq;    

    struct rk28_mmc_data *board;    
    unsigned int   iosclock;		//save the prev value of IOS
    unsigned int flags;
 
    /* flag for current bus settings */
    u32 bus_mode;

    /* DMA buffer used for transmitting */
    unsigned int* buffer;
    unsigned int total_length;
    
    int         dodma;   //0--no use dma; 1--use dma     
    int	        dma; //register the DMA chunne
    int         complete_dma; // 0--DMA request ok; 1-- DMA have been freed
    int         requestDmaError;  // 0--no request; 1--request success
    unsigned int		dma_dir;    
    unsigned int		dma_nents;

    /* Latest in the scatterlist that has been enabled for transfer */
    int transfer_index;

    unsigned int *pbuf;

    unsigned int    controllerNo;      //identify the host selected
    unsigned int    cmdr;           //store the command register parameter.    
    unsigned int    cmderror;           //command error
    
    
    /* SDMMC Host Controller Information */
    HOST_BUS_WIDTH_E  busWidth;       //
                                      //
                                      //
    SDC_INT_INFO_T    intInfo;        //
    uint32            cardFreq;       //

    int               lockscu;//0--no lock; 1--lock
    int               preCardState; //the state, 0--removal, 1--inserter

    /*some parameter for interrupt*/    
    spinlock_t		complete_lock;

    //for enable of disable the irq of removal or insertion
    struct work_struct	switch_work;
	struct timer_list	switch_timer;

};





 /* MMC / SD */
struct rk28_mmc_data {
	u8		    det_pin;	/* card detect IRQ */
	unsigned int wire4:1;	/* (SD) supports DAT0..DAT3 */
	u8		wp_pin;		/* (SD) writeprotect detect */
	u8		vcc_pin;	/* power switching (high == on) */
};


void 	rk28_sdmmc_Init(struct rk28mci_host *host);
int32 	rk28_sdmmc_hw_init(struct rk28mci_host *host);
void 	rk28_sdmmc_disable(struct rk28mci_host *host);

uint32  rk28_sdmmc_IsCardPresence(struct rk28mci_host *host);
uint32  rk28_sdmmc_start_command(struct rk28mci_host *host,struct mmc_command *cmd);
void    rk28_sdmmc_set_clock( struct rk28mci_host *host, u32 new_clock);
int32   rk28_sdmmc_SetHostBusWidth(struct rk28mci_host *host);

//void    rk28_sdmmc_send_request(struct mmc_host *mmc, struct mmc_request *mrq);
void    rk28_sdmmc_send_request(struct mmc_host *mmc);

void    rk28_sdmmc_tasklet(unsigned long data);
irqreturn_t rk28_sdmmc_irq(int irq ,  void *devid);
int32 rk28_mci_ChangeFreq(struct rk28mci_host *host, uint32 freqKHz);


// SD/MMC print function for debug
#define SDMMC_printk(msg...)  	//printk(msg)   // normal information
#define INTprintk(msg...)   	//printk(msg)   // interrupt information  
#define SDprintk(msg...)   	    //printk(msg)   // new print infomation


#endif


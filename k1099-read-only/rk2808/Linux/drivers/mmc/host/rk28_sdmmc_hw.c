/*
 **  linux/drivers/mmc/host/rk28_sdmmc_hw.c
 **
 **  Copyright (C),2009, Fuzhou Rockchip Electronics Co.,Ltd.
 **
 **  Auther : Xie Bangwang
 **  Desc   : driver for SD/MMC/SDIO media control interface.
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
#include <linux/mmc/card.h>

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

#include <asm/arch/hardware.h>



struct rk28mci_host *testhost; //the parameter is used for debug.

static int32 rk28_mci_ControlClock(struct rk28mci_host *host, bool enable);

extern int  sdmmc0_disable_Irq_ForRemoval;
extern spinlock_t  sdmmc0_spinlock; 
extern int resetTimes;

/*
* Debug: print the all register relative with the SD/MMC host
*/
void rk28_sdmmc_printk( void )
{
    struct rk28mci_host *host = testhost;
#if 1
    volatile unsigned int *pScu;
    volatile unsigned int *pGrf = (volatile unsigned int *)(REG_FILE_BASE_ADDR_VA);

    printk("#########################SDMMC_CTRL:%x #################################\n", rk28_mci_read(host, SDMMC_CTRL));
    printk("#########################SDMMC_PWREN:%x ################################\n", rk28_mci_read(host, SDMMC_PWREN));
    printk("#########################SDMMC_CLKDIV:%x ###############################\n", rk28_mci_read(host, SDMMC_CLKDIV));
    printk("#########################SDMMC_CLKSRC:%x ###############################\n", rk28_mci_read(host, SDMMC_CLKSRC));
    printk("#########################SDMMC_CLKENA:%x ###############################\n", rk28_mci_read(host, SDMMC_CLKENA));
    printk("#########################SDMMC_TMOUT:%x ################################\n", rk28_mci_read(host, SDMMC_TMOUT));
    printk("#########################SDMMC_CTYPE:%x ################################\n", rk28_mci_read(host, SDMMC_CTYPE));
    printk("#########################SDMMC_BLKSIZ:%x ###############################\n", rk28_mci_read(host, SDMMC_BLKSIZ));
    printk("#########################SDMMC_BYTCNT:%x ###############################\n", rk28_mci_read(host, SDMMC_BYTCNT));
    printk("#########################SDMMC_INTMASK:%x ##############################\n", rk28_mci_read(host, SDMMC_INTMASK));
    printk("#########################SDMMC_CMDARG:%x ###############################\n", rk28_mci_read(host, SDMMC_CMDARG));
    printk("#########################SDMMC_CMD:%x ##################################\n", rk28_mci_read(host, SDMMC_CMD));
    printk("#########################SDMMC_RESP0:%x ################################\n", rk28_mci_read(host, SDMMC_RESP0));
    printk("#########################SDMMC_RESP1:%x ################################\n", rk28_mci_read(host, SDMMC_RESP1));
    printk("#########################SDMMC_RESP2:%x ################################\n", rk28_mci_read(host, SDMMC_RESP2));
    printk("#########################SDMMC_RESP3:%x ################################\n", rk28_mci_read(host, SDMMC_RESP3));
    printk("#########################SDMMC_MINTSTS:%x ##############################\n", rk28_mci_read(host, SDMMC_MINTSTS));
    printk("#########################SDMMC_RINTSTS:%x ##############################\n", rk28_mci_read(host, SDMMC_RINTSTS));
    printk("#########################SDMMC_STATUS:%x ###############################\n", rk28_mci_read(host, SDMMC_STATUS));
    printk("#########################SDMMC_FIFOTH:%x ###############################\n", rk28_mci_read(host, SDMMC_FIFOTH));
    printk("#########################SDMMC_CDETECT:%x ##############################\n", rk28_mci_read(host, SDMMC_CDETECT));
    printk("#########################SDMMC_WRTPRT:%x ###############################\n", rk28_mci_read(host, SDMMC_WRTPRT));
    printk("#########################SDMMC_TCBCNT:%x ###############################\n", rk28_mci_read(host, SDMMC_TCBCNT));
    printk("#########################SDMMC_TBBCNT:%x ###############################\n", rk28_mci_read(host, SDMMC_TBBCNT));
    printk("#########################SDMMC_DEBNCE:%x ###############################\n", rk28_mci_read(host, SDMMC_DEBNCE));

    pScu = (volatile unsigned int *)(SCU_BASE_ADDR_VA + 0x14);
    printk("#########################SCU_CLKSEL0_CON:%x ##############################\n", *pScu);
    pScu = (volatile unsigned int *)(SCU_BASE_ADDR_VA + 0x1C);
    printk("#########################SCU_CLKGATE0_CON:%x ##############################\n", *pScu);
    pScu = (volatile unsigned int *)(SCU_BASE_ADDR_VA + 0x28);
    printk("#########################SCU_SOFTRST_CON:%x ##############################\n", *pScu);
    pGrf = (volatile unsigned int *)(REG_FILE_BASE_ADDR_VA + 0x20);
    printk("#########################    IOMUX_A_CON:%x ##############################\n", *pGrf);
    
    printk("#########################host->preCardState :%x ###############################\n", host->preCardState); 
    
    printk("#########################Insertion/removal  Irqstate=%d ###############################\n", sdmmc0_disable_Irq_ForRemoval);    
    printk("#########################Insertion/removal  resettimes=%d ###############################\n", resetTimes);    
    
#endif
}


/*****************************************************************************************/
//enalbe the clock for the SDMMC controller.
/*****************************************************************************************/
void SDPAM_SDCClkEnable(struct rk28mci_host *host, bool enable)
{
#if 0 // the SCU module will ensure that.
    eCLK_GATE gate;
    gate = (host->controllerNo== SDC0) ? CLK_GATE_SDMMC0 : CLK_GATE_SDMMC1;
    //printk("%s..%s..%d   **************xbw************\n",__FUNCTION__,__FILE__,__LINE__);
    if (enable)
    {
        SCUEnableClk(gate);
    }
    else
    {
        SCUDisableClk(gate);
    }
#endif

}



/*****************************************************************************************/
//set the iomux for SDMMC
/*****************************************************************************************/
bool   SDPAM_IOMUX_SetSDPort(struct rk28mci_host *host)
{

    switch (host->busWidth)
    {
        case BUS_WIDTH_1_BIT:
            rockchip_mux_api_set(GPIOH_MMC0_SEL_NAME, IOMUXA_SDMMC0_CMD_DATA0_CLKOUT);
            rockchip_mux_api_set(GPIOH_MMC0D_SEL_NAME, IOMUXA_GPIO1_D234);
            
            GPIOSetPinDirection(GPIOPortH_Pin1,GPIO_IN);
	        GPIOPullUpDown(GPIOPortH_Pin1,GPIOPullUp);	        
	        GPIOSetPinDirection(GPIOPortH_Pin2,GPIO_IN);
	        GPIOPullUpDown(GPIOPortH_Pin2,GPIOPullUp);
	        GPIOSetPinDirection(GPIOPortH_Pin3,GPIO_IN);
	        GPIOPullUpDown(GPIOPortH_Pin3,GPIOPullUp);
            break;
        case BUS_WIDTH_4_BIT:
            rockchip_mux_api_set(GPIOH_MMC0_SEL_NAME, IOMUXA_SDMMC0_CMD_DATA0_CLKOUT);
            rockchip_mux_api_set(GPIOH_MMC0D_SEL_NAME, IOMUXA_SDMMC0_DATA123);
            break;
        case BUS_WIDTH_8_BIT:
            rockchip_mux_api_set(GPIOH_MMC0_SEL_NAME, IOMUXA_SDMMC0_CMD_DATA0_CLKOUT);
            rockchip_mux_api_set(GPIOH_MMC0D_SEL_NAME, IOMUXA_SDMMC0_DATA123);
            rockchip_mux_api_set(GPIOB4_SPI0CS0_MMC0D4_NAME, IOMUXA_SDMMC0_DATA4);
            rockchip_mux_api_set(GPIOB_SPI0_MMC0_NAME, IOMUXA_SDMMC0_DATA567);
            break;
        default:
            return FALSE;
    }

    return TRUE;
}



/*****************************************************************************************/
//check the card`s presence
/*****************************************************************************************/
uint32 rk28_sdmmc_IsCardPresence(struct rk28mci_host *host)
{
    volatile uint32 value;

    value = rk28_mci_read(host,SDMMC_CDETECT);       
    if (value & NO_CARD_DETECT)
    {       
        //printk("%s....%d   *****  Card = 0   *******xbw*******\n",__FUNCTION__,__LINE__);   
        return 0;
    }
    else
    {    
        //printk("%s....%d   *****  Card = 1    *******xbw*******\n",__FUNCTION__,__LINE__);   
        return 1;
    }
}


int rk28_sdmmc_IsCardPresence_global( void )
{
    volatile uint32 value;
    
    value = __raw_readl(SDMMC0_BASE_ADDR_VA+SDMMC_CDETECT);    
    if (value & NO_CARD_DETECT)
        return 0;
    else
        return 1;
}




/***********************************************************************************************/
//analyse the input parameter, and generate the value for the SDMMC_CMD
/***********************************************************************************************/
uint32 rk28_GetCmdRegSet(uint32 cmd, uint32 cmdArg, uint32 flags, uint32 respType)
{
    uint32 cmdSet = 0;
    switch (cmd)
    {
        case SD_CMD0:    //
            cmdSet |= (SEND_INIT | NO_WAIT_PREV);
            break;

        case SD_CMD12:
            cmdSet |= (STOP_CMD | NO_WAIT_PREV);
            break;
        case SD_CMD13:
        case SD_CMD15:   //CMD15 detail need to be studyed more.
            cmdSet |= NO_WAIT_PREV;
            break;

        default:
            cmdSet |= WAIT_PREV;
            break;
    }


    /* response type */
    switch (respType)
    {
        case MMC_RSP_R1:
        case MMC_RSP_R1B:
            // case MMC_RSP_R5:  //R5,R6,R7 is same with the R1
            //case MMC_RSP_R6:
            // case R6m_TYPE:
            // case MMC_RSP_R7:
            cmdSet |= (CHECK_R_CRC | SHORT_R | R_EXPECT);
            break;
        case MMC_RSP_R3:
            //case MMC_RSP_R4:
            /* these response not contain crc7, so don't care crc error and response error */
            cmdSet |= (NO_CHECK_R_CRC | SHORT_R | R_EXPECT);
            break;
        case MMC_RSP_R2:
            cmdSet |= (CHECK_R_CRC | LONG_R | R_EXPECT);
            break;
        case MMC_RSP_NONE:
            cmdSet |= (NOCARE_R_CRC | NOCARE_R | NO_R_EXPECT);
            break;
        default:
            cmdSet |= (NOCARE_R_CRC | NOCARE_R | NO_R_EXPECT);
            break;
    }

    return (START_CMD | cmdSet | cmd);
}



/*****************************************************************************************/
//the sdmmc host open or close the internal clock
/*****************************************************************************************/
static int32 rk28_mci_ControlClock(struct rk28mci_host *host, bool enable)
{
    volatile uint32 value = 0;
    int32           timeOut = 0;

    //wait previous start to clear
    timeOut = 1000;
    while (((value = rk28_mci_read(host, SDMMC_CMD)) & START_CMD) && (timeOut > 0))
    {
        udelay(1);
        timeOut--;
    }

    if (timeOut == 0)
    {
        return SDC_SDC_ERROR;
    }

#if 0
    //做实验用。先不省电模式
    if (1)//(enable)
    {
        value = CCLK_ENABLE;
    }
    else
    {
        value =  CCLK_DISABLE;
    }
#else
    if (enable)
    {
        value = (CCLK_LOW_POWER | CCLK_ENABLE);
    }
    else
    {
        value = (CCLK_LOW_POWER | CCLK_DISABLE);
    }
#endif

    rk28_mci_write(host, SDMMC_CLKENA, value);
    rk28_cmd_start(host, (START_CMD | UPDATE_CLOCK | WAIT_PREV) );

    //wait until current start clear
    timeOut = 1000;
    while (((value = rk28_mci_read(host, SDMMC_CMD)) & START_CMD) && (timeOut > 0))
    {
        udelay(1);
        timeOut--;
    }
    if (timeOut == 0)
    {
        return SDC_SDC_ERROR;
    }

    return SDC_SUCCESS;
}


/*********************************************************************************/
//change the clock for card
/*********************************************************************************/
int32 rk28_mci_ChangeFreq(struct rk28mci_host *host, uint32 freqKHz)
{
    volatile uint32          value = 0;
    uint32          suitCclkInDiv = 0;
    int32           timeOut = 0;
    int32           ret = SDC_SUCCESS;
    int ahbfrq;

    if (freqKHz == 0)
    {
        return SDC_PARAM_ERROR;
    }

    ret = rk28_mci_ControlClock(host, FALSE);
    if (ret != SDC_SUCCESS)
    {
        return ret;
    }

    ahbfrq = __rockchip_clk_get_uint_clk( SCU_IPID_SDMMC0 )/1000;  //The unit of ahpfrq value is KHz
    
    if (freqKHz <= 400)
    {
        suitCclkInDiv = (ahbfrq/freqKHz) + ((( ahbfrq%freqKHz ) > 0) ? 1:0);     
    }
    else
    {
        suitCclkInDiv = 1;
    }


    //wait previous start to clear
    timeOut = 1000;
    while (((value = rk28_mci_read(host, SDMMC_CMD)) & START_CMD) && (timeOut > 0))
    {
        udelay(1);
        timeOut--;
    }
    if (timeOut == 0)
    {
        return SDC_SDC_ERROR;
    }

    if (suitCclkInDiv == 1)
    {
        value = 0;
    }
    else
    {
        value = (suitCclkInDiv >> 1);
    }
    
    rk28_mci_write(host, SDMMC_CLKDIV, value);   
    rk28_cmd_start(host, (START_CMD | UPDATE_CLOCK | WAIT_PREV));

    //wait until current start clear
    timeOut = 1000;
    while (((value = rk28_mci_read(host, SDMMC_CMD)) & START_CMD) && (timeOut > 0))
    {
        udelay(1);
        timeOut--;
    }
    if (timeOut == 0)
    {
        return SDC_SDC_ERROR;
    }

    return rk28_mci_ControlClock(host, TRUE);

}



int32 rk28_WaitCardBusy(struct rk28mci_host *host)
{
    volatile uint32	value = 0;
    uint32	timeout = 0;

    //wait busy
    timeout = 0;
    while ((value = rk28_mci_read(host, SDMMC_STATUS)) & DATA_BUSY)
    {
        udelay(1);
        timeout++;
        if (timeout > 250000) //max is 250ms
        {
            return SDC_BUSY_TIMEOUT;
        }
    }
    return SDC_SUCCESS;
}



void rk28_mci_dma_irq(int dma, void *devid)
{
    struct rk28mci_host *host = devid;
    host->intInfo.transLen = host->intInfo.desLen;
}



void rk28_SDC_ControlDMA(struct rk28mci_host *host, bool enable)
{
    volatile uint32 value;

    value = rk28_mci_read(host, SDMMC_CTRL);

    if (enable)
    {
        value |= ENABLE_DMA;
    }
    else
    {
        value &= ~(ENABLE_DMA);
    }

    rk28_mci_write(host, SDMMC_CTRL, value);
}

static int32 rk28_PrepareForWriteData(struct rk28mci_host *host, struct mmc_data *data)
{
    int     output;
    uint32    i = 0;
    uint32     dataLen;
    uint32     count, *pBuf = (uint32 *)host->pbuf;

    output = 0;
    dataLen = data->blocks*data->blksz;

    //SDMMC controller request the data is multiple of 4.
    count = (dataLen >> 2) + ((dataLen & 0x3) ? 1:0);

    if (count <= FIFO_DEPTH)
    {
        for (i=0; i<count; i++)
        {
            rk28_mci_write(host, SDMMC_FIFO, pBuf[i]);
        }
    }
    else
    {
        host->intInfo.desLen = count;
        host->intInfo.transLen = 0;
        host->intInfo.pBuf = (uint32 *)pBuf;

        host->complete_dma = 0;  //DMA still no request;
        host->requestDmaError = 0;//
        if ((output = rk28_dma_request(host->dma,  rk28_mci_dma_irq, host))!=0)
        {
            host->requestDmaError = 1;//   request error.
            printk("%s..%s..%d *******DMA request ERROR in *** dmaErr=%d****====xbw===*******\n",__FUNCTION__,__FILE__,__LINE__, output);
            return  SDC_FALSE;
        }

        //DMA request success
        host->complete_dma = 1;  //DMA has been request success, set 1 value.

        host->dma_dir = DMA_TO_DEVICE;
        host->dma_nents = dma_map_sg(mmc_dev(host->mmc), data->sg,
                                     data->sg_len,  host->dma_dir);

        data->sg->length = count*4;
        output = rk28_dma_setup_sg(host->dma,  data->sg, data->sg_len,  DMA_MODE_WRITE);
        if (0 != output)
        {
            printk("%s..%s..%d ***********DMA setup ERROR in WriteCmd***  dmaErr=%d******====xbw===******\n",__FUNCTION__,__FILE__,__LINE__,output);
            return SDC_FALSE;
        }

        rk28_SDC_ControlDMA(host, 1);
        rk28_dma_enable(host->dma);

    }


    return SDC_SUCCESS;
}



static int32 rk28_PrepareForReadData(struct rk28mci_host *host, struct mmc_data *data)
{
    uint32  count = 0;
    uint32  dataLen;
    int   output;

    output = 0;
    dataLen = data->blocks*data->blksz;

    //SDMMC controller request the data is multiple of 4.
    count = (dataLen >> 2) ;//+ ((dataLen & 0x3) ? 1:0);
    if (count > (RX_WMARK+1))  //datasheet error.actually, it can nont waken the interrupt when less and equal than RX_WMARK+1
    {
        host->intInfo.desLen = (dataLen >> 2);
        host->intInfo.transLen = 0;
        host->intInfo.pBuf = (uint32 *)host->pbuf;

        host->complete_dma = 0;  //DMA still no request;
        host->requestDmaError = 0;//
        if ((output = rk28_dma_request(host->dma,  rk28_mci_dma_irq, host))!=0)
        {
            host->requestDmaError = 1;//   request error.
            printk("%s..%s..%d ***********DMA request ERROR in ReadCmd***  dmaErr=%d******====xbw===**********\n",__FUNCTION__,__FILE__,__LINE__, output);
            return SDC_FALSE;
        }

        //DMA request success
        host->complete_dma = 1;  //DMA has been request success, set 1 value

        //if request DMA success, then begin to setup DMA.
        host->dma_dir = DMA_FROM_DEVICE;
        host->dma_nents = dma_map_sg(mmc_dev(host->mmc), data->sg,
                                     data->sg_len,  host->dma_dir);

        data->sg->length = count*4;
        output = rk28_dma_setup_sg(host->dma,  data->sg, data->sg_len,  DMA_MODE_READ);
        if (0 != output)
        {
            printk("%s..%s..%d ***********DMA setup ERROR in readCmd,*** dmaErr=%d******====xbw===*******************\n",__FUNCTION__,__FILE__,__LINE__,output);
            return SDC_FALSE;
        }

        rk28_SDC_ControlDMA(host, 1);
        rk28_dma_enable(host->dma);

    }

    return SDC_SUCCESS;
}



int32 rk28_ReadRemainData(struct rk28mci_host *host, uint32 originalLen, void *pDataBuf)
{
    volatile uint32  value = 0;

    uint32           i = 0;
    uint32          *pBuf = (uint32 *)pDataBuf;
    uint8           *pByteBuf = (uint8 *)pDataBuf;
    uint32           lastData = 0;

    //SDMMC controller must be multiple of 32. so if transfer 13, then actuall we should write or read 16 byte.
    uint32           count = (originalLen >> 2) + ((originalLen & 0x3) ? 1:0);

    //when use DMA, there are remain data only when datalen/4 less than  RX_WMARK+1 same as equaltion. or not multiple of 4
    if (!((value = rk28_mci_read(host, SDMMC_STATUS)) & FIFO_EMPTY))
    {
        if (count <= (RX_WMARK+1))
        {
            i = 0;
            while ((i<(originalLen >> 2))&&(!(value & FIFO_EMPTY)))
            {
                pBuf[i++] = rk28_mci_read(host, SDMMC_FIFO);
                value = rk28_mci_read(host, SDMMC_STATUS);
            }
        }

        if (count > (originalLen >> 2))
        {
            lastData = rk28_mci_read(host, SDMMC_FIFO);

            //fill the 1 to 3 byte.
            for (i=0; i<(originalLen & 0x3); i++)
            {
                pByteBuf[(originalLen & 0xFFFFFFFC) + i] = (uint8)((lastData >> (i << 3)) & 0xFF); //default little-endian
            }
        }
    }


    return SDC_SUCCESS;
}


/****************************************************************/
//init the SDMMC cotroller.
/****************************************************************/
void rk28_sdmmc_Init(struct rk28mci_host *host)
{
    /* init global variable */
    host->intInfo.transLen = 0;
    host->intInfo.desLen = 0;
    host->intInfo.pBuf = NULL;
    host->cardFreq = 0;

    host->busWidth = BUS_WIDTH_1_BIT; //use 1 bit in the initlization

    /* enable SDMMC port */
    SDPAM_IOMUX_SetSDPort(host);

    //set the detect pin to GPIO
    rockchip_mux_api_set(GPIOF3_APWM1_MMC0DETN_NAME, IOMUXA_SDMMC0_DETECT_N); //use interrupt
    
    host->preCardState = 0; /* default value, not exist */

    if (!rk28_sdmmc_IsCardPresence(host))
    {
        rk28_mci_ControlClock(host, FALSE);//if removal, close clock;  2009-07-20
        SDPAM_SDCClkEnable(host, FALSE);
        host->preCardState = 0; /* not exist */
    }
    else
        host->preCardState = 1; //default the card inserted.
}


int32 rk28_sdmmc_SetHostBusWidth(struct rk28mci_host *host)
{
    uint32  value = 0;

    switch (host->busWidth)
    {
        case BUS_WIDTH_1_BIT:
            value = BUS_1_BIT;
            break;
        case BUS_WIDTH_4_BIT:
            value = BUS_4_BIT;
            break;
        case BUS_WIDTH_8_BIT:    //Now,8bits-mode is not supported.
            value = BUS_8_BIT;
            break;
        default:
            return SDC_PARAM_ERROR;
    }
    SDPAM_IOMUX_SetSDPort(host);
    rk28_mci_write(host, SDMMC_CTYPE, value);

    return SDC_SUCCESS;
}



/****************************************************************/
//reset the SDMMC controller of the current host
/****************************************************************/
int32 SDC_ResetController(struct rk28mci_host *host)
{
    volatile uint32  value = 0;
    uint32          ahbFreq = __rockchip_clk_get_uint_clk( SCU_IPID_SDMMC0 )/1000 ;//rockchip_clk_get_ahb()/1000; SCU sure the clk after the devider.
    int32            timeOut = 0;

    /* reset SDMMC IP */
    SDPAM_SDCClkEnable(host, TRUE);
    udelay(10);
    //SDPAM_SDCReset(host); //reset the SDMMC controller from the SCU module. // the SCU module will ensure that.

    /* reset */
    rk28_mci_write(host, SDMMC_CTRL, (FIFO_RESET | SDC_RESET));

    timeOut = 1000;
    value = rk28_mci_read(host, SDMMC_CTRL);
    while (( value & (FIFO_RESET | SDC_RESET)) && (timeOut > 0))
    {
        udelay(1);
        timeOut--;
        value = rk28_mci_read(host, SDMMC_CTRL);
    }

    if (timeOut == 0)
    {
        SDPAM_SDCClkEnable(host, FALSE);
        return SDC_SDC_ERROR;
    }

    /* config FIFO */
    rk28_mci_write(host, SDMMC_FIFOTH, (SD_MSIZE_16 | (RX_WMARK << RX_WMARK_SHIFT) | (TX_WMARK << TX_WMARK_SHIFT)));
    rk28_mci_write(host, SDMMC_CTYPE, BUS_1_BIT);
    rk28_mci_write(host, SDMMC_CLKSRC, CLK_DIV_SRC_0);
    /* config debounce */
    rk28_mci_write(host, SDMMC_DEBNCE, (DEBOUNCE_TIME*ahbFreq)&0xFFFFFF);

    /* config interrupt */
    rk28_mci_write(host, SDMMC_RINTSTS, 0xFFFFFFFF);

    //value = (SBE_INT | HLE_INT | DTO_INT | CD_INT | CDT_INT );
    value = (SBE_INT | HLE_INT | DTO_INT | CDT_INT ); //

    rk28_mci_write(host, SDMMC_INTMASK, value);

    rk28_mci_write(host, SDMMC_CTRL, ENABLE_INT);

    return SDC_SUCCESS;
}


int32 rk28_sdmmc_hw_init(struct rk28mci_host *host)
{
    rk28_sdmmc_Init(host); //Modifyed by xbw@2009-11-04
    
    /* reset all card */
    SDC_ResetController(host);
    SDPAM_SDCClkEnable(host, TRUE);
    rk28_mci_ChangeFreq(host, FOD_FREQ);
    
    return SDC_SUCCESS;
}



/*
 * Disable the controller
 */
void rk28_sdmmc_disable(struct rk28mci_host *host)
{
    rk28_mci_ControlClock(host, FALSE);
    SDPAM_SDCClkEnable(host, FALSE);
}



/*
 * Before excute any command,it is best to check FIFO and Clear the FIFO register.
 */
uint32 rk28_sdmmc_clearFIFO(struct rk28mci_host *host)
{
    volatile uint32  value;
    int32   errorflag, timeOut;

    value = 0;
    errorflag = 0;
    timeOut = 0;

    //Clean FIFO
    value = rk28_mci_read(host, SDMMC_STATUS);
    if (!(value & FIFO_EMPTY))
    {
        value = rk28_mci_read(host, SDMMC_CTRL);
        value |= FIFO_RESET;
        rk28_mci_write(host, SDMMC_CTRL, value);

        timeOut = 1000;
        while (((value = rk28_mci_read(host, SDMMC_CTRL)) & (FIFO_RESET)) && (timeOut > 0))
        {
            udelay(1);

            timeOut--;
        }
        if (timeOut == 0)
        {
            host->cmd->error = -EIO;
            host->cmderror = host->cmd->error;
            errorflag=1;

        }
    }

    return errorflag;
}



void rk28_sdmmc_setup_data(struct rk28mci_host *host, struct mmc_data *data)
{
    unsigned int cmdr=0;

    if (data)
    {
        data->bytes_xfered = 0;
        host->pbuf = (u32*)sg_virt(data->sg);

        if (data->flags & MMC_DATA_STREAM)
        {
            cmdr |= STREAM_TRANS;  //the transfer_mode in the SDMMC_CMD.
        }
        else
        {
            cmdr |= BLOCK_TRANS;
        }

        if (data->flags & MMC_DATA_WRITE)
        {
            cmdr |= (WRITE_CARD | DATA_EXPECT);

            rk28_PrepareForWriteData(host, data);
        }
        else
        {
            cmdr |= (READ_CARD | DATA_EXPECT);

            rk28_PrepareForReadData(host, data);
        }

        rk28_mci_write(host, SDMMC_BLKSIZ, data->blksz);
        rk28_mci_write(host, SDMMC_BYTCNT, (data->blocks*data->blksz));
    }
    else
    {
        rk28_mci_write(host, SDMMC_BLKSIZ, 0);
        rk28_mci_write(host, SDMMC_BYTCNT, 0);
    }

    host->cmdr |= cmdr;
}


static void  rk28_sdmmc_release_resource(struct rk28mci_host *host)
{
    //printk("%s..%d ******complete_dma=%d********====xbw===******\n",__FUNCTION__,__LINE__, host->complete_dma);
    if (1==host->complete_dma)
    {
        host->complete_dma = 0;
        rk28_dma_free(host->dma);
    }
    
    //unlock SCU
    if (1==host->lockscu)
    {
        host->lockscu = 0;
        rockchip_clk_unlock_pll(SCU_IPID_SDMMC0);
    }

}

void  rk28_sdmmc0_set_frq(struct rk28mci_host *host)
{
    struct mmc_host *mmchost = container_of(host,  struct mmc_host, private);
    struct mmc_card	*card;
    struct mmc_ios *ios;
	unsigned int max_dtr;
    
    extern void mmc_set_clock(struct mmc_host *host, unsigned int hz);

    if(!mmchost)
        return;

    card = (struct mmc_card	*)mmchost->card;
    ios  = ( struct mmc_ios *)&mmchost->ios;

    if(!card && !ios)
        return;

    if(MMC_POWER_ON == ios->power_mode)
        return;

    max_dtr = (unsigned int)-1;
    
    if (mmc_card_highspeed(card)) 
    {
        if (max_dtr > card->ext_csd.hs_max_dtr)
            max_dtr = card->ext_csd.hs_max_dtr;
            
    }
    else if (max_dtr > card->csd.max_dtr) 
    {
        max_dtr = card->csd.max_dtr;
    }
    mmc_set_clock(mmchost, max_dtr);

}

/*
 * Prepare the parameter for the command before sending command. Then start command.
*/
uint32 rk28_sdmmc_start_command(struct rk28mci_host *host,struct mmc_command *cmd)
{
    unsigned int cmdr;
    struct mmc_data *data ;
    volatile uint32 value=0;
    int32           timeOut = 0;
    int  errorflag=0;
    
	unsigned long flags;
  
	spin_lock_irqsave(&host->complete_lock, flags); //disable_irq(host->irq); //not to close irq, 


    host->cmd = cmd;
    data = cmd->data;
    host->cmderror= cmd->error;

    cmdr = rk28_GetCmdRegSet(cmd->opcode, cmd->arg, cmd->flags, mmc_resp_type(cmd));
    if (host->flags & FL_SENT_STOP)
        cmdr |= STOP_CMD;

    if (!(cmdr & STOP_CMD))
    {
        errorflag = rk28_sdmmc_clearFIFO(host);
        if (0 != errorflag)
        {
            cmd->error = -ENOMEDIUM;
            host->cmderror = cmd->error;
            errorflag = 5;
            goto StarCMDError_exit;
        }
    }

    //store the cmdr
    host->cmdr = cmdr;

    if (cmdr & WAIT_PREV)
    {
        value = rk28_mci_read(host, SDMMC_STATUS);
        if (value & DATA_BUSY)
        {
            cmd->error = -ETIMEDOUT;
            host->cmderror = cmd->error;
            errorflag = 2;
            goto StarCMDError_exit;
        }
    }

    //Lock SCU
    // if (cmd->opcode != SD_CMD12)
    {
         rockchip_clk_lock_pll(SCU_IPID_SDMMC0);
         host->lockscu = 1;
         rk28_sdmmc0_set_frq(host);  //set the clock after lock scu.
    }

    rk28_sdmmc_setup_data(host, data);
    if (1==host->requestDmaError)
    {
        printk("%s..%s..%d ***********DMA request ERROR*********====xbw===*******************\n",__FUNCTION__,__FILE__,__LINE__);
        cmd->error = -ENOMEDIUM;
        host->cmderror = cmd->error;
        errorflag = 61;
        goto StarCMDError_exit;
    }

    //prepare the value for SDMMC_CMD register
    cmdr = host->cmdr;


    /*
     * Set the arguments and send the command
     */
    rk28_mci_write(host, SDMMC_CMDARG, cmd->arg);
    rk28_mci_write(host, SDMMC_CMD, cmdr);


#if 0//debug, printk CMD   

    //SDMMC_printk("Set the arguments and send the command****************** xbw  ******************\n");
    //if(data || (12==cmd->opcode))
    debug_print("Sending command = %d,  cmdr = %x , arg =  %x========xbw=====***************************=== \n", cmd->opcode, cmdr, cmd->arg);
    //SDMMC_printk("            arg =  %x\n", cmd->arg);
#endif


    //wait until current start clear
    timeOut = 2000;
    value = rk28_mci_read(host, SDMMC_CMD);
    while ((value & START_CMD) && (timeOut > 0))
    {
        udelay(10);
        timeOut--;
        value = rk28_mci_read(host, SDMMC_CMD);
    }
    if (timeOut ==0)
    {
        cmd->error = -ETIMEDOUT;
        host->cmderror = cmd->error;
        errorflag = 4;
        goto StarCMDError_exit;
    }

    /* Enable Interrupt */
    spin_unlock_irqrestore(&host->complete_lock, flags);   //enable_irq(host->irq);
    return errorflag;


StarCMDError_exit:
    printk("%s..%s..%d *******start cmd error====errorflag=%d==xbw=****************\n",__FUNCTION__,__FILE__,__LINE__, errorflag);

    /* Enable Interrupt */
    spin_unlock_irqrestore(&host->complete_lock, flags);//enable_irq(host->irq);

    return errorflag;

}


extern void mmc_force_remove(struct mmc_host *host);
#if 0
void rk28_mci_reset( struct rk28mci_host *host )
{
    struct mmc_host *mmc = host->mmc;
  
    if (mmc)
    {       
        if( mmc->card )
        {          
            mmc_force_remove( mmc );
        }
   
        if (rk28_sdmmc_IsCardPresence(host))
        {     
            printk("%s....%d   *******call reset for the SDcard .And the card is insertion ***  xbw  ****\n",__FUNCTION__,__LINE__);
            mmc_detect_change(mmc, 1);
        }
        else
        {
            printk("%s....%s....%d   *******There's not card when we want to reset the SDMMC0. ****xbw*****\n",__FUNCTION__,__FILE__,__LINE__);
        }
     }
     else
     {        
        printk("%s....%s....%d   *******the driver of SDMMC0 is not registered when we want to reset the SDMMC0. *****xbw****\n",__FUNCTION__,__FILE__,__LINE__);
     }
}
#else
int simulateRemoveSD=0;
void rk28_mci_reset( struct rk28mci_host *host )
{
    struct mmc_host *mmc = host->mmc;
  
    if (mmc)
    {  
        if (rk28_sdmmc_IsCardPresence(host))
        {     
            printk("%s....%d   *******call reset for the SDcard .And the card is insertion ***  xbw  ****\n",__FUNCTION__,__LINE__);

            //模拟remove操作           
            spin_lock( &sdmmc0_spinlock); 
            simulateRemoveSD = 1;//故意让detect失败，模拟成remove动作。            
            spin_unlock( &sdmmc0_spinlock);

            mmc_detect_change(mmc, 1);
        }
        else
        {
            printk("%s....%s....%d   *******There's not card when we want to reset the SDMMC0. ****xbw*****\n",__FUNCTION__,__FILE__,__LINE__);
        }
     }
     else
     {        
        printk("%s....%s....%d   *******the driver of SDMMC0 is not registered when we want to reset the SDMMC0. *****xbw****\n",__FUNCTION__,__FILE__,__LINE__);
     }
}

#endif

/* for debug function */
int rk28_sdmmc_reset( void )
{
    rk28_mci_reset( testhost );
    return 0x120;
}


irqreturn_t rk28_sdmmc_irq(int irq ,  void *devid)
{
    struct rk28mci_host *host = devid;
    uint32  value = 0;
    uint32  data = 0;
    int presecseFlag;
    unsigned long iflags;

    spin_lock_irqsave(&host->complete_lock, iflags);

    value = rk28_mci_read(host, SDMMC_MINTSTS);

    if (value & CDT_INT)
    {
        rk28_mci_write(host, SDMMC_RINTSTS, CDT_INT);//0xFFFFFFFF); // //  If CDT_INT, then clear all interrupt.
      
        //printk("%s....%s....%d   ******CDT_INT ========================= *******xbw*******\n",__FUNCTION__,__FILE__,__LINE__);   
        
        if ((presecseFlag = rk28_sdmmc_IsCardPresence(host)) != host->preCardState)
        {        
            printk("\n%s..%s..%d   ****Card state  prestate=%d, newState=%d, irqstate=%d****xbw*******\n",__FUNCTION__,__FILE__,__LINE__, host->preCardState, presecseFlag, sdmmc0_disable_Irq_ForRemoval);   
            host->preCardState = presecseFlag;
        
            //printk("%s....%s....%d   ******INT    call initlization  *******xbw*******\n",__FUNCTION__,__FILE__,__LINE__);           
        
           spin_lock( &sdmmc0_spinlock);
           
            resetTimes = 0;
           
            if(0 == sdmmc0_disable_Irq_ForRemoval)
            {   
                mmc_detect_change(host->mmc, 30);;
            }
            else
            {
                mod_timer(&host->switch_timer, jiffies +msecs_to_jiffies(RK28_SDMMC0_SWITCH_POLL_DELAY));
            }
            
            spin_unlock( &sdmmc0_spinlock);
            

        }

        
        spin_unlock_irqrestore(&host->complete_lock, iflags);

        return IRQ_HANDLED;

    }


    if (!host->request)
    {
        debug_print("%s....%s....%d   ******requestNULL , hardware have sickness.Please check hardware.********xbw*******\n",__FUNCTION__,__FILE__,__LINE__);
        rk28_mci_write(host, SDMMC_RINTSTS, 0xFFFFFFFF);

        spin_unlock_irqrestore(&host->complete_lock, iflags);
        return IRQ_HANDLED;
    }


    if (value & CD_INT)
    {
        //complete(host->done);  //The wait for command done have been replaced with requiry directry.
        rk28_mci_write(host, SDMMC_RINTSTS, CD_INT);
    }


    if (value & DTO_INT)
    {
        complete(host->done); 
        rk28_mci_write(host, SDMMC_RINTSTS, DTO_INT);
    }
    if (value & SBE_INT)
    {
        complete(host->done);  
        data = (HLE_INT | FRUN_INT | DTO_INT | CDT_INT);
        rk28_mci_write(host, SDMMC_INTMASK, data);
    }

    if (value & FRUN_INT)
    {
        INTprintk("%s....%s....%d   *******ImpossibleError,overrun or underrun  ***********    xbw ********\n",__FUNCTION__,__FILE__,__LINE__);
        complete(host->done); 
        rk28_mci_write(host, SDMMC_RINTSTS, FRUN_INT);
        spin_unlock_irqrestore(&host->complete_lock, iflags);
        return IRQ_HANDLED;
    }
    if (value & HLE_INT)
    {
        INTprintk("%s....%s....%d   ******* hardware locked write error  ************  xbw ***************\n",__FUNCTION__,__FILE__,__LINE__);
        complete(host->done); 
        rk28_mci_write(host, SDMMC_RINTSTS, HLE_INT);
        spin_unlock_irqrestore(&host->complete_lock, iflags);
        return IRQ_HANDLED;
    }

    spin_unlock_irqrestore(&host->complete_lock, iflags);
    return IRQ_HANDLED;
}


/*
 * After a command, store the response and recover the controller for next command.
*/
static uint32 rk28_mci_end_command(struct rk28mci_host *host, unsigned int cmdresult)
{
    struct mmc_command *cmd;
    unsigned int cmdr;
    uint32 value, errorflag;
    int32  ret = SDC_SUCCESS;

    value = 0;
    errorflag = 0;
    ret = cmdresult;
    cmd = host->cmd;
    cmdr = host->cmdr;

    value = rk28_mci_read(host, SDMMC_RINTSTS);
    rk28_mci_write(host, SDMMC_RINTSTS, 0xFFFFFFFF);

    if (ret == SDC_SUCCESS)
    {
        SDMMC_printk("rk28_mci_end_command: op success ************* xbw *****************\n");
        if ((mmc_resp_type(cmd) == MMC_RSP_R1B) || (cmd->opcode == SD_CMD12))
        {
            SDMMC_printk("rk28_mci_end_command:need wait busy after command ************* xbw *****************\n");
            ret = rk28_WaitCardBusy(host);
            if (ret != SDC_SUCCESS)
            {
                cmd->error = -EIO;
                host->cmderror = cmd->error;
                errorflag =9;
                printk("rk28_mci_end_command: wait busy timeout ************* xbw *****************\n");
                goto EndCMD_exit;
            }
        }

        //if need, get response
        if (cmd->flags & MMC_RSP_PRESENT)
        {
            if (cmd->flags & MMC_RSP_136)
            {
                cmd->resp[0] = rk28_mci_read(host, SDMMC_RESP3);
                cmd->resp[1] = rk28_mci_read(host, SDMMC_RESP2);
                cmd->resp[2] = rk28_mci_read(host, SDMMC_RESP1);
                cmd->resp[3] = rk28_mci_read(host, SDMMC_RESP0);

                SDMMC_printk("rk28_mci_end_command: long response:%x ************* xbw *****************\n", cmd->resp[0]);
                SDMMC_printk("rk28_mci_end_command:              :%x ************* xbw *****************\n", cmd->resp[1]);
                SDMMC_printk("rk28_mci_end_command:              :%x ************* xbw *****************\n", cmd->resp[2]);
                SDMMC_printk("rk28_mci_end_command:              :%x ************* xbw *****************\n", cmd->resp[3]);
            }
            else
            {
                cmd->resp[0] = rk28_mci_read(host, SDMMC_RESP0);
                SDMMC_printk("rk28_mci_end_command: short response:%x ************* xbw *****************\n", cmd->resp[0]);
            }
        }

        if (host->cardFreq > 400)
        {
            if (value & RCRC_INT)
            {
                cmd->error = -EIO;
                host->cmderror = cmd->error;

                errorflag = 10;
                SDMMC_printk("rk28_mci_end_command: freq > 400, response crc error ************* xbw *****************\n");
                goto EndCMD_exit;
            }
            if (value & RE_INT)
            {
                cmd->error = -EIO;
                host->cmderror = cmd->error;

                errorflag = 11;
                SDMMC_printk("rk28_mci_end_command: freq > 400, response error ************* xbw *****************\n");
                goto EndCMD_exit;
            }
        }
    }

    if (ret == SDC_SUCCESS)
    {
        cmd->error = 0;
        host->cmderror = cmd->error;
    }
    else
    {
        cmd->error = -EIO;
        host->cmderror = cmd->error;

        errorflag = 13;
        goto EndCMD_exit;
    }
    SDMMC_printk("rk28_mci_end_command: ****succeed to execute command.***********\n");

EndCMD_exit:
    return errorflag;

}



/*
 * Send a command
 */
void rk28_mci_send_command(struct rk28mci_host *host, struct mmc_command *cmd)
{
    unsigned int cmdr;
    struct mmc_data *data = cmd->data;
    uint32 value=0;
    int           ret = SDC_SUCCESS;
    int32           timeOut = 0;
    int  errorflag=0;
    //int32 stopCmd = 0;
    unsigned long waitTime;

    
    errorflag = rk28_sdmmc_start_command(host, cmd);
    cmdr = host->cmdr;
    if (0 != errorflag)
    {
        cmd->error = -ETIMEDOUT;
        host->cmderror = cmd->error;
        errorflag = 1;
        goto exit;
    }

    //require about CD_INT
    waitTime = 60000;
    while (!((  value = rk28_mci_read(host, SDMMC_RINTSTS)) & CD_INT) && waitTime)  /* if card not exist? */
    {
        udelay(1);
        if ( !rk28_sdmmc_IsCardPresence(host) )
        {
            cmd->error = -ETIMEDOUT;
            host->cmderror = cmd->error;
            errorflag = 2;
            goto exit;
        }
        waitTime--;
    }
    if(waitTime==0)
    {
        cmd->error = -ETIMEDOUT;
        host->cmderror = cmd->error;
        errorflag = 22;
        goto exit;
    }
    
    rk28_mci_write(host, SDMMC_RINTSTS, CD_INT);

    if (cmdr & STOP_CMD)
    {
        errorflag = rk28_sdmmc_clearFIFO(host);
        if (0 != errorflag)
        {
            SDMMC_printk("%s..%s..%d\n",__FUNCTION__,__FILE__,__LINE__);
            cmd->error = -ETIMEDOUT;
            host->cmderror = cmd->error;
            errorflag = 3;
            goto exit;
        }

    }
    SDMMC_printk("%s..%s..%d\n",__FUNCTION__,__FILE__,__LINE__);

    //check response error, or response crc error, or response timeout
    value = rk28_mci_read(host, SDMMC_RINTSTS);

    //if (value & RTO_INT)
    if ( (value & RTO_INT)
        &&(host->cmd->opcode != SD_CMD2) //MMC_ALL_SEND_CID) 
        && (host->cmd->opcode != SD_CMD1)//MMC_SEND_OP_COND)
        && (host->cmd->opcode != SD_CMD41)//SD_APP_OP_COND)
        && (host->cmd->opcode != SD_CMD55))//MMC_APP_CMD) )
    {
        SDMMC_printk("%s..%s..%d\n",__FUNCTION__,__FILE__,__LINE__);

        #if 0 
         //调试信息
        if(host->cmd->opcode==SD_CMD17 || host->cmd->opcode==SD_CMD18|| host->cmd->opcode==SD_CMD24|| host->cmd->opcode==SD_CMD25)
        {       
            printk("%s....%s....%d   ********做实验  cmd=%d*****  xbw  **************\n",__FUNCTION__,__FILE__,__LINE__, host->cmd->opcode);
            rk28_sdmmc_printk(); ////做实验
        }
        #endif
    
       // rk28_mci_write(host, SDMMC_RINTSTS, RTO_INT);
        rk28_mci_write(host, SDMMC_RINTSTS, 0xFFFFFFFF);   //if response timeout，stop tansfer data,and clear all initerrupt.
        
        value = rk28_mci_read(host, SDMMC_CTRL);/////
        if (value & ENABLE_DMA)
        {
            if (1 == host->complete_dma)
            {
                rk28_dma_disable(host->dma);
            }

            value &= ~(ENABLE_DMA);
            value |= FIFO_RESET;
            rk28_mci_write(host, SDMMC_CTRL, value);

            timeOut = 10000;//1000;
            while (((value = rk28_mci_read(host, SDMMC_CTRL)) & (FIFO_RESET)) && (timeOut > 0))
            {
                SDMMC_printk("%s..%s..%d\n",__FUNCTION__,__FILE__,__LINE__);
                udelay(1);//(10);
                timeOut--;
            }
            if (timeOut == 0)
            {
                host->cmd->error = -ENOMEM;
                host->cmderror = host->cmd->error;

                SDprintk("%s....%s....%d   **********fail_task ***********  xbw  **************\n",__FUNCTION__,__FILE__,__LINE__);
                errorflag = 44;
                goto exit;
            }
        }


        cmd->error = -EIO;
        host->cmderror = cmd->error;
        errorflag = 6;
/*
        if(rk28_sdmmc_IsCardPresence(host))
        {
            printk("%s..%s..%d   ********cardstate=1 , response timeout, cmd=%d, cmdr=%x, cardIrqstate=%d*********  xbw  *********\n",__FUNCTION__,__FILE__,__LINE__, cmd->opcode, host->cmdr, sdmmc0_disable_Irq_ForRemoval);
            rk28_sdmmc_printk();
            
            local_irq_disable();
            while(1); //测试, 出错的时候;直接停住            
        }*/

        goto exit;
    }

    SDMMC_printk("%s..%s..%d\n",__FUNCTION__,__FILE__,__LINE__);
    if (cmdr & DATA_EXPECT) 
    {
        //wait for DTO_INT
        waitTime = wait_for_completion_timeout(host->done, 800/*400*/); //if large than N*jiffies, to exit the function.
        if (waitTime == 0 )
        {
            cmd->error = -EIO;
            host->cmderror = cmd->error;
            SDMMC_printk("%s....%s....%d   **********fail_task ***********  xbw  **************\n",__FUNCTION__,__FILE__,__LINE__);

            errorflag = 12;
            goto exit;
        }

        if (cmdr & WRITE_CARD)
        {
            //deal with DMA
            value = rk28_mci_read(host, SDMMC_CTRL);
            if (value & ENABLE_DMA)
            {
                if (1 == host->complete_dma)
                {
                    rk28_dma_disable(host->dma);
                }

                value &= ~(ENABLE_DMA);
                rk28_mci_write(host, SDMMC_CTRL, value);
            }


            value = rk28_mci_read(host, SDMMC_RINTSTS);
            if (value & (DCRC_INT |EBE_INT))
            {
                ret = SDC_DATA_CRC_ERROR;
                
                SDMMC_printk("%s....%s....%d   **********fail_task ***********  xbw  **************\n",__FUNCTION__,__FILE__,__LINE__);
            }
            else
            {
                ret =  rk28_WaitCardBusy(host);
                if (ret != SDC_SUCCESS)
                {
                    host->cmd->error = -ETIMEDOUT ;
                    host->cmderror = host->cmd->error;
                    errorflag = 14;
                    SDMMC_printk("%s....%s....%d   **********fail_task ***********  xbw  **************\n",__FUNCTION__,__FILE__,__LINE__);
                    goto exit;
                }
                host->total_length = (host->request->cmd->data->blocks*host->request->cmd->data->blksz);
                host->transfer_index = host->cmd->data->sg_len;
                host->request->cmd->data->bytes_xfered = (host->request->cmd->data->blocks* host->request->cmd->data->blksz);
                ret = SDC_SUCCESS;
            }

        }
        else
        {
            //deal with DMA
            value = rk28_mci_read(host, SDMMC_CTRL);
            if (value & ENABLE_DMA)
            {
                if (1 == host->complete_dma)
                {
                    rk28_dma_disable(host->dma);
                }

                value &= ~(ENABLE_DMA);
                rk28_mci_write(host, SDMMC_CTRL, value);
            }


            value = rk28_mci_read(host, SDMMC_RINTSTS);
            if (value & SBE_INT)
            {
            #if 0
                stopCmd = rk28_GetCmdRegSet(SD_CMD12, 0, FALSE, MMC_RSP_R1);
                rk28_cmd_start(host, stopCmd);

                timeOut = 1000;
                while (((value = rk28_mci_read(host, SDMMC_CMD)) & START_CMD) && (timeOut > 0))
                {
                    udelay(1);
                    timeOut--;
                }
                if (timeOut == 0)
                {
                    cmd->error = -EIO;
                    host->cmderror = cmd->error;

                    errorflag = 8;
                    SDMMC_printk("rk28_mci_send_command: wait command done timeout ************* xbw *****************\n");
                    goto exit;
                }
                SDMMC_printk("rk28_mci_send_command:entry wait data trans end ************* xbw *****************\n");

                //require about CD_INT
                waitTime = 60000;
                while ((((value = rk28_mci_read(host, SDMMC_RINTSTS)) & CD_INT) != CD_INT) &&  waitTime)
                {
                    udelay(1);
                    if ( !rk28_sdmmc_IsCardPresence(host) )
                    {
                        cmd->error = -EIO;
                        host->cmderror = cmd->error;
                        errorflag = 81;
                        
                        SDMMC_printk("%s....%s....%d   **********fail_task ***********  xbw  **************\n",__FUNCTION__,__FILE__,__LINE__);
                        goto exit;
                    }
                }

		 if(waitTime==0)
		    {
		        cmd->error = -EIO;
		        host->cmderror = cmd->error;
		        errorflag = 82;
		        
                SDMMC_printk("%s....%s....%d   **********fail_task ***********  xbw  **************\n",__FUNCTION__,__FILE__,__LINE__);
		        goto exit;
		    }	
                rk28_mci_write(host, SDMMC_RINTSTS, CD_INT );


                //wait for DTO_INT
                waitTime = wait_for_completion_timeout(host->done, 800/*400*/); //if large than 20*jiffies, to exit the function.
                if (waitTime==0)
                {
                    cmd->error = -EIO;
                    host->cmderror = cmd->error;

                    errorflag = 11;
                    
                    SDMMC_printk("%s....%s....%d   **********fail_task ***********  xbw  **************\n",__FUNCTION__,__FILE__,__LINE__);
                    goto exit;
                }

                rk28_mci_write(host, SDMMC_RINTSTS, SBE_INT);
                //value = (SBE_INT | HLE_INT | FRUN_INT | DTO_INT | CD_INT |CDT_INT);
                value = (SBE_INT | HLE_INT | FRUN_INT | DTO_INT |CDT_INT);

                rk28_mci_write(host, SDMMC_INTMASK, value);
             #endif
             
                ret = SDC_START_BIT_ERROR;
                
                SDMMC_printk("%s....%s....%d   **********fail_task ***********  xbw  **************\n",__FUNCTION__,__FILE__,__LINE__);

            }
            else if ((value & EBE_INT) && (cmd->opcode!= SD_CMD14))
            {
                host->cmd->error = -EIO ;
                host->cmderror = host->cmd->error;
                errorflag = 18;
                ret = SDC_END_BIT_ERROR;
                
                SDMMC_printk("%s....%s....%d   **********fail_task ***********  xbw  **************\n",__FUNCTION__,__FILE__,__LINE__);
                goto exit;
            }
            else if (value & DRTO_INT)
            {
                host->cmd->error = -EIO ;
                host->cmderror = host->cmd->error;
                errorflag = 19;
                ret = SDC_DATA_READ_TIMEOUT;
                
                SDMMC_printk("%s....%s....%d   **********fail_task ***********  xbw  **************\n",__FUNCTION__,__FILE__,__LINE__);
                goto exit;
            }
            else if (value & DCRC_INT)
            {
                host->cmd->error = -EIO ;
                host->cmderror = host->cmd->error;
                errorflag = 20;
                ret = SDC_DATA_CRC_ERROR;
                
                SDMMC_printk("%s....%s....%d   **********fail_task ***********  xbw  **************\n",__FUNCTION__,__FILE__,__LINE__);
                goto exit;
            }
            else
            {
                ret = rk28_ReadRemainData(host, (data->blocks*data->blksz), host->pbuf);
                if (ret == SDC_SUCCESS)
                {
                    host->transfer_index = host->cmd->data->sg_len;
                    data->bytes_xfered = (data->blocks*data->blksz);

                }
            }
        }

    }
   
  
    errorflag = rk28_mci_end_command(host, ret);
    if ( !errorflag )
        return ;
    errorflag += 100;
exit:
    if(SD_CMD5 != cmd->opcode)
    {
        printk("%s,errorflag=%d,cmd=%d,cmdr=%x,ret=0x%x\n",__FUNCTION__, errorflag, cmd->opcode,
               host->cmdr , ret );
    }

    return;

}

/*
 * Process the next step in the request
 */
void rk28_sdmmc_send_request(struct mmc_host *mmc)
{
    struct rk28mci_host *host = mmc_priv(mmc);
    DECLARE_COMPLETION_ONSTACK(done);
#if 1
    //printk debug infomation
    if ((!host->request) || (!host->request->cmd))
    {
        debug_print("%s..%d*******host->request=Null, Hardware have sickness*************", __FILE__,__LINE__ );
        return;
    }
#endif
    host->flags = 0;
    host->done = &done;

    if (host->request->cmd)
    {
        host->flags |= FL_SENT_COMMAND;
        rk28_mci_send_command(host, host->request->cmd);
    }

    //if ((!host->cmderror) && (host->request) && (host->request->stop) )
    if ((host->request) && (host->request->stop))
    {
        host->flags |= FL_SENT_STOP;
        rk28_mci_send_command(host, host->request->stop);
    }

    host->request = NULL;
    rk28_sdmmc_release_resource(host); //release the DMA and SCU  if it had been requested succeedly.

}









/*
 * rockchip-iis.c  --  ALSA SoC ROCKCHIP IIS Audio Layer Platform driver
 *
 * Driver for rockchip iis audio
 *  Copyright (C) 2009 lhh
 *
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/clk.h>

#include <asm/dma.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/hw_common.h>
#include <asm/arch/iomux.h>
#include <asm/arch/scu.h> 

#include "rk28-pcm.h"
#include "rk28-iis.h"


#if 0
#define DBG(x...) printk(KERN_INFO x)
#else
#define DBG(x...) do { } while (0)
#endif

#define pheadi2s  ((pI2S_REG)(i2s->regs))

struct rockchip_i2s_info {
	struct device	*dev;
	void __iomem	*regs;
	struct clk	*iis_clk;
	struct clk	*iis_pclk;

	u32		 suspend_iismod;
	u32		 suspend_iiscon;
	u32		 suspend_iispsr;
};

static struct rockchip_i2s_info rockchip_i2s;

static struct rockchip_dma_client rockchip_dma_client_out = {
	.name		= "I2S PCM Stereo out"
};

static struct rockchip_dma_client rockchip_dma_client_in = {
	.name		= "I2S PCM Stereo in"
};

static struct rockchip_pcm_dma_params rockchip_i2s_pcm_stereo_out = {
	.client		= &rockchip_dma_client_out,
	.channel	= RK28_DMA_I2S_TXD, ///0,  //DMACH_I2S_OUT,
	.dma_addr	= I2S_BASE_ADDR + I2S_TXR_BUFF,
	.dma_size	= 4,
};

static struct rockchip_pcm_dma_params rockchip_i2s_pcm_stereo_in = {
	.client		= &rockchip_dma_client_in,
	.channel	= RK28_DMA_I2S_RXD,  ///1,  //DMACH_I2S_IN,
	.dma_addr	= I2S_BASE_ADDR + I2S_RXR_BUFF,
	.dma_size	= 4,
};




/* 
 *Turn on or off the transmission path. 
 */
static void rockchip_snd_txctrl(int on)
{
    struct rockchip_i2s_info *i2s = &rockchip_i2s;	  
	  u32 opr,fifosts;
    
    DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);
    
    opr = readl(&(pheadi2s->I2S_OPR));
    fifosts = readl(&(pheadi2s->I2S_FIFOSTS));
    fifosts = (fifosts & (~(0x0f<<16))) | TX_HALF_FULL | RX_HALF_FULL;
    writel(fifosts, &(pheadi2s->I2S_FIFOSTS));
    if (on) 
    {
        opr = (opr & (~(RESET_RX | I2S_DMA_REQ2_DISABLE | TX_START | RX_START))) | (RESET_TX | I2S_DMA_REQ1_DISABLE);
        writel(opr, &(pheadi2s->I2S_OPR));
        udelay(5);
        opr = (opr & (~(I2S_DMA_REQ1_DISABLE | I2S_DMA_REQ1_RX_ENABLE | RX_START))) | I2S_DMA_REQ1_ENABLE | I2S_DMA_REQ1_TX_ENABLE | TX_START;
        writel(opr, &(pheadi2s->I2S_OPR));
    }
    else
    {  
        opr = (opr & (~TX_START)) | I2S_DMA_REQ1_DISABLE;
        writel(opr, &(pheadi2s->I2S_OPR));
    }
}
static void rockchip_snd_rxctrl(int on)
{
    struct rockchip_i2s_info *i2s = &rockchip_i2s;
	u32 opr,fifosts;
	  
    DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);
    opr = readl(&(pheadi2s->I2S_OPR));
    fifosts = readl(&(pheadi2s->I2S_FIFOSTS));
    fifosts = (fifosts & (~(0x0f<<16))) | TX_HALF_FULL | RX_HALF_FULL;
    writel(fifosts, &(pheadi2s->I2S_FIFOSTS));
    if (on) 
    {
        opr = (opr & (~(RESET_TX | I2S_DMA_REQ1_DISABLE| TX_START | RX_START))) | (RESET_RX | I2S_DMA_REQ2_DISABLE);
        writel(opr, &(pheadi2s->I2S_OPR));
        udelay(5);
        opr = (opr & (~(I2S_DMA_REQ2_DISABLE | I2S_DMA_REQ2_TX_ENABLE | TX_START))) | I2S_DMA_REQ2_ENABLE | I2S_DMA_REQ2_RX_ENABLE | RX_START;
        writel(opr, &(pheadi2s->I2S_OPR));
    }
    else
    {
        opr = (opr & (~RX_START)) | I2S_DMA_REQ2_DISABLE;
        writel(opr, &(pheadi2s->I2S_OPR));
    }   
}

/*
 * Set Rockchip I2S DAI format
 */
static int rockchip_i2s_set_fmt(struct snd_soc_cpu_dai *cpu_dai,
		unsigned int fmt)
{
    struct rockchip_i2s_info *i2s = &rockchip_i2s;	
    u32 tx_ctl,rx_ctl;
    DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);
    
    tx_ctl = readl(&(pheadi2s->I2S_TXCTL));
    tx_ctl &= (~MASTER_MODE);
    
    switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	  case SND_SOC_DAIFMT_CBM_CFM:  	
	  	tx_ctl |= MASTER_MODE;  
	  	break;
	  case SND_SOC_DAIFMT_CBS_CFS:
	  	tx_ctl |= SLAVE_MODE;  
	  	break;
	  default:
	  	DBG("unknwon master/slave format\n");
	  	return -EINVAL;
	  }
    tx_ctl &= ~IISMOD_SDF_MASK;
    
    switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
    case SND_SOC_DAIFMT_RIGHT_J:
    	tx_ctl |= RIGHT_JUSTIFIED;
    	break;
    case SND_SOC_DAIFMT_LEFT_J:
    	tx_ctl |= LEFT_JUSTIFIED;
    	break;
    case SND_SOC_DAIFMT_I2S:
    	tx_ctl |= I2S_MODE;
    	break;
    default:
    	DBG("Unknown data format\n");
    	return -EINVAL;
	  }
	tx_ctl = tx_ctl & (~(0xff<<8)) & (~(0x03<<16)) & (~(1<<3));  
	tx_ctl = tx_ctl | OVERSAMPLING_RATE_64FS | SCK_RATE4 | STEREO_MODE;   
    writel(tx_ctl, &(pheadi2s->I2S_TXCTL));
    rx_ctl = tx_ctl | CLEAR_RXFIFO;
    writel(rx_ctl, &(pheadi2s->I2S_RXCTL));
    return 0;
}

static int rockchip_i2s_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct rockchip_i2s_info *i2s = &rockchip_i2s;	
	u32 iismod;
	  
    DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);
	/*by Vincent Hsiung for EQ Vol Change*/
	#define HW_PARAMS_FLAG_EQVOL_ON 0x21
	#define HW_PARAMS_FLAG_EQVOL_OFF 0x22
    if ((params->flags == HW_PARAMS_FLAG_EQVOL_ON)||(params->flags == HW_PARAMS_FLAG_EQVOL_OFF))
    	{
    		return 0;
    	}
           
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rtd->dai->cpu_dai->dma_data = &rockchip_i2s_pcm_stereo_out;
	else
		rtd->dai->cpu_dai->dma_data = &rockchip_i2s_pcm_stereo_in;
    
	/* Working copies of register */
	iismod = readl(&(pheadi2s->I2S_TXCTL));
    iismod &= (~SAMPLE_DATA_MASK);
	switch (params_format(params)) {
	  case SNDRV_PCM_FORMAT_S8:
	  	iismod |= SAMPLE_DATA_8bit;
	  	break;
	  case SNDRV_PCM_FORMAT_S16_LE:
	  	iismod |= SAMPLE_DATA_16bit;
	  	break;
	  } 
	/*stereo mode MCLK/SCK=4*/  
	iismod = iismod & (~(0xff<<8)) & (~(0x03<<16)) & (~(1<<3));
	iismod = iismod | OVERSAMPLING_RATE_64FS | SCK_RATE4 | STEREO_MODE; 
	  
	writel(iismod, &(pheadi2s->I2S_TXCTL));
    iismod = iismod | CLEAR_RXFIFO;
    writel(iismod, &(pheadi2s->I2S_RXCTL));
    return 0;
}


static int rockchip_i2s_trigger(struct snd_pcm_substream *substream, int cmd)
{    
    int ret = 0;

    DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);
    
    switch (cmd) {
    case SNDRV_PCM_TRIGGER_START:
    case SNDRV_PCM_TRIGGER_RESUME:
    case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:   
    	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
    		rockchip_snd_rxctrl(1);
    	else
    		rockchip_snd_txctrl(1);
    	break;
    case SNDRV_PCM_TRIGGER_STOP:
    case SNDRV_PCM_TRIGGER_SUSPEND:
    case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
    	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
    		rockchip_snd_rxctrl(0);
    	else
    		rockchip_snd_txctrl(0);
    	break;
    default:
    	ret = -EINVAL;
    	break;
    }
    
	return ret;
}
/*
 * Set Rockchip Clock source
 */
static int rockchip_i2s_set_sysclk(struct snd_soc_cpu_dai *cpu_dai,
	int clk_id, unsigned int freq, int dir)
{
    DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);
    /*add scu clk source and enable clk*/
    
    return 0;
}

/*
 * Set Rockchip Clock dividers
 */
static int rockchip_i2s_set_clkdiv(struct snd_soc_cpu_dai *cpu_dai,
	int div_id, int div)
{
    //u32 reg;
    
    DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);
    /*when i2s in master mode ,must set codec pll div*/
    switch (div_id) {
    case ROCKCHIP_DIV_BCLK:
    	//reg = readl(&(pheadi2s->I2S_TXCTL)) & ~S3C2410_IISMOD_FS_MASK;
    	//writel(reg | div, &(pheadi2s->I2S_TXCTL));
    	break;
    case ROCKCHIP_DIV_MCLK:
    	//reg = readl(rockchip_i2s.regs + S3C2410_IISMOD) & ~(S3C2410_IISMOD_384FS);
    	//writel(reg | div, s3c24xx_i2s.regs + S3C2410_IISMOD);
    	break;
    case ROCKCHIP_DIV_PRESCALER:
    	//writel(div, s3c24xx_i2s.regs + S3C2410_IISPSR);
    	//reg = readl(s3c24xx_i2s.regs + S3C2410_IISCON);
    	//writel(reg | S3C2410_IISCON_PSCEN, s3c24xx_i2s.regs + S3C2410_IISCON);
    	break;
    default:
    	return -EINVAL;
	}
    return 0;
}
/*
 * To avoid duplicating clock code, allow machine driver to
 * get the clockrate from here.
 */
u32 rockchip_i2s_get_clockrate(void)
{
    DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);
	return 0;  ///clk_get_rate(s3c24xx_i2s.iis_clk);
}
EXPORT_SYMBOL_GPL(rockchip_i2s_get_clockrate);

static int rockchip_i2s_probe(struct platform_device *pdev)
{
    DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);
    rockchip_i2s.dev = &pdev->dev;
    
    rockchip_i2s.regs = I2S_BASE_ADDR_VA;  ///ioremap(I2S_BASE_ADDR, 0x20);
    
	  if (rockchip_i2s.regs == NULL)
		return -ENXIO;
		
    rockchip_i2s.iis_clk = 12000000;///clk_get(&pdev->dev, "i2sclk");
    ///clk_enable(rockchip_i2s.iis_clk);
    //rockchip_scu_apbunit_register( SCU_IPID_I2S , "i2sclk", NULL );
    /* Configure the I2S pins in correct mode */
    rockchip_mux_api_set(CXGPIO_I2S_SEL_NAME,IOMUXB_I2S_INTERFACE);
    
    rockchip_snd_txctrl(0);
	rockchip_snd_rxctrl(0);
    
    return 0;
}

#ifdef CONFIG_PM
int rockchip_i2s_suspend(struct platform_device *pdev,
		struct snd_soc_cpu_dai *cpu_dai)
{
    DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);
    //clk_disable(clk);
	  return 0;
}

int rockchip_i2s_resume(struct platform_device *pdev,
		struct snd_soc_cpu_dai *cpu_dai)
{
    DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);
    //clk_enable(clk);
    return 0;
}		
#else
#define rockchip_i2s_suspend NULL
#define rockchip_i2s_resume NULL
#endif


#define ROCKCHIP_I2S_RATES \
	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_16000 | \
	SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
	SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000)

struct snd_soc_cpu_dai rockchip_i2s_dai = {
	.name = "rockchip-i2s",
	.id = 0,
	.type = SND_SOC_DAI_I2S,   
	.probe = rockchip_i2s_probe,
	.suspend = rockchip_i2s_suspend,
	.resume = rockchip_i2s_resume,
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = ROCKCHIP_I2S_RATES,
		.formats = SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE,},
	.capture = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = ROCKCHIP_I2S_RATES,
		.formats = SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE,},
	.ops = {
		.trigger = rockchip_i2s_trigger,
		.hw_params = rockchip_i2s_hw_params,},
	.dai_ops = {
		.set_fmt = rockchip_i2s_set_fmt,
		.set_clkdiv = rockchip_i2s_set_clkdiv,
		.set_sysclk = rockchip_i2s_set_sysclk,
	},
};


EXPORT_SYMBOL_GPL(rockchip_i2s_dai);

/* Module information */
MODULE_AUTHOR("lhh lhh@rock-chips.com");
MODULE_DESCRIPTION("ROCKCHIP IIS ASoC Interface");
MODULE_LICENSE("GPL");

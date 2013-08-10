/*
 * rockchip_wm8988.c  --  SoC audio for rockchip
 *
 * Driver for rockchip wm8988 audio
 *  Copyright (C) 2009 lhh
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *
 */

#include <linux/module.h>
#include <linux/device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/hw_common.h>

#include "../codecs/wm8988.h"
#include "rk28-pcm.h"
#include "rk28-iis.h"

#if 1
#define	DBG(x...)	printk(KERN_INFO x)
#else
#define	DBG(x...)
#endif



static int rockchip_startup(struct snd_pcm_substream *substream)
{    
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_cpu_dai *cpu_dai = rtd->dai->cpu_dai;
	int ret;
	  
    DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);   
    
	  /* cpu clock is the rockchip master clock sent to the I2S */
	  ret = cpu_dai->dai_ops.set_sysclk(cpu_dai, 0,  ///AT91_SYSCLK_MCK,
	  	75000000, SND_SOC_CLOCK_IN);
	  if (ret < 0)
	  	return ret;
    
	  /* codec system clock is supplied by PCK1, set to 12MHz */
	  ret = codec_dai->dai_ops.set_sysclk(codec_dai, 0,  ///WM8988_SYSCLK,
	  	12000000, SND_SOC_CLOCK_IN);
	  if (ret < 0)
	  	return ret;
    
	  /* Start PCK1 clock. */
	//  clk_enable(pck1_clk);
	DBG("pck1 started\n");
    
	return 0;
}
static void rockchip_shutdown(struct snd_pcm_substream *substream)
{
    DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);
    /* Stop PCK1 clock. */
	  //clk_disable(pck1_clk);
	DBG("pck1 stopped\n");
}
static int rockchip_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_cpu_dai *cpu_dai = rtd->dai->cpu_dai;
	int ret;
	  
    DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);    
	/*by Vincent Hsiung for EQ Vol Change*/
	#define HW_PARAMS_FLAG_EQVOL_ON 0x21
	#define HW_PARAMS_FLAG_EQVOL_OFF 0x22
    if ((params->flags == HW_PARAMS_FLAG_EQVOL_ON)||(params->flags == HW_PARAMS_FLAG_EQVOL_OFF))
    {
    	ret = codec_dai->ops.hw_params(substream,params); //by Vincent
    }
    else
    {
	    /* set codec DAI configuration */
	    #if defined (CONFIG_SND_ROCKCHIP_SOC_MASTER) 
	    ret = codec_dai->dai_ops.set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
	    	SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS); 
	    #endif	
	    #if defined (CONFIG_SND_ROCKCHIP_SOC_SLAVE) 
	    ret = codec_dai->dai_ops.set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
	    	SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM ); 
	    #endif
	    if (ret < 0)
	    	  return ret; 
	    /* set cpu DAI configuration */
	    #if defined (CONFIG_SND_ROCKCHIP_SOC_MASTER) 
	    ret = cpu_dai->dai_ops.set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
	    	SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	    #endif	
	    #if defined (CONFIG_SND_ROCKCHIP_SOC_SLAVE) 
	    ret = cpu_dai->dai_ops.set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
	    	SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);	
	    #endif		
	    if (ret < 0)
	    	  return ret;
	  }
    
	  return 0;
}

static const struct snd_soc_dapm_widget rockchip_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Int Mic", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
};

static const char *audio_map[][3] = {

	/* speaker connected to LHPOUT */
	{"Ext Spk", NULL, "LHPOUT"},

	/* mic is connected to Mic Jack, with WM8988 Mic Bias */
	{"MICIN", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Int Mic"},

	/* terminator */
	{NULL, NULL, NULL},
};

/*
 * Logic for a wm8988 as connected on a rockchip board.
 */
static int rockchip_wm8988_init(struct snd_soc_codec *codec)
{
	int i;
	  
    DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);
    
    /* Add specific widgets */
    for(i = 0; i < ARRAY_SIZE(rockchip_dapm_widgets); i++) {
    	  snd_soc_dapm_new_control(codec, &rockchip_dapm_widgets[i]);
    }
    
    /* Set up specific audio path audio_mapnects */
    for(i = 0; audio_map[i][0] != NULL; i++) {
    	snd_soc_dapm_connect_input(codec, audio_map[i][0],
    		audio_map[i][1], audio_map[i][2]);
    }
    
    /* not connected */
    snd_soc_dapm_set_endpoint(codec, "RLINEIN", 0);
    snd_soc_dapm_set_endpoint(codec, "LLINEIN", 0);
    
    /* always connected */
    snd_soc_dapm_set_endpoint(codec, "Int Mic", 1);
    snd_soc_dapm_set_endpoint(codec, "Ext Spk", 1);
    
    snd_soc_dapm_sync_endpoints(codec);
    
    return 0;
}
static struct snd_soc_ops rockchip_ops = {
	  .startup = rockchip_startup,
	  .hw_params = rockchip_hw_params,
	  .shutdown = rockchip_shutdown,
};

static struct snd_soc_dai_link rockchip_dai = {
	  .name = "WM8988",
	  .stream_name = "WM8988 PCM",
	  .cpu_dai = &rockchip_i2s_dai,
	  .codec_dai = &wm8988_dai,
	  .init = rockchip_wm8988_init,
	  .ops = &rockchip_ops,
};

static struct snd_soc_machine snd_soc_machine_rockchip = {
	  .name = "ROCKCHIP_WM8988",
	  .dai_link = &rockchip_dai,
	  .num_links = 1,
};

static struct wm8988_setup_data rockchip_wm8988_setup = {
	  .i2c_address = 0x1a,
};

static struct snd_soc_device rockchip_snd_devdata = {
	  .machine = &snd_soc_machine_rockchip,
	  .platform = &rockchip_soc_platform,
	  .codec_dev = &soc_codec_dev_wm8988,
	  .codec_data = &rockchip_wm8988_setup,
};

static struct platform_device *rockchip_snd_device;

static int __init audio_card_init(void)
{
	int ret =0;	
    DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);
    if (!request_mem_region(I2S_BASE_ADDR, 0x20, "soc-audio")) {
	  	  DBG("I2S memory region is busy\n");
	  	  return -EBUSY;
	  }
	  rockchip_snd_device = platform_device_alloc("soc-audio", -1);
	  if (!rockchip_snd_device) {
	  	  DBG("platform device allocation failed\n");
	  	  ret = -ENOMEM;
	  	  goto fail_release_mem;
	  }
	  platform_set_drvdata(rockchip_snd_device, &rockchip_snd_devdata);
	  rockchip_snd_devdata.dev = &rockchip_snd_device->dev;
	  ret = platform_device_add(rockchip_snd_device);
	  if (ret) {
		    DBG("platform device add failed\n");
		    platform_device_put(rockchip_snd_device);
		goto fail_release_mem;
	  }
	  return ret;
	
fail_release_mem:
	  release_mem_region(I2S_BASE_ADDR, 0x20);
	  return ret;
}
static void __exit audio_card_exit(void)
{
    DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);
	platform_device_unregister(rockchip_snd_device);
}

module_init(audio_card_init);
module_exit(audio_card_exit);
/* Module information */
MODULE_AUTHOR("lhh lhh@rock-chips.com");
MODULE_DESCRIPTION("ROCKCHIP IIS ASoC Interface");
MODULE_LICENSE("GPL");

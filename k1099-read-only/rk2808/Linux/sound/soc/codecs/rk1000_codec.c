/*
 * rk1000.c -- RK1000 ALSA SoC audio driver
 *
 * Copyright (C) 2009 rockchip lhh
 *
 *
 * Based on RK1000.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <asm/arch/gpio.h>
#include <asm/arch/iomux.h>
#include <asm/arch/rk28_scu.h>

#include "rk1000_codec.h"

#ifdef CONFIG_ANX7150
#include <asm/arch/anx7150.h>
#endif

#define AUDIO_NAME "RK1000_CODEC"
#define RK1000_VERSION "1.0"
#ifdef CONFIG_MACH_PWS700AA
#define SPK_CTRL_PIN          GPIOPortB_Pin3
#define SPK_IOMUX_PIN_NAME       GPIOB3_U0RTSN_SEL_NAME
#define SPK_IOMUX_PIN_DIR        IOMUXB_GPIO0_B3
#else
#define SPK_CTRL_PIN          GPIOPortF_Pin7
#define SPK_IOMUX_PIN_NAME       GPIOE_SPI1_SEL_NAME
#define SPK_IOMUX_PIN_DIR        IOMUXA_GPIO1_A1237
#endif
/*
 * Debug
 */
#if 0
#define	DBG(x...)	printk(KERN_INFO x)
#else
#define	DBG(x...)
#endif

#define err(format, arg...) \
	printk(KERN_ERR AUDIO_NAME ": " format "\n" , ## arg)
#define info(format, arg...) \
	printk(KERN_INFO AUDIO_NAME ": " format "\n" , ## arg)
	
#define OUT_CAPLESS  (0)   //是否为无电容输出，1:无电容输出，0:有电容输出	

static u32 gVolReg = 0x0f;  ///0x0f; //用于记录音量寄存器
//static u32 gCodecVol = 0x0f;
static u8 gR0AReg = 0;  //用于记录R0A寄存器的值，用于改变采样率前通过R0A停止clk
static u8 gR0BReg = 0;  //用于记录R0B寄存器的值，用于改变采样率前通过R0B停止interplate和decimation
static u8 gR1314Reg = 0;  //用于记录R13,R14寄存器的值，用于FM音量为0时

#if defined (CONFIG_I2C) || defined (CONFIG_I2C_MODULE)
static unsigned short normal_i2c[] = { 0, I2C_CLIENT_END };

/* Magic definition of all other variables and things */
I2C_CLIENT_INSMOD;
#endif

/* codec private data */
struct rk1000_codec_priv {
	unsigned int sysclk;
};

/*
 * rk1000 register cache
 * We can't read the RK1000 register space when we
 * are using 2 wire for device control, so we cache them instead.
 */
static const u16 rk1000_codec_reg[] = {
	0x0005, 0x0004, 0x00fd, 0x00f3,  /*  0 */
	0x0003, 0x0000, 0x0000, 0x0000,  /*  4 */
	0x0000, 0x0005, 0x0000, 0x0000,  /*  8 */
	0x0097, 0x0097, 0x0097, 0x0097,  /* 12 */
	0x0097, 0x0097, 0x00cc, 0x0000,  /* 16 */
	0x0000, 0x00f1, 0x0090, 0x00ff,  /* 20 */
	0x00ff, 0x00ff, 0x009c, 0x0000,  /* 24 */
	0x0000, 0x00ff, 0x00ff, 0x00ff,  /* 28 */
};

/*
 * read rk1000 register cache
 */
static inline unsigned int rk1000_codec_read_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u16 *cache = codec->reg_cache;
	if (reg > RK1000_CACHE_REGNUM)
		return -1;
	return cache[reg];
}

/*
 * write rk1000 register cache
 */
static inline void rk1000_codec_write_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg, unsigned int value)
{
	u16 *cache = codec->reg_cache;
	if (reg > RK1000_CACHE_REGNUM)
		return;
	cache[reg] = value;
}

static int rk1000_codec_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	u8 data[2];
	struct i2c_client *i2c;
    //DBG("%s[%d] reg=0x%x value=0x%x\n",__FUNCTION__,__LINE__,reg,value);
	data[0] = value & 0x00ff;
	data[1] = data[0];
	rk1000_codec_write_reg_cache (codec, reg, value);
	i2c = (struct i2c_client *)codec->control_data;
	i2c->addr = (i2c->addr & 0x60)|reg;
	if (codec->hw_write(codec->control_data, data, 2) == 2)
		return 0;
	else
		return -EIO;
}

static int rk1000_codec_read(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int *value)
{
	u8 data[2];
	struct i2c_client *i2c;

	i2c = (struct i2c_client *)codec->control_data;
	i2c->addr = (i2c->addr & 0x60)|(reg+1);
	
	if (codec->hw_read(codec->control_data, data, 2) == 2)
	{
		*value = *((short*)&data[0]);
		return 0;
	}
	else
	{
		printk("codec read error!");
		return -EIO;
	}
}

static const struct snd_kcontrol_new rk1000_codec_snd_controls[] = {

SOC_DOUBLE_R("Capture Volume", ACCELCODEC_R0C, ACCELCODEC_R0D, 0, 15, 0),
SOC_DOUBLE_R("Capture Switch", ACCELCODEC_R0C, ACCELCODEC_R0D, 7, 1, 1),

SOC_DOUBLE_R("PCM Volume", ACCELCODEC_R0D, ACCELCODEC_R0E, 0, 7, 0),

//SOC_SINGLE("Left ADC Capture Volume", ACCELCODEC_R17, 0, 63, 0),
//SOC_SINGLE("Right ADC Capture Volume", ACCELCODEC_R18, 0, 63, 0),


};

/* add non dapm controls */
static int rk1000_codec_add_controls(struct snd_soc_codec *codec)
{
	int err, i;
    DBG("%s[%d]\n",__FUNCTION__,__LINE__);
	for (i = 0; i < ARRAY_SIZE(rk1000_codec_snd_controls); i++) {
		err = snd_ctl_add(codec->card,
				snd_soc_cnew(&rk1000_codec_snd_controls[i],codec, NULL));
		if (err < 0)
			return err;
	}
	return 0;
}


/* Left Mixer */
static const struct snd_kcontrol_new rk1000_codec_left_mixer_controls[] = {
SOC_DAPM_SINGLE("Playback Switch", ACCELCODEC_R15, 6, 1, 0),
SOC_DAPM_SINGLE("Left Bypass Switch", ACCELCODEC_R15, 2, 1, 0),

};

/* Right Mixer */
static const struct snd_kcontrol_new rk1000_codec_right_mixer_controls[] = {
SOC_DAPM_SINGLE("Playback Switch", ACCELCODEC_R15, 7, 1, 0),
SOC_DAPM_SINGLE("Left Bypass Switch", ACCELCODEC_R15, 3, 1, 0),

};


static const struct snd_soc_dapm_widget rk1000_codec_dapm_widgets[] = {
	SND_SOC_DAPM_MIXER("Left Mixer", SND_SOC_NOPM, 0, 0,
		&rk1000_codec_left_mixer_controls[0],
		ARRAY_SIZE(rk1000_codec_left_mixer_controls)),
	SND_SOC_DAPM_MIXER("Right Mixer", SND_SOC_NOPM, 0, 0,
		&rk1000_codec_right_mixer_controls[0],
		ARRAY_SIZE(rk1000_codec_right_mixer_controls)),
    
	//SND_SOC_DAPM_PGA("Right Out 1", ACCELCODEC_R1E, 0, 0, NULL, 0),
	//SND_SOC_DAPM_PGA("Left Out 1", ACCELCODEC_R1E, 1, 0, NULL, 0),
	//SND_SOC_DAPM_DAC("Right DAC", "Right Playback", ACCELCODEC_R1F, 1, 0),
	//SND_SOC_DAPM_DAC("Left DAC", "Left Playback", ACCELCODEC_R1F, 2, 0),
    
	SND_SOC_DAPM_ADC("ADC", "Capture", ACCELCODEC_R1D, 6, 1),
	SND_SOC_DAPM_ADC("ADC BUFF", "Capture BUFF", ACCELCODEC_R1D, 2, 0),
    
     
	SND_SOC_DAPM_OUTPUT("LOUT1"),
	SND_SOC_DAPM_OUTPUT("ROUT1"),
    
	SND_SOC_DAPM_INPUT("LINPUT1"),
	SND_SOC_DAPM_INPUT("RINPUT1"),
};

static const char *audio_map[][3] = {
	/* left mixer */
	{"Left Mixer", "Playback Switch", "Left DAC"},
	{"Left Mixer", "Left Bypass Switch", "Left Line Mux"},
	//{"Left Mixer", "Right Playback Switch", "Right DAC"},
	//{"Left Mixer", "Right Bypass Switch", "Right Line Mux"},
    
	/* right mixer */
	//{"Right Mixer", "Left Playback Switch", "Left DAC"},
	//{"Right Mixer", "Left Bypass Switch", "Left Line Mux"},
	{"Right Mixer", "Playback Switch", "Right DAC"},
	{"Right Mixer", "Right Bypass Switch", "Right Line Mux"},
    
	/* left out 1 */
	{"Left Out 1", NULL, "Left Mixer"},
	{"LOUT1", NULL, "Left Out 1"},
    
    
	/* right out 1 */
	{"Right Out 1", NULL, "Right Mixer"},
	{"ROUT1", NULL, "Right Out 1"},
    
	/* Left Line Mux */
	{"Left Line Mux", "Line 1", "LINPUT1"},
	{"Left Line Mux", "PGA", "Left PGA Mux"},
	{"Left Line Mux", "Differential", "Differential Mux"},
    
	/* Right Line Mux */
	{"Right Line Mux", "Line 1", "RINPUT1"},
	{"Right Line Mux", "PGA", "Right PGA Mux"},
	{"Right Line Mux", "Differential", "Differential Mux"},
    
	/* Left PGA Mux */
	{"Left PGA Mux", "Line 1", "LINPUT1"},
	{"Left PGA Mux", "Line 2", "LINPUT2"},
	{"Left PGA Mux", "Line 3", "LINPUT3"},
	{"Left PGA Mux", "Differential", "Differential Mux"},
    
	/* Right PGA Mux */
	{"Right PGA Mux", "Line 1", "RINPUT1"},
	{"Right PGA Mux", "Differential", "Differential Mux"},
    
	/* Differential Mux */
	{"Differential Mux", "Line 1", "LINPUT1"},
	{"Differential Mux", "Line 1", "RINPUT1"},
    
	/* Left ADC Mux */
	{"Left ADC Mux", "Stereo", "Left PGA Mux"},
	{"Left ADC Mux", "Mono (Left)", "Left PGA Mux"},
	{"Left ADC Mux", "Digital Mono", "Left PGA Mux"},
    
	/* Right ADC Mux */
	{"Right ADC Mux", "Stereo", "Right PGA Mux"},
	{"Right ADC Mux", "Mono (Right)", "Right PGA Mux"},
	{"Right ADC Mux", "Digital Mono", "Right PGA Mux"},
    
	/* ADC */
	{"Left ADC", NULL, "Left ADC Mux"},
	{"Right ADC", NULL, "Right ADC Mux"},
    
	/* terminator */
	{NULL, NULL, NULL},
};

static int rk1000_codec_add_widgets(struct snd_soc_codec *codec)
{
	int i;
    DBG("%s[%d]\n",__FUNCTION__,__LINE__);
	for(i = 0; i < ARRAY_SIZE(rk1000_codec_dapm_widgets); i++) {
		snd_soc_dapm_new_control(codec, &rk1000_codec_dapm_widgets[i]);
	}
    
	/* set up audio path audio_mapnects */
	for(i = 0; audio_map[i][0] != NULL; i++) {
		snd_soc_dapm_connect_input(codec, audio_map[i][0],
			audio_map[i][1], audio_map[i][2]);
	}
    
	snd_soc_dapm_new_widgets(codec);
	return 0;
}

struct _coeff_div {
	u32 mclk;
	u32 rate;
	u16 fs;
	u8 sr:5;
	u8 usb:1;
	u8 bclk;
};

/* codec hifi mclk clock divider coefficients */
static const struct _coeff_div coeff_div[] = {
	/* 8k */
	{12288000, 8000, 1536, 0x6, 0x0,ASC_BCLKDIV_16},
	{11289600, 8000, 1408, 0x16, 0x0,ASC_BCLKDIV_16},
	{18432000, 8000, 2304, 0x7, 0x0,ASC_BCLKDIV_16},
	{16934400, 8000, 2112, 0x17, 0x0,ASC_BCLKDIV_16},
	{8192000, 8000, 1024, 0x0, 0x0,ASC_BCLKDIV_16},
	{12000000, 8000, 1500, 0x6, 0x1,ASC_BCLKDIV_16},
    
	/* 11.025k */
	{11289600, 11025, 1024, 0x18, 0x0,ASC_BCLKDIV_16},
	{16934400, 11025, 1536, 0x19, 0x0,ASC_BCLKDIV_16},
	{12000000, 11025, 1088, 0x19, 0x1,ASC_BCLKDIV_16},
    
    /* 12k */
	{12288000, 12000, 1024, 0x8, 0x0,ASC_BCLKDIV_16},
	{18432000, 12000, 1536, 0x9, 0x0,ASC_BCLKDIV_16},
	{12000000, 12000, 1000, 0x8, 0x1,ASC_BCLKDIV_16},
    
	/* 16k */
	{12288000, 16000, 768, 0xa, 0x0,ASC_BCLKDIV_8},
	{18432000, 16000, 1152, 0xb, 0x0,ASC_BCLKDIV_8},
	{12000000, 16000, 750, 0xa, 0x1,ASC_BCLKDIV_8},
    
	/* 22.05k */
	{11289600, 22050, 512, 0x1a, 0x0,ASC_BCLKDIV_8},
	{16934400, 22050, 768, 0x1b, 0x0,ASC_BCLKDIV_8},
	{12000000, 22050, 544, 0x1b, 0x1,ASC_BCLKDIV_8},
    
    /* 24k */
	{12288000, 24000, 512, 0x1c, 0x0,ASC_BCLKDIV_8},
	{18432000, 24000, 768, 0x1d, 0x0,ASC_BCLKDIV_8},
	{12000000, 24000, 500, 0x1c, 0x1,ASC_BCLKDIV_8},
	
	/* 32k */
	{12288000, 32000, 384, 0xc, 0x0,ASC_BCLKDIV_8},
	{18432000, 32000, 576, 0xd, 0x0,ASC_BCLKDIV_8},
	{12000000, 32000, 375, 0xa, 0x1,ASC_BCLKDIV_8},
    
	/* 44.1k */
	{11289600, 44100, 256, 0x10, 0x0,ASC_BCLKDIV_8},
	{16934400, 44100, 384, 0x11, 0x0,ASC_BCLKDIV_8},
	{12000000, 44100, 272, 0x11, 0x1,ASC_BCLKDIV_8},
    
	/* 48k */
	{12288000, 48000, 256, 0x0, 0x0,ASC_BCLKDIV_4},
	{18432000, 48000, 384, 0x1, 0x0,ASC_BCLKDIV_4},
	{12000000, 48000, 250, 0x0, 0x1,ASC_BCLKDIV_4},
    
	/* 88.2k */
	{11289600, 88200, 128, 0x1e, 0x0,ASC_BCLKDIV_4},
	{16934400, 88200, 192, 0x1f, 0x0,ASC_BCLKDIV_4},
	{12000000, 88200, 136, 0x1f, 0x1,ASC_BCLKDIV_4},
    
	/* 96k */
	{12288000, 96000, 128, 0xe, 0x0,ASC_BCLKDIV_4},
	{18432000, 96000, 192, 0xf, 0x0,ASC_BCLKDIV_4},
	{12000000, 96000, 125, 0xe, 0x1,ASC_BCLKDIV_4},
};

static inline int get_coeff(int mclk, int rate)
{
	int i;
    
	//printk("<7>mclk = %d, rate = %d\n", mclk, rate);
	for (i = 0; i < ARRAY_SIZE(coeff_div); i++) {
		if (coeff_div[i].rate == rate && coeff_div[i].mclk == mclk)
			return i;
	}
    
	printk(KERN_ERR "rk1000: could not get coeff for mclk %d @ rate %d\n",
		mclk, rate);
	return -EINVAL;
}

static int rk1000_codec_set_dai_sysclk(struct snd_soc_codec_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct rk1000_codec_priv *rk1000 = codec->private_data;
    DBG("%s[%d]\n",__FUNCTION__,__LINE__);

	switch (freq) {
	case 11289600:
	case 12000000:
	case 12288000:
	case 16934400:
	case 18432000:
		rk1000->sysclk = freq;
		return 0;
	}
	return -EINVAL;
}

static int rk1000_codec_set_dai_fmt(struct snd_soc_codec_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 iface = 0;
#ifdef CONFIG_ANX7150
	if(anx7150_get_output_status() == HDMI){
		return 0;
	}
#endif
    DBG("%s[%d]\n",__FUNCTION__,__LINE__);
	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		iface = 0x0040;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
	    iface = 0x0020;
		break;
	default:
		return -EINVAL;
	}
    
	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iface |= 0x0002;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface |= 0x0001;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface |= 0x0003;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		iface |= 0x0013;
		break;
	default:
		return -EINVAL;
	}
    
	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		iface |= 0x0090;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		iface |= 0x0080;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		iface |= 0x0010;
		break;
	default:
		return -EINVAL;
	}
    
	rk1000_codec_write(codec, ACCELCODEC_R09, iface);
	return 0;
}

static int rk1000_codec_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	struct rk1000_codec_priv *rk1000 = codec->private_data;

#ifdef CONFIG_ANX7150
	if(anx7150_get_output_status() == HDMI){
		return 0;
	}
#endif
    DBG("````%s[%d]\n",__FUNCTION__,__LINE__);

	/*by Vincent Hsiung for EQ Vol Change*/
	#define HW_PARAMS_FLAG_EQVOL_ON 0x21
	#define HW_PARAMS_FLAG_EQVOL_OFF 0x22
	if (params->flags == HW_PARAMS_FLAG_EQVOL_ON)
	{
		u16 r17 = rk1000_codec_read_reg_cache(codec, ACCELCODEC_R17);
		u16 r18 = rk1000_codec_read_reg_cache(codec, ACCELCODEC_R17);
		
		r17 &= (~0x3f); //6db
		r18 &= (~0x3f); //6db
		
		rk1000_codec_write(codec, ACCELCODEC_R17, r17);
		rk1000_codec_write(codec, ACCELCODEC_R18, r18);
		
		return 0;
	}
	else if (params->flags == HW_PARAMS_FLAG_EQVOL_OFF)
	{
		u16 r17 = rk1000_codec_read_reg_cache(codec, ACCELCODEC_R17);
		u16 r18 = rk1000_codec_read_reg_cache(codec, ACCELCODEC_R17);
		
		r17 &= (~0x3f); 
		r17 |= 0x0f; //0db
		
		r18 &= (~0x3f); 
		r18 |= 0x0f; //0db
		
		rk1000_codec_write(codec, ACCELCODEC_R17, r17);
		rk1000_codec_write(codec, ACCELCODEC_R18, r18);
		return 0;
	} 
	
	u16 iface = rk1000_codec_read_reg_cache(codec, ACCELCODEC_R09) & 0xf3;
	u16 srate = rk1000_codec_read_reg_cache(codec, ACCELCODEC_R00) & 0xcf;
	int coeff = get_coeff(rk1000->sysclk, params_rate(params));
    DBG("%s[%d]\n",__FUNCTION__,__LINE__);
	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		iface |= 0x0004;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		iface |= 0x0008;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		iface |= 0x000c;
		break;
	}
	
    rk1000_codec_write(codec,ACCELCODEC_R0C, 0x17);  
    rk1000_codec_write(codec,ACCELCODEC_R04, ASC_INT_MUTE_L|ASC_INT_MUTE_R|ASC_SIDETONE_L_OFF|ASC_SIDETONE_R_OFF);   //soft mute

    //必须先将clk和EN_INT都disable掉，否则切换bclk分频值可能导致codec内部时序混乱掉，
    //表现出来的现象是，以后的音乐都变成了噪音，而且就算把输入codec的I2S_DATAOUT断开也一样出噪音
    rk1000_codec_write(codec,ACCELCODEC_R0B, ASC_DEC_DISABLE|ASC_INT_DISABLE);  //0x00
    rk1000_codec_write(codec,ACCELCODEC_R0A, (gR0AReg & 0x7F)); //disable clk
        
	/* set iface & srate */
	rk1000_codec_write(codec, ACCELCODEC_R09, iface);
	if (coeff >= 0){
		rk1000_codec_write(codec, ACCELCODEC_R0A, (coeff_div[coeff].sr << 1) | coeff_div[coeff].usb|ASC_CLKNODIV|ASC_CLK_ENABLE);
	    rk1000_codec_write(codec, ACCELCODEC_R00, srate|coeff_div[coeff].bclk);
	}		
    rk1000_codec_write(codec,ACCELCODEC_R0B, gR0BReg);
	return 0;
}

static struct snd_soc_device *rk1000_codec_socdev;

#ifdef CONFIG_ANX7150
int rk1000_codec_set_clk(int mclk, int rate)
{
	struct snd_soc_codec *codec = rk1000_codec_socdev->codec;

	u16 srate = rk1000_codec_read_reg_cache(codec, ACCELCODEC_R00) & 0xcf;
	int coeff = get_coeff(mclk, rate);
	
	DBG("%s[%d]\n",__FUNCTION__,__LINE__); 

	if(coeff < 0){
		DBG("get coeff err!\n");
		return coeff;
	}
	
    rk1000_codec_write(codec,ACCELCODEC_R0B, ASC_DEC_DISABLE|ASC_INT_DISABLE);  //0x00
    rk1000_codec_write(codec,ACCELCODEC_R0A, (gR0AReg & 0x7F)); //disable clk

	if(mclk == 12000000){
		__rockchip_scu_set_parent( SCU_IPID_I2S , SCU_IPID_12M , SCU_MODE_NONE, 0 );
	}
	else if(mclk == 12288000){
		__rockchip_clk_set_unit_clock(SCU_IPID_CODEC, SCU_CLK_122880);
		__rockchip_scu_set_parent( SCU_IPID_I2S , SCU_IPID_CODEC , SCU_MODE_SETDIV , 25 );
	}
	else{
		DBG("invalid mclk, mclk = %d\n", mclk);
	}
	
	/* set srate */
	if (coeff >= 0){
		rk1000_codec_write(codec, ACCELCODEC_R0A, (coeff_div[coeff].sr << 1) | coeff_div[coeff].usb|ASC_CLKNODIV|ASC_CLK_ENABLE);
		rk1000_codec_write(codec, ACCELCODEC_R00, srate|coeff_div[coeff].bclk);
	}
    rk1000_codec_write(codec,ACCELCODEC_R0B, gR0BReg);

	return 0;
}
#endif

void PhaseOut(struct snd_soc_codec *codec,u32 nStep, u32 us)
{
    DBG("%s[%d]\n",__FUNCTION__,__LINE__); 
   // while(1)
   // {
     //   gVolReg = gVolReg + nStep;  //虽然是加，但音量是在减小
     //   if(gVolReg < 0x3F)
     //   {
      //      rk1000_codec_write(codec,ACCELCODEC_R17, gVolReg|ASC_OUTPUT_ACTIVE|ASC_CROSSZERO_EN);  //AOL
      //      rk1000_codec_write(codec,ACCELCODEC_R18, gVolReg|ASC_OUTPUT_ACTIVE|ASC_CROSSZERO_EN);  //AOR
      //      udelay(us);
      //  }
      //  else
      //  {
            rk1000_codec_write(codec,ACCELCODEC_R17, 0x0F|ASC_OUTPUT_ACTIVE|ASC_CROSSZERO_EN);  //AOL
            rk1000_codec_write(codec,ACCELCODEC_R18, 0x0F|ASC_OUTPUT_ACTIVE|ASC_CROSSZERO_EN);  //AOR
            udelay(us);
         //   break;
       // }
   // }
}

void PhaseIn(struct snd_soc_codec *codec,u32 nStep, u32 us)
{
   // u32 vol = 0x3F;
    DBG("%s[%d]\n",__FUNCTION__,__LINE__); 
    //while(1)
   // {
       // vol = vol - nStep; //虽然是减，但音量是在增加
       // if(vol < gVolReg)
       // {
         //   rk1000_codec_write(codec,ACCELCODEC_R17, vol|ASC_OUTPUT_ACTIVE|ASC_CROSSZERO_EN);  //AOL
         //   rk1000_codec_write(codec,ACCELCODEC_R18, vol|ASC_OUTPUT_ACTIVE|ASC_CROSSZERO_EN);  //AOR
          //  udelay(us);
       // }
        //else
       // {
            rk1000_codec_write(codec,ACCELCODEC_R17, 0x0f|ASC_OUTPUT_ACTIVE|ASC_CROSSZERO_EN);  //AOL gVolReg|ASC_OUTPUT_ACTIVE|ASC_CROSSZERO_EN);  //AOL
            rk1000_codec_write(codec,ACCELCODEC_R18, 0x0f|ASC_OUTPUT_ACTIVE|ASC_CROSSZERO_EN); //gVolReg|ASC_OUTPUT_ACTIVE|ASC_CROSSZERO_EN);  //AOR
            udelay(us);
           // break;
        //}
    //}
}

int rk1000_codec_mute(struct snd_soc_codec_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
    DBG("%s[%d]\n",__FUNCTION__,__LINE__); 
	DBG("mute = %d\n", mute);
#ifdef CONFIG_ANX7150
	if(anx7150_get_output_status() == HDMI){
		mute = 1;
	}
#endif

	if (mute){
	    PhaseOut(codec,1, 5000);
		rk1000_codec_write(codec,ACCELCODEC_R19, 0xFF);  //AOM
        rk1000_codec_write(codec,ACCELCODEC_R04, ASC_INT_MUTE_L|ASC_INT_MUTE_R|ASC_SIDETONE_L_OFF|ASC_SIDETONE_R_OFF);  //soft mute   
	}else{		
		rk1000_codec_write(codec,ACCELCODEC_R1D, 0x2a);  //setup Vmid and Vref, other module power down
		rk1000_codec_write(codec,ACCELCODEC_R1E, 0x40);  ///|ASC_PDASDML_ENABLE);
        rk1000_codec_write(codec,ACCELCODEC_R1F, 0x09|ASC_PDMIXM_ENABLE|ASC_PDPAM_ENABLE);  ///|ASC_PDMICB_ENABLE|ASC_PDMIXM_ENABLE);
        PhaseIn(codec,1, 5000);
		///if(gCodecVol != 0){
        rk1000_codec_write(codec,ACCELCODEC_R04, ASC_INT_ACTIVE_L|ASC_INT_ACTIVE_R|ASC_SIDETONE_L_OFF|ASC_SIDETONE_R_OFF);
        //}
        rk1000_codec_write(codec,ACCELCODEC_R19, 0x7F);  //AOM
        #if 1
	    /*disable speaker */
	    rockchip_mux_api_set(SPK_IOMUX_PIN_NAME, SPK_IOMUX_PIN_DIR);
	    GPIOSetPinDirection(SPK_CTRL_PIN,GPIO_OUT);
	    GPIOSetPinLevel(SPK_CTRL_PIN,GPIO_HIGH);
       #endif
	}
	return 0;
}

static int rk1000_codec_dapm_event(struct snd_soc_codec *codec, int event)
{
	u16 pwr_reg = rk1000_codec_read_reg_cache(codec, ACCELCODEC_R1D) & 0xff;
    DBG("%s[%d]\n",__FUNCTION__,__LINE__);
	switch (event) {
	case SNDRV_CTL_POWER_D0: /* full On */
		/* set vmid to 50k and unmute dac */
		rk1000_codec_write(codec, ACCELCODEC_R1D, pwr_reg & 0x007ff);
		break;
	case SNDRV_CTL_POWER_D1: /* partial On */
	case SNDRV_CTL_POWER_D2: /* partial On */
		/* set vmid to 5k for quick power up */
		rk1000_codec_write(codec, ACCELCODEC_R1D, pwr_reg & 0x007ff);  //0x01c1);
		break;
	case SNDRV_CTL_POWER_D3hot: /* Off, with power */
		/* mute dac and set vmid to 500k, enable VREF */
		rk1000_codec_write(codec, ACCELCODEC_R1D, pwr_reg & 0x007ff);  //0x0141);
		break;
	case SNDRV_CTL_POWER_D3cold: /* Off, without power */
		rk1000_codec_write(codec, ACCELCODEC_R1D, pwr_reg & 0x007ff);
		break;
	}
	codec->dapm_state = event;
	return 0;
}

#define RK1000_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_44100 | \
		SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000)

#define RK1000_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
	 SNDRV_PCM_FMTBIT_S24_LE)

struct snd_soc_codec_dai rk1000_codec_dai = {
	.name = "RK1000_CODEC",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = RK1000_RATES,
		.formats = RK1000_FORMATS,},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = RK1000_RATES,
		.formats = RK1000_FORMATS,},
	.ops = {
		.hw_params = rk1000_codec_pcm_hw_params,
	},
	.dai_ops = {
		.digital_mute = rk1000_codec_mute,
		.set_fmt = rk1000_codec_set_dai_fmt,
		.set_sysclk = rk1000_codec_set_dai_sysclk,
	},
};
EXPORT_SYMBOL_GPL(rk1000_codec_dai);

static void rk1000_codec_work(struct work_struct *work)
{
	struct snd_soc_codec *codec =
		container_of(work, struct snd_soc_codec, delayed_work.work);
	rk1000_codec_dapm_event(codec, codec->dapm_state);
	DBG("%s[%d]\n",__FUNCTION__,__LINE__);
}

static int rk1000_codec_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
    DBG("%s[%d]\n",__FUNCTION__,__LINE__);
    
    #if 1
	/*disable speaker and save current speaker stauts*/
	rockchip_mux_api_set(SPK_IOMUX_PIN_NAME, SPK_IOMUX_PIN_DIR);/*speaker disable pin PF7*/
	GPIOSetPinDirection(SPK_CTRL_PIN,GPIO_OUT);
	GPIOSetPinLevel(SPK_CTRL_PIN,GPIO_LOW);
	//printk("Speaker status == %d!!\n",GPIOGetPinLevel(GPIOPortF_Pin7));
    #endif
	//rk1000_codec_dapm_event(codec, SNDRV_CTL_POWER_D3cold);
	#if OUT_CAPLESS
    rk1000_codec_write(codec,ACCELCODEC_R15, 0x01);
    #else
    rk1000_codec_write(codec,ACCELCODEC_R15, 0x31);
    #endif
    rk1000_codec_write(codec,ACCELCODEC_R0B, ASC_DEC_DISABLE|ASC_INT_DISABLE);
    gR0BReg = ASC_DEC_DISABLE|ASC_INT_DISABLE;
    #if 1
    rk1000_codec_write(codec,ACCELCODEC_R1E, 0xFC);
    rk1000_codec_write(codec,ACCELCODEC_R1D, 0x2a|ASC_PDSDL_ENABLE|ASC_PDBSTL_ENABLE|ASC_PDPGAL_ENABLE);  //setup Vmid and Vref, other module power down
    #if OUT_CAPLESS
    rk1000_codec_write(codec,ACCELCODEC_R1F, 0x5F);
    #else
    rk1000_codec_write(codec,ACCELCODEC_R1F, 0xDF);
    #endif
    #endif    
	return 0;
}

static int rk1000_codec_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	struct i2c_client *i2c;
	int i;
	u8 data[2];
	u16 *cache = codec->reg_cache;
    DBG("%s[%d]\n",__FUNCTION__,__LINE__);
	/* Sync reg_cache with the hardware */
	for (i = 0; i < ARRAY_SIZE(rk1000_codec_reg); i++) {
		data[0] = cache[i] & 0x00ff;
		data[1] = data[0];
		i2c = (struct i2c_client *)codec->control_data;
	    i2c->addr = (i2c->addr & 0x60)|i;
		codec->hw_write(codec->control_data, data, 2);
	}
   
	//rk1000_codec_dapm_event(codec, SNDRV_CTL_POWER_D3hot);
   
	/* charge rk1000 caps */
	if (codec->suspend_dapm_state == SNDRV_CTL_POWER_D0) {
		///rk1000_codec_dapm_event(codec, SNDRV_CTL_POWER_D2);
		codec->dapm_state = SNDRV_CTL_POWER_D0;
		schedule_delayed_work(&codec->delayed_work, msecs_to_jiffies(1000));
	}
	#if 1
	//2power down useless module
    rk1000_codec_write(codec,ACCELCODEC_R1D, 0x2a|ASC_PDSDL_ENABLE|ASC_PDBSTL_ENABLE|ASC_PDPGAL_ENABLE);  //setup Vmid and Vref, other module power down
    rk1000_codec_write(codec,ACCELCODEC_R1E, 0x40);  ///|ASC_PDASDML_ENABLE);
    #if OUT_CAPLESS
    rk1000_codec_write(codec,ACCELCODEC_R1F, 0x09|ASC_PDMICB_ENABLE|ASC_PDMIXM_ENABLE);
    #else
    rk1000_codec_write(codec,ACCELCODEC_R1F, 0x09|ASC_PDMICB_ENABLE|ASC_PDMIXM_ENABLE|ASC_PDPAM_ENABLE);
    #endif
    #endif
    //2other
    rk1000_codec_write(codec,ACCELCODEC_R0B, ASC_DEC_DISABLE|ASC_INT_ENABLE);
    gR0BReg = ASC_DEC_ENABLE|ASC_INT_ENABLE;  //ASC_DEC_DISABLE|ASC_INT_ENABLE;
		rk1000_codec_write(codec,ACCELCODEC_R15, 0xc1);
    #if 1
	/*set speaker stauts to  previously status*/
	rockchip_mux_api_set(SPK_IOMUX_PIN_NAME, SPK_IOMUX_PIN_DIR);/*speaker disable pin PF7*/
	GPIOSetPinDirection(SPK_CTRL_PIN,GPIO_OUT);
	GPIOSetPinLevel(SPK_CTRL_PIN,GPIO_HIGH);
	//printk("Speaker status == %d!!\n",GPIOGetPinLevel(GPIOPortF_Pin7));
    #endif
	return 0;
}

/*
 * initialise the RK1000 driver
 * register the mixer and dsp interfaces with the kernel
 */
static int rk1000_codec_init(struct snd_soc_device *socdev)
{
	struct snd_soc_codec *codec = socdev->codec;
	int ret = 0;
    DBG("%s[%d]\n",__FUNCTION__,__LINE__);
	codec->name = "RK1000_CODEC";
	codec->owner = THIS_MODULE;
	codec->read = rk1000_codec_read_reg_cache;
	codec->write = rk1000_codec_write;
	codec->dapm_event = rk1000_codec_dapm_event;
	codec->dai = &rk1000_codec_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = sizeof(rk1000_codec_reg);
	codec->reg_cache = kmemdup(rk1000_codec_reg, sizeof(rk1000_codec_reg), GFP_KERNEL);
	if (codec->reg_cache == NULL)
		return -ENOMEM;    
	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "rk1000: failed to create pcms\n");
		goto pcm_err;
	}
    #if 1
	/*disable speaker */
	rockchip_mux_api_set(SPK_IOMUX_PIN_NAME, SPK_IOMUX_PIN_DIR);
	GPIOSetPinDirection(SPK_CTRL_PIN,GPIO_OUT);
	GPIOSetPinLevel(SPK_CTRL_PIN,GPIO_LOW);
	//printk("Speaker status == %d!!\n",GPIOGetPinLevel(GPIOPortF_Pin7));
    #endif
	/* charge output caps */
	rk1000_codec_dapm_event(codec, SNDRV_CTL_POWER_D2);
	codec->dapm_state = SNDRV_CTL_POWER_D3hot;
	schedule_delayed_work(&codec->delayed_work, msecs_to_jiffies(1000));
    rk1000_codec_write(codec,ACCELCODEC_R1D, 0x00);
    rk1000_codec_write(codec,ACCELCODEC_R17, 0xFF);  //AOL
    rk1000_codec_write(codec,ACCELCODEC_R18, 0xFF);  //AOR
    rk1000_codec_write(codec,ACCELCODEC_R19, 0xFF);  //AOM

    rk1000_codec_write(codec,ACCELCODEC_R1F, 0xDF);
    mdelay(10);
    rk1000_codec_write(codec,ACCELCODEC_R1F, 0x5F);
    rk1000_codec_write(codec,ACCELCODEC_R19, 0x7F);  //AOM
    rk1000_codec_write(codec,ACCELCODEC_R15, 0xC1);//rk1000_codec_write(codec,ACCELCODEC_R15, 0xCD);//by Vincent Hsiung
    rk1000_codec_write(codec,ACCELCODEC_R1A, 0x1C);
    mdelay(100);
    rk1000_codec_write(codec,ACCELCODEC_R1F, 0x09);
    rk1000_codec_write(codec,ACCELCODEC_R1E, 0x00);
    mdelay(10);
    rk1000_codec_write(codec,ACCELCODEC_R1A, 0x14);
    rk1000_codec_write(codec,ACCELCODEC_R1D, 0xFE);
    rk1000_codec_write(codec,ACCELCODEC_R17, 0xBF);  //AOL
    rk1000_codec_write(codec,ACCELCODEC_R18, 0xBF);  //AOR
    rk1000_codec_write(codec,ACCELCODEC_R19, 0x7F);  //AOM
    rk1000_codec_write(codec,ACCELCODEC_R1F, 0xDF);

    //2soft mute
    rk1000_codec_write(codec,ACCELCODEC_R04, ASC_INT_MUTE_L|ASC_INT_MUTE_R|ASC_SIDETONE_L_OFF|ASC_SIDETONE_R_OFF);   //soft mute
    
    //2set default SR and clk
    rk1000_codec_write(codec,ACCELCODEC_R0A, ASC_USB_MODE|FREQ48kHz|ASC_CLKNODIV|ASC_CLK_DISABLE);
    gR0AReg = ASC_USB_MODE|FREQ48kHz|ASC_CLKNODIV|ASC_CLK_DISABLE;
    //2Config audio  interface
    rk1000_codec_write(codec,ACCELCODEC_R09, ASC_I2S_MODE|ASC_16BIT_MODE|ASC_NORMAL_LRCLK|ASC_LRSWAP_DISABLE|ASC_MASTER_MODE|ASC_NORMAL_BCLK);
    rk1000_codec_write(codec,ACCELCODEC_R00, ASC_HPF_ENABLE|ASC_DSM_MODE_DISABLE|ASC_SCRAMBLE_ENABLE|ASC_DITHER_ENABLE|ASC_BCLKDIV_8);  //BCLK div 8
    //2volume,input,outpu
    rk1000_codec_write(codec,ACCELCODEC_R05, 0x0e);
    rk1000_codec_write(codec,ACCELCODEC_R06, 0x42);
    rk1000_codec_write(codec,ACCELCODEC_R07, 0x0e);
    rk1000_codec_write(codec,ACCELCODEC_R08, 0x42);
    
    rk1000_codec_write(codec,ACCELCODEC_R0C, 0x10|ASC_INPUT_VOL_0DB|ASC_INPUT_MUTE);   //LIL
    rk1000_codec_write(codec,ACCELCODEC_R0D, 0x10|ASC_INPUT_VOL_0DB);   //LIR
    rk1000_codec_write(codec,ACCELCODEC_R0E, 0x10|ASC_INPUT_VOL_0DB);   //MIC
    rk1000_codec_write(codec,ACCELCODEC_R12, 0x4c|ASC_MIC_INPUT|ASC_MIC_BOOST_20DB);  //mic input and boost 20dB
    rk1000_codec_write(codec,ACCELCODEC_R13, ASC_LPGAMX_DISABLE|ASC_ALMX_DISABLE|((LINE_2_MIXER_GAIN & 0x7) << 4)|0x0);
    rk1000_codec_write(codec,ACCELCODEC_R14, ASC_RPGAMX_DISABLE|ASC_ARMX_DISABLE|((LINE_2_MIXER_GAIN & 0x7) << 4)|0x0);
    gR1314Reg = ASC_RPGAMX_DISABLE|ASC_ARMX_DISABLE|((LINE_2_MIXER_GAIN & 0x7) << 4)|0x0;

    //2other
    rk1000_codec_write(codec,ACCELCODEC_R0B, ASC_DEC_DISABLE|ASC_INT_DISABLE);  //0x00
    gR0BReg = ASC_DEC_DISABLE|ASC_INT_DISABLE;
    rk1000_codec_write(codec,ACCELCODEC_R15, \
                    0x01|ASC_RLPFMX_DISABLE|ASC_LLPFMX_DISABLE|ASC_LDAMX_DISABLE|ASC_RDAMX_DISABLE|ASC_LSCF_ACTIVE|ASC_RSCF_ACTIVE);  //0x3c
    rk1000_codec_write(codec,ACCELCODEC_R1B, 0x32);
    rk1000_codec_write(codec,ACCELCODEC_R1C, ASC_DEM_ENABLE);  ///0x00);  //use default value
    
    ///dac mode
    rk1000_codec_write(codec,ACCELCODEC_R17, 0xBF);  //AOL  音量最低
    rk1000_codec_write(codec,ACCELCODEC_R18, 0xBF);  //AOR
        
    //2power down useless module
    rk1000_codec_write(codec,ACCELCODEC_R1D, 0x2a|ASC_PDSDL_ENABLE|ASC_PDBSTL_ENABLE|ASC_PDPGAL_ENABLE);  //setup Vmid and Vref, other module power down
    rk1000_codec_write(codec,ACCELCODEC_R1E, 0x40|ASC_PDASDML_ENABLE);
    #if OUT_CAPLESS
    rk1000_codec_write(codec,ACCELCODEC_R1F, 0x09|ASC_PDMICB_ENABLE|ASC_PDMIXM_ENABLE);
    #else
    rk1000_codec_write(codec,ACCELCODEC_R1F, 0x09|ASC_PDMICB_ENABLE|ASC_PDMIXM_ENABLE|ASC_PDPAM_ENABLE);
    #endif

    //2other
   rk1000_codec_write(codec,ACCELCODEC_R0B, ASC_DEC_DISABLE|ASC_INT_ENABLE);
    gR0BReg = ASC_DEC_ENABLE|ASC_INT_ENABLE;  //ASC_DEC_DISABLE|ASC_INT_ENABLE;
    rk1000_codec_write(codec,ACCELCODEC_R15, 0xC1);//rk1000_codec_write(codec,ACCELCODEC_R15, 0xCD);//by Vincent Hsiung
    rk1000_codec_write(codec,ACCELCODEC_R0C, 0x10|ASC_INPUT_VOL_0DB|ASC_INPUT_MUTE);   //LIL
    rk1000_codec_write(codec,ACCELCODEC_R0D, 0x10|ASC_INPUT_VOL_0DB);   //LIR
    rk1000_codec_write(codec,ACCELCODEC_R0E, 0x10|ASC_INPUT_VOL_0DB);   //MIC
    rk1000_codec_write(codec,ACCELCODEC_R12, 0x4c|ASC_MIC_INPUT|ASC_MIC_BOOST_20DB);  //mic input and boost 20dB
    rk1000_codec_write(codec,ACCELCODEC_R13, 0x00);
    rk1000_codec_write(codec,ACCELCODEC_R14, 0x00);
    gR1314Reg = 0x00;
    rk1000_codec_write(codec,ACCELCODEC_R1C, ASC_DEM_ENABLE);  //0x00);  //use default value

  //  rk1000_codec_write(codec,ACCELCODEC_R17, gVolReg|ASC_CROSSZERO_EN|ASC_OUTPUT_ACTIVE);  //AOL
  //  rk1000_codec_write(codec,ACCELCODEC_R18, gVolReg|ASC_CROSSZERO_EN|ASC_OUTPUT_ACTIVE);  //AOR

	rk1000_codec_add_controls(codec);
	rk1000_codec_add_widgets(codec);
	ret = snd_soc_register_card(socdev);
	if (ret < 0) {
		printk(KERN_ERR "rk1000: failed to register card\n");
		goto card_err;
	}
	return ret;

card_err:
	  snd_soc_free_pcms(socdev);
	  snd_soc_dapm_free(socdev);
pcm_err:
	  kfree(codec->reg_cache);
	  return ret;
}

/* If the i2c layer weren't so broken, we could pass this kind of data
   around */
//static struct snd_soc_device *rk1000_codec_socdev;

#if defined (CONFIG_I2C) || defined (CONFIG_I2C_MODULE)

static struct i2c_driver rk1000_codec_i2c_driver;
static struct i2c_client client_template;

static int rk1000_codec_set_probe(struct i2c_adapter *adap, int addr, int kind)
{
	struct snd_soc_device *socdev = rk1000_codec_socdev;
	struct rk1000_codec_setup_data *setup = socdev->codec_data;
	struct snd_soc_codec *codec = socdev->codec;
	struct i2c_client *i2c;
	int ret;
    DBG("%s[%d]\n",__FUNCTION__,__LINE__);
	if (addr != setup->i2c_address)
		return -ENODEV;
    
	client_template.adapter = adap;
	client_template.addr = addr;
    
	i2c = kmemdup(&client_template, sizeof(client_template), GFP_KERNEL);
	if (i2c == NULL) {
		kfree(codec);
		return -ENOMEM;
	}
	i2c->mode = DIRECTMODE;
	i2c->Channel = I2C_CH0;
	i2c->speed = 150;
	i2c->addressBit=I2C_7BIT_ADDRESS_8BIT_REG;
	i2c_set_clientdata(i2c, codec);
	codec->control_data = i2c;
    
	ret = i2c_attach_client(i2c);
	if (ret < 0) {
		err("failed to attach codec at addr %x\n", addr);
		goto err;
	}
    
	ret = rk1000_codec_init(socdev);
	if (ret < 0) {
	err("failed to initialise RK1000\n");
		goto err;
	}
	
	return ret;
    
err:
	kfree(codec);
	kfree(i2c);
	return ret;
}

static int rk1000_codec_i2c_detach(struct i2c_client *client)
{
	struct snd_soc_codec *codec = i2c_get_clientdata(client);
	i2c_detach_client(client);
	kfree(codec->reg_cache);
	kfree(client);
	return 0;
}
static void rk1000_codec_shutdown(struct i2c_client *client)
{
	struct snd_soc_codec *codec = i2c_get_clientdata(client);
     	DBG("%s[%d]\n",__FUNCTION__,__LINE__);
	#if OUT_CAPLESS
    	rk1000_codec_write(codec,ACCELCODEC_R1A, ASC_VMDSCL_SLOWEST|ASC_MICBIAS_09|ASC_L2M_DISABLE|ASC_R2M_DISABLE|ASC_CAPLESS_ENABLE); //0x90
    #else
    	rk1000_codec_write(codec,ACCELCODEC_R1A, ASC_VMDSCL_SLOWEST|ASC_MICBIAS_09|ASC_L2M_DISABLE|ASC_R2M_DISABLE|ASC_CAPLESS_DISABLE);
    #endif
    	rk1000_codec_write(codec,ACCELCODEC_R17, 0xFF);
    	rk1000_codec_write(codec,ACCELCODEC_R18, 0xFF);
    	rk1000_codec_write(codec,ACCELCODEC_R1E, 0xFF);
    	rk1000_codec_write(codec,ACCELCODEC_R1F, 0xFF);
    	rk1000_codec_write(codec,ACCELCODEC_R1D, 0xFF);	
		mdelay(10);
		#if 1
		/*set speaker stauts to  previously status*/
		rockchip_mux_api_set(SPK_IOMUX_PIN_NAME, SPK_IOMUX_PIN_DIR);/*speaker disable pin PF7*/
		GPIOSetPinDirection(SPK_CTRL_PIN,GPIO_OUT);
		GPIOSetPinLevel(SPK_CTRL_PIN,GPIO_HIGH);
		//printk("Speaker status == %d!!\n",GPIOGetPinLevel(GPIOPortF_Pin7));
    	#endif
}
static int rk1000_codec_i2c_attach(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, rk1000_codec_set_probe);
}

/* corgi i2c codec control layer */
static struct i2c_driver rk1000_codec_i2c_driver = {
	.driver = {
		.name = "RK1000 I2C Codec",
		.owner = THIS_MODULE,
	},
	.id =             I2C_DRIVERID_RK1000_CODEC,
	.attach_adapter = rk1000_codec_i2c_attach,
	.detach_client =  rk1000_codec_i2c_detach,
	.command =        NULL,
	.shutdown = rk1000_codec_shutdown,
};

static struct i2c_client client_template = {
	.name =   "RK1000_CODEC",
	.driver = &rk1000_codec_i2c_driver,
};
#endif

static int rk1000_codec_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct rk1000_codec_setup_data *setup = socdev->codec_data;
	struct snd_soc_codec *codec;
	struct rk1000_codec_priv *rk1000;
	int ret = 0;
    
    DBG("%s[%d]\n",__FUNCTION__,__LINE__);
    
	info("RK1000 Audio Codec %s", RK1000_VERSION);
	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;
    
	rk1000 = kzalloc(sizeof(struct rk1000_codec_priv), GFP_KERNEL);
	if (rk1000 == NULL) {
		kfree(codec);
		return -ENOMEM;
	}
    
	codec->private_data = rk1000;
	socdev->codec = codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);
	rk1000_codec_socdev = socdev;
	INIT_DELAYED_WORK(&codec->delayed_work, rk1000_codec_work);
	
#if defined (CONFIG_I2C) || defined (CONFIG_I2C_MODULE)
	if (setup->i2c_address) {
		normal_i2c[0] = setup->i2c_address;
		codec->hw_write = (hw_write_t)i2c_master_send;
		codec->hw_read = (hw_read_t)i2c_master_recv; //by Vincent Hsiung
		ret = i2c_add_driver(&rk1000_codec_i2c_driver);
		if (ret != 0)
			printk(KERN_ERR "can't add i2c driver");
	}
#else
		/* Add other interfaces here */
#endif

	return ret;
}

/*
 * This function forces any delayed work to be queued and run.
 */
static int run_delayed_work(struct delayed_work *dwork)
{
	int ret;
    DBG("%s[%d]\n",__FUNCTION__,__LINE__);
	/* cancel any work waiting to be queued. */
	ret = cancel_delayed_work(dwork);
    
	/* if there was any work waiting then we run it now and
	 * wait for it's completion */
	if (ret) {
		schedule_delayed_work(dwork, 0);
		flush_scheduled_work();
	}
	return ret;
}

/* power down chip */
static int rk1000_codec_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
    DBG("%s[%d]\n",__FUNCTION__,__LINE__);
	if (codec->control_data)
		rk1000_codec_dapm_event(codec, SNDRV_CTL_POWER_D3cold);
	run_delayed_work(&codec->delayed_work);
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
#if defined (CONFIG_I2C) || defined (CONFIG_I2C_MODULE)
	i2c_del_driver(&rk1000_codec_i2c_driver);
#endif
	kfree(codec->private_data);
	kfree(codec);
   
	return 0;
}

struct snd_soc_codec_device soc_codec_dev_rk1000 = {
	.probe = 	rk1000_codec_probe,
	.remove = 	rk1000_codec_remove,
	.suspend = 	rk1000_codec_suspend,
	.resume =	rk1000_codec_resume,
};

EXPORT_SYMBOL_GPL(soc_codec_dev_rk1000);

MODULE_DESCRIPTION("ASoC RK1000 CODEC driver");
MODULE_AUTHOR("lhh lhh@rock-chips.com");
MODULE_LICENSE("GPL");

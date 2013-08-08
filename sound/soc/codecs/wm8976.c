/*
 * wm8976.c  --  WM8976 ALSA Soc Audio driver
 *
 * Copyright 2006 Wolfson Microelectronics PLC.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

//#define DEBUG

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <asm/div64.h>

//#include "../../../drivers/urbetter/power_gpio.h"

#include "wm8976.h"

#if 0
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/freezer.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/utsname.h>
#include <scsi/scsi.h>
#include <scsi/scsi_cmnd.h>
#include <scsi/scsi_device.h>
#endif
#include <linux/proc_fs.h>
#define AUDIO_NAME "wm8976"
#define WM8976_VERSION "0.4"

//#define CONFIG_SND_DEBUG
//#undef CONFIG_SND_DEBUG
#ifdef CONFIG_SND_DEBUG
#define s3cdbg(x...) printk("[wm8976 drv]"x)
#define BDG(x...)  	printk("=========[wm8976 drv]  %s(); line=%d\n", __func__, __LINE__)
#else
#define s3cdbg(x...) do { } while(0)
#define BDG(x...) do { } while(0)
#endif

struct snd_soc_codec_device soc_codec_dev_wm8976;
struct snd_soc_codec *g_codec;
static int wm8976_flag = 1;
static int wm8976_set_register_sound();
static int wm8976_set_register(struct snd_soc_dai *dai);
unsigned int reg_buf[58] ={0};

/*
 * wm8976 register cache
 * We can't read the WM8976 register space when we are
 * using 2 wire for device control, so we cache them instead.
 */
#if 0
static const u16 wm8976_reg[WM8976_CACHEREGNUM] = {

    0x0000, 0x0000, 0x0000, 0x0000,
    0x0050, 0x0000, 0x0140, 0x0000,
    0x0000, 0x0000, 0x0000, 0x00ff,
    0x00ff, 0x0000, 0x0100, 0x01ff,             //r15 bit 8 set 1 zengsiling
    0x00ff, 0x0000, 0x012c, 0x002c,
    0x002c, 0x002c, 0x002c, 0x0000,
    0x0032, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000,
    0x0038, 0x000b, 0x0032, 0x0000,
    0x0008, 0x000c, 0x0093, 0x00e9,
    0x0000, 0x0000, 0x0000, 0x0000,
    0x0033, 0x0010, 0x0010, 0x0100,
    0x0100, 0x0002, 0x0001, 0x0001,
    0x0032, 0x0032, 0x0039, 0x0039,
    
    0x0001, 0x0001,
    
};
#endif
/****wangyulu***********/
#if 1
static const u16 wm8976_reg[WM8976_CACHEREGNUM] = {
//  R0      R1      R2      R3
    0x0000, 0x0000, 0x0000, 0x0000,
//  R4      R5      R6      R7
    0x0050, 0x0000, 0x0140, 0x0000,
//  R8      R9      R10     R11
    0x0000, 0x0000, 0x0000, 0x00ff,
//  R12     R13     R14     R15
    0x00ff, 0x0000, 0x0100, 0x01ff,             //r15 bit 8 set 1 zengsiling
//  R16     R17     R18     R19
    0x00ff, 0x0000, 0x012c, 0x002c,
//  R20     R21     R22     R23
    0x002c, 0x002c, 0x002c, 0x0000,
//  R24     R25     R26     R27
    0x0032, 0x0000, 0x0000, 0x0000,
//  R28     R29     R30     R31
    0x0000, 0x0000, 0x0000, 0x0000,
//  R32     R33     R34     R35
    0x0038, 0x000b, 0x0032, 0x0000,
//  R36     R37     R38     R39
    0x0008, 0x000c, 0x0093, 0x00e9,
//  R40     R41     R42     43
    0x0000, 0x0000, 0x0000, 0x0000,
//  R44     R45     R46     R47
    0x0033, 0x0010, 0x0010, 0x0100,
//  R48     R49     R50     R51
    0x0100, 0x0002, 0x0001, 0x0001,
//  R52     R53     R54     R55
    0x0032, 0x0032, 0x0039, 0x0039,
//  R56     R57
    0x0001, 0x0001,
};
#endif
/**************wangyulu add down********/
/********* creat mode8976 in proc ******/
int sound8976_galley_select_flag = 0;

static struct proc_dir_entry * s_proc = NULL; 
static int modem_switch_writeproc(struct file *file,const char *buffer,
                           unsigned long count, void *data)
{
	 int value; 
	 value = 0; 
	 sscanf(buffer, "%d", &value);
	switch(value)
	{
		case 0:
			wm8976_set_register_sound(value);
			break;
		case 1:
			wm8976_set_register_sound(value);
			break;
		default:
			printk("************************urbetter sound8976_galley_select_flag ==null***********************\n");
			break;
	}
	return count;
}
static int modem_switch_readproc(char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	int len;
	len = sprintf(page, "%d\n", sound8976_galley_select_flag==0?0:(sound8976_galley_select_flag==1?1:(sound8976_galley_select_flag==2?2:3))); //wangyulu
	if (off + count >= len)
		*eof = 1;	
	if (len < off)
			return 0;
	*start = page + off;
	return ((count < len - off) ? count : len - off);
}
//******wangyulu  add  up****/
/*
 * read wm8976 register cache
 */
static inline unsigned int wm8976_read_reg_cache(struct snd_soc_codec  *codec,
	unsigned int reg)
{
	u16 *cache = codec->reg_cache;
	if (reg == WM8976_RESET)
		return 0;
	if (reg >= WM8976_CACHEREGNUM)
		return -1;
	   //printk("************************wangyulu 1************************\n");
	return cache[reg];
}

/*
 * write wm8976 register cache
 */
static inline void wm8976_write_reg_cache(struct snd_soc_codec  *codec,
	u16 reg, unsigned int value)
{
	u16 *cache = codec->reg_cache;
	if (reg >= WM8976_CACHEREGNUM)
		return;
	
		cache[reg] = value;
}

/*
 * write to the WM8976 register space
 */
int wm8976_write(struct snd_soc_codec  *codec, unsigned int reg,
	unsigned int value)
{
	u8 data[2];
	if (sound8976_galley_select_flag!=1) {
		/* data is
		 *   D15..D9 WM8976 register offset
		 *   D8...D0 register data
		 */
		data[0] = (reg << 1) | ((value >> 8) & 0x0001);
		data[1] = value & 0x00ff;
	     
		wm8976_write_reg_cache (codec, reg, value);
		if (codec->hw_write(codec->control_data, data, 2) == 2)
			return 0;
		else
			return -1;
	} else {
		return 0;
	}
}
EXPORT_SYMBOL_GPL(wm8976_write);
#define wm8976_reset(c)	wm8976_write(c, WM8976_RESET, 0)

static const char *wm8976_companding[] = { "Off", "NC", "u-law", "A-law" };
//static const char *wm8976_deemp[] = { "None", "32kHz", "44.1kHz", "48kHz" };
static const char *wm8976_eqmode[] = { "Capture", "Playback" };
static const char *wm8976_bw[] = {"Narrow", "Wide" };
static const char *wm8976_eq1[] = {"80Hz", "105Hz", "135Hz", "175Hz" };
static const char *wm8976_eq2[] = {"230Hz", "300Hz", "385Hz", "500Hz" };
static const char *wm8976_eq3[] = {"650Hz", "850Hz", "1.1kHz", "1.4kHz" };
static const char *wm8976_eq4[] = {"1.8kHz", "2.4kHz", "3.2kHz", "4.1kHz" };
static const char *wm8976_eq5[] = {"5.3kHz", "6.9kHz", "9kHz", "11.7kHz" };
static const char *wm8976_alc[] = {"ALC", "Limiter" };

static const struct soc_enum wm8976_enum[] = {
	SOC_ENUM_SINGLE(WM8976_COMP, 1, 4, wm8976_companding),	/* adc */
	SOC_ENUM_SINGLE(WM8976_COMP, 3, 4, wm8976_companding),	/* dac */

	SOC_ENUM_SINGLE(WM8976_EQ1,  8, 2, wm8976_eqmode),
	SOC_ENUM_SINGLE(WM8976_EQ1,  5, 4, wm8976_eq1),

	SOC_ENUM_SINGLE(WM8976_EQ2,  8, 2, wm8976_bw),
	SOC_ENUM_SINGLE(WM8976_EQ2,  5, 4, wm8976_eq2),

	SOC_ENUM_SINGLE(WM8976_EQ3,  8, 2, wm8976_bw),
	SOC_ENUM_SINGLE(WM8976_EQ3,  5, 4, wm8976_eq3),

	SOC_ENUM_SINGLE(WM8976_EQ4,  8, 2, wm8976_bw),
	SOC_ENUM_SINGLE(WM8976_EQ4,  5, 4, wm8976_eq4),

	SOC_ENUM_SINGLE(WM8976_EQ5,  8, 2, wm8976_bw),
	SOC_ENUM_SINGLE(WM8976_EQ5,  5, 4, wm8976_eq5),
	
	SOC_ENUM_SINGLE(WM8976_ALC3,  8, 2, wm8976_alc),
};

static const struct snd_kcontrol_new wm8976_snd_controls[] = 
{

	SOC_SINGLE("Digital Loopback Switch", WM8976_COMP, 0, 1, 0),

	SOC_ENUM("DAC Companding", wm8976_enum[1]),
	SOC_ENUM("ADC Companding", wm8976_enum[0]),

	SOC_SINGLE("High Pass Filter Switch", WM8976_ADC, 8, 1, 0),	
	SOC_SINGLE("High Pass Cut Off", WM8976_ADC, 4, 7, 0),	
	SOC_DOUBLE("ADC Inversion Switch", WM8976_ADC, 0, 1, 1, 0),
	SOC_SINGLE("Capture Volume", WM8976_ADCVOL,  0, 255, 0),
	SOC_SINGLE("Capture Boost(+20dB)", WM8976_ADCBOOST, 8, 1, 0),
	SOC_SINGLE("Capture PGA ZC Switch", WM8976_INPPGA,  7, 1, 0),
	SOC_SINGLE("Capture PGA Volume", WM8976_INPPGA,  0, 63, 0),

        SOC_SINGLE("ALC Enable Switch", WM8976_ALC1,  8, 1, 0),
        SOC_SINGLE("ALC Capture Max Gain", WM8976_ALC1,  3, 7, 0),
        SOC_SINGLE("ALC Capture Min Gain", WM8976_ALC1,  0, 7, 0),
        SOC_SINGLE("ALC Capture ZC Switch", WM8976_ALC2,  8, 1, 0),
        SOC_SINGLE("ALC Capture Hold", WM8976_ALC2,  4, 7, 0),
        SOC_SINGLE("ALC Capture Target", WM8976_ALC2,  0, 15, 0),
        SOC_ENUM("ALC Capture Mode", wm8976_enum[12]),
        SOC_SINGLE("ALC Capture Decay", WM8976_ALC3,  4, 15, 0),
        SOC_SINGLE("ALC Capture Attack", WM8976_ALC3,  0, 15, 0),
        SOC_SINGLE("ALC Capture Noise Gate Switch", WM8976_NGATE,  3, 1, 0),
        SOC_SINGLE("ALC Capture Noise Gate Threshold", WM8976_NGATE,  0, 7, 0),

	SOC_ENUM("Eq-3D Mode Switch", wm8976_enum[2]),	
	SOC_ENUM("Eq1 Cut-Off Frequency", wm8976_enum[3]),	
	SOC_SINGLE("Eq1 Volume", WM8976_EQ1,  0, 31, 1),	
	
	SOC_ENUM("Eq2 BandWidth Switch", wm8976_enum[4]),	
	SOC_ENUM("Eq2 Centre Frequency", wm8976_enum[5]),	
	SOC_SINGLE("Eq2 Volume", WM8976_EQ2,  0, 31, 1),

	SOC_ENUM("Eq3 BandWidth Switch", wm8976_enum[6]),	
	SOC_ENUM("Eq3 Centre Frequency", wm8976_enum[7]),	
	SOC_SINGLE("Eq3 Volume", WM8976_EQ3,  0, 31, 1),

	SOC_ENUM("Eq4 BandWidth Switch", wm8976_enum[8]),	
	SOC_ENUM("Eq4 Centre Frequency", wm8976_enum[9]),	
	SOC_SINGLE("Eq4 Volume", WM8976_EQ4,  0, 31, 1),
	
	SOC_ENUM("Eq5 BandWidth Switch", wm8976_enum[10]),	
	SOC_ENUM("Eq5 Centre Frequency", wm8976_enum[11]),	
	SOC_SINGLE("Eq5 Volume", WM8976_EQ5,  0, 31, 1),
	SOC_DOUBLE_R("PCM Playback Volume", WM8976_DACVOLL, WM8976_DACVOLR, 0, 127, 0),

	SOC_DOUBLE_R("Headphone Playback Switch", WM8976_HPVOLL,  WM8976_HPVOLR, 6, 1, 1),
	SOC_DOUBLE_R("Headphone Playback Volume", WM8976_HPVOLL,  WM8976_HPVOLR, 0, 62, 0),

	SOC_DOUBLE_R("Speaker Playback Switch", WM8976_SPKVOLL,  WM8976_SPKVOLR, 6, 1, 1),
	SOC_DOUBLE_R("Speaker Playback Volume", WM8976_SPKVOLL,  WM8976_SPKVOLR, 0, 62, 0),

};

/* add non dapm controls */
static int wm8976_add_controls(struct snd_soc_codec *codec)
{
	int err, i;

	//BDG();
	for (i = 0; i < ARRAY_SIZE(wm8976_snd_controls); i++) {
		err = snd_ctl_add(codec->card, snd_soc_cnew(&wm8976_snd_controls[i],codec, NULL));
		if (err < 0)
			return err;
	}

	return 0;
}

/* Left Output Mixer */
static const struct snd_kcontrol_new wm8976_left_mixer_controls[] = {
	SOC_DAPM_SINGLE("Left Playback Switch", WM8976_OUTPUT, 6, 1, 0),
	SOC_DAPM_SINGLE("Right Playback Switch", WM8976_MIXL, 0, 1, 0),
	SOC_DAPM_SINGLE("Bypass Playback Switch", WM8976_MIXL, 1, 1, 0),
	SOC_DAPM_SINGLE("Left Aux Switch", WM8976_MIXL, 5, 1, 0),
};

/* Right Output Mixer */
static const struct snd_kcontrol_new wm8976_right_mixer_controls[] = {
	SOC_DAPM_SINGLE("Left Playback Switch", WM8976_OUTPUT, 5, 1, 0),
	SOC_DAPM_SINGLE("Right Playback Switch", WM8976_MIXR, 0, 1, 0),
	SOC_DAPM_SINGLE("Right Aux Switch", WM8976_MIXR, 5, 1, 0),
};

/* Out4 Mixer */
static const struct snd_kcontrol_new wm8976_out4_mixer_controls[] = {

	SOC_DAPM_SINGLE("VMID", WM8976_MONOMIX, 6, 1, 0),
	SOC_DAPM_SINGLE("Out4 LeftMixer Switch", WM8976_MONOMIX, 4, 1, 0),
	SOC_DAPM_SINGLE("Out4 LeftDac Switch", WM8976_MONOMIX, 3, 1, 0),
	SOC_DAPM_SINGLE("Out4 RightMixer Switch", WM8976_MONOMIX, 1, 1, 0),	
	SOC_DAPM_SINGLE("Out4 RightDac Switch", WM8976_MONOMIX, 0, 1, 0),

};

/* Out3 Mixer */
static const struct snd_kcontrol_new wm8976_out3_mixer_controls[] = {

	SOC_DAPM_SINGLE("VMID", WM8976_OUT3MIX, 6, 1, 0),
	SOC_DAPM_SINGLE("Out3 Out4Mixer Switch", WM8976_OUT3MIX, 3, 1, 0),
	SOC_DAPM_SINGLE("Out3 BypassADC Switch", WM8976_OUT3MIX, 2, 1, 0),
	SOC_DAPM_SINGLE("Out3 LeftMixer Switch", WM8976_OUT3MIX, 1, 1, 0),
	SOC_DAPM_SINGLE("Out3 LeftDac Switch", WM8976_OUT3MIX, 0, 1, 0),

};

static const struct snd_kcontrol_new wm8976_boost_controls[] = {
	SOC_DAPM_SINGLE("Mic PGA Switch", WM8976_INPPGA,  6, 1, 1), 
	SOC_DAPM_SINGLE("AuxL Volume", WM8976_ADCBOOST, 0, 7, 0),
	SOC_DAPM_SINGLE("L2 Volume", WM8976_ADCBOOST, 4, 7, 0),
};

static const struct snd_kcontrol_new wm8976_micpga_controls[] = {
	SOC_DAPM_SINGLE("L2 Switch", WM8976_INPUT, 2, 1, 0),
	SOC_DAPM_SINGLE("MICN Switch", WM8976_INPUT, 1, 1, 0),
	SOC_DAPM_SINGLE("MICP Switch", WM8976_INPUT, 0, 1, 0),
};

static const struct snd_soc_dapm_widget wm8976_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("MICN"),
	SND_SOC_DAPM_INPUT("MICP"),
	SND_SOC_DAPM_INPUT("AUXL"),
	SND_SOC_DAPM_INPUT("AUXR"),
	SND_SOC_DAPM_INPUT("L2"),
	
	SND_SOC_DAPM_MICBIAS("Mic Bias", WM8976_POWER1, 4, 0),

	
	SND_SOC_DAPM_MIXER("Left Mixer", WM8976_POWER3, 2, 0,
		&wm8976_left_mixer_controls[0], ARRAY_SIZE(wm8976_left_mixer_controls)),
	SND_SOC_DAPM_PGA("Left Out 1", WM8976_POWER2, 7, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Left Out 2", WM8976_POWER3, 6, 0, NULL, 0),
	SND_SOC_DAPM_DAC("Left DAC", "Left HiFi Playback", WM8976_POWER3, 0, 0),

	SND_SOC_DAPM_MIXER("Right Mixer", WM8976_POWER3, 3, 0,
		&wm8976_right_mixer_controls[0], ARRAY_SIZE(wm8976_right_mixer_controls)),
	SND_SOC_DAPM_PGA("Right Out 1", WM8976_POWER2, 8, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Right Out 2", WM8976_POWER3, 5, 0, NULL, 0),
	SND_SOC_DAPM_DAC("Right DAC", "Right HiFi Playback", WM8976_POWER3, 1, 0),

	SND_SOC_DAPM_ADC("ADC", "HiFi Capture", WM8976_POWER2, 0, 0),

     //PGA out control 
     //lulu   SND_SOC_DAPM_PGA("Mic PGA", WM8976_POWER2, 2, 0,
     //lulu   &wm8976_micpga_controls[0],ARRAY_SIZE(wm8976_micpga_controls)),//input pga is disable
	SND_SOC_DAPM_MIXER("Mic PGA", WM8976_POWER2, 2, 0,
	&wm8976_micpga_controls[0],ARRAY_SIZE(wm8976_micpga_controls)),	

	SND_SOC_DAPM_MIXER("Boost Mixer", WM8976_POWER2, 4, 0,
		&wm8976_boost_controls[0], ARRAY_SIZE(wm8976_boost_controls)),
	
	SND_SOC_DAPM_OUTPUT("LOUT1"),
	SND_SOC_DAPM_OUTPUT("ROUT1"),
	SND_SOC_DAPM_OUTPUT("LOUT2"),
	SND_SOC_DAPM_OUTPUT("ROUT2"),
	
    
	SND_SOC_DAPM_MIXER("Out3 Mixer", WM8976_POWER1, 6, 0,
		&wm8976_out3_mixer_controls[0], ARRAY_SIZE(wm8976_out3_mixer_controls)),	
	SND_SOC_DAPM_PGA("Out 3", WM8976_POWER1, 7, 0, NULL, 0),
	SND_SOC_DAPM_OUTPUT("OUT3"),
	
	SND_SOC_DAPM_MIXER("Out4 Mixer", WM8976_POWER1, 7, 0,
		&wm8976_out4_mixer_controls[0], ARRAY_SIZE(wm8976_out4_mixer_controls)),
	SND_SOC_DAPM_PGA("Out 4", WM8976_POWER3, 8, 0, NULL, 0),
	SND_SOC_DAPM_OUTPUT("OUT4"),
    
};

static const struct snd_soc_dapm_route audio_map[] = {
	/* left mixer */
	{"Left Mixer", "Left Playback Switch", "Left DAC"},
	{"Left Mixer", "Right Playback Switch", "Right DAC"},
	{"Left Mixer", "Bypass Playback Switch", "Boost Mixer"},
	{"Left Mixer", "Left Aux Switch", "AUXL"},

	/* right mixer */
	{"Right Mixer", "Right Playback Switch", "Right DAC"},
	{"Right Mixer", "Left Playback Switch", "Left DAC"},
	{"Right Mixer", "Right Aux Switch", "AUXR"},
	
	/* left out */
	{"Left Out 1", NULL, "Left Mixer"},
	{"Left Out 2", NULL, "Left Mixer"},
	{"LOUT1", NULL, "Left Out 1"},
	{"LOUT2", NULL, "Left Out 2"},
	
	/* right out */
	{"Right Out 1", NULL, "Right Mixer"},
	{"Right Out 2", NULL, "Right Mixer"},
	{"ROUT1", NULL, "Right Out 1"},
	{"ROUT2", NULL, "Right Out 2"},

	/* Microphone PGA */
	{"Mic PGA", "MICN Switch", "MICN"},
	{"Mic PGA", "MICP Switch", "MICP"},
	{"Mic PGA", "L2 Switch", "L2" },
	
	/* Boost Mixer */
	{"Boost Mixer", "Mic PGA Switch", "Mic PGA"},
	{"Boost Mixer", "AuxL Volume", "AUXL"},
	{"Boost Mixer", "L2 Volume", "L2"},
	
	{"ADC", NULL, "Boost Mixer"},



	/* out 3 */
	{"Out3 Mixer", "VMID", "Out4 Mixer"},
	{"Out3 Mixer", "Out3 Out4Mixer Switch", "Out4 Mixer"},
	{"Out3 Mixer", "Out3 BypassADC Switch", "ADC"},
	{"Out3 Mixer", "Out3 LeftMixer Switch", "Left Mixer"},
	{"Out3 Mixer", "Out3 LeftDac Switch", "Left DAC"},
	{"Out 3", NULL, "Out3 Mixer"},
	{"OUT3", NULL, "Out 3"},
	
	/* out 4 */
	{"Out4 Mixer", "VMID", "Out3 Mixer"},
	{"Out4 Mixer", "Out4 LeftMixer Switch", "Left Mixer"},
	{"Out4 Mixer", "Out4 LeftDac Switch", "Left DAC"},
	{"Out4 Mixer", "Out4 RightMixer Switch", "Right Mixer"},	
	{"Out4 Mixer", "Out4 RightDac Switch", "Right DAC"},
	{"Out 4", NULL, "Out4 Mixer"},
	{"OUT4", NULL, "Out 4"},
};

static int wm8976_add_widgets(struct snd_soc_codec *codec)
{
     printk("************************urbetter wm8976_add_widgets************************\n");
	snd_soc_dapm_new_controls(codec, wm8976_dapm_widgets,
				  ARRAY_SIZE(wm8976_dapm_widgets));

	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_new_widgets(codec);

	return 0;
}

struct _pll_div {
	unsigned int pre:4; /* prescale - 1 */
	unsigned int n:4;
	unsigned int k;
};

static struct _pll_div pll_div;

/* The size in bits of the pll divide multiplied by 10
 * to allow rounding later */
#define FIXED_PLL_SIZE ((1 << 24) * 10)

static void pll_factors(unsigned int target, unsigned int source)
{
	unsigned long long Kpart;
	unsigned int K, Ndiv, Nmod;


	Ndiv = target / source;
	if (Ndiv < 6) {
		source >>= 1;
		pll_div.pre = 1;
		Ndiv = target / source;
	} else
		pll_div.pre = 0;

	if ((Ndiv < 6) || (Ndiv > 12))
		printk(KERN_WARNING"WM8976 N value outwith recommended range! N = %d\n",Ndiv);

	pll_div.n = Ndiv;
	Nmod = target % source;
	Kpart = FIXED_PLL_SIZE * (long long)Nmod;

	do_div(Kpart, source);

	K = Kpart & 0xFFFFFFFF;

	/* Check if we need to round */
	if ((K % 10) >= 5)
		K += 5;

	/* Move down to proper range now rounding is done */
	K /= 10;

	pll_div.k = K;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static int wm8976_set_dai_pll(struct snd_soc_dai *codec_dai,
		int pll_id, unsigned int freq_in, unsigned int freq_out)
#else		
static int wm8976_set_dai_pll(struct snd_soc_dai *codec_dai,
		int pll_id, int source, unsigned int freq_in, unsigned int freq_out)
#endif		
{
     printk("************************urbetter wm8976_set_dai_pll************************\n");
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 reg;

//	BDG();
	if(freq_in == 0 || freq_out == 0) {
		reg = wm8976_read_reg_cache(codec, WM8976_POWER1);
		wm8976_write(codec, WM8976_POWER1, reg & 0x1df);
		return 0;
	}

	pll_factors(freq_out * 8, freq_in);
#if 1
	wm8976_write(codec, WM8976_PLLN, (pll_div.pre << 4) | pll_div.n);
	wm8976_write(codec, WM8976_PLLK1, pll_div.k >> 18);
	wm8976_write(codec, WM8976_PLLK1, (pll_div.k >> 9) && 0x1ff);
	wm8976_write(codec, WM8976_PLLK1, pll_div.k && 0x1ff);
	reg = wm8976_read_reg_cache(codec, WM8976_POWER1);
	wm8976_write(codec, WM8976_POWER1, reg | 0x020);
#endif	
	
	return 0;
}

static int wm8976_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	//BDG();
	return 0;
}

static int wm8976_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
#if 1
	u16 iface = wm8976_read_reg_cache(codec, WM8976_IFACE) & 0x7;
	u16 clk = wm8976_read_reg_cache(codec, WM8976_CLOCK) & 0xfffe;

	//BDG();
	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) 
	{
	case SND_SOC_DAIFMT_CBM_CFM:
		clk |= 0x0001;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK)
	{
	case SND_SOC_DAIFMT_I2S:
		iface |= 0x0010;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		iface |= 0x0000;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface |= 0x0008;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface |= 0x0018;
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		iface |= 0x0180;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		iface |= 0x0100;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		iface |= 0x0080;
		break;
	default:
		return -EINVAL;
	}

	wm8976_write(codec, WM8976_IFACE, iface);
	wm8976_write(codec, WM8976_CLOCK, clk);
#endif
}
/***wangyulu*****/
static int wm8976_set_register_sound( int val)
{   
	int i;
	unsigned int reg;
#if 1
	if ((val ==1) && (wm8976_flag == 1)){
		for (i=0; i<58; i++) {	
			reg = wm8976_read_reg_cache(g_codec, i);
			reg_buf[i] = reg;
			//printk("reg_buf=0x%02X\n", reg_buf[i]); 
		}
	}
#endif
	//************ urbetter ***************
	sound8976_galley_select_flag = 0;	
	if(val == 0){
		  //R0
	      reg = wm8976_read_reg_cache(g_codec,WM8976_RESET );
          wm8976_write(g_codec, WM8976_RESET,  (reg |0x0000) & 0x0000);
	      //R1
	      reg = wm8976_read_reg_cache(g_codec, WM8976_POWER1);
          wm8976_write(g_codec, WM8976_POWER1, (reg | 0x001D) & 0x0000);
	      //R2
	      reg = wm8976_read_reg_cache(g_codec, WM8976_POWER2);
	      wm8976_write(g_codec, WM8976_POWER2, (reg | 0x0015) & 0x0000);
	      //R3
	      reg = wm8976_read_reg_cache(g_codec, WM8976_POWER3);
          wm8976_write(g_codec, WM8976_POWER3, (reg | 0x006E) & 0x0000);
		  for(i=4 ; i<58; i++) {
		  wm8976_write(g_codec, i, reg_buf[i]);
		  //printk("reg_buf=0x%02X\n", reg_buf[i]); 
		  wm8976_flag =3;
	   }
	}
      else if(val == 1) {
	 printk("**********************urbetter wm8976_set_register_sound ====1***********************\n");  
      printk("urbetter wm8976 LIN_LIP\n");
	 wm8976_flag = 2;
	 #if 1
	 //R0
	 reg = wm8976_read_reg_cache(g_codec,WM8976_RESET );
      wm8976_write(g_codec, WM8976_RESET,  (reg |0x0000) & 0x0000);
	 //R1
	 reg = wm8976_read_reg_cache(g_codec, WM8976_POWER1);
      //wm8976_write(g_codec, WM8976_POWER1, (reg | 0x001D) & 0x001D);//yuanlai
      wm8976_write(g_codec, WM8976_POWER1, (reg | 0x01D9) & 0x01D9);//LU
	 //R2
	 reg = wm8976_read_reg_cache(g_codec, WM8976_POWER2);
	 wm8976_write(g_codec, WM8976_POWER2, (reg | 0x0195) & 0x0195);//yuanlai
	 //R3
	 reg = wm8976_read_reg_cache(g_codec, WM8976_POWER3);
      //wm8976_write(g_codec, WM8976_POWER3, (reg | 0x006E) & 0x006E);//yuanlai
      wm8976_write(g_codec, WM8976_POWER3, (reg | 0x01ef) & 0x01ef);  //LU
	 //R4
	 reg = wm8976_read_reg_cache(g_codec,WM8976_IFACE );
      wm8976_write(g_codec, WM8976_IFACE, (reg | 0x0010) & 0x0010);
	 //R5
	 reg = wm8976_read_reg_cache(g_codec,WM8976_COMP );
      wm8976_write(g_codec, WM8976_COMP,  (reg |  0x0000) & 0x0000);
	 //R6
	 reg = wm8976_read_reg_cache(g_codec,WM8976_CLOCK );
      wm8976_write(g_codec, WM8976_CLOCK, reg |  0x0000);
	 //R7
	 reg = wm8976_read_reg_cache(g_codec,WM8976_ADD);
      wm8976_write(g_codec, WM8976_ADD,  reg | 0x0000);
	 //R8
	 reg = wm8976_read_reg_cache(g_codec,WM8976_GPIO );
      wm8976_write(g_codec, WM8976_GPIO,  reg | 0x0000);
	 //R9
	 reg = wm8976_read_reg_cache(g_codec,WM8976_JACK1);
      wm8976_write(g_codec, WM8976_JACK1,  reg | 0x0000 );
	 //R10
	 reg = wm8976_read_reg_cache(g_codec,WM8976_DAC);
      wm8976_write(g_codec, WM8976_DAC,  reg | 0x0000);	
	 //R11
	 reg = wm8976_read_reg_cache(g_codec,WM8976_DACVOLL);
      wm8976_write(g_codec, WM8976_DACVOLL,  reg |0x01ff );
	 //R12
	 reg = wm8976_read_reg_cache(g_codec,WM8976_DACVOLR );
      wm8976_write(g_codec, WM8976_DACVOLR,  reg | 0x01ff);
	 //R13
	 reg = wm8976_read_reg_cache(g_codec,WM8976_JACK2);
      wm8976_write(g_codec, WM8976_JACK2,  reg |0x0000);
	 //R14
	 reg = wm8976_read_reg_cache(g_codec,WM8976_ADC);
      wm8976_write(g_codec, WM8976_ADC,  reg | 0x0100);
	 //R15
	 reg = wm8976_read_reg_cache(g_codec,WM8976_ADCVOL);
      wm8976_write(g_codec, WM8976_ADCVOL,  reg | 0x01ff);
	 /*
	 //R16
	 reg = wm8976_read_reg_cache(g_codec,WM8976_EQ1);
      wm8976_write(g_codec, WM8976_EQ1,  reg | );
	 //R17
	 reg = wm8976_read_reg_cache(g_codec,WM8976_EQ2);
      wm8976_write(g_codec, WM8976_EQ2,  reg | );
      */
	 //R18
	 reg = wm8976_read_reg_cache(g_codec,WM8976_EQ1);
      wm8976_write(g_codec, WM8976_EQ1,  reg | 0x012c);
	 //R19
	 reg = wm8976_read_reg_cache(g_codec,WM8976_EQ2);
      wm8976_write(g_codec, WM8976_EQ2,  reg | 0x002c);
	 //R20
	 reg = wm8976_read_reg_cache(g_codec,WM8976_EQ3);
      wm8976_write(g_codec, WM8976_EQ3,  reg | 0x002c);
	 //R21
	 reg = wm8976_read_reg_cache(g_codec,WM8976_EQ4);
      wm8976_write(g_codec, WM8976_EQ4,  reg | 0x002c);
	 //R22
	 reg = wm8976_read_reg_cache(g_codec,WM8976_EQ5);
      wm8976_write(g_codec, WM8976_EQ5,  reg | 0x002c);
	 /*
	 //R23
	 reg = wm8976_read_reg_cache(g_codec,WM8976_DACLIM2);
      wm8976_write(g_codec, WM8976_DACLIM2,  reg | );
      */
	 //R24
	 reg = wm8976_read_reg_cache(g_codec,WM8976_DACLIM1);
      wm8976_write(g_codec, WM8976_DACLIM1,  reg | 0x0032);
	 //R25
	 reg = wm8976_read_reg_cache(g_codec,WM8976_DACLIM2);
      wm8976_write(g_codec, WM8976_DACLIM2,  reg | 0x0000);
	 /*
	 //R26
	 reg = wm8976_read_reg_cache(g_codec,WM8976_NOTCH3);
      wm8976_write(g_codec, WM8976_NOTCH3,  reg | );
      */
	 //R27
	 reg = wm8976_read_reg_cache(g_codec,WM8976_NOTCH1);
      wm8976_write(g_codec, WM8976_NOTCH1,  reg | 0x0000);
	 //R28
	 reg = wm8976_read_reg_cache(g_codec,WM8976_NOTCH2);
      wm8976_write(g_codec, WM8976_NOTCH2,  reg | 0x0000);
	 //R29
	 reg = wm8976_read_reg_cache(g_codec,WM8976_NOTCH3);
      wm8976_write(g_codec, WM8976_NOTCH3,  reg | 0x0000);
	 //R30
	 reg = wm8976_read_reg_cache(g_codec,WM8976_NOTCH4);
      wm8976_write(g_codec, WM8976_NOTCH4,  reg | 0x0000);
	 /*
	 //R31
	 reg = wm8976_read_reg_cache(g_codec,WM8976_NGATE);
      wm8976_write(g_codec, WM8976_NGATE,  reg | );
      */
	 //R32
	 reg = wm8976_read_reg_cache(g_codec, WM8976_ALC1);
	 wm8976_write(g_codec, WM8976_ALC1, (reg | 0x0038) & 0x0038);
	 //R33
	 reg = wm8976_read_reg_cache(g_codec,WM8976_ALC2);
      wm8976_write(g_codec, WM8976_ALC2,  reg | 0x000b );
	 //R34
	 reg = wm8976_read_reg_cache(g_codec,WM8976_ALC3);
      wm8976_write(g_codec, WM8976_ALC3,  reg | 0x0032);
	 //R35
	 reg = wm8976_read_reg_cache(g_codec,WM8976_NGATE);
      wm8976_write(g_codec, WM8976_NGATE,  reg | 0x0000);
	 //R36
	 reg = wm8976_read_reg_cache(g_codec,WM8976_PLLN);
      wm8976_write(g_codec, WM8976_PLLN,  reg | 0x0008);
	 //R37
	 reg = wm8976_read_reg_cache(g_codec,WM8976_PLLK1);
      wm8976_write(g_codec, WM8976_PLLK1,  reg | 0x000c);
	 //R38
	 reg = wm8976_read_reg_cache(g_codec,WM8976_PLLK2);
      wm8976_write(g_codec, WM8976_PLLK2,  reg | 0x0093);
	 //R39
	 reg = wm8976_read_reg_cache(g_codec,WM8976_PLLK3);
      wm8976_write(g_codec, WM8976_PLLK3,  reg | 0x00e9);
	 /*
	 //R40
	 reg = wm8976_read_reg_cache(g_codec,WM8976_DAC);
      wm8976_write(g_codec, WM8976_DAC,  reg | );
      */
	 //R41
	 reg = wm8976_read_reg_cache(g_codec,WM8976_3D);
      wm8976_write(g_codec, WM8976_3D,  reg | 0x0000 );
	 /*
	 //R42
	 reg = wm8976_read_reg_cache(g_codec,WM8976_DAC);
      wm8976_write(g_codec, WM8976_DAC,  reg | );
	 */
	 //R43
	 reg = wm8976_read_reg_cache(g_codec,WM8976_BEEP);
      wm8976_write(g_codec, WM8976_BEEP,  reg | 0x0010);
      //R44
	 reg = wm8976_read_reg_cache(g_codec, WM8976_INPUT);
	 wm8976_write(g_codec, WM8976_INPUT, reg | 0x0003);
	 //R45
	 //reg = wm8976_read_reg_cache(g_codec, WM8976_INPPGA);
	 //wm8976_write(g_codec, WM8976_INPPGA, (reg | 0x01B8) & 0x01B8);//yuan
	 //wm8976_write(g_codec, WM8976_INPPGA, (reg | 0x01f8) & 0x01f8);
	 //lu
	 //R45  lu  out4
	 reg = wm8976_read_reg_cache(g_codec, WM8976_INPPGA);
	 //wm8976_write(g_codec, WM8976_INPPGA, (reg | 0x007f) & 0x0000); 
	 wm8976_write(g_codec, WM8976_INPPGA, (reg | 0x007f) & 0xFF3F); 
	 /*
	 //R46
	 reg = wm8976_read_reg_cache(g_codec,WM8976_DAC);
      wm8976_write(g_codec, WM8976_DAC,  reg | );
	 */
	 //R47 
	 reg = wm8976_read_reg_cache(g_codec, WM8976_ADCBOOST);
      wm8976_write(g_codec, WM8976_ADCBOOST, (reg | 0x0100) & 0x0100);
	 /*
      //R48
      reg = wm8976_read_reg_cache(g_codec,WM8976_DAC);
      wm8976_write(g_codec, WM8976_DAC,  reg | );
      */
      //R49
      reg = wm8976_read_reg_cache(g_codec, WM8976_OUTPUT);
      wm8976_write(g_codec, WM8976_OUTPUT, (reg | 0x0018) & 0x0018);//yuan
	 //R50 //yuanlai
      //reg = wm8976_read_reg_cache(g_codec, WM8976_MIXL);
      //wm8976_write(g_codec, WM8976_MIXL, (reg | 0x01E0) & 0x01E0); 
      //R50 //lu youyong out3
      reg = wm8976_read_reg_cache(g_codec, WM8976_MIXL);
      //wm8976_write(g_codec, WM8976_MIXL, (reg | 0xffff) & 0xffff); 
      wm8976_write(g_codec, WM8976_MIXL, (reg | 0xfffd) & 0xfffd);
	 //R51
      reg = wm8976_read_reg_cache(g_codec, WM8976_MIXR);
      wm8976_write(g_codec, WM8976_MIXR, (reg | 0x01e0) & 0x01e0);
      //R52 
	 reg = wm8976_read_reg_cache(g_codec, WM8976_HPVOLL);
	 wm8976_write(g_codec, WM8976_HPVOLL, (reg | 0x0135) & 0x0135);
	 //R53 
	 reg = wm8976_read_reg_cache(g_codec, WM8976_HPVOLR);
      wm8976_write(g_codec, WM8976_HPVOLR, (reg | 0x0135) & 0x0135);
	 //R54
	 reg = wm8976_read_reg_cache(g_codec,WM8976_SPKVOLL);
      wm8976_write(g_codec, WM8976_SPKVOLL,  reg | 0x013a);
	 //R55
	 reg = wm8976_read_reg_cache(g_codec,WM8976_SPKVOLR);
      wm8976_write(g_codec, WM8976_SPKVOLR,  reg | 0x013a);
	 //R56 
	 reg = wm8976_read_reg_cache(g_codec, WM8976_OUT3MIX);
	 //wm8976_write(g_codec, WM8976_OUT3MIX, (reg | 0x000f) & 0x000f); 
	 wm8976_write(g_codec, WM8976_OUT3MIX, (reg | 0x0004) & 0x0004); 
      //R57
      reg = wm8976_read_reg_cache(g_codec, WM8976_MONOMIX);
      wm8976_write(g_codec, WM8976_MONOMIX, (reg | 0x0019) & 0x0019);   
      
#endif 
#if 0//out3 shihaode 
     
//lulu  wm8976_write(g_codec, WM8976_POWER1, pwr_reg|0x01D);//off
	   printk("************************urbetter sound8976_galley_select_flag ====1***********************\n");
        //R0
	   reg = wm8976_read_reg_cache(g_codec,WM8976_RESET );
        wm8976_write(g_codec, WM8976_RESET,  (reg |0x0000) & 0x0000);
	   //R1
	   reg = wm8976_read_reg_cache(g_codec, WM8976_POWER1);
        //wm8976_write(g_codec, WM8976_POWER1, (reg | 0x001D) & 0x001D);//yuanlai
        wm8976_write(g_codec, WM8976_POWER1, (reg | 0x01D9) & 0x01D9);//LU
	   //R2
	   reg = wm8976_read_reg_cache(g_codec, WM8976_POWER2);
	   wm8976_write(g_codec, WM8976_POWER2, (reg | 0x0195) & 0x0195);//yuanlai
	   //R3
	   reg = wm8976_read_reg_cache(g_codec, WM8976_POWER3);
        //wm8976_write(g_codec, WM8976_POWER3, (reg | 0x006E) & 0x006E);//yuanlai
        wm8976_write(g_codec, WM8976_POWER3, (reg | 0x01ef) & 0x01ef);  //LU
        //R32
	   reg = wm8976_read_reg_cache(g_codec, WM8976_ALC1);
	   wm8976_write(g_codec, WM8976_ALC1, (reg | 0x0000) & 0xfeff);
	   //R44
	   reg = wm8976_read_reg_cache(g_codec, WM8976_INPUT);
	   wm8976_write(g_codec, WM8976_INPUT, (reg | 0x0003) & 0xffff);
	   
	   //R45 out3 
	   reg = wm8976_read_reg_cache(g_codec, WM8976_INPPGA);
	   wm8976_write(g_codec, WM8976_INPPGA, (reg | 0x007f) & 0xFF3F);
	   //R47
	   reg = wm8976_read_reg_cache(g_codec, WM8976_ADCBOOST);
        wm8976_write(g_codec, WM8976_ADCBOOST, (reg | 0x0170) & 0xFFF0);   
	   //R47
	   reg = wm8976_read_reg_cache(g_codec, WM8976_ADCBOOST);
        wm8976_write(g_codec, WM8976_ADCBOOST, (reg | 0x01F0) & 0xFFF0);
	   //R49
        reg = wm8976_read_reg_cache(g_codec, WM8976_OUTPUT);
        wm8976_write(g_codec, WM8976_OUTPUT, (reg | 0x001b) & 0xFEFF);
	   printk("urbetter wm8976 MIX_out4 lululu\n");
        //R50
        reg = wm8976_read_reg_cache(g_codec, WM8976_MIXL);
        wm8976_write(g_codec, WM8976_MIXL, reg | 0x01FE);
	   //R50
        reg = wm8976_read_reg_cache(g_codec, WM8976_MIXL);
        //wm8976_write(codec, WM8976_MIXL, (reg | 0x01FE) & 0xfffd);
        wm8976_write(g_codec, WM8976_MIXL, (reg | 0xffff) & 0xffff);
        //R51
        reg = wm8976_read_reg_cache(g_codec, WM8976_MIXR);
        //wm8976_write(codec, WM8976_MIXR, reg | 0x0020);
        wm8976_write(g_codec, WM8976_MIXR, reg | 0x01e0);
        //R52
	   reg = wm8976_read_reg_cache(g_codec, WM8976_HPVOLL);
	   wm8976_write(g_codec, WM8976_HPVOLL, (reg | 0x003F) & 0xFFBF);
	   //R53
	   reg = wm8976_read_reg_cache(g_codec, WM8976_HPVOLR);
        wm8976_write(g_codec, WM8976_HPVOLR, (reg | 0x003F)& 0xFFBF);	

        printk("urbetter wm8976 MIX_out3\n");
	   //R56
	   reg = wm8976_read_reg_cache(g_codec, WM8976_OUT3MIX);
	   wm8976_write(g_codec, WM8976_OUT3MIX, (reg | 0x0006) & 0xffbf); //6=0
        //R57
        reg = wm8976_read_reg_cache(g_codec, WM8976_MONOMIX);
        wm8976_write(g_codec, WM8976_MONOMIX, (reg | 0x0000) & 0xff00);
#endif
	 //************ urbetter ***************	 
    }
    sound8976_galley_select_flag = val;
}
static int wm8976_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	//wm8976_set_bias_level(codec, SND_SOC_BIAS_PREPARE);
	//************ urbetter ***************
	u16 reg;
#if 0
     //////////////urbetter ///////////////////

#endif
#if 1 //lu
	reg = wm8976_read_reg_cache(codec, WM8976_DACVOLL);
	wm8976_write(codec, WM8976_DACVOLL, reg | 0x0100);
	reg = wm8976_read_reg_cache(codec, WM8976_DACVOLR);
	wm8976_write(codec, WM8976_DACVOLR, reg | 0x0100);
	reg = wm8976_read_reg_cache(codec, WM8976_ADCVOL);
	wm8976_write(codec, WM8976_ADCVOL, reg | 0x01ff);
	//Reserved a reg
	reg = wm8976_read_reg_cache(codec, WM8976_INPPGA);
	wm8976_write(codec, WM8976_INPPGA, reg | 0x01B8);  //63/63 % PGA  0x01BF
	reg = wm8976_read_reg_cache(codec, WM8976_HPVOLL);
	wm8976_write(codec, WM8976_HPVOLL, reg | 0x0100);
	reg = wm8976_read_reg_cache(codec, WM8976_HPVOLR);
	wm8976_write(codec, WM8976_HPVOLR, reg | 0x0100);
	reg = wm8976_read_reg_cache(codec, WM8976_SPKVOLL);
	wm8976_write(codec, WM8976_SPKVOLL, reg | 0x0100);
	reg = wm8976_read_reg_cache(codec, WM8976_SPKVOLR);
	wm8976_write(codec, WM8976_SPKVOLR, reg | 0x0100);
#endif
	//*************************************
	u16 iface = wm8976_read_reg_cache(codec, WM8976_IFACE) & 0xff9f;
	u16 adn = wm8976_read_reg_cache(codec, WM8976_ADD) & 0x1f1;


	s3cdbg("%s:%d Entering...\n",__FUNCTION__, __LINE__);

	/* bit size */
	switch (params_format(params))
	{
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		iface |= 0x0020;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		iface |= 0x0040;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		iface |= 0x0060;
		break;
	}

	/* filter coefficient */
	switch (params_rate(params))
	{
	//case SNDRV_PCM_RATE_8000:
	case 8000:
		adn |= 0x5 << 1;
		break;
	//case SNDRV_PCM_RATE_11025:
	case 11025:
		adn |= 0x4 << 1;
		break;
	//case SNDRV_PCM_RATE_16000:
	case 16000:
		adn |= 0x3 << 1;
		break;
	//case SNDRV_PCM_RATE_22050:
	case 22050:
		adn |= 0x2 << 1;
		break;
	//case SNDRV_PCM_RATE_32000:
	case 32000:
		adn |= 0x1 << 1;
		break;
	case 44100:
	case 48000:
		//adn |= 0x0 << 1;
		break;
	}

	/* set iface */
	wm8976_write(codec, WM8976_IFACE, iface);
	wm8976_write(codec, WM8976_ADD, adn);
	return 0;
}

static int wm8976_set_dai_clkdiv(struct snd_soc_dai *codec_dai,
		int div_id, int div)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 reg;

//	BDG();
	s3cdbg("%s:%d Entering...\n",__FUNCTION__, __LINE__);
	
	switch (div_id) {
	case WM8976_MCLKDIV:
		reg = wm8976_read_reg_cache(codec, WM8976_CLOCK) & 0x11f;
		wm8976_write(codec, WM8976_CLOCK, reg | div);
		break;
	case WM8976_BCLKDIV:
		reg = wm8976_read_reg_cache(codec, WM8976_CLOCK) & 0x1c7;
		wm8976_write(codec, WM8976_CLOCK, reg | div);
		break;
	case WM8976_OPCLKDIV:
		reg = wm8976_read_reg_cache(codec, WM8976_GPIO) & 0x1cf;
		wm8976_write(codec, WM8976_GPIO, reg | div);
		break;
	case WM8976_DACOSR:
		reg = wm8976_read_reg_cache(codec, WM8976_DAC) & 0x1f7;
		wm8976_write(codec, WM8976_DAC, reg | div);
		break;
	case WM8976_ADCOSR:
		reg = wm8976_read_reg_cache(codec, WM8976_ADC) & 0x1f7;
		wm8976_write(codec, WM8976_ADC, reg | div);
		break;
	case WM8976_MCLKSEL:
		reg = wm8976_read_reg_cache(codec, WM8976_CLOCK) & 0x0ff;
		wm8976_write(codec, WM8976_CLOCK, reg | div);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int wm8976_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	u16 mute_reg = wm8976_read_reg_cache(codec, WM8976_DAC) & 0xffbf;

//	BDG();
	s3cdbg("%s:%d mute = %d...\n",__FUNCTION__, __LINE__, mute);

	if(mute)
	{
		wm8976_write(codec, WM8976_DAC, mute_reg | 0x40);
#if 0		//for test
		write_power_item_value(POWER_SPEAKER, 0);
#endif

	}
	else{
		wm8976_write(codec, WM8976_DAC, mute_reg);
#if 0		//for test
		write_power_item_value(POWER_SPEAKER, 1);
#endif

	}

	return 0;
}

void wm8976_set_mute(int mute)
{
    	u16 mute_reg = wm8976_read_reg_cache(g_codec, WM8976_DAC) & 0xffbf;
    	u16 mute_reg1 = wm8976_read_reg_cache(g_codec, WM8976_HPVOLL) & 0xffbf;
    	u16 mute_reg2 = wm8976_read_reg_cache(g_codec, WM8976_HPVOLR) & 0xffbf;
    	u16 mute_reg3 = wm8976_read_reg_cache(g_codec, WM8976_SPKVOLL) & 0xffbf;
    	u16 mute_reg4 = wm8976_read_reg_cache(g_codec, WM8976_SPKVOLR) & 0xffbf;
		
//	BDG();
	if(mute)
        {
            wm8976_write(g_codec, WM8976_DAC, mute_reg | 0x40);
            wm8976_write(g_codec, WM8976_HPVOLL, mute_reg1 | 0x40);
            wm8976_write(g_codec, WM8976_HPVOLR, mute_reg2 | 0x40);
            wm8976_write(g_codec, WM8976_SPKVOLL, mute_reg3 | 0x40);
            wm8976_write(g_codec, WM8976_SPKVOLR, mute_reg4 | 0x40);
        }
	else
        {
            wm8976_write(g_codec, WM8976_DAC, mute_reg);
            wm8976_write(g_codec, WM8976_HPVOLL, mute_reg1);
            wm8976_write(g_codec, WM8976_HPVOLR, mute_reg2);
            wm8976_write(g_codec, WM8976_SPKVOLL, mute_reg3);
            wm8976_write(g_codec, WM8976_SPKVOLR, mute_reg4);
        }

}
EXPORT_SYMBOL_GPL(wm8976_set_mute);



/* TODO: liam need to make this lower power with dapm */
static int wm8976_set_bias_level(struct snd_soc_codec *codec,
	enum snd_soc_bias_level level)
{
     u16 reg;	
	int i;
#if 1
	u16 pwr_reg = wm8976_read_reg_cache(codec,WM8976_POWER1) & 0x0fc;

//	BDG();
//	s3cdbg("level =======[%04x]%d;\n", pwr_reg, level);
	
	wm8976_write(codec, WM8976_INPUT, 0x03);
	switch (level) {
	case SND_SOC_BIAS_ON:
		/*set vmid to 75k for*/
	     wm8976_write(codec, WM8976_POWER1, (pwr_reg|0x01D) & 0x01D);//microphone bias is open 	
		#if 0
		for (i=0; i<58; i++) {	
			reg = wm8976_read_reg_cache(codec, i);
			reg_buf[i] = reg;
			printk("reg_buf=0x%02X\n", reg_buf[i]); 
		     }
		     printk("************************urbetter  0n0n0n0n0n0n00n0n0n0n0n0n************************\n");
		#endif
		break;
	case SND_SOC_BIAS_STANDBY:
		printk("************************urbetter 13 SND_SOC_BIAS_STANDBY************************\n");
            /* set vmid to 5k for quick power up */
	      wm8976_write(codec, WM8976_POWER1, (pwr_reg|0x001F) & 0X001F);//microphone bias is open 
	     #if 0
		for (i=0; i<58; i++) {	
			reg = wm8976_read_reg_cache(codec, i);
			reg_buf[i] = reg;
			printk("reg_buf=0x%02X\n", reg_buf[i]); 
		     }
		     printk("************************urbetter  ofofofofofofofofofofofofofofoof************************\n");
          #endif
		break;
	case SND_SOC_BIAS_PREPARE:
		printk("************************utbetter 13 SND_SOC_BIAS_PREPARE************************\n");
		 wm8976_write(codec, WM8976_POWER1, (pwr_reg|0x001E) & 0X001E);//microphone bias is open 
		#if 0
				for (i=0; i<58; i++) {	
					reg = wm8976_read_reg_cache(codec, i);
					reg_buf[i] = reg;
					printk("reg_buf=0x%02X\n", reg_buf[i]); 
					}
					printk("************************urbetter  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^************************\n");
		#endif
		break;
	case SND_SOC_BIAS_OFF:
		//wm8976_write(codec, WM8976_POWER1, 0x0);
		//wm8976_write(codec, WM8976_POWER2, 0x0);
		//wm8976_write(codec, WM8976_POWER3, 0x0);
		break;
		
	}
	codec->bias_level = level;
	return 0;
#endif
}



#define WM8976_RATES \
	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_16000 | \
	SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
	SNDRV_PCM_RATE_48000)

#define WM8976_FORMATS \
	(SNDRV_PCM_FORMAT_S16_LE | SNDRV_PCM_FORMAT_S20_3LE | \
	SNDRV_PCM_FORMAT_S24_3LE | SNDRV_PCM_FORMAT_S24_LE)


static struct snd_soc_dai_ops wm8976_dai_ops = {
	.hw_params	= wm8976_hw_params,
	.set_fmt	= wm8976_set_dai_fmt,
	.set_clkdiv	= wm8976_set_dai_clkdiv,
	.set_pll	= wm8976_set_dai_pll,
	.digital_mute	= wm8976_mute,
	.set_sysclk	= wm8976_set_dai_sysclk, //wang
};

struct snd_soc_dai wm8976_dai = {

	.name = "WM8976 HiFi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = WM8976_RATES,
		.formats = WM8976_FORMATS,},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = WM8976_RATES,
		.formats = WM8976_FORMATS,},
	.ops = &wm8976_dai_ops,

};
EXPORT_SYMBOL_GPL(wm8976_dai);

#if 0
static int wm8976_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	wm8976_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int wm8976_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	int i;
	u8 data[2];
	u16 *cache = codec->reg_cache;

	/* Sync reg_cache with the hardware */
	for (i = 0; i < ARRAY_SIZE(wm8976_reg); i++)
	{
		data[0] = (i << 1) | ((cache[i] >> 8) & 0x0001);
		data[1] = cache[i] & 0x00ff;
		codec->hw_write(codec->control_data, data, 2);
	}

	wm8976_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	wm8976_set_bias_level(codec, codec->suspend_bias_level);
	return 0;
}
#endif

static struct snd_soc_device *wm8976_socdev;

static int s_model = 710;
static void check_model(void)
{
/*	int value = 710;
	extern char g_selected_utmodel[];
	if(sscanf(g_selected_utmodel, "%d", &value) == 1)
	{
		if(value > 0) 
			s_model = value;
		printk("select audio  model %d\n", s_model);
	} */
}


static int wm8976_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	int ret = 0;

	s3cdbg("Entering %s...\n", __func__);

	if (g_codec == NULL) {
		dev_err(&pdev->dev, "Codec device not registered\n");
		return -ENODEV;
	}

	check_model();

	socdev->card->codec = g_codec;
	codec = g_codec;

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if(ret < 0) {
		printk(KERN_ERR "wm8976: failed to create pcms\n");
		goto pcm_err;
	}

	wm8976_add_controls(codec);

	wm8976_add_widgets(codec);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35) //wang
	s3cdbg("%s init_card...\n", __func__);
	ret = snd_soc_init_card(socdev);
	if (ret < 0) {
		dev_err(codec->dev, "failed to register card: %d\n", ret);
		goto card_err;
	}
#endif
	return ret;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)//wang
card_err:
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
#endif	
pcm_err:
	return ret;
}


/* power down chip */
static int wm8976_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_wm8976 = {
	.probe = 	wm8976_probe,
	.remove = 	wm8976_remove,
//	.suspend = 	wm8976_suspend,
//	.resume =	wm8976_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_wm8976);


static int wm8976_register(struct snd_soc_codec *codec)
{
	int ret = 0;
	u16 reg;
	
	//BDG();
	s3cdbg("Entering %s...\n", __func__);

	if (g_codec) {
		dev_err(codec->dev, "Another WM8580 is registered\n");
		ret = -EINVAL;
		goto err;
	}

	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	codec->name = "WM8976";
	codec->owner = THIS_MODULE;
	codec->read = wm8976_read_reg_cache;
	codec->write = wm8976_write;
	codec->set_bias_level = wm8976_set_bias_level;
	codec->dai = &wm8976_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = ARRAY_SIZE(wm8976_reg);
	codec->reg_cache = kmemdup(wm8976_reg, sizeof(wm8976_reg), GFP_KERNEL);

	if (codec->reg_cache == NULL)
		return -ENOMEM;

	ret = wm8976_reset(codec);
	if(ret != 0)
	{
		s3cdbg("%s:Failed to reset codec,ret=%d\n", __func__, ret);
	}
	else
	{
		s3cdbg("%s:reset codec,ret=%d\n", __func__, ret);
	}

	wm8976_set_bias_level(codec, SND_SOC_BIAS_PREPARE);
	codec->bias_level = SND_SOC_BIAS_STANDBY;
	msleep(msecs_to_jiffies(2000));

#if 1
        wm8976_write(codec, WM8976_IFACE, 0x010);
	wm8976_write(codec, WM8976_CLOCK, 0x000);
	wm8976_write(codec, WM8976_BEEP, 0x010);

        /* set the update bits */
        reg = wm8976_read_reg_cache(codec, WM8976_DACVOLL);
        wm8976_write(codec, WM8976_DACVOLL, reg | 0x0100);
        reg = wm8976_read_reg_cache(codec, WM8976_DACVOLR);
        wm8976_write(codec, WM8976_DACVOLR, reg | 0x0100);
		
        reg = wm8976_read_reg_cache(codec, WM8976_ADCVOL);
        wm8976_write(codec, WM8976_ADCVOL, reg | 0x01ff);
        /*Reserved a reg*/
        reg = wm8976_read_reg_cache(codec, WM8976_INPPGA);
        wm8976_write(codec, WM8976_INPPGA, reg | 0x01B8);  // 63/63 % PGA  0x01BF


		if(s_model == 708){                                //09 20 
		        reg = wm8976_read_reg_cache(codec, WM8976_HPVOLL);
		        //wm8976_write(codec, WM8976_HPVOLL, reg | 0x0100);
		        wm8976_write(codec, WM8976_HPVOLL, reg | 0x0139);		//urbetter volume for aigo 0920
		        reg = wm8976_read_reg_cache(codec, WM8976_HPVOLR);
		        //wm8976_write(codec, WM8976_HPVOLR, reg | 0x0100);		
		        wm8976_write(codec, WM8976_HPVOLR, reg | 0x0139);		//urbetter volume for aigo 0920
			}
		else{  
		        reg = wm8976_read_reg_cache(codec, WM8976_HPVOLL);
		        //wm8976_write(codec, WM8976_HPVOLL, reg | 0x0100);
		        wm8976_write(codec, WM8976_HPVOLL, reg | 0x0001);		//urbetter volume for aigo 0920
		        reg = wm8976_read_reg_cache(codec, WM8976_HPVOLR);
		        //wm8976_write(codec, WM8976_HPVOLR, reg | 0x0100);		
		        wm8976_write(codec, WM8976_HPVOLR, reg | 0x0001);		//urbetter volume for aigo 0920
			}
		
        reg = wm8976_read_reg_cache(codec, WM8976_SPKVOLL);
        wm8976_write(codec, WM8976_SPKVOLL, reg | 0x0100);
        reg = wm8976_read_reg_cache(codec, WM8976_SPKVOLR);
        wm8976_write(codec, WM8976_SPKVOLR, reg | 0x0100);
//add by urbetter 110802
      //wm8976_write(codec, WM8976_MIXL, ((7<<6)|(1<<5)|(1<<0)));
      //wm8976_write(codec, WM8976_MIXR, ((7<<6)|(1<<5)|(1<<0)));


        //wm8976_write(codec, WM8976_EQ1, 0x0109);		//urbetter for EQ1 BASS
        wm8976_write(codec, WM8976_EQ5, 0x010D);		//urbetter for EQ5
#endif

#if 0
	wm8976_write(codec, WM8976_GPIO, 0x008);     /* R8*/	
	wm8976_write(codec, WM8976_ADD, 0x001);      /* slow clock enabled */	
	wm8976_write(codec, WM8976_JACK1, 0x050);    /* selected GPIO2 as jack detection input and Enable*/	
	wm8976_write(codec, WM8976_JACK2, 0x021);    /* OUT2_EN_0 and OUT2_EN_1 */
#endif


    g_codec=codec;;
	
	/* power on device */
	//wm8976_set_bias_level(codec, SND_SOC_BIAS_OFF);
	
	wm8976_write(codec, WM8976_INPUT, 0x03);

	ret = snd_soc_register_codec(codec);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register codec: %d\n", ret);
		goto err;
	}

	wm8976_dai.dev = codec->dev;
	ret = snd_soc_register_dais(&wm8976_dai, 1);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register DAI: %d\n", ret);
		goto err_codec;
	}

	return 0;

err_codec:
	snd_soc_unregister_codec(codec);
err:
	kfree(codec);
	return ret;

}

static void wm8976_unregister(struct snd_soc_codec *codec)
{
	//BDG();
	wm8976_set_bias_level(codec, SND_SOC_BIAS_OFF);
	snd_soc_unregister_dais(&wm8976_dai, 1);
	snd_soc_unregister_codec(codec);
	kfree(codec);
	g_codec = NULL;
}


#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
static int wm8976_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct snd_soc_codec *codec;

	s3cdbg("Entering %s...\n", __func__);

	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	codec->hw_write = (hw_write_t)i2c_master_send;

	i2c_set_clientdata(i2c, codec);
	codec->control_data = i2c;

	codec->dev = &i2c->dev;

	return wm8976_register(codec);
}

static int wm8976_i2c_remove(struct i2c_client *client)
{
	struct snd_soc_codec *codec = i2c_get_clientdata(client);
	wm8976_unregister(codec);
	return 0;
}

static const struct i2c_device_id wm8976_i2c_id[] = {
	{ "wm8976", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, wm8976_i2c_id);

static struct i2c_driver wm8976_i2c_driver = {
	.driver = {
		.name = "wm8976",//wang
		.owner = THIS_MODULE,
	},
	.probe =    wm8976_i2c_probe,
	.remove =   wm8976_i2c_remove,
	.id_table = wm8976_i2c_id,
};
#endif

static int __init wm8976_modinit(void)
{
	int ret;

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	ret = i2c_add_driver(&wm8976_i2c_driver);
	if (ret != 0) {
		pr_err("Failed to register WM8976 I2C driver: %d\n", ret);
	}
#endif
/*********wangyulu add down******/
	//extern char g_selected_codec[];
	//if(!strcmp(g_selected_codec, "wm8976")){
	if(1){
		 #define MODEM_SWITCH_PROC_NAME "sound8976_galley_select"
		 #define PROC_NAME "sound8976"
		extern struct proc_dir_entry proc_root;
		struct proc_dir_entry *root_entry;
		struct proc_dir_entry *entry;
		
		root_entry = proc_mkdir(PROC_NAME, &proc_root);
		s_proc = create_proc_entry(MODEM_SWITCH_PROC_NAME, 0666, root_entry);
		if (s_proc != NULL){
			s_proc->write_proc = modem_switch_writeproc;
			s_proc->read_proc = modem_switch_readproc;
		}
	}
	printk(KERN_INFO "Initializing wm8976_modinit...\n");
/********urbetter add up********/
	return 0;
}
module_init(wm8976_modinit);

static void __exit wm8976_exit(void)
{
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	i2c_del_driver(&wm8976_i2c_driver);
#endif
}
module_exit(wm8976_exit);


MODULE_DESCRIPTION("ASoC WM8976 driver");
MODULE_AUTHOR("urbetter");
MODULE_LICENSE("GPL");


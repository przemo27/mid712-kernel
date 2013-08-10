/*
 *
 * Copyright (C) 2009 rockchip lhh
 *
 * Based on WM8750.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef _WM8988_H
#define _WM8988_H


/*ADC & DAC Control  (R5)*/
#define 	WM_ADCPOL_DEF		0x00	/*Polarity not inverted*/
#define 	WM_ADCPOL_Lpolin		0x20	/*L polarity	invert*/
#define 	WM_ADCPOL_Rpolin		0x40	/*R polarity invert*/
#define 	WM_ADCPOL_LRpolin		0x60	/*L and R polarity invert*/

#define 	WM_HPOR 			0x10	/*Store dc offset when high-pass		0:clear offset	1:store offset*/
#define 	WM_DACMU				0x08	/*Digital Soft MUte 	0:n mute(signal active) 1:mute*/

#define 	WM_DEEMP_DEF			0x00	/*NO De-emphasis*/
#define 	WM_DEEMP_32kSamRate 0x02	/*32kHz sample rate*/
#define 	WM_DEEMP_44KSamRate 0x04	/*44kHz sample rate*/
#define 	WM_DEEMP_48kSamRate 0x06	/*48KHz sample rate*/

#define WM_ADCHPD		0x0 	/**/

/*WM8987_R7   Digital Audio Interface Format*/
#define   WM_MASTER_MODE 		0x40
#define   WM_SLAVE_MODE 		0x00
#define  	WM_16BIT_MODE 		(0x0<<2)
#define  	WM_24BIT_MODE 		(0x2<<2)
#define  	WM_DSP_MODE  			0x03
#define 	WM_I2S_MODE  			0x02

/*WM8988_R8  Clocking and Sample Rate*/
#define   WM_BCKdiv4			   (1<<7)
#define   WM_BCKdiv8			   (2<<7)
#define 	WM_CLKDIV2		0x40

#define 	FREQ96kHz			(0x0e<<1) 
#define 	FREQ48kHz			(0x00<<1)
#define 	FREQ441kHz			(0x11<<1)
#define 	FREQ32kHz			(0x0c<<1)
#define 	FREQ24kHz			(0x1c<<1)
#define 	FREQ2205kHz 	(0x1b<<1)
#define 	FREQ16kHz			(0x0a<<1)
#define 	FREQ12kHz			(0x08<<1)
#define 	FREQ11025kHz		(0x19<<1)
//#define	FREQ9k6Hz			0x09
#define 	FREQ8kHz			(0x06<<1)
#define 	WM_USB_MODE 	0x01|WM_BCKdiv8

/*WM8988_R10^^WM8988_R11 Left and Right  Channel Digital Volume*/
#define   WM_VOL_MUTE		0x00
#define   WM_VOL_0DB		0xff
#define   WM_UPDATE_VOL 0x100

/*PM1 WM8988_PWR1 (R25) */

//#define 	WM_VMID50k	0x80	    /* Vmid divider enable and select  00-Vmid disabled  01- 50O divider enabled 
//                                    10- 500O divider enabled  11- 5K O divider enabled*/
#define 	WM_VMID_OFF	0x000       //For Vmid disabled (for OFF mode)
#define 	WM_VMID50K	0x080       //For playback / record
#define 	WM_VMID500K	0x100       //For low-power standby
#define 	WM_VMID5K	0x180       //For fast start-up

#define 	WM_VREF 		0x40	/*VREF(necessary for all other functions)*/
#define 	WM_AINL 		0x20	/*Analogue in PGA Left*/
#define 	WM_AINR 		0x10	/*Analogue in PGA Right*/
#define 	WM_ADCL 		0x08	/*ADC Left*/
#define 	WM_ADCR 		0x04	/*ADC Right*/
#define 	WM_MICB 		0x02	/*NULL*/
#define     WM_DIGENB 		0x01	/*Master clock disable  0:master clock enabled 1:master clock disabled*/

/*PM2 WM8988_PWR2 (R26) */

#define 	WM_DACL		0x100	/*DAC Left */
#define  	WM_DACR		0x80	/*DAC Right*/
#define	    WM_LOUT1		0x40	/*LOUT1 Output Buffer*/
#define 	WM_ROUT1		0x20	/*ROUT1	Output Buffer*/
#define 	WM_LOUT2		0x10	/*LOUT2 Output Buffer*/
#define 	WM_ROUT2		0x08	/*ROUT2 Output Buffer*/

/*WM8988_R34  Left out mix1	*/
#define  	WM_LD2LO				0x100
#define  	WM_LI2LO				0x80
#define     WM_LO0DB				0x20
#define  	WM_LO6DB				0x00
/*WM8988_R37  Right out Mix1*/
#define  	WM_RD2RO				0x100
#define  	WM_RI2RO				0x80
#define  	WM_RO0DB				0x20
#define  	WM_RO6DB				0x00

#define ASC_BCLKDIV_4               (0x1 << 4)
#define ASC_BCLKDIV_8               (0x2 << 4)
#define ASC_BCLKDIV_16              (0x3 << 4)

/* WM8988 register space */

#define WM8988_LINVOL    0x00
#define WM8988_RINVOL    0x01
#define WM8988_LOUT1V    0x02
#define WM8988_ROUT1V    0x03
#define WM8988_ADCDAC    0x05
#define WM8988_IFACE     0x07
#define WM8988_SRATE     0x08
#define WM8988_LDAC      0x0a
#define WM8988_RDAC      0x0b
#define WM8988_BASS      0x0c
#define WM8988_TREBLE    0x0d
#define WM8988_RESET     0x0f  
#define WM8988_3D        0x10
#define WM8988_ALC1      0x11
#define WM8988_ALC2      0x12
#define WM8988_ALC3      0x13  
#define WM8988_NGATE     0x14
#define WM8988_LADC      0x15			/*Left ADC volume*/
#define WM8988_RADC      0x16		/*Right ADC volume*/
#define WM8988_ADCTL1    0x17		/*Additional control(1)*/
#define WM8988_ADCTL2    0x18		/*Additional control(2)*/
#define WM8988_PWR1      0x19		/*Pwr Mgmt(1)*/
#define WM8988_PWR2      0x1a		/*Pwr Mgmt(2)*/
#define WM8988_ADCTL3    0x1b
#define WM8988_ADCIN     0x1f		/*ADC input Mode*/
#define WM8988_LADCIN    0x20		/*ADCL signal path*/
#define WM8988_RADCIN    0x21		/*ADCR signal path*/
#define WM8988_LOUTM1    0x22
#define WM8988_LOUTM2    0x23
#define WM8988_ROUTM1    0x24
#define WM8988_ROUTM2    0x25
#define WM8988_MOUTM1    0x26
#define WM8988_MOUTM2    0x27
#define WM8988_LOUT2V    0x28
#define WM8988_ROUT2V    0x29
#define WM8988_MOUTV     0x2a

#define WM8988_CACHE_REGNUM 0x2a

#define	WM8988_LOW_POWER_PLAYBACK		0x43 

struct wm8988_setup_data {
	unsigned short i2c_address;
};

struct wm8988_platform_data {
    unsigned int speed;
    unsigned int mode;
    unsigned int reg_byte_cnt;
};

extern struct snd_soc_codec_dai wm8988_dai;
extern struct snd_soc_codec_device soc_codec_dev_wm8988;

#endif

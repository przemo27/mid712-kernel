/*
 * rtc class driver for the HYM8563 chip
 *
 *
 * based on previously existing rtc class drivers
 *
 * 2007 (c) MontaVista, Software, Inc.	This file is licensed under
 * the terms of the GNU General Public License version 2.  This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
 
#ifndef _HYM8563_H
#define _HYM8563_H
 
#define   HYM_ADDR		0xA2
#define   RTC_SPEED 		200
		
#define   RTC_CTL1		0x00
#define   RTC_CTL2		0x01
#define   RTC_SEC		0x02
#define   RTC_MIN		0x03
#define   RTC_HOUR		0x04
#define   RTC_DAY		0x05
#define   RTC_WEEK		0x06
#define   RTC_MON		0x07
#define   RTC_YEAR		0x08
#define   RTC_A_MIN 		0x09
#define   RTC_A_HOUR	0x0A
#define   RTC_A_DAY 		0x0B
#define   RTC_A_WEEK	0x0C
#define   RTC_CLKOUT	0x0D
#define   RTC_T_CTL 	0x0E
#define   RTC_T_COUNT	0x0F
#define   CENTURY	0x80
#define   TI		0x10
#define   AF		0x08
#define   TF		0x04
#define   AIE		0x02
#define   TIE		0x01
#define   FE		0x80
#define   TE		0x80
#define   FD1		0x02
#define   FD0		0x01
#define   TD1		0x02
#define   TD0		0x01
#define   VL		0x80

#define HYM8563_REG_LEN 	0x10
#define HYM8563_RTC_SECTION_LEN	0x07

struct hym8563_platform_data {
    unsigned int speed;
    unsigned int mode;
    unsigned int reg_byte_cnt;
};

#endif

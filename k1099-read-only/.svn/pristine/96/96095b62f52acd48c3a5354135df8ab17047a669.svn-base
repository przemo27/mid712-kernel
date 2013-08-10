/*
 * linux/drivers/video/s3c2410fb.h
 * Copyright (c) Arnaud Patard
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *	    S3C2410 LCD Controller Frame Buffer Driver
 *	    based on skeletonfb.c, sa1100fb.h
 *
 * ChangeLog
 *
 * 2004-12-04: Arnaud Patard <arnaud.patard@rtp-net.org>
 *      - Moved dprintk to s3c2410fb.c
 *
 * 2004-09-07: Arnaud Patard <arnaud.patard@rtp-net.org>
 * 	- Renamed from h1940fb.h to s3c2410fb.h
 * 	- Changed h1940 to s3c2410
 *
 * 2004-07-15: Arnaud Patard <arnaud.patard@rtp-net.org>
 *	- First version
 */

#ifndef __RK28FB_H
#define __RK28FB_H

struct rk28fb_gpio{
    u32 display_on;
    u32 lcd_cs;
    u32 lcd_standby;
};

struct rk28fb_iomux{
    char *data16;
    char *data18;
    char *data24;
    char *den;
    char *vsync;
};


struct rk28fb_mach_info {
    struct rk28fb_gpio *gpio;
    struct rk28fb_iomux *iomux;
};


extern void __init rk28_add_device_lcdc(void);


#endif

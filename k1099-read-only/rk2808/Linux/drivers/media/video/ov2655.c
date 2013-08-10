/*
o* Driver for MT9M001 CMOS Image Sensor from Micron
 *
 * Copyright (C) 2008, Guennadi Liakhovetski <kernel@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/videodev2.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/log2.h>
#include <linux/gpio.h>

#include <media/v4l2-common.h>
#include <media/v4l2-chip-ident.h>
#include <media/soc_camera.h>
#include <asm/arch/iomux.h>
#include <asm/arch/gpio.h>
#include <linux/delay.h>

struct reginfo{
   u16 reg;
   u8 val;
};

/* init 800X600 SVGA */
static struct reginfo ov2655_init_data[] = {
#if 1// sensor image will mirror if modified as 0
{0x308c, 0x80},
{0x308d, 0x0e},
{0x360b, 0x00},
{0x30b0, 0xff},
{0x30b1, 0xff},
{0x30b2, 0x24},

{0x300e, 0x34},
{0x300f, 0xa6},
{0x3010, 0x81},
{0x3082, 0x01},
{0x30f4, 0x01},
{0x3090, 0x3b},//0x33},
{0x3091, 0xc0},
{0x30ac, 0x42},

{0x30d1, 0x08},
{0x30a8, 0x56},
{0x3015, 0x03},
{0x3093, 0x00},
{0x307e, 0xe5},
{0x3079, 0x00},
{0x30aa, 0x42},
{0x3017, 0x40},
{0x30f3, 0x82},
{0x306a, 0x0c},
{0x306d, 0x00},
{0x336a, 0x3c},
{0x3076, 0x6a},
{0x30d9, 0x8c},
{0x3016, 0x82},
{0x3601, 0x30},
{0x304e, 0x88},
{0x30f1, 0x82},
{0x306f, 0x14},

{0x3012, 0x10},
{0x3011, 0x01},
{0x302A, 0x02},
{0x302B, 0xE6},
{0x3028, 0x07},
{0x3029, 0x93},

{0x3391, 0x06},
{0x3394, 0x38},
{0x3395, 0x38},

{0x3015, 0x02},
{0x302d, 0x00},
{0x302e, 0x00},

{0x3013, 0xf7},
{0x3018, 0x80},
{0x3019, 0x70},
{0x301a, 0xd4},

{0x30af, 0x00},
{0x3048, 0x1f}, 
{0x3049, 0x4e},  
{0x304a, 0x20},  
{0x304f, 0x20},  
{0x304b, 0x02}, 
{0x304c, 0x00},  
{0x304d, 0x02},  
{0x304f, 0x20},  
{0x30a3, 0x10},  
{0x3013, 0xf7}, 
{0x3014, 0x84},  
{0x3071, 0x00},
{0x3070, 0x5d},
{0x3073, 0x00},
{0x3072, 0x5d},
{0x301c, 0x07},
{0x301d, 0x07},
{0x304d, 0x42},     
{0x304a, 0x40},  
{0x304f, 0x40},  
{0x3095, 0x07},  
{0x3096, 0x16}, 
{0x3097, 0x1d}, 

{0x3020, 0x01},
{0x3021, 0x18},
{0x3022, 0x00},
{0x3023, 0x06},
{0x3024, 0x06},
{0x3025, 0x58},
{0x3026, 0x02},
{0x3027, 0x5e},
{0x3088, 0x03},
{0x3089, 0x20},
{0x308a, 0x02},
{0x308b, 0x58},
{0x3316, 0x64},
{0x3317, 0x25},
{0x3318, 0x80},
{0x3319, 0x08},
{0x331a, 0x64},
{0x331b, 0x4b},
{0x331c, 0x00},
{0x331d, 0x38},
{0x3100, 0x00},

{0x3320, 0xfa},
{0x3321, 0x11},
{0x3322, 0x92},
{0x3323, 0x01},
{0x3324, 0x97},
{0x3325, 0x02},
{0x3326, 0xff},
{0x3327, 0x0c},
{0x3328, 0x10}, 
{0x3329, 0x10},
{0x332a, 0x58},
{0x332b, 0x56},
{0x332c, 0xbe},
{0x332d, 0xe1},
{0x332e, 0x3a},
{0x332f, 0x36},
{0x3330, 0x4d},
{0x3331, 0x44},
{0x3332, 0xf8},
{0x3333, 0x0a},
{0x3334, 0xf0},
{0x3335, 0xf0},
{0x3336, 0xf0},
{0x3337, 0x40},
{0x3338, 0x40},
{0x3339, 0x40},
{0x333a, 0x00},
{0x333b, 0x00}, 

{0x3380, 0x28}, 
{0x3381, 0x48}, 
{0x3382, 0x10}, 
{0x3383, 0x22}, 
{0x3384, 0xc0}, 
{0x3385, 0xe2}, 
{0x3386, 0xe2}, 
{0x3387, 0xf2}, 
{0x3388, 0x10}, 
{0x3389, 0x98}, 
{0x338a, 0x00}, 

{0x3340, 0x04},
{0x3341, 0x07},
{0x3342, 0x19},
{0x3343, 0x34},
{0x3344, 0x4a},
{0x3345, 0x5a},
{0x3346, 0x67},
{0x3347, 0x71},
{0x3348, 0x7c},
{0x3349, 0x8c},
{0x334a, 0x9b},
{0x334b, 0xa9},
{0x334c, 0xc0},
{0x334d, 0xd5},
{0x334e, 0xe8},
{0x334f, 0x20},

{0x3350, 0x37},//0x33},  
{0x3351, 0x27},//0x28},  
{0x3352, 0x00}, 
{0x3353, 0x16},	 
{0x3354, 0x00}, 
{0x3355, 0x85},  
{0x3356, 0x35},  
{0x3357, 0x28},  
{0x3358, 0x00}, 
{0x3359, 0x13},  
{0x335a, 0x00}, 
{0x335b, 0x85},  
{0x335c, 0x37},//0x34},  
{0x335d, 0x28},  
{0x335e, 0x00}, 
{0x335f, 0x13},  
{0x3360, 0x00}, 
{0x3361, 0x85},  
{0x3363, 0x70}, 
{0x3364, 0x7f}, 
{0x3365, 0x00}, 
{0x3366, 0x00}, 
{0x3362, 0x90},

{0x3301, 0xff},
{0x338B, 0x11},
{0x338c, 0x10},
{0x338d, 0x40},

{0x3370, 0xd0},
{0x3371, 0x00},
{0x3372, 0x00},
{0x3373, 0x30},
{0x3374, 0x10},
{0x3375, 0x10},
{0x3376, 0x04},
{0x3377, 0x00},
{0x3378, 0x04},
{0x3379, 0x80},

{0x3069, 0x84},
#if(defined(CONFIG_BOARD_NX7005))
{0x307c, 0x13},//0x10},
#else
{0x307c, 0x10},//0x10},0x13
#endif
{0x3087, 0x02},

{0x3300, 0xfc},
{0x3302, 0x11},
{0x3400, 0x02},
{0x3606, 0x20},
{0x3601, 0x30},
{0x30f3, 0x83},
{0x304e, 0x88},

{0x30aa, 0x72},
{0x30a3, 0x80},
{0x30a1, 0x41},

{0x3086, 0x0f},
{0x3086, 0x00},
    
{0x0, 0x0},   //end flag
#else//mirror
{0x308c, 0x80},
{0x308d, 0x0e},
{0x360b, 0x00},
{0x30b0, 0xff},
{0x30b1, 0xff},
{0x30b2, 0x24},

{0x300e, 0x34},
{0x300f, 0xa6},
{0x3010, 0x81},
{0x3082, 0x01},
{0x30f4, 0x01},
{0x3090, 0x33},
{0x3091, 0xc0},
{0x30ac, 0x42},

{0x30d1, 0x08},
{0x30a8, 0x56},
{0x3015, 0x03},
{0x3093, 0x00},
{0x307e, 0xe5},
{0x3079, 0x00},
{0x30aa, 0x42},
{0x3017, 0x40},
{0x30f3, 0x82},
{0x306a, 0x0c},
{0x306d, 0x00},
{0x336a, 0x3c},
{0x3076, 0x6a},
{0x30d9, 0x8c},
{0x3016, 0x82},
{0x3601, 0x30},
{0x304e, 0x88},
{0x30f1, 0x82},
{0x306f, 0x14},

{0x3012, 0x10},
{0x3011, 0x01},
{0x302A, 0x02},
{0x302B, 0xE6},
{0x3028, 0x07},
{0x3029, 0x93},

{0x3391, 0x06},
{0x3394, 0x38},
{0x3395, 0x38},

{0x3015, 0x02},
{0x302d, 0x00},
{0x302e, 0x00},

{0x3013, 0xf7},
{0x3018, 0x80},
{0x3019, 0x70},
{0x301a, 0xd4},

{0x30af, 0x00},
{0x3048, 0x1f}, 
{0x3049, 0x4e},  
{0x304a, 0x20},  
{0x304f, 0x20},  
{0x304b, 0x02}, 
{0x304c, 0x00},  
{0x304d, 0x02},  
{0x304f, 0x20},  
{0x30a3, 0x10},  
{0x3013, 0xf7}, 
{0x3014, 0x84},  
{0x3071, 0x00},
{0x3070, 0x5d},
{0x3073, 0x00},
{0x3072, 0x5d},
{0x301c, 0x07},
{0x301d, 0x07},
{0x304d, 0x42},     
{0x304a, 0x40},  
{0x304f, 0x40},  
{0x3095, 0x07},  
{0x3096, 0x16}, 
{0x3097, 0x1d}, 

{0x3020, 0x01},
{0x3021, 0x18},
{0x3022, 0x00},
{0x3023, 0x06},
{0x3024, 0x06},
{0x3025, 0x58},
{0x3026, 0x02},
{0x3027, 0x5e},
{0x3088, 0x03},
{0x3089, 0x20},
{0x308a, 0x02},
{0x308b, 0x58},
{0x3316, 0x64},
{0x3317, 0x25},
{0x3318, 0x80},
{0x3319, 0x08},
{0x331a, 0x64},
{0x331b, 0x4b},
{0x331c, 0x00},
{0x331d, 0x38},
{0x3100, 0x00},

{0x3320, 0xfa},
{0x3321, 0x11},
{0x3322, 0x92},
{0x3323, 0x01},
{0x3324, 0x97},
{0x3325, 0x02},
{0x3326, 0xff},
{0x3327, 0x0c},
{0x3328, 0x10}, 
{0x3329, 0x10},
{0x332a, 0x58},
{0x332b, 0x56},
{0x332c, 0xbe},
{0x332d, 0xe1},
{0x332e, 0x3a},
{0x332f, 0x36},
{0x3330, 0x4d},
{0x3331, 0x44},
{0x3332, 0xf8},
{0x3333, 0x0a},
{0x3334, 0xf0},
{0x3335, 0xf0},
{0x3336, 0xf0},
{0x3337, 0x40},
{0x3338, 0x40},
{0x3339, 0x40},
{0x333a, 0x00},
{0x333b, 0x00}, 

{0x3380, 0x28}, 
{0x3381, 0x48}, 
{0x3382, 0x10}, 
{0x3383, 0x22}, 
{0x3384, 0xc0}, 
{0x3385, 0xe2}, 
{0x3386, 0xe2}, 
{0x3387, 0xf2}, 
{0x3388, 0x10}, 
{0x3389, 0x98}, 
{0x338a, 0x00}, 

{0x3340, 0x04},
{0x3341, 0x07},
{0x3342, 0x19},
{0x3343, 0x34},
{0x3344, 0x4a},
{0x3345, 0x5a},
{0x3346, 0x67},
{0x3347, 0x71},
{0x3348, 0x7c},
{0x3349, 0x8c},
{0x334a, 0x9b},
{0x334b, 0xa9},
{0x334c, 0xc0},
{0x334d, 0xd5},
{0x334e, 0xe8},
{0x334f, 0x20},

{0x3350, 0x33},  
{0x3351, 0x28},  
{0x3352, 0x00}, 
{0x3353, 0x16},	 
{0x3354, 0x00}, 
{0x3355, 0x85},  
{0x3356, 0x35},  
{0x3357, 0x28},  
{0x3358, 0x00}, 
{0x3359, 0x13},  
{0x335a, 0x00}, 
{0x335b, 0x85},  
{0x335c, 0x34},  
{0x335d, 0x28},  
{0x335e, 0x00}, 
{0x335f, 0x13},  
{0x3360, 0x00}, 
{0x3361, 0x85},  
{0x3363, 0x70}, 
{0x3364, 0x7f}, 
{0x3365, 0x00}, 
{0x3366, 0x00}, 
{0x3362, 0x90},

{0x3301, 0xff},
{0x338B, 0x11},
{0x338c, 0x10},
{0x338d, 0x40},

{0x3370, 0xd0},
{0x3371, 0x00},
{0x3372, 0x00},
{0x3373, 0x30},
{0x3374, 0x10},
{0x3375, 0x10},
{0x3376, 0x04},
{0x3377, 0x00},
{0x3378, 0x04},
{0x3379, 0x80},

{0x3069, 0x84},
{0x307c, 0x10},
{0x3087, 0x02},

{0x3300, 0xfc},
{0x3302, 0x11},
{0x3400, 0x02},
{0x3606, 0x20},
{0x3601, 0x30},
{0x30f3, 0x83},
{0x304e, 0x88},

{0x30aa, 0x72},
{0x30a3, 0x80},
{0x30a1, 0x41},

{0x3086, 0x0f},
{0x3086, 0x00},
    
{0x0, 0x0},   //end flag
#endif
};

/* 1600X1200 UXGA */
static struct reginfo ov2655_uxga[] = {
    
{0x300E, 0x34}, 
{0x3011, 0x01},
{0x3012, 0x00},
{0x302a, 0x05},
{0x302b, 0xCB},
{0x306f, 0x54}, 
{0x3362, 0x80},

{0x3070, 0x5d},
{0x3072, 0x5d},
{0x301c, 0x0f},
{0x301d, 0x0f},

{0x3020, 0x01},
{0x3021, 0x18},
{0x3022, 0x00},
{0x3023, 0x0A},
{0x3024, 0x06},
{0x3025, 0x58},
{0x3026, 0x04},
{0x3027, 0xbc},
{0x3088, 0x06},
{0x3089, 0x40},
{0x308A, 0x04},
{0x308B, 0xB0},
{0x3316, 0x64},
{0x3317, 0x4B},
{0x3318, 0x00},
{0x3319, 0x6C},
{0x331A, 0x64},
{0x331B, 0x4B},
{0x331C, 0x00},
{0x331D, 0x6C},
{0x3302, 0x01},
    
    {0x0, 0x0}, 
};

/* 1280X1024 SXGA */
static struct reginfo ov2655_sxga[] = {
    
{0x300E, 0x34}, 
{0x3011, 0x01},
{0x3012, 0x00},
{0x302a, 0x05},
{0x302b, 0xCB},
{0x306f, 0x54}, 
{0x3362, 0x80},

{0x3070, 0x5d},
{0x3072, 0x5d},
{0x301c, 0x0f},
{0x301d, 0x0f},

{0x3020, 0x01},
{0x3021, 0x18},
{0x3022, 0x00},
{0x3023, 0x0A},
{0x3024, 0x06},
{0x3025, 0x58},
{0x3026, 0x04},
{0x3027, 0xbc},
{0x3088, 0x05},
{0x3089, 0x00},
{0x308A, 0x04},
{0x308B, 0x00},
{0x3316, 0x64},
{0x3317, 0x4B},
{0x3318, 0x00},
{0x3319, 0x6C},
{0x331A, 0x50},
{0x331B, 0x40},
{0x331C, 0x00},
{0x331D, 0x6C},
{0x3302, 0x11},
    
    {0x0, 0x0}, 
};

/* 800X600 SVGA*/
static struct reginfo ov2655_svga[] = {
    
{0x300E, 0x34},
{0x3011, 0x01},
{0x3012, 0x10},
{0x302a, 0x02},
{0x302b, 0xE6},
{0x306f, 0x14},
{0x3362, 0x90},

{0x3070, 0x5d},
{0x3072, 0x5d},
{0x301c, 0x07},
{0x301d, 0x07},

{0x3020, 0x01},
{0x3021, 0x18},
{0x3022, 0x00},
{0x3023, 0x06},
{0x3024, 0x06},
{0x3025, 0x58},
{0x3026, 0x02},
{0x3027, 0x5E},
{0x3088, 0x03},
{0x3089, 0x20},
{0x308A, 0x02},
{0x308B, 0x58},
{0x3316, 0x64},
{0x3317, 0x25},
{0x3318, 0x80},
{0x3319, 0x08},
{0x331A, 0x64},
{0x331B, 0x4B},
{0x331C, 0x00},
{0x331D, 0x38},
{0x3302, 0x11},
    
    {0x0, 0x0}, 
};

/* 640X480 VGA */
static struct reginfo ov2655_vga[] = {
    
{0x300E, 0x34},
{0x3011, 0x01},
{0x3012, 0x10},
{0x302a, 0x02},
{0x302b, 0xE6},
{0x306f, 0x14},
{0x3362, 0x90},

{0x3070, 0x5D},
{0x3072, 0x5D},
{0x301c, 0x07},
{0x301d, 0x07},

{0x3020, 0x01},
{0x3021, 0x18},
{0x3022, 0x00},
{0x3023, 0x06},
{0x3024, 0x06},
{0x3025, 0x58},
{0x3026, 0x02},
{0x3027, 0x61},
{0x3088, 0x02},
{0x3089, 0x88},
{0x308A, 0x01},
{0x308B, 0xe0},
{0x3316, 0x64},
{0x3317, 0x25},
{0x3318, 0x80},
{0x3319, 0x08},
{0x331A, 0x28},
{0x331B, 0x1e},
{0x331C, 0x08},
{0x331D, 0x38},
{0x3302, 0x11},
    
    {0x0, 0x0}, 
};

/* 352X288 CIF */
static struct reginfo ov2655_cif[] = {
    
{0x300E, 0x34}, 
{0x3011, 0x01},
{0x3012, 0x10},
{0x302a, 0x02},
{0x302b, 0xE6},
{0x306f, 0x14}, 
{0x3362, 0x90},

{0x3070, 0x5d},
{0x3072, 0x5d},
{0x301c, 0x07},
{0x301d, 0x07},

{0x3020, 0x01},
{0x3021, 0x18},
{0x3022, 0x00},
{0x3023, 0x06},
{0x3024, 0x06},
{0x3025, 0x58},
{0x3026, 0x02},
{0x3027, 0x61},
{0x3088, 0x01},
{0x3089, 0x68},
{0x308a, 0x01},
{0x308b, 0x20},
{0x3316, 0x64},
{0x3317, 0x25},
{0x3318, 0x80},
{0x3319, 0x08},
{0x331a, 0x16},
{0x331b, 0x12},
{0x331c, 0x08},
{0x331d, 0x38},
{0x3100, 0x00},
    {0x3302, 0x11},
    
    {0x0, 0x0}, 
};

/* 320*240 QVGA */
static  struct reginfo ov2655_qvga[] = {
    
{0x300E, 0x34},
{0x3011, 0x01},
{0x3012, 0x10},
{0x302a, 0x02},
{0x302b, 0xE6},
{0x306f, 0x14},
{0x3362, 0x90},

{0x3070, 0x5D},
{0x3072, 0x5D},
{0x301c, 0x07},
{0x301d, 0x07},

{0x3020, 0x01},
{0x3021, 0x18},
{0x3022, 0x00},
{0x3023, 0x06},
{0x3024, 0x06},
{0x3025, 0x58},
{0x3026, 0x02},
{0x3027, 0x61},
{0x3088, 0x01},
{0x3089, 0x40},
{0x308A, 0x00},
{0x308B, 0xf0},
{0x3316, 0x64},
{0x3317, 0x25},
{0x3318, 0x80},
{0x3319, 0x08},
{0x331A, 0x14},
{0x331B, 0x0f},
{0x331C, 0x00},
{0x331D, 0x38},
{0x3302, 0x11},
    
    {0x0, 0x0}, 
};

/* 160X120 QQVGA*/
static struct reginfo ov2655_qqvga[] = {
    
{0x300E, 0x34},
{0x3011, 0x01},
{0x3012, 0x10},
{0x302a, 0x02},
{0x302b, 0xE6},
{0x306f, 0x14},
{0x3362, 0x90},

{0x3070, 0x5d},
{0x3072, 0x5d},
{0x301c, 0x07},
{0x301d, 0x07},

{0x3020, 0x01},
{0x3021, 0x18},
{0x3022, 0x00},
{0x3023, 0x06},
{0x3024, 0x06},
{0x3025, 0x58},
{0x3026, 0x02},
{0x3027, 0x61},
{0x3088, 0x00},
{0x3089, 0xa0},
{0x308a, 0x00},
{0x308b, 0x78},
{0x3316, 0x64},
{0x3317, 0x25},
{0x3318, 0x80},
{0x3319, 0x08},
{0x331a, 0x0a},
{0x331b, 0x07},
{0x331c, 0x80},
{0x331d, 0x38},
{0x3100, 0x00},
{0x3302, 0x11},
    
    {0x0, 0x0}, 
};

static const struct soc_camera_data_format ov2655_colour_formats[] = {
     {
	/* Order important: first natively supported,
	 * second supported with a GPIO extender */
		.name		= "ov2655 UYVY",
		.depth		= 8,
		.fourcc		= V4L2_PIX_FMT_UYVY,
     },

     {
               .name = "ov2655 YUYV",
               .depth = 8,
               .fourcc = V4L2_PIX_FMT_YUYV,
     },
};


struct ov2655{
	struct i2c_client *client;
	struct soc_camera_device icd;
	int model;	/* V4L2_IDENT_MT9M001* codes from v4l2-chip-ident.h */
	int switch_gpio;
	unsigned char autoexposure;
	unsigned char datawidth;

};

#define OV2655_IIC_ADDR 	    0x60  

static unsigned short normal_i2c[] = {OV2655_IIC_ADDR >> 1 , I2C_CLIENT_END};
static unsigned short ignore = I2C_CLIENT_END;
static struct i2c_client_address_data addr_data_ov2655 = {
	.normal_i2c	= normal_i2c,
	.probe		= &ignore,
	.ignore		= &ignore,
};

static int ov2655_probe(struct i2c_adapter *adapter, int addr, int kind);
static int ov2655_video_probe(struct soc_camera_device *);
static void ov2655_video_remove(struct soc_camera_device *);
static int ov2655_get_control(struct soc_camera_device *, struct v4l2_control *);
static int ov2655_set_control(struct soc_camera_device *, struct v4l2_control *);

/* ov2655 register write */
static int ov2655_write(struct i2c_client *client, u16 reg, u8 val)
{
   int err;
   u8 buf[3];
   struct i2c_msg msg[1];
   
   buf[0] = reg >> 8;     
   buf[1] = reg & 0xFF;
   buf[2] = val; 
   
   msg->addr = client->addr;
   msg->flags = 0;
   msg->buf = buf;
   msg->len = sizeof(buf);

   err = i2c_transfer(client->adapter, msg, 1);

   if (err >= 0)
   {
      return 0;
   }

   return err;
}

/* ov2655 register read */
static int ov2655_read(struct i2c_client *client, u16 reg, u8 *val)
{
   int err;
   u8 buf[2];
   struct i2c_msg msg[1];
   
   buf[0] = reg >> 8; 
   buf[1] = reg & 0xFF;
   
   msg->addr = client->addr;
   msg->flags |= I2C_M_RD;
   msg->buf = buf;
   msg->len = sizeof(buf);

   err = i2c_transfer(client->adapter, msg, 1);

   if(err >= 0)
   {
      *val = buf[0];
      return 0;
   } 
   
   return err;
}

/* write a array of registers  */ 
static int ov2655_write_array(struct i2c_client *client, struct reginfo *regarray)
{
   int err;
   int i = 0;
   
   while(regarray[i++].reg != 0)
   {
      err = ov2655_write(client, regarray[i].reg, regarray[i].val);
      if (err != 0)
      {
          printk("write failed current i = %d\n", i);
          return err;
      } 
   }
   return 0; 
}

static int ov2655_init(struct soc_camera_device *icd)
{
    struct ov2655 *ov2655 = container_of(icd, struct ov2655, icd);
    u8 val;
    int ret;
    u16 pid = 0;

    /* soft reset */
    ret = ov2655_write(ov2655->client, 0x3012, 0x80);
    if (ret != 0)
    {
       printk("soft reset ov2655 failed\n");
       return -ENODEV;
    }
     
    udelay(5 * 1000);  //delay 5 microseconds 
    
    /* check if it is an ov2655 sensor */ 
    ret = ov2655_read(ov2655->client, 0x300a, &val);    
    if (ret != 0)
    {
       printk("read chip id high byte failed\n");
       return -ENODEV;
    }  
    
    pid |= (val << 8);

    ret = ov2655_read(ov2655->client, 0x300b, &val);
    if (ret != 0)
    {
       printk("read chip id low byte failed\n");
       return -ENODEV;
    }
    
    pid |= (val & 0xff);
    
    printk("sensor product id is %x\n",pid); 
/*
    if (pid != 0x2656)
    {
       printk("error: devicr mismatched \n");
       return -ENODEV; 
    }
 */   
    ret = ov2655_write_array(ov2655->client, ov2655_init_data);
    if (ret != 0)
    {
       printk("error: ov2655 initial failed\n");
       return ret;
    } 

    return 0;
}

static int ov2655_release(struct soc_camera_device *icd)
{
	return 0;
}

static int ov2655_start_capture(struct soc_camera_device *icd)
{
	return 0;
}

static int ov2655_stop_capture(struct soc_camera_device *icd)
{
	return 0;
}

#if (defined(CONFIG_BOARD_IPAD)||defined(CONFIG_BOARD_E700))
#define SENSOR_RESET   	GPIOPortB_Pin7		// SENSOR RESET
#define SENSOR_POWERDN   GPIOPortH_Pin6		// SENSOR POWER
#elif defined(CONFIG_BOARD_IPAD8)
#define SENSOR_RESET   	GPIOPortF_Pin4		// SENSOR RESET
#define SENSOR_POWERDN   GPIOPortH_Pin6		// SENSOR POWER
#endif

static int bus_switch_request(struct ov2655 *ov2655,
			      struct soc_camera_link *icl)
{
    /*
	unsigned int gpio = icl->gpio;

    int ret = gpio_request(gpio, "ov2655");
    if (ret < 0) {
        dev_err(&ov2655->client->dev, "Cannot get GPIO %u\n",
            gpio);
        return ret;
    }

    ret = gpio_direction_output(gpio, 0);
    if (ret < 0) {
        dev_err(&ov2655->client->dev,
            "Cannot set GPIO %u to output\n", gpio);
        gpio_free(gpio);
        return ret;
    }
	ov2655->switch_gpio = gpio;
    */

#if(defined(CONFIG_BOARD_NM701)||defined(CONFIG_BOARD_NX7005))
#if(defined(CONFIG_RK28_I2C_GPIO_EXPANDERS))
	pca955x_gpio_direction_output(PCA955X_Pin17, GPIO_LOW);
	pca955x_gpio_direction_output(PCA955X_Pin16, GPIO_LOW);
	mdelay(500);
	pca955x_gpio_direction_output(PCA955X_Pin16, GPIO_HIGH);
#endif    
#else
    rockchip_mux_api_set(GPIOH6_IQ_SEL_NAME, IOMUXB_GPIO1_D6);
    gpio_direction_output(GPIOPortH_Pin6, GPIO_OUT);
    __gpio_set(GPIOPortH_Pin6, GPIO_LOW);
    
#if (defined(CONFIG_BOARD_IPAD)||defined(CONFIG_BOARD_E700)||defined(CONFIG_BOARD_IPAD8)||defined(CONFIG_BOARD_NM701))
	GPIOSetPinDirection(SENSOR_RESET,GPIO_OUT);
	GPIOSetPinLevel(SENSOR_RESET,GPIO_LOW);	//
	mdelay(500);
	GPIOSetPinLevel(SENSOR_RESET,GPIO_HIGH);	//
#endif    
#endif
    
	return 0;
}

static void bus_switch_release(struct ov2655 *ov2655)
{

}

static int ov2655_set_bus_param(struct soc_camera_device *icd,
				 unsigned long flags)
{
       //struct ov2655 *ov2655 = container_of(icd, struct ov2655, icd);
	

	return 0;
}

static unsigned long ov2655_query_bus_param(struct soc_camera_device *icd)
{
	/* 0v9650 has all capture_format parameters fixed */
	return SOCAM_PCLK_SAMPLE_RISING |
		SOCAM_HSYNC_ACTIVE_HIGH |
		SOCAM_VSYNC_ACTIVE_LOW|
		SOCAM_SENSOR_UYVY;
}

static int ov2655_set_fmt_cap(struct soc_camera_device *icd,
		__u32 pixfmt, struct v4l2_rect *rect)
{
    struct ov2655 *ov2655 = container_of(icd, struct ov2655, icd);
    int ret;
    struct reginfo *ov2655_win;
    
    //printk("rect width = %d, rect height = %d\n", rect->width, rect->height);
#if 1    
    if ((rect->width <= 320) && (rect->height <= 240)) {
       ov2655_win = ov2655_qvga;
    } else if ((rect->width <= 352) && (rect->height <= 288)){
 //       ov2655_win = ov2655_cif;
        ov2655_win = ov2655_vga;
    } else if ((rect->width <= 640) && (rect->height <= 480)){
#else
    if ((rect->width <= 640) && (rect->height <= 480)){
#endif    
        ov2655_win = ov2655_vga;
    } else if((rect->width <= 800) && (rect->height <= 600)){
       ov2655_win = ov2655_svga;
    } else if((rect->width <= 1280) && (rect->height <= 1024)) {
        ov2655_win = ov2655_sxga;
    }else {
        ov2655_win = ov2655_uxga;
    }
    
    ret = ov2655_write_array(ov2655->client, ov2655_win); 
    if (ret != 0)
    {
       printk("ov2655 set format capability failed\n");
       return ret;
    }
    mdelay(250);
    //printk("\n%s..%s..%d    ******** nzy *********%d %d\n",__FUNCTION__,__FILE__,__LINE__,rect->width,rect->height);

    return 0;
}

static int ov2655_try_fmt_cap(struct soc_camera_device *icd,
			       struct v4l2_format *f)
{
	if (f->fmt.pix.height < 32 + icd->y_skip_top)
		f->fmt.pix.height = 32 + icd->y_skip_top;
	if (f->fmt.pix.height > 1200 + icd->y_skip_top)
		f->fmt.pix.height = 1200 + icd->y_skip_top;
	if (f->fmt.pix.width < 48)
		f->fmt.pix.width = 48;
	if (f->fmt.pix.width > 1600)
		f->fmt.pix.width = 1600;
	f->fmt.pix.width &= ~0x01; /* has to be even, unsure why was ~3 */

	return 0;
}

static int ov2655_get_chip_id(struct soc_camera_device *icd,
			       struct v4l2_chip_ident *id)
{
	struct ov2655 *ov2655 = container_of(icd, struct ov2655, icd);

	if (id->match_type != V4L2_CHIP_MATCH_I2C_ADDR)
		return -EINVAL;

	if (id->match_chip != ov2655->client->addr)
		return -ENODEV;

	id->ident	= ov2655->model;
	id->revision	= 0;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov2655_get_register(struct soc_camera_device *icd,
				struct v4l2_register *reg)
{
	struct ov2655 *ov2655 = container_of(icd, struct ov2655, icd);

	if (reg->match_type != V4L2_CHIP_MATCH_I2C_ADDR || reg->reg > 0xff)
		return -EINVAL;

	if (reg->match_chip != ov2655->client->addr)
		return -ENODEV;

    char reg = reg->reg;
    int ret = ov2655_rx_data(ov2655->client, &reg, 1);
    if (!ret)
        reg->val = reg;

	return ret;
}

static int ov2655_set_register(struct soc_camera_device *icd,
				struct v4l2_register *reg)
{
	struct ov2655 *ov2655 = container_of(icd, struct ov2655, icd);

	if (reg->match_type != V4L2_CHIP_MATCH_I2C_ADDR || reg->reg > 0xff)
		return -EINVAL;

	if (reg->match_chip != ov2655->client->addr)
		return -ENODEV;

    char reg[2];
    reg[0] = reg->reg;
    reg[1] = reg->val;
    int ret = ov2655_tx_data(ov2655->client, reg, 2);

	return ret;
}
#endif

static const struct v4l2_queryctrl ov2655_controls[] = {
	{
		.id		= V4L2_CID_VFLIP,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Flip Vertically",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 0,
	}, {
		.id		= V4L2_CID_GAIN,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Gain",
		.minimum	= 0,
		.maximum	= 127,
		.step		= 1,
		.default_value	= 64,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
	}, {
		.id		= V4L2_CID_EXPOSURE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Exposure",
		.minimum	= 1,
		.maximum	= 255,
		.step		= 1,
		.default_value	= 255,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
	}, {
		.id		= V4L2_CID_EXPOSURE_AUTO,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Automatic Exposure",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 1,
	}
};

static int ov2655_get_control(struct soc_camera_device *icd, struct v4l2_control *ctrl)
{
	//struct ov2655 *ov2655 = container_of(icd, struct ov2655, icd);
	//int data;

	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		break;
	}
	return 0;
}

static struct soc_camera_ops ov2655_ops = {
	.owner			= THIS_MODULE,
	.probe			= ov2655_video_probe,
	.remove			= ov2655_video_remove,
	.init			= ov2655_init,
	.release		= ov2655_release,
	.start_capture		= ov2655_start_capture,
	.stop_capture		= ov2655_stop_capture,
	.set_fmt_cap		= ov2655_set_fmt_cap,
	.try_fmt_cap		= ov2655_try_fmt_cap,
	.set_bus_param		= ov2655_set_bus_param,
	.query_bus_param	= ov2655_query_bus_param,
	.controls		= ov2655_controls,
	.num_controls		= ARRAY_SIZE(ov2655_controls),
	.get_control		= ov2655_get_control,
	.set_control		= ov2655_set_control,
	.get_chip_id		= ov2655_get_chip_id,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.get_register		= ov2655_get_register,
	.set_register		= ov2655_set_register,
#endif
};

static int ov2655_set_control(struct soc_camera_device *icd, struct v4l2_control *ctrl)
{
	//struct ov2655 *ov2655 = container_of(icd, struct ov2655, icd);
	const struct v4l2_queryctrl *qctrl;
	//int data;

	qctrl = soc_camera_find_qctrl(&ov2655_ops, ctrl->id);

	if (!qctrl)
		return -EINVAL;

	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
		break;
	case V4L2_CID_GAIN:
		break;
	case V4L2_CID_EXPOSURE:
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		break;
	}
	return 0;
}

/* Interface active, can use i2c. If it fails, it can indeed mean, that
 * this wasn't our capture interface, so, we wait for the right one */
static int ov2655_video_probe(struct soc_camera_device *icd)
{
	struct ov2655 *ov2655 = container_of(icd, struct ov2655, icd);
	int ret;

	/* We must have a parent by now. And it cannot be a wrong one.
	 * So this entire test is completely redundant. */
	if (!icd->dev.parent ||
	    to_soc_camera_host(icd->dev.parent)->nr != icd->iface)
		return -ENODEV;

        ov2655->model = V4L2_IDENT_OV2655;
        icd->formats = ov2655_colour_formats;
        icd->num_formats = ARRAY_SIZE(ov2655_colour_formats);

	/* Now that we know the model, we can start video */
	ret = soc_camera_video_start(icd);
	if (ret)
		goto eisis;

	return 0;

eisis:

	return ret;
}

static void ov2655_video_remove(struct soc_camera_device *icd)
{
	struct ov2655 *ov2655 = container_of(icd, struct ov2655, icd);

	dev_dbg(&icd->dev, "Video %x removed: %p, %p\n", ov2655->client->addr,
		ov2655->icd.dev.parent, ov2655->icd.vdev);
	soc_camera_video_stop(&ov2655->icd);
}

static int ov2655_remove(struct i2c_client *client)
{
	struct ov2655 *ov2655 = i2c_get_clientdata(client);

	soc_camera_device_unregister(&ov2655->icd);
	kfree(ov2655);

	return 0;
}

static int ov2655_detach_client(struct i2c_client *client)
{
	ov2655_remove(client);
#if(defined(CONFIG_BOARD_NX7005))
#if defined(CONFIG_RK28_I2C_GPIO_EXPANDERS)
	pca955x_gpio_direction_output(PCA955X_Pin17,GPIO_HIGH);
    
	pca955x_gpio_direction_output(PCA955X_Pin16,GPIO_LOW);
#endif
#endif
	return i2c_detach_client(client);
}

static int ov2655_attach_adapter(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data_ov2655, ov2655_probe);
}

static struct i2c_driver ov2655_driver = {
	.driver = {
		.name = "ov2655",
	    },
	.id 	= OV2655_IIC_ADDR,
	.attach_adapter = &ov2655_attach_adapter,
	.detach_client  = &ov2655_detach_client,
};

static struct soc_camera_link iclink = {//nzy add
    .bus_id = 33, /* Must match with the camera ID above */
    .gpio   = 1,
};
    
static int ov2655_probe(struct i2c_adapter *adapter, int addr, int kind)
{
	struct ov2655 *ov2655;
	struct soc_camera_device *icd;
	struct soc_camera_link *icl;
	struct i2c_client *client = NULL;
	int ret;
	
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)){
	    ret = -EIO;
            goto exit_check_functionality_failed;
	}

	client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (!client) {
	    ret = -ENOMEM;
            goto exit_alloc_client_failed;
	}

	strlcpy(client->name, "ov2655", I2C_NAME_SIZE);
	client->addr = addr;
	client->adapter = adapter;
	client->driver = &ov2655_driver;
	client->addressBit = I2C_7BIT_ADDRESS_16BIT_REG;
	client->mode = NORMALMODE;
	client->Channel = I2C_CH1;
	client->speed = 100;	
	ret = i2c_attach_client(client);
	if (ret) {
            goto exit_i2c_attach_client_failed;
	}
        printk("\n%s..%s..%d    ******** nzy *********\n",__FUNCTION__,__FILE__,__LINE__);
	
	ov2655 = kzalloc(sizeof(struct ov2655), GFP_KERNEL);
	if (!ov2655) {
       	    ret = -ENOMEM;
            goto exit_alloc_data_failed;
	}

	ov2655->client = client;
	i2c_set_clientdata(client, ov2655);
        //icl = &iclink;//client->dev.platform_data;

	/* Second stage probe - when a capture adapter is there */
	icd = &ov2655->icd;
	icd->ops	= &ov2655_ops;
	icd->control	= &client->dev;
	icd->x_min	= 0;
	icd->y_min	= 0;
	icd->x_current	= 0;
	icd->y_current	= 0;
	icd->width_min	= 48;
	icd->width_max	= 1600;
	icd->height_min	= 32;
	icd->height_max	= 1200;
	icd->y_skip_top	= 1;
	icd->iface	= 33;//icl->bus_id;
	/* Default datawidth - this is the only width this camera (normally)
	 * supports. It is only with extra logic that it can support
	 * other widths. Therefore it seems to be a sensible default. */
	ov2655->datawidth = 8;
	/* Simulated autoexposure. If enabled, we calculate shutter width
	 * ourselves in the driver based on vertical blanking and frame width */
	ov2655->autoexposure = 1;

	ret = bus_switch_request(ov2655, icl);
	if (ret)
		goto exit_bus_switch_request_failed;
		
	ret = soc_camera_device_register(icd);
	if (ret)
        goto exit_soc_camera_device_register_failed;
                
        printk("\n%s..%s..%d    ******** nzy *********\n",__FUNCTION__,__FILE__,__LINE__);
	return 0;

exit_soc_camera_device_register_failed:
    bus_switch_release(ov2655);
exit_bus_switch_request_failed:
	kfree(ov2655);
exit_alloc_data_failed:
   i2c_detach_client(client);
exit_i2c_attach_client_failed:
	kfree(client);
exit_alloc_client_failed:
exit_check_functionality_failed:
	return ret;
}

static int __init ov2655_mod_init(void)
{
	return i2c_add_driver(&ov2655_driver);
}

static void __exit ov2655_mod_exit(void)
{
	i2c_del_driver(&ov2655_driver);
}

#if(defined(CONFIG_BOARD_NX7005))
late_initcall(ov2655_mod_init);
#else
module_init(ov2655_mod_init);
#endif
module_exit(ov2655_mod_exit);

MODULE_DESCRIPTION("OV2655 Camera sensor driver");
MODULE_AUTHOR("lbt <kernel@rock-chips>");
MODULE_LICENSE("GPL");


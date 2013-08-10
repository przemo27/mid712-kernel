/*
 * include/linux/iomux.h
 *
 *  Driver for rockchip iomux
 *  Copyright (C) 2009 lhh
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ROCKCHIP_IOMUX_H__
#define __ROCKCHIP_IOMUX_H__

//IOMUX_A_CON
#define  IOMUXA_I2C0                        (0)
#define  IOMUXA_GPIO1_A45                   (1)
#define  IOMUXA_GPIO1_A67                   (0)
#define  IOMUXA_UART1_SIR                   (1)
#define  IOMUXA_I2C1                        (2)
#define  IOMUXA_GPIO1_B1                    (0)
#define  IOMUXA_UART1_SOUT                  (1)
#define  IOMUXA_CX_TIMER1_PMW               (2)
#define  IOMUXA_GPIO1_B0                    (0)
#define  IOMUXA_UART1_SIN                   (1)
#define  IOMUXA_CX_TIMER0_PWM               (2)
#define  IOMUXA_GPIO1_C237                  (0)
#define  IOMUXA_SDMMC1_CMD_DATA0_CLKOUT     (1)
#define  IOMUXA_GPIO1_C456                  (0)
#define  IOMUXA_SDMMC1_DATA123              (1)
#define  IOMUXA_GPIO1_A1237                 (0)
#define  IOMUXA_SPI1                        (1)                      
#define  IOMUXA_GPIO0_B0                    (0)
#define  IOMUXA_SPI0_CSN1                   (1)
#define  IOMUXA_SDMMC1_PWR_EN               (2)
#define  IOMUXA_GPIO1_C1                    (0)
#define  IOMUXA_UART0_SOUT                  (1)
#define  IOMUXA_SDMMC1_WRITE_PRT            (2)
#define  IOMUXA_GPIO1_C0                    (0)
#define  IOMUXA_UART0_SIN                   (1)
#define  IOMUXA_SDMMC1_DETECT_N             (2)
#define  IOMUXA_GPIO1_B4                    (0)
#define  IOMUXA_PWM2                        (1)
#define  IOMUXA_SDMMC0_WRITE_PRT            (2)
#define  IOMUXA_GPIO1_B3                    (0)
#define  IOMUXA_PWM1                        (1)
#define  IOMUXA_SDMMC0_DETECT_N             (2)
#define  IOMUXA_GPIO0_B1                    (0)
#define  IOMUXA_SM_CS1_N                    (1)
#define  IOMUXA_SDMMC0_PWR_EN               (2)
#define  IOMUXA_GPIO1_D234                  (0)
#define  IOMUXA_SDMMC0_DATA123              (1)
#define  IOMUXA_GPIO1_D015                  (0)
#define  IOMUXA_SDMMC0_CMD_DATA0_CLKOUT     (1)
#define  IOMUXA_GPIO0_B567                  (0)
#define  IOMUXA_SPI0                        (1)
#define  IOMUXA_SDMMC0_DATA567              (2)
#define  IOMUXA_GPIO0_B4                    (0)
#define  IOMUXA_SPI0_CSN0                   (1)
#define  IOMUXA_SDMMC0_DATA4                (2)

///IOMUX_B_CON
#define  IOMUXB_GPIO2_24                    (0)
#define  IOMUXB_GPS_CLK                     (1)
#define  IOMUXB_HSADC_CLKOUT                (2)
#define  IOMUXB_HSADC_DATA_I98              (0)
#define  IOMUXB_TS_FAIL_TS_VALID            (1)
#define  IOMUXB_GPIO0_A7                    (0)
#define  IOMUXB_FLASH_CS3                   (1)
#define  IOMUXB_GPIO0_A6                    (0)
#define  IOMUXB_FLASH_CS2                   (1)
#define  IOMUXB_GPIO0_A5                    (0)
#define  IOMUXB_FLASH_CS1                   (1)
#define  IOMUXB_GPIO1_B5                    (0)
#define  IOMUXB_PWM3                        (1)
#define  IOMUXB_DEMOD_PWM_OUT               (2)
#define  IOMUXB_GPIO0_B3                    (0)
#define  IOMUXB_UART0_RTS_N                 (1)
#define  IOMUXB_GPIO0_B2                    (0)
#define  IOMUXB_UART0_CTS_N                 (1)
#define  IOMUXB_GPIO1_B2                    (0)
#define  IOMUXB_PWM0                        (1)
#define  IOMUXB_GPIO0_D0_7                  (0)
#define  IOMUXB_LCDC_DATA8_15               (1)
#define  IOMUXB_GPIO0_C2_7                  (0)
#define  IOMUXB_LCDC_DATA18_23              (1)
#define  IOMUXB_GPIO0_C01                   (0)
#define  IOMUXB_LCDC_DATA16_17              (1)
#define  IOMUXB_GPIO2_26                    (0)
#define  IOMUXB_LCDC_DENABLE                (1)
#define  IOMUXB_GPIO2_25                    (0)
#define  IOMUXB_LCDC_VSYNC                  (1)
#define  IOMUXB_GPIO2_14_23                 (0)
#define  IOMUXB_HSADC_DATA9_0               (1)
#define  IOMUXB_GPIO2_0_13                  (0)
#define  IOMUXB_HOST_INTERFACE              (1)
#define  IOMUXB_GPIO1_D7                    (0)
#define  IOMUXB_HSADC_CLKIN                 (1)
#define  IOMUXB_GPIO1_D6                    (0)
#define  IOMUXB_EXT_IQ_INDEX                (1)
#define  IOMUXB_I2S_INTERFACE               (0)
#define  IOMUXB_GPIO2_27_31                 (1)
#define  IOMUXB_GPIO1_B6                    (0)
#define  IOMUXB_VIP_CLKOUT                  (1)

#define IOMUX_A_CON_REG 0x20
#define IOMUX_B_CON_REG 0x24
#define DEFAULT			0
#define INITIAL			1



#define GPIOE_I2C0_SEL_NAME                     "gpioe_i2c0_sel"   
#define GPIOE_U1IR_I2C1_NAME                    "gpioe_u1ir_i2c1" 
#define GPIOF1_UART1_CPWM1_NAME                 "gpiof1_uart1_cpwm1"
#define GPIOF0_UART1_CPWM0_NAME                 "gpiof0_uart1_cpwm0"
#define GPIOG_MMC1_SEL_NAME                     "gpiog_mmc1_sel" 
#define GPIOG_MMC1D_SEL_NAME                    "gpiog_mmc1d_sel" 
#define GPIOE_SPI1_SEL_NAME                     "gpioe_spi1_sel"   
#define GPIOB0_SPI0CSN1_MMC1PCA_NAME            "gpiob0_spi0csn1_mmc1pca"
#define GPIOG1_UART0_MMC1WPT_NAME               "gpiog1_uart0_mmc1wpt"	 
#define GPIOG0_UART0_MMC1DET_NAME               "gpiog0_uart0_mmc1det"	 
#define GPIOF4_APWM2_MMC0WPT_NAME               "gpiof4_apwm2_mmc0wpt"	 
#define GPIOF3_APWM1_MMC0DETN_NAME              "gpiof3_apwm1_mmc0detn"	
#define GPIOB1_SMCS1_MMC0PCA_NAME               "gpiob1_smcs1_mmc0pca"	 
#define GPIOH_MMC0D_SEL_NAME                    "gpioh_mmc0d_sel"
#define GPIOH_MMC0_SEL_NAME                     "gpioh_mmc0_sel"   
#define GPIOB_SPI0_MMC0_NAME                    "gpiob_spi0_mmc0"
#define GPIOB4_SPI0CS0_MMC0D4_NAME              "gpiob4_spi0cs0_mmc0d4"		
                           
#define CXGPIO_GPSCLK_HSADCCLKOUT_NAME          "cxgpio_gpsclk_hsadcclkout"
#define HSADCDATA_TSCON_SEL_NAME                "hsadcdata_tscon_sel"			
#define GPIOA7_FLASHCS3_SEL_NAME                "gpioa7_flashcs3_sel"			
#define GPIOA6_FLASHCS2_SEL_NAME                "gpioa6_flashcs2_sel"			
#define GPIOA5_FLASHCS1_SEL_NAME                "gpioa5_flashcs1_sel"			
#define GPIOF5_APWM3_DPWM3_NAME                 "gpiof5_apwm3_dpwm"
#define GPIOB3_U0RTSN_SEL_NAME                  "gpiob3_u0rtsn_sel"
#define GPIOB2_U0CTSN_SEL_NAME                  "gpiob2_u0ctsn_sel"
#define GPIOF2_APWM0_SEL_NAME                   "gpiof2_apwm0_sel"
#define GPIOC_LCDC16BIT_SEL_NAME                "gpiod_lcdc16bit_sel"			
#define GPIOC_LCDC24BIT_SEL_NAME                "gpioc_lcdc24bit_sel"			
#define GPIOC_LCDC18BIT_SEL_NAME                "gpioc_lcdc18bit_sel"			
#define CXGPIO_LCDDEN_SEL_NAME                  "cxgpio_lcdden_sel"
#define CXGPIO_LCDVSYNC_SEL_NAME                "cxgpio_lcdvsync_sel"			
#define CXGPIO_HSADC_SEL_NAME                   "cxgpio_hsadc_sel"
#define CXGPIO_HOST_SEL_NAME                    "cxgpio_host_sel"
#define GPIOH7_HSADCCLK_SEL_NAME                "gpioh7_hsadcclk_sel"			
#define GPIOH6_IQ_SEL_NAME                      "gpioh6_iq_sel"   
#define CXGPIO_I2S_SEL_NAME                     "cxgpio_i2s_sel" 
#define GPIOF6_VIPCLK_SEL_NAME                  "gpiof6_vipclk_sel"



#define MUX_CFG(desc,reg,off,interl,mux_mode,bflags)	\
{						  	\
        .name = desc,                                   \
        .offset = off,                               	\
        .interleave = interl,                       	\
        .mux_reg = ROCKCHIP_IOMUX_##reg##_CON,          \
        .mode = mux_mode,                               \
        .flags = bflags,				\
},

struct mux_config {
	char *name;
	const unsigned int offset;
	unsigned int mode;
	const unsigned int mux_reg;
	const unsigned int interleave;
	unsigned int flags;
};

  
/*GRF Registers*/
typedef volatile struct tagGRF_REG
{
    unsigned int  CPU_APB_REG0;
    unsigned int  CPU_APB_REG1;
    unsigned int  CPU_APB_REG2;
    unsigned int  CPU_APB_REG3;
    unsigned int  CPU_APB_REG4;
    unsigned int  CPU_APB_REG5;
    unsigned int  CPU_APB_REG6;
    unsigned int  CPU_APB_REG7;
    unsigned int  IOMUX_A_CON;
    unsigned int  IOMUX_B_CON;
    unsigned int  GPIO0_AB_PU_CON;
    unsigned int  GPIO0_CD_PU_CON;
    unsigned int  GPIO1_AB_PU_CON;
    unsigned int  GPIO1_CD_PU_CON;
    unsigned int  OTGPHY_CON0;
    unsigned int  OTGPHY_CON1;
}GRF_REG, *pGRF_REG,*pAPB_REG;

extern int rockchip_iomux_init(void);
extern void rockchip_mux_api_set(char *name, unsigned int mode);
#endif


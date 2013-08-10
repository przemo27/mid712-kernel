/*
 *  linux/include/asm-arm/arch-rockchip/dma.h
 *
 */

#ifndef __ASM_ARCH_RK28_DMA_H
#define __ASM_ARCH_RK28_DMA_H




/******dam registers*******/
//cfg low word
#define         B_CFGL_CH_PRIOR(P)       ((P)<<5)//pri = 0~2
#define         B_CFGL_CH_SUSP           (1<<8)
#define         B_CFGL_FIFO_EMPTY        (1<<9)
#define         B_CFGL_H_SEL_DST         (0<<10)
#define         B_CFGL_S_SEL_DST         (1<<10)
#define         B_CFGL_H_SEL_SRC         (0<<11)
#define         B_CFGL_S_SEL_SRC         (1<<11)
#define         B_CFGL_LOCK_CH_L_OTF     (0<<12)
#define         B_CFGL_LOCK_CH_L_OBT     (1<<12)
#define         B_CFGL_LOCK_CH_L_OTN     (2<<12)
#define         B_CFGL_LOCK_B_L_OTF      (0<<14)
#define         B_CFGL_LOCK_B_L_OBT      (1<<14)
#define         B_CFGL_LOCK_B_L_OTN      (2<<14)
#define         B_CFGL_LOCK_CH_EN        (0<<16)
#define         B_CFGL_LOCK_B_EN         (0<<17)
#define         B_CFGL_DST_HS_POL_H      (0<<18)
#define         B_CFGL_DST_HS_POL_L      (1<<18)
#define         B_CFGL_SRC_HS_POL_H      (0<<19)
#define         B_CFGL_SRC_HS_POL_L      (1<<19)
#define         B_CFGL_RELOAD_SRC        (1<<30)
#define         B_CFGL_RELOAD_DST        (1<<31)
//cfg high word
#define         B_CFGH_FCMODE            (1<<0)
#define         B_CFGH_FIFO_MODE         (1<<1)
#define         B_CFGH_PROTCTL           (1<<2)
#define         B_CFGH_DS_UPD_EN         (1<<5)
#define         B_CFGH_SS_UPD_EN         (1<<6)
#define         B_CFGH_SRC_PER(HS)       ((HS)<<7)
#define         B_CFGH_DST_PER(HS)       ((HS)<<11)
    
//ctl low word
#define         B_CTLL_INT_EN            (1<<0)
#define         B_CTLL_DST_TR_WIDTH_8    (0<<1)
#define         B_CTLL_DST_TR_WIDTH_16   (1<<1)
#define         B_CTLL_DST_TR_WIDTH_32   (2<<1)
#define         B_CTLL_DST_TR_WIDTH(W)   ((W)<<1)
#define         B_CTLL_SRC_TR_WIDTH_8    (0<<4)
#define         B_CTLL_SRC_TR_WIDTH_16   (1<<4)
#define         B_CTLL_SRC_TR_WIDTH_32   (2<<4)
#define         B_CTLL_SRC_TR_WIDTH(W)   ((W)<<4)
#define         B_CTLL_DINC_INC          (0<<7)
#define         B_CTLL_DINC_DEC          (1<<7)
#define         B_CTLL_DINC_UNC          (2<<7)
#define         B_CTLL_DINC(W)           ((W)<<7)
#define         B_CTLL_SINC_INC          (0<<9)
#define         B_CTLL_SINC_DEC          (1<<9)
#define         B_CTLL_SINC_UNC          (2<<9)
#define         B_CTLL_SINC(W)           ((W)<<9)
#define         B_CTLL_DST_MSIZE_1       (0<<11)
#define         B_CTLL_DST_MSIZE_4       (1<<11)
#define         B_CTLL_DST_MSIZE_8       (2<<11)
#define         B_CTLL_DST_MSIZE_16      (3<<11)
#define         B_CTLL_DST_MSIZE_32      (4<<11)
#define         B_CTLL_SRC_MSIZE_1       (0<<14)
#define         B_CTLL_SRC_MSIZE_4       (1<<14)
#define         B_CTLL_SRC_MSIZE_8       (2<<14)
#define         B_CTLL_SRC_MSIZE_16      (3<<14)
#define         B_CTLL_SRC_MSIZE_32      (4<<14)
#define         B_CTLL_SRC_GATHER        (1<<17)
#define         B_CTLL_DST_SCATTER       (1<<18)
#define         B_CTLL_MEM2MEM_DMAC      (0<<20)
#define         B_CTLL_MEM2PER_DMAC      (1<<20)
#define         B_CTLL_PER2MEM_DMAC      (2<<20)
#define         B_CTLL_PER2MEM_PER       (4<<20)
#define         B_CTLL_DMS_EXP           (0<<23)
#define         B_CTLL_DMS_ARMD          (1<<23)
#define         B_CTLL_SMS_EXP           (0<<25)
#define         B_CTLL_SMS_ARMD          (1<<25)
#define         B_CTLL_LLP_DST_EN        (1<<27)
#define         B_CTLL_LLP_SRC_EN        (1<<28)

#define         DWDMA_SAR(chn)      0x00+0x58*(chn)
#define         DWDMA_DAR(chn)      0x08+0x58*(chn)
#define         DWDMA_LLP(chn)      0x10+0x58*(chn)
#define         DWDMA_CTLL(chn)     0x18+0x58*(chn)
#define         DWDMA_CTLH(chn)     0x1c+0x58*(chn)
#define         DWDMA_SSTAT(chn)    0x20+0x58*(chn)
#define         DWDMA_DSTAT(chn)    0x28+0x58*(chn)
#define         DWDMA_SSTATAR(chn)  0x30+0x58*(chn)
#define         DWDMA_DSTATAR(chn)  0x38+0x58*(chn)
#define         DWDMA_CFGL(chn)     0x40+0x58*(chn)
#define         DWDMA_CFGH(chn)     0x44+0x58*(chn)
#define         DWDMA_SGR(chn)      0x48+0x58*(chn)
#define         DWDMA_DSR(chn)      0x50+0x58*(chn)

#define         DWDMA_RawTfr        0x2c0
#define         DWDMA_RawBlock      0x2c8
#define         DWDMA_RawSrcTran    0x2d0
#define         DWDMA_RawDstTran    0x2d8
#define         DWDMA_RawErr        0x2e0
#define         DWDMA_StatusTfr     0x2e8
#define         DWDMA_StatusBlock   0x2f0
#define         DWDMA_StatusSrcTran 0x2f8
#define         DWDMA_StatusDstTran 0x300
#define         DWDMA_StatusErr     0x308
#define         DWDMA_MaskTfr       0x310
#define         DWDMA_MaskBlock     0x318
#define         DWDMA_MaskSrcTran   0x320
#define         DWDMA_MaskDstTran   0x328
#define         DWDMA_MaskErr       0x330
#define         DWDMA_ClearTfr      0x338
#define         DWDMA_ClearBlock    0x340
#define         DWDMA_ClearSrcTran  0x348
#define         DWDMA_ClearDstTran  0x350
#define         DWDMA_ClearErr      0x358
#define         DWDMA_StatusInt     0x360
#define         DWDMA_ReqSrcReg     0x368
#define         DWDMA_ReqDstReg     0x370
#define         DWDMA_SglReqSrcReg  0x378
#define         DWDMA_SglReqDstReg  0x380
#define         DWDMA_LstSrcReg     0x388
#define         DWDMA_LstDstReg     0x390
#define         DWDMA_DmaCfgReg     0x398
#define         DWDMA_ChEnReg       0x3a0
#define         DWDMA_DmaIdReg      0x3a8
#define         DWDMA_DmaTestReg    0x3b0
/**************************/


#define RK28_DMA_CHANNELS  3
#define RK28_DMA_CH0      0
#define RK28_DMA_CH1      1
#define RK28_DMA_CH2      2

#define RK28_DMA_IRQ_NUM   0
#define RK28_DMA_LLPS      100

#define RK28_DMA_CH0A1_MAX_LEN      4095U
#define RK28_DMA_CH2_MAX_LEN        2047U

#define RK28_DMA_BUSY        1
#define RK28_DMA_IDLE        0


typedef unsigned int rk28_dmach_t;
typedef unsigned long rk28_flags_t;


struct rk28_dma_llp {
    u32      sar;
    u32      dar;
    struct      rk28_dma_llp  *llp;
    u32      ctll;
    u32      size; 
};

struct rk28_dma_dev {
    const char *name;  //device name
    u32      hd_if;   //hardware interface
    u32      dev_addr;   //device basic addresss 
    u32      fifo_width;  // fifo width of device 
    u32      burst_width;  // burst width of fifo 
    u32      dma_phy_ch;  // current dma channel which allocated by device
    u32      dma_dft_ch;  // default dma channel which allocated by device
    u32      priority;   // priority of physical channel which requested by device
};

struct rk28_dma_channel {
	struct rk28_dma_dev *dev_info;// basic address of sg in memory
    struct rk28_dma_llp *dma_llp_vir;  // virtual cpu addrress of linked list
    u32 dma_llp_phy;                   // physical bus address of linked list 
	void (*irq_handler) (s32, void *); //call back function
	void *data;            // the second input value of  irq_handler
	struct scatterlist *sg;// basic address of sg in memory
	u32 sgcount;     // count of sg 
	u32 dma_block;     // current transfer block  
	s32 res_block;     // residue block of current sg 
	u32  dma_mode;  // dma mode of write or read
	u32  dma_status;  // identify if DMA is using	
};

struct rk28_dma_infor {
	u32 reg_vir_base;      // virtual cpu basic address of dwdma channel register 
	u32 dma_irq_num;
	u32 dma_flag[RK28_DMA_CHANNELS]; // device which already request specifical dma physical channel 
};

extern int rockchip_dma_getposition(rk28_dmach_t channel, rk28_dmach_t *src, rk28_dmach_t *dst);


extern s32 rk28_dma_enable(rk28_dmach_t dma_ch);
extern s32 rk28_dma_disable(rk28_dmach_t dma_ch);
extern s32 rk28_dma_request(rk28_dmach_t dma_ch, 
                                   void (*irq_handler) (s32, void *),
		                           void *data);
extern s32 rk28_dma_free(rk28_dmach_t dma_ch);
extern s32 rk28_dma_setup_sg(rk28_dmach_t dma_ch,
		                        struct scatterlist *sg, 
		                        u32 sgcount,
		                        u32 dmamode);	
extern s32 rk28_dma_test(void);






/*devicd id list*/
#define RK28_DMA_SD_MMC0       0
#define RK28_DMA_LCDC_R0       1
#define RK28_DMA_LCDC_R1       2
#define RK28_DMA_LCDC_R2       3
#define RK28_DMA_LCDC_R3       4
#define RK28_DMA_SD_MMC1       5
#define RK28_DMA_I2S_TXD       6
#define RK28_DMA_I2S_RXD       7
#define RK28_DMA_SPI_M_TXD     8
#define RK28_DMA_SPI_M_RXD     9
#define RK28_DMA_SPI_S_TXD     10
#define RK28_DMA_SPI_S_RXD     11
#define RK28_DMA_URAT0_TXD     12
#define RK28_DMA_URAT0_RXD     13
#define RK28_DMA_URAT1_TXD     14
#define RK28_DMA_URAT1_RXD     15
#define RK28_DMA_SDRAM         16

#define RK28_DMA_MAX        17
#define RK28_DMA_NULL       0xff



#endif	/* _ASM_ARCH_RK28_DMA_H */

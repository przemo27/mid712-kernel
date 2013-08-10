/*
 * include/linux/rk28_spim.h
 *
 * Copyright (C) rockchip
 *
 * SPIM registers.
 * Based on RK28XX datasheet revision 0.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef ROCKCHIP_SPIM_H
#define ROCKCHIP_SPIM_H

//SPIM_SPIENR SPIS_SPIENR
#define SPI_ENABLE                  (1)

//SPIM_CTRLR0  SPIS_CTRLR0
#define NORMAL_MODE_OPERATION       (0)
#define TEST_MODE_OPERATION         (1<<11)
#define TRANSMIT_RECEIVE            (0)
#define TRANSMIT_ONLY               (1<<8)
#define RECEIVE_ONLY                (2<<8)
#define SPIM_E2PROM_READ            (3<<8)
#define SERIAL_CLOCK_POLARITY_LOW   (0)
#define SERIAL_CLOCK_POLARITY_HIGH  (1<<7)
#define SERIAL_CLOCK_PHASE_MIDDLE   (0)
#define SERIAL_CLOCK_PHASE_START    (1<<6)
#define MOTOROLA_SPI                (0)
#define TEXAS_INSTRUMENTS_SSP       (1<<4)
#define NATIONAL_SEMI_MICROWIRE     (2<<4)
#define SPI_DATA_WIDTH4             (3)
#define SPI_DATA_WIDTH5             (4)
#define SPI_DATA_WIDTH6             (5)
#define SPI_DATA_WIDTH7             (6)
#define SPI_DATA_WIDTH8             (7)
#define SPI_DATA_WIDTH9             (8)
#define SPI_DATA_WIDTH10            (9)
#define SPI_DATA_WIDTH11            (0xa)
#define SPI_DATA_WIDTH12            (0xb)
#define SPI_DATA_WIDTH13            (0xc)
#define SPI_DATA_WIDTH14            (0xd)
#define SPI_DATA_WIDTH15            (0xe)
#define SPI_DATA_WIDTH16            (0xf)   


///SPIM_SR  SPIS_SR
#define RECEIVE_FIFO_FULL           (1<<4)
#define RECEIVE_FIFO_NOT_EMPTY      (1<<3)
#define TRANSMIT_FIFO_EMPTY         (1<<2)
#define TRANSMIT_FIFO_NOT_FULL      (1<<1)
#define SPI_BUSY_FLAG               (1)

//SPIM_IMR
#define RECEIVE_FIFO_FULL_INT_NOTMASK       (1<<4)
#define RECEIVE_FIFO_OVERFLOW_INT_NOTMASK   (1<<3)
#define RECEIVE_FIFO_UNDERFLOW_INT_NOTMASK  (1<<2)
#define TRANSMIT_FIFO_OVERFLOW_INT_NOTMASK  (1<<1)
#define TRANSMIT_FIFO_EMPTY_INT_NOTMASK     (1)

//SPIM_DMACR SPIS_DMACR
#define TRANSMIT_DMA_ENABLE         (1<<1)
#define RECEIVE_DMA_ENABLE          (1)


//SPI MASTER Registers
typedef volatile struct tagSPI_MASTER_STRUCT
{
    unsigned int SPIM_CTRLR0;
    unsigned int SPIM_CTRLR1;
    unsigned int SPIM_SPIENR;
    unsigned int SPIM_MWCR;
    unsigned int SPIM_SER;
    unsigned int SPIM_BAUDR;
    unsigned int SPIM_TXFTLR;
    unsigned int SPIM_RXFTLR;
    unsigned int SPIM_TXFLR;
    unsigned int SPIM_RXFLR;
    unsigned int SPIM_SR;
    unsigned int SPIM_IMR;
    unsigned int SPIM_ISR;
    unsigned int SPIM_RISR;
    unsigned int SPIM_TXOICR;
    unsigned int SPIM_RXOICR;
    unsigned int SPIM_RXUICR;
    unsigned int SPIM_MSTICR;
    unsigned int SPIM_ICR;
    unsigned int SPIM_DMACR;
    unsigned int SPIM_DMATDLR;
    unsigned int SPIM_DMARDLR;
    unsigned int SPIM_IDR;
    unsigned int SPIM_COMP_VERSION;
    unsigned int SPIM_DR0;
    unsigned int SPIM_DR1;
    unsigned int SPIM_DR2;
    unsigned int SPIM_DR3;
    unsigned int SPIM_DR4;
    unsigned int SPIM_DR5;
    unsigned int SPIM_DR6;
    unsigned int SPIM_DR7;
    unsigned int SPIM_DR8;
    unsigned int SPIM_DR9;
    unsigned int SPIM_DR10;
    unsigned int SPIM_DR11;
    unsigned int SPIM_DR12;
    unsigned int SPIM_DR13;
    unsigned int SPIM_DR14;
    unsigned int SPIM_DR15;
}SPIM_REG,*pSPIM_REG;


extern void __init rockchip_add_device_spi_master(struct spi_board_info *devices, int nr_devices);

#endif

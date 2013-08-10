/*
 * include/linux/rk28_serial.h
 *
 * Copyright (C) SAN People
 *
 * USART registers.
 * Based on RK28XX datasheet revision 0.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef RK28_SERIAL_H
#define RK28_SERIAL_H

typedef volatile struct tagUART_STRUCT
{
    unsigned int UART_RBR;
    unsigned int UART_DLH;
    unsigned int UART_IIR;
    unsigned int UART_LCR;
    unsigned int UART_MCR;
    unsigned int UART_LSR;
    unsigned int UART_MSR;
    unsigned int UART_SCR;
    unsigned int RESERVED1[(0x30-0x20)/4];
    unsigned int UART_SRBR[(0x70-0x30)/4];
    unsigned int UART_FAR;
    unsigned int UART_TFR;
    unsigned int UART_RFW;
    unsigned int UART_USR;
    unsigned int UART_TFL;
    unsigned int UART_RFL;
    unsigned int UART_SRR;
    unsigned int UART_SRTS;
    unsigned int UART_SBCR;
    unsigned int UART_SDMAM;
    unsigned int UART_SFE;
    unsigned int UART_SRT;
    unsigned int UART_STET;
    unsigned int UART_HTX;
    unsigned int UART_DMASA;
    unsigned int RESERVED2[(0xf4-0xac)/4];
    unsigned int UART_CPR;
    unsigned int UART_UCV;
    unsigned int UART_CTR;
} UART_REG, *pUART_REG;

//#define UART_RBR            0x00
//#define UART_DLH            0x04


//#define UART_IIR            0x08
 #define   IR_MODEM_STATUS                    (0)
 #define   NO_INT_PENDING                     (1)
 #define   THR_EMPTY                          (2)
 #define   RECEIVER_DATA_AVAILABLE            (0x04)
 #define   RECEIVER_LINE_AVAILABLE            (0x06)
 #define   BUSY_DETECT                        (0x07)
 #define   CHARACTER_TIMEOUT                  (0x0c)

//#define UART_LCR            0x0c
#define  LCR_DLA_EN                         (1<<7)
#define  BREAK_CONTROL_BIT                  (1<<6)
#define  EVEN_PARITY_SELECT                 (1<<4)
#define  EVEN_PARITY                        (1<<4)
#define  ODD_PARITY                          (0)
#define  PARITY_DISABLED                     (0)
#define  PARITY_ENABLED                     (1<<3)
#define  ONE_STOP_BIT                        (0)
#define  ONE_HALF_OR_TWO_BIT                (1<<2)
#define  LCR_WLS_5                           (0x00)
#define  LCR_WLS_6                           (0x01)
#define  LCR_WLS_7                           (0x02)
#define  LCR_WLS_8                           (0x03)
#define  UART_DATABIT_MASK                   (0x03)

//#define UART_MCR            0x10

//#define UART_LSR            0x14
#define  UART_TRANSMIT_EMPTY                (1<<6) /*Transmitter Empty bit.*/

#define UART_MSR            0x18
#define UART_SCR            0x20
#define UART_SRBR           0x30
#define UART_FAR            0x70
#define UART_TFR            0x74
#define UART_RFW            0x78 
 
//#define UART_USR            0x7c
#define  UART_RECEIVE_FIFO_FULL              (0)
#define  UART_RECEIVE_FIFO_NOT_FULL          (1<<4)
#define  UART_RECEIVE_FIFO_EMPTY             (0)
#define  UART_RECEIVE_FIFO_NOT_EMPTY         (1<<3)
#define  UART_TRANSMIT_FIFO_NOT_EMPTY        (0)
#define  UART_TRANSMIT_FIFO_EMPTY            (1<<2)
#define  UART_TRANSMIT_FIFO_FULL             (0)
#define  UART_TRANSMIT_FIFO_NOT_FULL         (1<<1)
#define  UART_USR_BUSY                       (1)


//#define UART_TFL            0x80
//#define UART_RFL            0x84
//#define UART_SRR            0x88
 #define UART_XMIT_FIFO_SET (1<<2)
 #define UART_RCVR_FIFO_SET (1<<1)
 #define UART_RESET         1
//#define UART_SRTS           0x8c
//#define UART_SBCR           0x90
//#define UART_SDMAM          0x94
//#define UART_SFE            0x98
//#define UART_SRT            0x9c
//#define UART_STET           0xa0
//#define UART_HTX            0xa4
//#define UART_DMASA          0xa8
//#define UART_CPR            0xf4
//#define UART_UCV            0xf8
//#define UART_CTR            0xfc

#define UART_THR            UART_RBR
#define UART_DLL            UART_RBR
#define UART_IER            UART_DLH
#define UART_FCR            UART_IIR
#define UART_STHR           UART_SRBR 

#define NR_PORTS (2)

extern void __init rockchip_register_uart(unsigned id, unsigned portnr, unsigned pins);
extern void __init rockchip_set_serial_console(unsigned portnr);

struct rock_uart_config {
	unsigned short	console_tty;	/* tty number of serial console */
	unsigned short	nr_tty;		/* number of serial tty's */
	short		tty_map[];	/* map UART to tty number */
};

extern struct platform_device *rockchip_default_console_device;
extern void   rockchip_init_serial(struct rock_uart_config *config);
extern void __init rockchip_add_device_serial(void); 



#endif

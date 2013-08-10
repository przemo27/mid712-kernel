/*************************************************************
 *Copyright: ROCKCHIP Inc
 *
 *Author :WQQ
 *
 *Date	: 2009-04-21
 *
 **************************************************************/

#ifndef I2C_RK28_H
#define I2C_RK28_H


#define RK28_BASE_I2C 0x1800C000 //base
#define RK28_ID_I2C 11 /*i2c interface intc*/

#define RK28_I2C_MTXR    0x00
#define RK28_I2C_MRXR    0x04
#define RK28_I2C_STXR    0x08
#define RK28_I2C_SRXR    0x0C
#define RK28_I2C_SADDR   0x10
#define RK28_I2C_IER     0x14
#define RK28_I2C_ISR     0x18
#define RK28_I2C_LCMR    0x1C
#define RK28_I2C_LSR     0x20
#define RK28_I2C_CONR    0x24
#define RK28_I2C_OPR     0x28





#define START_BIT                (1)
#define NO_START_BIT             (0)

//I2C_IER
#define I2C_ARBITR_LOSE_ENABLE   (1<<7)  //Arbitration lose interrupt

//I2C_ISR
#define I2C_ARBITR_LOSE_STATUS   (1<<7)  //Arbitration lose STATUS
#define I2C_RECE_INT_MACKP       (1<<1) //Master ACK period interrupt status bit
#define I2C_RECE_INT_MACK        (1)     //Master receives ACK interrupt status bit


//I2C_LSR
#define I2C_LSR_RECE_NACK        (1<<1)

#define I2C_LSR_BUSY_START        (1)
#define I2C_LSR_BUSY_STOP        (0)

//I2C_LCMR
#define I2C_LCMR_RESUME          (1<<2)
#define I2C_LCMR_STOP            (1<<1)
#define I2C_LCMR_START           (1)

//I2C_CONR
#define I2C_CON_ACK              (0)
#define I2C_CON_NACK             (1<<4)
#define I2C_MASTER_TRAN_MODE     (1<<3)
#define I2C_MASTER_RECE_MODE     (0)
#define I2C_MASTER_PORT_ENABLE   (1<<2)
#define I2C_SLAVE_RECE_MODE      (0)
#define I2C_SLAVE_TRAN_MODE      (1<<1)
#define I2C_SLAVE_PORT_ENABLE    (1)


//I2C_OPR
#define SLAVE_7BIT_ADDRESS_MODE  (0)
#define SLAVE_10BIT_ADDRESS_MODE  (1<<8)
#define RESET_I2C_STATUS         (1<<7)
#define I2C_CORE_ENABLE          (1<<6)
#define I2C_CORE_DISABLE         (0)


#define I2C_READ_BIT             (1)
#define I2C_WRITE_BIT            (0)




#endif

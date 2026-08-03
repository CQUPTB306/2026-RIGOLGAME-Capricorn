/**
 * @file    soft_i2c_simple.h
 * @brief   软件 I2C 驱动 (MSPM0G3507) — 多总线版本
 * @note    总线1 (OLED):          PA15(SDA) PA16(SCL) [PINCM37/PINCM38]
 *          总线2 (MPU6050):      PA28(SDA) PA31(SCL)
 *          总线3 (循迹):         PB03(SDA) PB02(SCL)
 */

#ifndef __SOFT_I2C_H
#define __SOFT_I2C_H

#include "../ti_msp_dl_config.h"

#define SOFT_I2C_WRITE 0
#define SOFT_I2C_READ  1

typedef struct {
    GPIO_Regs *scl_port;
    uint32_t   scl_pin;
    uint32_t   scl_iomux;
    GPIO_Regs *sda_port;
    uint32_t   sda_pin;
    uint32_t   sda_iomux;
    void     (*delay)(void);
} SoftI2C_Obj;

void    SoftI2C_Init(SoftI2C_Obj *obj);
void    SoftI2C_Start(SoftI2C_Obj *obj);
void    SoftI2C_Stop(SoftI2C_Obj *obj);
uint8_t SoftI2C_WaitAck(SoftI2C_Obj *obj);
void    SoftI2C_Ack(SoftI2C_Obj *obj);
void    SoftI2C_NAck(SoftI2C_Obj *obj);
void    SoftI2C_SendByte(SoftI2C_Obj *obj, uint8_t data);
uint8_t SoftI2C_ReadByte(SoftI2C_Obj *obj, uint8_t ack);

uint8_t SoftI2C_WriteOneByte(SoftI2C_Obj *obj, uint8_t dev_addr, uint8_t reg, uint8_t data);
uint8_t SoftI2C_ReadOneByte(SoftI2C_Obj *obj, uint8_t dev_addr, uint8_t reg);
void    SoftI2C_WriteBuf(SoftI2C_Obj *obj, uint8_t dev_addr, uint8_t reg, uint8_t len, uint8_t *buf);
void    SoftI2C_ReadBuf(SoftI2C_Obj *obj, uint8_t dev_addr, uint8_t reg, uint8_t len, uint8_t *buf);

/* delay_ms 由 board.c 提供 */

/*========== 三个总线实例 ==========*/
extern SoftI2C_Obj i2c_pca9685;   /* PA00(SDA) PA01(SCL) — PCA9685(0x40)+OLED(0x3C) */
extern SoftI2C_Obj i2c_mpu6050;   /* PA28(SDA) PA31(SCL) */
extern SoftI2C_Obj i2c_track;     /* PB03(SDA) PB02(SCL) */

#endif

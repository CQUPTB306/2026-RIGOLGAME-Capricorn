/**
 * @file    soft_i2c_track.h
 * @brief   软件 I2C — 循迹传感器 (PA0=SDA, PA1=SCL)
 */

#ifndef __SOFT_I2C_TRACK_H
#define __SOFT_I2C_TRACK_H

#include "ti_msp_dl_config.h"

/* ── 引脚: PA0=SDA, PA1=SCL ── */
#define TRACK_I2C_SDA_PORT   GPIOA
#define TRACK_I2C_SDA_PIN    DL_GPIO_PIN_0
#define TRACK_I2C_SCL_PORT   GPIOA
#define TRACK_I2C_SCL_PIN    DL_GPIO_PIN_1

#define TRACK_I2C_WRITE  0
#define TRACK_I2C_READ   1

void TRACK_I2C_Start(void);
void TRACK_I2C_Stop(void);
uint8_t TRACK_I2C_Wait_Ack(void);
void TRACK_I2C_Send_Byte(uint8_t txd);
uint8_t TRACK_I2C_Read_Byte(unsigned char ack);
uint8_t TRACK_I2C_Read_One_Byte(uint8_t addr, uint8_t reg);

#endif

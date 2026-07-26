/**
 * @file    soft_i2c_track.h
 * @brief   循迹传感器专用软件 I2C — 开漏输出, 对齐 trace_i2c 工作项目
 * @note    引脚: PA28(SDA) PA31(SCL) — I2C0 硬件引脚
 *          SDA = 开漏 (HiZ 切换, 不驱 HIGH)
 *          SCL = 推挽 (主机驱动)
 */

#ifndef __SOFT_I2C_TRACK_H
#define __SOFT_I2C_TRACK_H

#include <stdint.h>

/*==================== 硬件配置 ====================*/

#define TRACK_I2C_SCL_PORT    GPIOA
#define TRACK_I2C_SCL_PIN     DL_GPIO_PIN_31
#define TRACK_I2C_SCL_IOMUX   IOMUX_PINCM6      /* PA31 -> PINCM6 */

#define TRACK_I2C_SDA_PORT    GPIOA
#define TRACK_I2C_SDA_PIN     DL_GPIO_PIN_28
#define TRACK_I2C_SDA_IOMUX   IOMUX_PINCM3      /* PA28 -> PINCM3 */

/* I2C 读写方向 */
#define TRACK_I2C_WRITE  0
#define TRACK_I2C_READ   1

/*==================== 函数声明 ====================*/

void    Track_I2C_Init(void);
void    Track_I2C_Start(void);
void    Track_I2C_Stop(void);
uint8_t Track_I2C_WaitAck(void);
void    Track_I2C_Ack(void);
void    Track_I2C_NAck(void);
void    Track_I2C_SendByte(uint8_t data);
uint8_t Track_I2C_ReadByte(uint8_t ack);

uint8_t Track_I2C_WriteOneByte(uint8_t addr, uint8_t reg, uint8_t data);
uint8_t Track_I2C_ReadOneByte(uint8_t addr, uint8_t reg);

#endif

#ifndef __SOFT_I2C_H
#define __SOFT_I2C_H

#include "stm32f1xx_hal.h"  // 根据你的芯片型号修改，如f4/f7/g4等

//==================== 引脚配置 ====================
#define I2C_SCL_PIN GPIO_PIN_8
#define I2C_SCL_GPIO_PORT GPIOB
#define I2C_SDA_PIN GPIO_PIN_9
#define I2C_SDA_GPIO_PORT GPIOB
//==================================================

// I2C 读写方向
#define I2C_WRITE 0
#define I2C_READ  1

// 函数声明
void I2C_Start(void);
void I2C_Stop(void);
uint8_t I2C_Wait_Ack(void);
void I2C_Ack(void);
void I2C_NAck(void);
void I2C_Send_Byte(uint8_t txd);
uint8_t I2C_Read_Byte(unsigned char ack);

// 常用读写接口
uint8_t I2C_Write_One_Byte(uint8_t addr, uint8_t reg, uint8_t data);
uint8_t I2C_Read_One_Byte(uint8_t addr, uint8_t reg);
void I2C_Write_Buf(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
void I2C_Read_Buf(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

#endif

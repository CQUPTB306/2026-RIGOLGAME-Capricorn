#ifndef __MPU_SOFT_I2C_H
#define __MPU_SOFT_I2C_H

#include "stm32f1xx_hal.h"   // 根据您的MCU系列调整，如f4/f7/g4等

//==================== 引脚配置 ====================
#define MPU_I2C_SCL_PIN      GPIO_PIN_10
#define MPU_I2C_SCL_PORT     GPIOB
#define MPU_I2C_SDA_PIN      GPIO_PIN_11
#define MPU_I2C_SDA_PORT     GPIOB
//==================================================

// I2C 读写方向
#define MPU_I2C_WRITE  0
#define MPU_I2C_READ   1

// 函数声明
void mpu_I2C_Start(void);
void mpu_I2C_Stop(void);
uint8_t mpu_I2C_Wait_Ack(void);
void mpu_I2C_Ack(void);
void mpu_I2C_NAck(void);
void mpu_I2C_Send_Byte(uint8_t txd);
uint8_t mpu_I2C_Read_Byte(uint8_t ack);

// 通用读写接口（寄存器级）
uint8_t mpu_I2C_Write_One_Byte(uint8_t dev_addr, uint8_t reg, uint8_t data);
uint8_t mpu_I2C_Read_One_Byte(uint8_t dev_addr, uint8_t reg);
void mpu_I2C_Write_Buf(uint8_t dev_addr, uint8_t reg, uint8_t len, uint8_t *buf);
void mpu_I2C_Read_Buf(uint8_t dev_addr, uint8_t reg, uint8_t len, uint8_t *buf);

// 初始化I2C引脚（配置为开漏输出模式，兼有输入功能）
void mpu_I2C_Init(void);

#endif

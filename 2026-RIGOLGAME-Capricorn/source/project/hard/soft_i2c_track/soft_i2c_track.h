#ifndef __SOFT_I2C_TRACK_H
#define __SOFT_I2C_TRACK_H

#include "stm32f1xx_hal.h"  // 请根据你的实际芯片型号修改（如 stm32f4xx_hal.h）

/*======================== 引脚配置 ========================*/
/* 用户可根据实际硬件连接修改以下宏定义                     */
#define TRACK_I2C_SCL_PIN       GPIO_PIN_5
#define TRACK_I2C_SCL_GPIO_PORT GPIOC
#define TRACK_I2C_SDA_PIN       GPIO_PIN_4
#define TRACK_I2C_SDA_GPIO_PORT GPIOC
/*==========================================================*/

/* I2C 读写方向 */
#define TRACK_I2C_WRITE  0
#define TRACK_I2C_READ   1

/* 函数声明 */
void TRACK_I2C_Start(void);
void TRACK_I2C_Stop(void);
uint8_t TRACK_I2C_Wait_Ack(void);
void TRACK_I2C_Ack(void);
void TRACK_I2C_NAck(void);
void TRACK_I2C_Send_Byte(uint8_t txd);
uint8_t TRACK_I2C_Read_Byte(unsigned char ack);

/* 常用读写接口 */
uint8_t TRACK_I2C_Write_One_Byte(uint8_t addr, uint8_t reg, uint8_t data);
uint8_t TRACK_I2C_Read_One_Byte(uint8_t addr, uint8_t reg);
void TRACK_I2C_Write_Buf(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
void TRACK_I2C_Read_Buf(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

#endif

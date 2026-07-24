#ifndef __OLED_H
#define __OLED_H

#include "stdint.h"
#include "soft_i2c.h"

// I2C函数名宏定义映射（适配soft_i2c的函数名）
#define Soft_I2C_Start    I2C_Start
#define Soft_I2C_SendByte I2C_Send_Byte
#define Soft_I2C_Stop     I2C_Stop
#define Soft_I2C_WaitAck  I2C_Wait_Ack

/* 函数声明 */
void OLED_Init(void);                                  // OLED初始化
void OLED_Clear(void);                                 // OLED清屏
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);  // 显示单个字符
void OLED_ShowString(uint8_t Line, uint8_t Column, char* String);  // 显示字符串
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);  // 显示十进制数字
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);  // 显示带符号十进制数字
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);   // 显示十六进制数字
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);   // 显示二进制数字
void OLED_ShowFloatNum(uint8_t Line, uint8_t Column, float Number, uint8_t IntLength, uint8_t DecLength);  // 显示浮点型数字
void OLED_SetCursor(uint8_t Y, uint8_t X);
void OLED_WriteData(uint8_t Data);
#endif

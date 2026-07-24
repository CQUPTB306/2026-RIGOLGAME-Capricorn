#ifndef __OLED_H
#define __OLED_H

#include "stm32f1xx_hal.h"
#define OLED_I2C_HANDLE &hi2c1
#define OLED_I2C_ADDR 0x78  // 0.96¥ÁI2C OLEDƒ¨»œµÿ÷∑


void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowString(uint8_t x, uint8_t y, char *str);
void OLED_ShowNum(uint8_t x, uint8_t y, int32_t num, uint8_t len);
void OLED_Show_Order_Setting(char *order_str, uint8_t index);
void OLED_Show_State(const char *state);
void OLED_Show_AlignResult(uint8_t success);
void OLED_Clear_Line(uint8_t line);

#endif
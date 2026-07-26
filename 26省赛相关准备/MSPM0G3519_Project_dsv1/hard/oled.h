/**
 * @file    oled.h
 * @brief   0.96寸 OLED 驱动 (SSD1306) — MSPM0G3507 适配版
 * @note    使用软件 I2C (soft_i2c_simple.h) 驱动
 *          I2C 总线: PA04(SDA), PA05(SCL) — 详见 soft_i2c_simple.c::i2c_oled
 *          分辨率: 128x64, 8x16 字体, 4行×16列
 *          改编自 STM32 最终代码的 OLED 驱动
 */

#ifndef __OLED_H
#define __OLED_H

#include "stdint.h"
#include "soft_i2c_simple.h"

/*==================== 函数声明 ====================*/

/**
 * @brief  OLED 初始化 (上电→寄存器配置→清屏)
 * @param  obj  软件 I2C 总线对象指针 (&i2c_oled)
 */
void OLED_Init(SoftI2C_Obj *obj);

/**
 * @brief  OLED 清屏 (全屏填充 0x00)
 * @param  obj  软件 I2C 总线对象指针
 */
void OLED_Clear(SoftI2C_Obj *obj);

/**
 * @brief  OLED 显示单个 ASCII 字符 (8x16 字体)
 * @param  obj    软件 I2C 总线对象指针
 * @param  Line   行号 (1~4)
 * @param  Column 列号 (1~16)
 * @param  Char   要显示的 ASCII 字符
 */
void OLED_ShowChar(SoftI2C_Obj *obj, uint8_t Line, uint8_t Column, char Char);

/**
 * @brief  OLED 显示字符串
 * @param  obj    软件 I2C 总线对象指针
 * @param  Line   起始行 (1~4)
 * @param  Column 起始列 (1~16)
 * @param  String 要显示的字符串 (以 '\0' 结尾)
 */
void OLED_ShowString(SoftI2C_Obj *obj, uint8_t Line, uint8_t Column, char *String);

/**
 * @brief  OLED 显示无符号十进制数字
 * @param  obj    软件 I2C 总线对象指针
 * @param  Line   行号 (1~4)
 * @param  Column 起始列 (1~16)
 * @param  Number 数字 (0~4294967295)
 * @param  Length 显示位数 (1~10)
 */
void OLED_ShowNum(SoftI2C_Obj *obj, uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

/**
 * @brief  OLED 显示带符号十进制数字
 * @param  obj    软件 I2C 总线对象指针
 * @param  Line   行号 (1~4)
 * @param  Column 起始列 (1~16)
 * @param  Number 数字 (-2147483648~2147483647)
 * @param  Length 显示位数 (1~10)
 */
void OLED_ShowSignedNum(SoftI2C_Obj *obj, uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);

/**
 * @brief  OLED 显示十六进制数字
 * @param  obj    软件 I2C 总线对象指针
 * @param  Line   行号 (1~4)
 * @param  Column 起始列 (1~16)
 * @param  Number 数字 (0~0xFFFFFFFF)
 * @param  Length 显示位数 (1~8)
 */
void OLED_ShowHexNum(SoftI2C_Obj *obj, uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

/**
 * @brief  OLED 显示二进制数字
 * @param  obj    软件 I2C 总线对象指针
 * @param  Line   行号 (1~4)
 * @param  Column 起始列 (1~16)
 * @param  Number 数字 (0~65535)
 * @param  Length 显示位数 (1~16)
 */
void OLED_ShowBinNum(SoftI2C_Obj *obj, uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

/**
 * @brief  OLED 显示浮点数
 * @param  obj       软件 I2C 总线对象指针
 * @param  Line      行号 (1~4)
 * @param  Column    起始列 (1~16)
 * @param  Number    浮点数
 * @param  IntLength 整数部分位数 (1~9)
 * @param  DecLength 小数部分位数 (1~6)
 */
void OLED_ShowFloatNum(SoftI2C_Obj *obj, uint8_t Line, uint8_t Column, float Number, uint8_t IntLength, uint8_t DecLength);

/**
 * @brief  OLED 写数据 (底层接口, 外部一般不需要调用)
 * @param  obj  软件 I2C 总线对象指针
 * @param  Data 要写入的字节数据
 */
uint8_t OLED_WriteData(SoftI2C_Obj *obj, uint8_t Data);

/**
 * @brief  OLED 设置光标位置
 * @param  obj 软件 I2C 总线对象指针
 * @param  Y   页地址 (0~7)
 * @param  X   列地址 (0~127)
 */
void OLED_SetCursor(SoftI2C_Obj *obj, uint8_t Y, uint8_t X);

#endif /* __OLED_H */

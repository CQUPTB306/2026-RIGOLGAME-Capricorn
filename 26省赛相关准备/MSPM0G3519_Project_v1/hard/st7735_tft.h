/**
 * @file    st7735_tft.h
 * @brief   ST7735 TFT LCD 驱动头文件 (MSPM0G3507 + TI DriverLib)
 * @author  基于 STM32 原版移植至 MSPM0G3507
 *
 * 接线定义 (MSPM0G3507 开发板, 对齐 empty.syscfg):
 *  GND  -> 电源地
 *  VCC  -> 3.3V
 *  SCL  -> PA.27 (软件模拟 SPI 时钟)
 *  SDA  -> PA.26 (软件模拟 SPI 数据 MOSI)
 *  RES  -> PB.27 (复位)
 *  DC   -> PB.26 (数据/命令选择)
 *  CS   -> PB.25 (片选)
 *  BLK  -> 不使用 (背光接 VCC 常亮 或 外部 PWM 控制)
 */

#ifndef __ST7735_TFT_H
#define __ST7735_TFT_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * 引脚端口宏定义 (基于 MSPM0G3507 开发板 SPI 例程)
 *===========================================================================*/
#define TFT_SCLK_PORT       GPIOA
#define TFT_SCLK_PIN        DL_GPIO_PIN_27
#define TFT_SCLK_IOMUX      IOMUX_PINCM60

#define TFT_MOSI_PORT       GPIOA
#define TFT_MOSI_PIN        DL_GPIO_PIN_26
#define TFT_MOSI_IOMUX      IOMUX_PINCM59

#define TFT_RES_PORT        GPIOB
#define TFT_RES_PIN         DL_GPIO_PIN_27
#define TFT_RES_IOMUX       IOMUX_PINCM58

#define TFT_DC_PORT         GPIOB
#define TFT_DC_PIN          DL_GPIO_PIN_26
#define TFT_DC_IOMUX        IOMUX_PINCM57

#define TFT_CS_PORT         GPIOB
#define TFT_CS_PIN          DL_GPIO_PIN_25
#define TFT_CS_IOMUX        IOMUX_PINCM56

/*===========================================================================
 * 引脚电平控制宏 (软件模拟 SPI, 直接操作 DriverLib)
 *===========================================================================*/
#define TFT_CS_H    DL_GPIO_setPins(TFT_CS_PORT, TFT_CS_PIN)
#define TFT_CS_L    DL_GPIO_clearPins(TFT_CS_PORT, TFT_CS_PIN)

#define TFT_SCLK_H  DL_GPIO_setPins(TFT_SCLK_PORT, TFT_SCLK_PIN)
#define TFT_SCLK_L  DL_GPIO_clearPins(TFT_SCLK_PORT, TFT_SCLK_PIN)

#define TFT_MOSI_H  DL_GPIO_setPins(TFT_MOSI_PORT, TFT_MOSI_PIN)
#define TFT_MOSI_L  DL_GPIO_clearPins(TFT_MOSI_PORT, TFT_MOSI_PIN)

#define TFT_RES_H   DL_GPIO_setPins(TFT_RES_PORT, TFT_RES_PIN)
#define TFT_RES_L   DL_GPIO_clearPins(TFT_RES_PORT, TFT_RES_PIN)

#define TFT_DC_H    DL_GPIO_setPins(TFT_DC_PORT, TFT_DC_PIN)
#define TFT_DC_L    DL_GPIO_clearPins(TFT_DC_PORT, TFT_DC_PIN)

/*===========================================================================
 * 屏幕参数 (ST7735 128x160)
 *===========================================================================*/
#define TFT_WIDTH   128
#define TFT_HEIGHT  160

/*===========================================================================
 * 颜色定义 (RGB565)
 *===========================================================================*/
#define TFT_RED         0xF800
#define TFT_GREEN       0x07E0
#define TFT_BLUE        0x001F
#define TFT_BLUE2       0x1C9F
#define TFT_PINK        0xD8A7
#define TFT_ORANGE      0xFA20
#define TFT_WHITE       0xFFFF
#define TFT_BLACK       0x0000
#define TFT_YELLOW      0xFFE0
#define TFT_CYAN        0x07FF
#define TFT_PURPLE      0xF81F
#define TFT_PURPLE2     0xDB92
#define TFT_PURPLE3     0x8811
#define TFT_GRAY0       0xEF7D
#define TFT_GRAY1       0x8410
#define TFT_GRAY2       0x4208
#define TFT_BROWN       0xBC40
#define TFT_BRRED       0xFC07
#define TFT_DARKBLUE    0x01CF
#define TFT_LIGHTBLUE   0x7D7C
#define TFT_GRAYBLUE    0x5458
#define TFT_LIGHTGREEN  0x841F
#define TFT_LGRAY       0xC618
#define TFT_LGRAYBLUE   0xA651
#define TFT_LBBLUE      0x2B12

/*===========================================================================
 * 函数声明
 *===========================================================================*/

/* 初始化和基础控制 */
void TFT_GPIO_Init(void);
void TFT_Init(void);
void TFT_Reset(void);
void TFT_SetRotation(uint8_t rotation);

/* SPI 底层通信 */
void TFT_SPI_SendByte(uint8_t data);
void TFT_SendCommand(uint8_t cmd);
void TFT_SendData(uint8_t data);
void TFT_SendData16(uint16_t data);
void TFT_SendCommandData(uint8_t cmd, uint8_t data);

/* 区域和光标 */
void TFT_SetRegion(uint16_t x_start, uint16_t y_start,
                   uint16_t x_end, uint16_t y_end);
void TFT_SetCursor(uint16_t x, uint16_t y);

/* 绘图函数 */
void TFT_Clear(uint16_t color);
void TFT_Fill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void TFT_DrawPoint(uint16_t x, uint16_t y, uint16_t color);
void TFT_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
                  uint16_t color);
void TFT_DrawCircle(uint16_t cx, uint16_t cy, uint16_t r, uint16_t color);
void TFT_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                  uint16_t color);
void TFT_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                  uint16_t color);

/* 文字显示 */
void TFT_ShowChar(uint8_t x, uint8_t y, uint16_t fc, uint16_t bc, char c);
void TFT_ShowString(uint8_t x, uint8_t y, uint16_t fc, uint16_t bc, char *str);
void TFT_ShowNumber(uint8_t x, uint8_t y, uint16_t fc, uint16_t bc,
                    int32_t num);

/* 图片显示 */
void TFT_ShowImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                   const unsigned char *p);

#ifdef __cplusplus
}
#endif

#endif /* __ST7735_TFT_H */

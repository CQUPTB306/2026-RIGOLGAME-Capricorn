/**
 * @file    grayscale.h
 * @brief   8路灰度传感器 MUX 驱动 — MSPM0G3519
 * @note    通过 AD0/AD1/AD2 三根地址线选择通道 (0-7),
 *          读取 OUT 引脚获得数字量 (0=黑线, 1=白底)
 *
 *          引脚:
 *            AD0  — PA12 (通道选择 bit0)
 *            AD1  — PB23 (通道选择 bit1)
 *            AD2  — PB27 (通道选择 bit2)
 *            OUT  — PB8  (传感器数据输出)
 */

#ifndef __GRAYSCALE_H__
#define __GRAYSCALE_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 通道数 ── */
#define GRAYSCALE_CHANNELS  8

/* ── 引脚宏: 由 ti_msp_dl_config.h 统一定义, 此处不再重复 ──
 *     (GRAY_AD0_PORT, GRAY_AD0_PIN, GRAY_AD0_IOMUX,
 *      GRAY_AD1_PORT, GRAY_AD1_PIN, GRAY_AD1_IOMUX,
 *      GRAY_AD2_PORT, GRAY_AD2_PIN, GRAY_AD2_IOMUX,
 *      GRAY_OUT_PORT, GRAY_OUT_PIN, GRAY_OUT_IOMUX) */

/* ── MUX 控制宏 (兼容参考代码风格) ── */

#define GRAY_AD0_WRITE(state)  do { \
    if (state) DL_GPIO_setPins(GRAY_AD0_PORT, GRAY_AD0_PIN); \
    else       DL_GPIO_clearPins(GRAY_AD0_PORT, GRAY_AD0_PIN); \
} while(0)

#define GRAY_AD1_WRITE(state)  do { \
    if (state) DL_GPIO_setPins(GRAY_AD1_PORT, GRAY_AD1_PIN); \
    else       DL_GPIO_clearPins(GRAY_AD1_PORT, GRAY_AD1_PIN); \
} while(0)

#define GRAY_AD2_WRITE(state)  do { \
    if (state) DL_GPIO_setPins(GRAY_AD2_PORT, GRAY_AD2_PIN); \
    else       DL_GPIO_clearPins(GRAY_AD2_PORT, GRAY_AD2_PIN); \
} while(0)

#define GRAY_OUT_READ()     (!!(DL_GPIO_readPins(GRAY_OUT_PORT, GRAY_OUT_PIN)))

/* ── API ── */

void    Grayscale_Init(void);
void    Grayscale_Read_All(uint8_t sensor[8]);
uint8_t Grayscale_Read_Single(uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif /* __GRAYSCALE_H__ */

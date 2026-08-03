/**
 * @file    key.h
 * @brief   按键驱动 — MSPM0G3519, 2 路开关量输入
 * @note    外围上拉 + 内部上拉, 按下 = 低电平
 *          KEY1: PB11 (PINCM28)
 *          KEY2: PA30 (PINCM5)
 */

#ifndef __KEY_H__
#define __KEY_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 引脚定义 ── */
#define KEY1_PORT       GPIOB
#define KEY1_PIN        DL_GPIO_PIN_11
#define KEY1_IOMUX      IOMUX_PINCM28

#define KEY2_PORT       GPIOA
#define KEY2_PIN        DL_GPIO_PIN_30
#define KEY2_IOMUX      IOMUX_PINCM5

/* ── 按键编号 ── */
#define KEY_NONE        0
#define KEY1_PRESS      1
#define KEY2_PRESS      2

/* ── 去抖时间 (ms) ── */
#define KEY_DEBOUNCE_MS  20

/* ── API ── */
void    Key_Init(void);
uint8_t Key_Read(void);
uint8_t Key_GetState(uint8_t key_id);

#ifdef __cplusplus
}
#endif

#endif /* __KEY_H__ */
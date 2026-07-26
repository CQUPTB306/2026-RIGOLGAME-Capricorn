/**
 * @file    encoder.h
 * @brief   MSPM0G3519 双路硬件正交编码器 — TIMG QEI
 *
 *          左编码器: PA24(PHA) + PA26(PHB) → QEI_LEFT  (TIMGx)
 *          右编码器: PA2(PHA)  + PB7(PHB)  → QEI_RIGHT (TIMGy)
 *
 *          SysConfig 需配置两个 TIMG 为 QEI 模式 (4倍频).
 *          若 ENC2 跨端口无法配硬件 QEI, 则软件 GPIO 中断解码.
 */

#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ====================================================================
 *   编码器物理参数
 * ==================================================================== */

#define ENCODER_LINES           13          /* 编码器线数 (每圈脉冲)     */
#define GEAR_RATIO              30          /* 减速比                    */
#define WHEEL_DIAMETER          65.0f       /* 车轮直径 (mm)             */
#define PULSE_PER_ROUND         ((uint16_t)(ENCODER_LINES * GEAR_RATIO * 4))
#define MM_PER_PULSE            (3.1415926f * WHEEL_DIAMETER / PULSE_PER_ROUND)

/* 编码器方向修正 (+1 或 -1), 若实际方向相反则改符号 */
#define ENC_LEFT_DIR            1
#define ENC_RIGHT_DIR           1

/* 控制周期 (s), 必须与 TIMG0 定时器周期一致 */
#define CONTROL_PERIOD_S        0.01f

/* ====================================================================
 *   硬件 QEI 实例 (由 SysConfig 生成, 见 ti_msp_dl_config.h)
 * ==================================================================== */

/* 左编码器 — PA24(PHA) + PA26(PHB) */
#define QEI_LEFT_INST           QEI_0_INST
#define QEI_LEFT_PHA_PORT       GPIOA
#define QEI_LEFT_PHA_PIN        DL_GPIO_PIN_24
#define QEI_LEFT_PHB_PORT       GPIOA
#define QEI_LEFT_PHB_PIN        DL_GPIO_PIN_26

/* 右编码器 — PA2(PHA) + PB7(PHB)
 * 若 SysConfig 无法配置跨端口 QEI, 改 ENC2_USE_SOFTWARE 为 1 */
#define ENC2_USE_SOFTWARE       1   /* 0=硬件QEI, 1=GPIO中断软件解码 (默认1: PA2+PB7跨端口硬件QEI不可行) */

#if ENC2_USE_SOFTWARE
#define QEI_RIGHT_INST          0   /* 不使用硬件实例 */
#else
#define QEI_RIGHT_INST          QEI_1_INST
#endif
#define QEI_RIGHT_PHA_PORT      GPIOA
#define QEI_RIGHT_PHA_PIN       DL_GPIO_PIN_2
#define QEI_RIGHT_PHB_PORT      GPIOB
#define QEI_RIGHT_PHB_PIN       DL_GPIO_PIN_7

/* ====================================================================
 *                        公共 API
 * ==================================================================== */

void    Encoder_Init(void);
void    Encoder_GetSpeed(float *speed_l, float *speed_r);
float   Encoder_GetSpeedLeft(void);
float   Encoder_GetSpeedRight(void);
int32_t Encoder_GetDistance(void);
uint32_t Encoder_GetCallCount(void);
void    Encoder_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H__ */

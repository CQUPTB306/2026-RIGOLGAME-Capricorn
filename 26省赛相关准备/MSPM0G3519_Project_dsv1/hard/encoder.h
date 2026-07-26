/**
 * @file    encoder.h
 * @brief   MSPM0G3507 硬件正交编码器 — 单路 TIMG8 QEI
 *
 *          SysConfig 已配置 TIMG8 为 QEI 模式 (4倍频):
 *            PHA: PB21 (IOMUX_PINCM49)
 *            PHB: PB22 (IOMUX_PINCM50)
 *
 *          硬件自动计数，无需 GPIO 中断，无需软件解码。
 *          读取 DL_TimerG_getTimerCount(QEI_0_INST) 即可获得脉冲数。
 */

#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ====================================================================
 *   编码器参数 (根据实际硬件修改)
 * ==================================================================== */

/** @brief 编码器线数 (每圈脉冲数, 未倍频) */
#define ENCODER_LINES           13

/** @brief 减速比 (电机转一圈 = 车轮转 1/GEAR_RATIO 圈) */
#define GEAR_RATIO              30

/** @brief 车轮直径 (mm) */
#define WHEEL_DIAMETER          65.0f

/** @brief 每圈脉冲数 (4倍频) */
#define PULSE_PER_ROUND         ((uint16_t)(ENCODER_LINES * GEAR_RATIO * 4))

/** @brief 每个脉冲对应的距离 (mm) */
#define MM_PER_PULSE            (3.1415926f * WHEEL_DIAMETER / PULSE_PER_ROUND)

/** @brief 编码器方向修正 (+1 或 -1) */
#define ENC_DIR                 1

/** @brief 控制周期 (s), 需与 SysConfig TIMER_0 周期一致 */
#define CONTROL_PERIOD_S        0.01f

/* ====================================================================
 *   硬件引脚 (SysConfig 生成, 见 ti_msp_dl_config.h)
 * ==================================================================== */

#define QEI_INST                QEI_0_INST      /**< TIMG8 QEI 实例  */
#define QEI_PHA_PORT            GPIOB            /**< A 相端口       */
#define QEI_PHA_PIN             DL_GPIO_PIN_21   /**< A 相引脚 PB21  */
#define QEI_PHA_IOMUX           (IOMUX_PINCM49)  /**< A 相 IOMUX     */
#define QEI_PHB_PORT            GPIOB            /**< B 相端口       */
#define QEI_PHB_PIN             DL_GPIO_PIN_22   /**< B 相引脚 PB22  */
#define QEI_PHB_IOMUX           (IOMUX_PINCM50)  /**< B 相 IOMUX     */

/* ====================================================================
 *                        公共 API
 * ==================================================================== */

/**
 * @brief  初始化硬件 QEI 编码器
 * @note   需在 SYSCFG_DL_init() 之后调用。
 *         TIMG8 的时钟和 QEI 模式已由 SysConfig 配置,
 *         此函数仅启动计数器并清零累计值。
 */
void Encoder_Init(void);

/**
 * @brief  读取当前编码器原始计数值 (4倍频脉冲数)
 * @return 当前计数器值 (int16_t, 正=前进, 负=后退)
 * @note   硬件自动处理方向, 计数器值直接反映旋转方向和脉冲数
 */
int16_t Encoder_GetCount(void);

/**
 * @brief  获取自上次调用以来的脉冲增量
 * @return 脉冲增量 (正=前进, 负=后退)
 * @note   每次调用会更新内部 last_count, 用于速度计算
 */
int16_t Encoder_GetDelta(void);

/**
 * @brief  获取当前速度 (硬件累加 + 周期换算)
 * @return 速度 (mm/s), 正=前进, 负=后退
 * @note   在每个控制周期调用一次, 内部自动计算 delta 并换算速度
 */
float Encoder_GetSpeed(void);

/**
 * @brief  获取累计行驶距离
 * @return 累计距离 (mm)
 * @note   内部累加, 前进增/后退减
 */
float Encoder_GetDistance(void);

/**
 * @brief  重置编码器计数和累计距离
 */
void Encoder_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H__ */

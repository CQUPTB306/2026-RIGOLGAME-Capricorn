/**
 * @file    encoder.c
 * @brief   MSPM0G3507 硬件正交编码器 — 单路 TIMG8 QEI 实现
 *
 *          TIMG8 硬件 QEI 模式 (SysConfig 配置):
 *            - PHA: PB21 (IOMUX_PINCM49), PHB: PB22 (IOMUX_PINCM50)
 *            - 4 倍频: 每个编码器线产生 4 个计数脉冲
 *            - 硬件自动判断方向 (正转加, 反转减)
 *            - LoadValue = 65535 (16位计数器)
 *
 *          与软件 GPIO 中断方式相比:
 *            - 无中断开销, 无丢脉冲风险
 *            - 方向由硬件判別, 无需软件读 B 相电平
 *            - 只需在读周期内调用 DL_TimerG_getTimerCount() 即可
 */

#include "encoder.h"

/* ====================================================================
 *           内部状态变量
 * ==================================================================== */

static int16_t  s_last_count  = 0;      /* 上次读取的计数值             */
static float    s_speed       = 0.0f;   /* 当前速度 (mm/s)              */
static float    s_distance    = 0.0f;   /* 累计距离 (mm)                */
static uint32_t s_call_count  = 0;      /* 调用次数 (调试用)            */

/* ====================================================================
 *           初始化
 * ==================================================================== */

void Encoder_Init(void)
{
    /*
     * TIMG8 的 QEI 模式已由 SYSCFG_DL_QEI_0_init() 完成:
     *   - DL_TimerG_setClockConfig(QEI_0_INST, ...)
     *   - DL_TimerG_configQEI(QEI_0_INST, DL_TIMER_QEI_MODE_2_INPUT, ...)
     *   - DL_TimerG_setLoadValue(QEI_0_INST, 65535)
     *   - DL_TimerG_enableClock(QEI_0_INST)
     *
     * 只需启动计数器。
     */
    DL_TimerG_startCounter(QEI_INST);

    /* 清零内部状态 */
    s_last_count  = (int16_t)DL_TimerG_getTimerCount(QEI_INST);
    s_speed       = 0.0f;
    s_distance    = 0.0f;
    s_call_count  = 0;
}

/* ====================================================================
 *           原始计数值读取
 * ==================================================================== */

int16_t Encoder_GetCount(void)
{
    /*
     * DL_TimerG_getTimerCount 返回 32 位值,
     * 但计数器配置为 16 位 (LoadValue=65535),
     * 取低 16 位转换为有符号 int16_t
     */
    return (int16_t)DL_TimerG_getTimerCount(QEI_INST);
}

/* ====================================================================
 *           增量计算
 * ==================================================================== */

int16_t Encoder_GetDelta(void)
{
    int16_t count = Encoder_GetCount();
    int16_t delta = count - s_last_count;

    s_last_count = count;
    return delta;
}

/* ====================================================================
 *           累计距离更新 (内部辅助, 必须在调用者之前定义)
 * ==================================================================== */

/**
 * @brief  更新累计距离
 * @note   每次控制周期累加一次, 前进增/后退减
 */
static void Encoder_UpdateDistance(int16_t delta)
{
    s_distance += delta * MM_PER_PULSE * ENC_DIR;
}

/* ====================================================================
 *           速度计算 (mm/s)
 *
 *  公式推导:
 *    delta         = 脉冲增量 (4倍频后)
 *    mm_per_pulse  = π × D / (编码器线数 × 减速比 × 4)
 *    speed (mm/s)  = delta × mm_per_pulse / CONTROL_PERIOD_S
 *
 *  不需要编码器方向修正, 硬件 QEI 已自动处理正反转方向。
 *  若实际方向与预期相反, 修改 ENC_DIR 宏 (或交换 PHA/PHB 接线)。
 * ==================================================================== */

float Encoder_GetSpeed(void)
{
    int16_t delta = Encoder_GetDelta();

    /* 方向修正 */
    delta *= ENC_DIR;

    /* 速度 = 距离增量 / 时间 */
    float speed = (delta * MM_PER_PULSE) / CONTROL_PERIOD_S;

    s_speed = speed;

    /* 同步更新累计距离 */
    Encoder_UpdateDistance(delta);

    s_call_count++;

    return speed;
}

/* ====================================================================
 *           累计距离 (mm)
 * ==================================================================== */

float Encoder_GetDistance(void)
{
    return s_distance;
}

/* ====================================================================
 *           综合读取 (速度 + 距离同步更新)
 *
 *  推荐用法: 在每个控制周期调用 Encoder_GetSpeed(),
 *           同时获得速度和累计距离, 避免两次读取导致计数不一致。
 * ==================================================================== */

/**
 * @brief  读取速度并同步更新累计距离
 * @param[out] speed    当前速度 (mm/s), 可为 NULL
 * @param[out] distance 累计距离 (mm),   可为 NULL
 * @note   内部只调用一次 DL_TimerG_getTimerCount(),
 *         确保速度和距离来自同一次计数快照。
 */
void Encoder_Update(float *speed, float *distance)
{
    int16_t delta = Encoder_GetDelta();

    /* 累计距离 */
    Encoder_UpdateDistance(delta);

    s_speed = (delta * MM_PER_PULSE * ENC_DIR) / CONTROL_PERIOD_S;

    if (speed != NULL)
        *speed = s_speed;

    if (distance != NULL)
        *distance = s_distance;

    s_call_count++;
}

/* ====================================================================
 *           重置
 * ==================================================================== */

void Encoder_Reset(void)
{
    /*
     * 重置硬件计数器: 写 0 到 VAL 不会重置计数器,
     * 但可以通过设置 LOAD 值间接重置, 或者直接记录偏移量。
     * 这里采用软件偏移方式: 记录当前值作为零点。
     */
    s_last_count  = (int16_t)DL_TimerG_getTimerCount(QEI_INST);
    s_speed       = 0.0f;
    s_distance    = 0.0f;
    s_call_count  = 0;
}

/* ====================================================================
 *           调试辅助
 * ==================================================================== */

/**
 * @brief  获取调用次数 (调试用, 确认 ISR / 控制循环在运行)
 * @return 自 Init/Reset 以来的 Encoder_GetSpeed 调用次数
 */
uint32_t Encoder_GetCallCount(void)
{
    return s_call_count;
}

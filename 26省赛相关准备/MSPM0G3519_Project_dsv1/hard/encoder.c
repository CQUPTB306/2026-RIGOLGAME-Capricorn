/**
 * @file    encoder.c
 * @brief   MSPM0G3519 双路硬件正交编码器实现
 *
 *          左编码器 (ENC1): TIMGx QEI, PA24(PHA)+PA26(PHB)
 *          右编码器 (ENC2): TIMGy QEI, PA2(PHA)+PB7(PHB)
 *                         或 GPIO 中断软件解码 (ENC2_USE_SOFTWARE=1)
 */

#include "encoder.h"

/* ====================================================================
 *           内部状态
 * ==================================================================== */

static int16_t  s_last_count_l  = 0;
static int16_t  s_last_count_r  = 0;
static float    s_speed_l       = 0.0f;
static float    s_speed_r       = 0.0f;
static float    s_distance      = 0.0f;
static uint32_t s_call_count    = 0;

/* ====================================================================
 *           初始化
 * ==================================================================== */

void Encoder_Init(void)
{
    /* 左编码器 QEI — 启动计数器 */
    DL_TimerG_startCounter(QEI_LEFT_INST);
    s_last_count_l = (int16_t)DL_TimerG_getTimerCount(QEI_LEFT_INST);

    /* 右编码器 — 硬件 QEI 或软件中断 */
#if ENC2_USE_SOFTWARE
    /* 软件解码: 配置 PA2/PB7 为双边沿中断输入 (由 SysConfig 处理) */
    /* 中断处理在 stm32f1xx_it.c 对应文件中实现 */
    s_last_count_r = 0;
#else
    DL_TimerG_startCounter(QEI_RIGHT_INST);
    s_last_count_r = (int16_t)DL_TimerG_getTimerCount(QEI_RIGHT_INST);
#endif

    s_speed_l    = 0.0f;
    s_speed_r    = 0.0f;
    s_distance   = 0.0f;
    s_call_count = 0;
}

/* ====================================================================
 *           速度读取 (mm/s)
 * ==================================================================== */

/**
 * @brief 同时读取左右轮速度
 * @param[out] speed_l  左轮速度 (mm/s), 正=前进; 可为 NULL
 * @param[out] speed_r  右轮速度 (mm/s), 正=前进; 可为 NULL
 *
 * 公式: speed = delta * MM_PER_PULSE * ENC_DIR / CONTROL_PERIOD_S
 * delta = 当前计数值 - 上次计数值 (int16_t 减法自动处理 16位翻转)
 */
void Encoder_GetSpeed(float *speed_l, float *speed_r)
{
    /* ── 左编码器 ── */
    int16_t count_l = (int16_t)DL_TimerG_getTimerCount(QEI_LEFT_INST);
    int16_t delta_l = (count_l - s_last_count_l) * ENC_LEFT_DIR;
    s_last_count_l  = count_l;
    float spd_l     = (delta_l * MM_PER_PULSE) / CONTROL_PERIOD_S;

    /* ── 右编码器 ── */
#if ENC2_USE_SOFTWARE
    /* 软件解码: delta 由 GPIO 中断累加, 此处读取并清零 */
    int16_t delta_r = 0;  /* TODO: 从中断累加变量读取 */
    float spd_r     = (delta_r * MM_PER_PULSE) / CONTROL_PERIOD_S;
#else
    int16_t count_r = (int16_t)DL_TimerG_getTimerCount(QEI_RIGHT_INST);
    int16_t delta_r = (count_r - s_last_count_r) * ENC_RIGHT_DIR;
    s_last_count_r  = count_r;
    float spd_r     = (delta_r * MM_PER_PULSE) / CONTROL_PERIOD_S;
#endif

    /* 保存 */
    s_speed_l = spd_l;
    s_speed_r = spd_r;

    /* 累计距离 (两轮同向时累加平均) */
    if (spd_l > 0.0f && spd_r > 0.0f)
        s_distance += (float)((delta_l + delta_r) / 2) * MM_PER_PULSE;
    else if (spd_l < 0.0f && spd_r < 0.0f)
        s_distance -= (float)((delta_l + delta_r) / 2) * MM_PER_PULSE;

    s_call_count++;

    if (speed_l != NULL) *speed_l = spd_l;
    if (speed_r != NULL) *speed_r = spd_r;
}

/* ── 单独读取 ── */

float Encoder_GetSpeedLeft(void)
{
    float spd;
    Encoder_GetSpeed(&spd, NULL);
    return spd;
}

float Encoder_GetSpeedRight(void)
{
    float spd;
    Encoder_GetSpeed(NULL, &spd);
    return spd;
}

/* ====================================================================
 *           累计距离
 * ==================================================================== */

int32_t Encoder_GetDistance(void)
{
    return (int32_t)s_distance;
}

/* ====================================================================
 *           重置
 * ==================================================================== */

void Encoder_Reset(void)
{
    s_last_count_l = (int16_t)DL_TimerG_getTimerCount(QEI_LEFT_INST);
#if !ENC2_USE_SOFTWARE
    s_last_count_r = (int16_t)DL_TimerG_getTimerCount(QEI_RIGHT_INST);
#endif
    s_speed_l    = 0.0f;
    s_speed_r    = 0.0f;
    s_distance   = 0.0f;
    s_call_count = 0;
}

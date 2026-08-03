/**
 * @file    motor.c
 * @brief   双路电机 + 速度 PI
 */

#include "motor.h"

Motor_t g_motor_l;
Motor_t g_motor_r;

/* ── PI (定点 Q12, 无浮点) ── */

void PI_Init(PI_t *pi, int32_t kp, int32_t ki, int32_t kd, int32_t omax)
{
    pi->Kp = kp;
    pi->Ki = ki;
    pi->Kd = kd;
    pi->integral   = 0;
    pi->prev_error = 0;
    pi->out_max    = omax;
}

int32_t PI_Calc(PI_t *pi, int32_t setpoint, int32_t measured)
{
    int32_t err = setpoint - measured;
    int32_t out = (pi->Kp * err) >> 12;

    pi->integral += err;
    if      (pi->integral >  2048000) pi->integral =  2048000;   /* 500*4096 */
    else if (pi->integral < -2048000) pi->integral = -2048000;
    out += (pi->Ki * pi->integral) >> 12;

    out += (pi->Kd * (err - pi->prev_error)) >> 12;
    pi->prev_error = err;

    if      (out >  pi->out_max) out =  pi->out_max;
    else if (out < -pi->out_max) out = -pi->out_max;

    return out;
}

/* ── 电机驱动 ── */

static uint16_t g_ramp_step = 1;    /* 缓启动步长, 每 Tick 变化量 */
static int16_t  g_l_cur    = 0;      /* 左轮当前实际速度 */
static int16_t  g_r_cur    = 0;      /* 右轮当前实际速度 */
static uint8_t  g_l_ramping = 0;     /* 左轮缓启动进行中 */
static uint8_t  g_r_ramping = 0;     /* 右轮缓启动进行中 */
static uint8_t  g_ramp_once_done = 0; /* 一次性启动缓启锁存: 只在首次启动生效 */

void Motor_Init(void)
{
    CIN1_LOW(); CIN2_LOW();
    DIN1_LOW(); DIN2_LOW();
    PWM_L_SET(PWM_MAX);
    PWM_R_SET(PWM_MAX);
    DL_TimerA_startCounter(PWM_0_INST);
    g_l_cur = 75; g_l_ramping = 0;
    g_r_cur = 50; g_r_ramping = 0;
}

void Motor_Stop(void)
{
    CIN1_LOW(); CIN2_LOW();
    DIN1_LOW(); DIN2_LOW();
    PWM_L_SET(PWM_MAX);
    PWM_R_SET(PWM_MAX);
    g_l_cur = 0; g_l_ramping = 0;
    g_r_cur = 0; g_r_ramping = 0;
}

void Motor_SetRamp(uint16_t step)
{
    if (step > 0 && step <= PWM_MAX)
        g_ramp_step = step;
}

/**
 * @brief  电机驱动
 * @note   静止→启动: 每 Tick 向目标靠近 g_ramp_step, 直到追上目标
 *         已行驶中:   直接设为目标, 保证循迹响应速度
 *         Motor_Stop() 后重新触发缓启动
 */
void Motor(int16_t l_target, int16_t r_target)
{
    int16_t l = l_target, r = r_target;

    /* 钳位 */
    if (l > PWM_MAX) l = PWM_MAX; else if (l < -PWM_MAX) l = -PWM_MAX;
    if (r > PWM_MAX) r = PWM_MAX; else if (r < -PWM_MAX) r = -PWM_MAX;

    /* ── 一次性启动缓启: 首次收到非零目标 → 进入缓启动, 之后锁存不再触发 ── */
    if (!g_ramp_once_done && (l != 0 || r != 0))
    {
        g_ramp_once_done = 1;
        g_l_ramping = 1;
        g_r_ramping = 1;
    }

    /* ── 左轮 ── */
    // if (g_l_cur == 0 && l != 0 && !g_l_ramping)
    //     g_l_ramping = 1;                    /* 从静止启动, 进入缓启动 */
    if (g_l_ramping)
    {
        if (g_l_cur < l) {
            g_l_cur += (int16_t)g_ramp_step;
            if (g_l_cur >= l) { g_l_cur = l; g_l_ramping = 0; }
        } else if (g_l_cur > l) {
            g_l_cur -= (int16_t)g_ramp_step;
            if (g_l_cur <= l) { g_l_cur = l; g_l_ramping = 0; }
        } else {
            g_l_ramping = 0;                /* 已到达 */
         }
     }
    else
    {
        g_l_cur = l;                        /* 行驶中, 直接响应 */
    }

    /* ── 右轮 ── */
    // if (g_r_cur == 0 && r != 0 && !g_r_ramping)
    //     g_r_ramping = 1;

    if (g_r_ramping)
    {
        if (g_r_cur < r) {
            g_r_cur += (int16_t)g_ramp_step;
            if (g_r_cur >= r) { g_r_cur = r; g_r_ramping = 0; }
        } else if (g_r_cur > r) {
            g_r_cur -= (int16_t)g_ramp_step;
            if (g_r_cur <= r) { g_r_cur = r; g_r_ramping = 0; }
        } else {
            g_r_ramping = 0;
         }
    }
    else
    {
        g_r_cur = r;
    }

    /* ── 实际 PWM 输出 ── */
    if (g_l_cur >= 0) {
        CIN1_HIGH(); CIN2_LOW();
        PWM_L_SET((uint16_t)(PWM_MAX - g_l_cur) - (g_l_cur < PWM_MAX ? 1 : 0));
    } else {
        CIN1_LOW(); CIN2_HIGH();
        PWM_L_SET((uint16_t)(PWM_MAX + g_l_cur));
    }

    if (g_r_cur >= 0) {
        DIN1_HIGH(); DIN2_LOW();
        PWM_R_SET((uint16_t)(PWM_MAX - g_r_cur) - (g_r_cur < PWM_MAX ? 1 : 0));
    } else {
        DIN1_LOW(); DIN2_HIGH();
        PWM_R_SET((uint16_t)(PWM_MAX + g_r_cur));
    }
}


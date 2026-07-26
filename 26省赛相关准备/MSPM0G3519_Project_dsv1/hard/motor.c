/**
 * @file    motor.c
 * @brief   MSPM0G3519 双路直流电机驱动 + 速度闭环实现
 *
 *          级联 PID 架构:
 *            外环循迹 PID → Motor_SetSpeed(target_l, target_r) → 设定目标速度
 *            内环速度 PID → Motor_Control_Loop() → Motor(pwm_l, pwm_r) → TB6612
 *
 *          参考: D:\!ziv\Nova\2026-RIGOLGAME-Capricorn\source\project\hard\motor\motor.c
 */

#include "motor.h"

/* ====================================================================
 *                        全局变量
 * ==================================================================== */

SpeedCtrl g_speed_left;
SpeedCtrl g_speed_right;
volatile uint8_t g_motor_control_flag = 0;
volatile uint8_t track_flag = 0;
volatile uint8_t turn_flag  = 0;

/* ====================================================================
 *                        PID 控制器
 * ==================================================================== */

/**
 * @brief 初始化 PID 控制器
 * @param out_max  输出上限 (同时也是下限的绝对值)
 */
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float out_max)
{
    pid->Kp          = kp;
    pid->Ki          = ki;
    pid->Kd          = kd;
    pid->integral    = 0.0f;
    pid->prev_error  = 0.0f;
    pid->last_output = 0.0f;
    pid->out_max     = out_max;
    pid->out_min     = -out_max;
}

/**
 * @brief PID 计算 (位置式 + 积分限幅 + 一阶低通滤波)
 *
 *  公式:
 *    raw = Kp*e + Ki*∫e + Kd*de
 *    output = 0.08 * last_output + 0.92 * raw   (一阶低通)
 *
 *  参考 Capricorn motor.c 的 PID_Calculate()
 */
float PID_Calc_Motor(PID_Controller *pid, float setpoint, float measurement)
{
    float error = setpoint - measurement;

    /* 比例 */
    float Pout = pid->Kp * error;

    /* 积分 (累加, 不乘 dt — 与 Capricorn 一致) */
    pid->integral += error;

    /* 积分限幅 ±1000, 防止编码器故障飞车 */
    if (pid->integral > 1000.0f)  pid->integral = 1000.0f;
    if (pid->integral < -1000.0f) pid->integral = -1000.0f;

    float Iout = pid->Ki * pid->integral;

    /* 微分 (不除 dt — 与 Capricorn 一致) */
    float derivative = error - pid->prev_error;
    float Dout = pid->Kd * derivative;

    /* 原始 PID 输出 */
    float raw_output = Pout + Iout + Dout;

    /* 一阶低通滤波: 0.08*last + 0.92*raw */
    float filtered_output = 0.08f * pid->last_output + 0.92f * raw_output;

    /* 输出限幅 */
    if (filtered_output > pid->out_max) filtered_output = pid->out_max;
    if (filtered_output < pid->out_min) filtered_output = pid->out_min;

    /* 保存状态 */
    pid->prev_error  = error;
    pid->last_output = filtered_output;

    return filtered_output;
}

/* ====================================================================
 *                        底层电机控制
 * ==================================================================== */

/**
 * @brief 电机初始化 — GPIO 方向引脚 + PWM 启动
 */
void Motor_Init(void)
{
    /* 方向引脚初始低电平 (H 桥关断) */
    CIN1_LOW();
    CIN2_LOW();
    DIN1_LOW();
    DIN2_LOW();

    /* PWM 输出最大占空比 → CCP=0, 但方向引脚全低, 电机不转 */
    PWM_L_SET(PWM_MAX);
    PWM_R_SET(PWM_MAX);

    DL_TimerA_startCounter(PWM_0_INST);
}

/**
 * @brief 停止所有电机
 */
void Motor_Stop(void)
{
    CIN1_LOW();
    CIN2_LOW();
    DIN1_LOW();
    DIN2_LOW();

    PWM_L_SET(PWM_MAX);
    PWM_R_SET(PWM_MAX);
}

/**
 * @brief 底层电机 PWM 驱动 (参考 Capricorn Motor())
 *
 * @param left_speed   正=前进, 负=后退, 范围 -PWM_MAX ~ +PWM_MAX
 * @param right_speed  正=前进, 负=后退, 范围 -PWM_MAX ~ +PWM_MAX
 */
void Motor(int16_t left_speed, int16_t right_speed)
{
    /* 限幅 */
    if (left_speed > PWM_MAX)   left_speed = PWM_MAX;
    if (left_speed < -PWM_MAX)  left_speed = -PWM_MAX;
    if (right_speed > PWM_MAX)  right_speed = PWM_MAX;
    if (right_speed < -PWM_MAX) right_speed = -PWM_MAX;

    /* 左轮 */
    if (left_speed >= 0) {
        CIN1_HIGH(); CIN2_LOW();
        PWM_L_SET((uint16_t)left_speed);
    } else {
        CIN1_LOW(); CIN2_HIGH();
        PWM_L_SET((uint16_t)(-left_speed));
    }

    /* 右轮 */
    if (right_speed >= 0) {
        DIN1_HIGH(); DIN2_LOW();
        PWM_R_SET((uint16_t)right_speed);
    } else {
        DIN1_LOW(); DIN2_HIGH();
        PWM_R_SET((uint16_t)(-right_speed));
    }
}

/* ====================================================================
 *                        速度闭环
 * ==================================================================== */

/**
 * @brief 初始化速度闭环控制
 *
 * 调用顺序: SYSCFG_DL_init() → Motor_Init() → Motor_Control_Init() → 主循环
 */
void Motor_Control_Init(void)
{
    /* 初始化编码器 (双路 QEI) */
    Encoder_Init();

    /* 初始化左右速度 PID — 参数继承 Capricorn 调试值 */
    PID_Init(&g_speed_left.pid,  2.15f, 1.1f, 0.001f, (float)PWM_MAX);
    PID_Init(&g_speed_right.pid, 2.15f, 1.1f, 0.001f, (float)PWM_MAX);

    /* 清零状态 */
    g_speed_left.target_speed   = 0.0f;
    g_speed_right.target_speed  = 0.0f;
    g_speed_left.current_speed  = 0.0f;
    g_speed_right.current_speed = 0.0f;
    g_speed_left.output         = 0;
    g_speed_right.output        = 0;

    g_motor_control_flag = 0;
    track_flag = 0;
    turn_flag  = 0;
}

/**
 * @brief 设定目标速度 (mm/s), 由循迹 PID 外环调用
 */
void Motor_SetSpeed(float left_mm_s, float right_mm_s)
{
    g_speed_left.target_speed  = left_mm_s;
    g_speed_right.target_speed = right_mm_s;
}

/**
 * @brief 速度控制主循环 — 每个控制周期 (100Hz) 调用一次
 *
 * 调用链:
 *   1. Encoder_GetSpeed() → 读取左右轮实际速度
 *   2. PID_Calc_Motor()   → 速度 PID 计算 (仅循迹模式)
 *   3. Motor()            → PWM 输出
 *
 * track_flag/turn_flag 控制输出模式 (参考 Capricorn):
 *   - turn_flag==1: 使用目标速度直接作为 PWM, 忽略 PID
 *   - track_flag==1: 使用速度 PID 输出
 *   - 默认: 停止电机
 */
void Motor_Control_Loop(void)
{
    float speed_l, speed_r;

    /* 1. 读取双编码器速度 (始终读取, 可用于诊断) */
    Encoder_GetSpeed(&speed_l, &speed_r);
    g_speed_left.current_speed  = speed_l;
    g_speed_right.current_speed = speed_r;

    /* 2. 速度 PID 计算 (仅循迹模式需要, 转弯模式直接使用目标值) */
    if (track_flag == 1)
    {
        float out_l = PID_Calc_Motor(&g_speed_left.pid,
                                      g_speed_left.target_speed, speed_l);
        float out_r = PID_Calc_Motor(&g_speed_right.pid,
                                      g_speed_right.target_speed, speed_r);

        g_speed_left.output  = (int16_t)out_l;
        g_speed_right.output = (int16_t)out_r;
    }

    /* 3. 电机输出 (根据模式标志选择) */
    if (turn_flag == 1)
    {
        /* 转弯模式: 使用目标速度直接作为 PWM
         * (Track_Run 通过 Motor_SetSpeed 设定为 ±TURN_SPEED_MM_S 范围,
         *  不经过 PID, 直接输出到 Motor) */
        Motor((int16_t)g_speed_left.target_speed,
              (int16_t)g_speed_right.target_speed);
    }
    else if (track_flag == 1)
    {
        /* 循迹模式: 使用速度 PID 输出 */
        Motor(g_speed_left.output, g_speed_right.output);
    }
    else
    {
        /* 空闲: 停止电机 */
        Motor_Stop();
    }
}

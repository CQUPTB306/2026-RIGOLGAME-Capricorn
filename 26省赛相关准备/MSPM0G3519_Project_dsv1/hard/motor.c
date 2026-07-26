/**
 * @file    motor.c
 * @brief   MSPM0G3507 双路直流电机驱动实现
 *
 *          开环 / 闭环双模式，通过 motor.h 中的 MOTOR_USE_ENCODER 切换。
 *
 *          硬件连接:
 *          - 电机驱动芯片: TB6612 或兼容 (CIN1/CIN2→C路, DIN1/DIN2→D路)
 *          - PWM: TIMA1 CCP0=PA15(左), CCP1=PA16(右), period=1000
 *          - 编码器 (闭环模式): TIMG8 QEI, PB21(PHA) + PB22(PHB)
 *          - 控制定时器 (闭环模式): TIMG0 → 100Hz 周期中断 (SysConfig)
 */

#include "ti_msp_dl_config.h"
#include "motor.h"

#if MOTOR_USE_ENCODER
#include "encoder.h"
#endif

/* ====================================================================
 *                        全局变量
 * ==================================================================== */

#if MOTOR_USE_ENCODER
SpeedCtrl g_speed_left;
SpeedCtrl g_speed_right;
volatile uint8_t g_motor_control_flag = 0;
#endif

/* ====================================================================
 *               MotorA / MotorB 单路底层控制 (内部使用)
 * ==================================================================== */

/**
 * @brief C路(左轮)方向+速度控制
 */
void MotorC_Control(uint8_t dir, uint16_t speed)
{
    /* 钳位: 确保 speed < PWM_MAX, 避免 PWM_MAX-speed-1 下溢为负数 */
    if (speed >= PWM_MAX) speed = PWM_MAX - 1;

    if (dir == 0) { CIN1_LOW();  CIN2_HIGH(); }
    else         { CIN1_HIGH(); CIN2_LOW();  }

    PWM_C_SET(PWM_MAX - (speed) - 1);
}

/**
 * @brief D路(右轮)方向+速度控制
 */
void MotorD_Control(uint8_t dir, uint16_t speed)
{
    /* 钳位: 确保 speed < PWM_MAX, 避免 PWM_MAX-speed-1 下溢为负数 */
    if (speed >= PWM_MAX) speed = PWM_MAX - 1;

    /* 右轮方向反转: 接线与左轮相反, dir==1→后退, dir==0→前进 */
    if (dir == 0) { DIN1_HIGH(); DIN2_LOW();  }
    else         { DIN1_LOW();  DIN2_HIGH(); }

    PWM_D_SET(PWM_MAX - (speed) - 1);
}

/* ====================================================================
 *                        公共 API 实现
 * ==================================================================== */

/**
 * @brief 电机初始化
 */
void Motor_Init(void)
{
    CIN1_LOW();
    CIN2_LOW();
    DIN1_LOW();
    DIN2_LOW();

    /* PWM_MAX → CCP=0 → ~100%占空比, 但方向引脚全低(H桥关断), 电机不转 */
    PWM_C_SET(PWM_MAX);
    PWM_D_SET(PWM_MAX);

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

    PWM_C_SET(PWM_MAX);
    PWM_D_SET(PWM_MAX);
}

/**
 * @brief 统一电机驱动接口
 *
 * 正数 = 前进, 负数 = 后退, 范围 -PWM_MAX ~ +PWM_MAX.
 */
void Motor_Set(int16_t left_speed, int16_t right_speed)
{
    if (left_speed > 0)
        MotorC_Control(0, left_speed);
    else
        MotorC_Control(1, -left_speed);

    if (right_speed > 0)
        MotorD_Control(0, right_speed);
    else
        MotorD_Control(1, -right_speed);
}

/* ── 辅助行驶 ── */

void Motor_Forward(uint16_t pwm_l, uint16_t pwm_r)
{
    Motor_Set((int16_t)pwm_l, (int16_t)pwm_r);
}

void Motor_Backward(uint16_t pwm_l, uint16_t pwm_r)
{
    Motor_Set(-(int16_t)pwm_l, -(int16_t)pwm_r);
}

void Motor_Turn_Left(uint16_t pwm)
{
    uint16_t pwm_left = pwm * 60 / 100;
    if (pwm_left > PWM_MAX) pwm_left = PWM_MAX;
    if (pwm > PWM_MAX) pwm = PWM_MAX;
    Motor_Set((int16_t)pwm_left, (int16_t)pwm);
}

void Motor_Turn_Right(uint16_t pwm)
{
    uint16_t pwm_right = pwm * 60 / 100;
    if (pwm_right > PWM_MAX) pwm_right = PWM_MAX;
    if (pwm > PWM_MAX) pwm = PWM_MAX;
    Motor_Set((int16_t)pwm, (int16_t)pwm_right);
}

/* ====================================================================
 *                 闭环模式实现 (MOTOR_USE_ENCODER = 1)
 * ==================================================================== */

#if MOTOR_USE_ENCODER

/* ====================================================================
 *                        PID 控制器
 * ==================================================================== */
/*
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float out_max)
{
    pid->Kp         = kp;
    pid->Ki         = ki;
    pid->Kd         = kd;
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->out_max    = out_max;
    pid->out_min    = -out_max;
}
*/
/**
 * @brief PID 计算 (位置式 + 积分抗饱和)
 *        公式: output = Kp*e + Ki*∫e*dt + Kd*de/dt
 */
float PID_Calc_Motor(PID_Controller *pid, float setpoint, float measurement)
{
    float error = setpoint - measurement;

    /* 比例 */
    float Pout = pid->Kp * error;

    /* 积分 (梯形积分 + 限幅防饱和) */
    pid->integral += error * CONTROL_PERIOD_S;

    /* 硬限制: 积分最多贡献 out_max 的 80%, 防止编码器故障时飞车 */
    float max_integral = 0.8f * pid->out_max;
    if (pid->Ki > 0.0001f) {
        float ki_limit = pid->out_max / pid->Ki;
        if (ki_limit < max_integral) max_integral = ki_limit;
    }
    if (pid->integral >  max_integral) pid->integral =  max_integral;
    if (pid->integral < -max_integral) pid->integral = -max_integral;

    float Iout = pid->Ki * pid->integral;

    /* 微分 */
    float derivative = (error - pid->prev_error) / CONTROL_PERIOD_S;
    float Dout = pid->Kd * derivative;
    pid->prev_error = error;

    /* 合成 + 输出限幅 */
    float output = Pout + Iout + Dout;
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    return output;
}

/* ====================================================================
 *                     速度闭环控制
 * ==================================================================== */

/**
 * @brief 初始化速度闭环控制
 *
 * 调用顺序: SYSCFG_DL_init() → Motor_Init() → Motor_Control_Init() → 主循环
 *
 * 需在 SysConfig 中提前配置:
 *   - 编码器: TIMG8 → QEI Mode (PB21 PHA + PB22 PHB)
 *   - 控制定时器: TIMA1 → Periodic Interrupt, 100Hz, 并使能 NVIC 中断
 */
void Motor_Control_Init(void)
{
    /* (1) 初始化硬件 QEI 编码器 (TIMG8) */
    Encoder_Init();

    /* (2) 初始化 PID — 左右独立 PID, 共享编码器反馈 */
    PID_Init(&g_speed_left.pid,  0.5f, 0.3f, 0.0f,  (float)PWM_MAX);
    PID_Init(&g_speed_right.pid, 0.5f, 0.3f, 0.0f,  (float)PWM_MAX);

    /* (3) 清零状态 */
    g_speed_left.target_speed  = 0.0f;
    g_speed_right.target_speed = 0.0f;
    g_speed_left.current_speed  = 0.0f;
    g_speed_right.current_speed = 0.0f;
    g_speed_left.output  = 0;
    g_speed_right.output = 0;
    g_speed_left.test  = 0;
    g_speed_right.test = 0;

    g_motor_control_flag = 0;
}

/**
 * @brief 设定目标速度 (mm/s)
 *
 * 正 = 前进, 负 = 后退.  典型范围: ±300 mm/s.
 */
void Motor_SetSpeed(float left_mm_s, float right_mm_s)
{
    g_speed_left.target_speed  = left_mm_s;
    g_speed_right.target_speed = right_mm_s;
}

/**
 * @brief 获取累计距离 (mm)
 */
int32_t Motor_GetDistance(void)
{
    return (int32_t)Encoder_GetDistance();
}

/**
 * @brief 速度控制主循环 — 每个控制周期调用一次
 *
 * 推荐调用方式 (在主循环中轮询, 不在 ISR 中做浮点运算):
 * @code
 *   while (1) {
 *       if (g_motor_control_flag) {
 *           g_motor_control_flag = 0;
 *           Motor_Control_Loop();
 *       }
 *       // 其他任务 ...
 *   }
 * @endcode
 *
 * @note  单编码器模式:
 *        只有一路 TIMG8 QEI 编码器, 测量一个轮子的速度作为反馈。
 *        左右轮共享同一速度测量值, 各自独立 PID 控制。
 *        如果需要精确的双轮独立控制, 需要再加一路 QEI 编码器。
 */
void Motor_Control_Loop(void)
{
    /*
     * 从 TIMG8 硬件 QEI 读取当前速度 (并同步更新累计距离)
     *
     * Encoder_GetSpeed() 内部:
     *   1. 读取 DL_TimerG_getTimerCount(QEI_INST) → 当前脉冲计数值
     *   2. 计算 delta = count - last_count (考虑 16位翻转)
     *   3. 速度 = delta × MM_PER_PULSE / CONTROL_PERIOD_S
     *   4. 累加距离
     */
    float speed = Encoder_GetSpeed();

    /* 更新左右轮当前速度 (共享同一编码器反馈) */
    g_speed_left.current_speed  = speed;
    g_speed_right.current_speed = speed;

    /* 左右独立 PID 计算, 共享编码器速度反馈 */
    float out_l = PID_Calc_Motor(&g_speed_left.pid,
                                  g_speed_left.target_speed, speed);
    float out_r = PID_Calc_Motor(&g_speed_right.pid,
                                  g_speed_right.target_speed, speed);

    g_speed_left.output  = (int16_t)out_l;
    g_speed_right.output = (int16_t)out_r;

    /* 驱动电机 */
    Motor_Set(g_speed_left.output, g_speed_right.output);
}

/* ====================================================================
 *              控制定时器中断服务程序 (ISR)
 *
 * ISR 名称: TIMG0_IRQHandler (由 SysConfig 生成)
 *
 * 中断源: CONTROL_TIMER_INST (TIMG0) 的 Zero/Load 事件
 *         (SysConfig → Timer → "Enable Zero IRQ")
 *
 * 用法 — 在 main.c 中实现:
 * @code
 *   void TIMG0_IRQHandler(void)
 *   {
 *       switch (DL_TimerG_getPendingInterrupt(CONTROL_TIMER_INST))
 *       {
 *           case DL_TIMER_IIDX_ZERO:
 *               g_motor_control_flag = 1;
 *               break;
 *           default:
 *               break;
 *       }
 *   }
 * @endcode
 * ==================================================================== */

#endif /* MOTOR_USE_ENCODER */

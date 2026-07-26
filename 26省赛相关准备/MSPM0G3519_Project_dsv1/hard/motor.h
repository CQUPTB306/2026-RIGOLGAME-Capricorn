/**
 * @file    motor.h
 * @brief   MSPM0G3507 双路直流电机驱动
 *
 *          两种工作模式，通过 MOTOR_USE_ENCODER 宏切换:
 *          - 开环模式 (默认, MOTOR_USE_ENCODER=0): 仅 Motor() 接口,
 *            直接 PWM 控制，无需编码器。
 *          - 闭环模式 (MOTOR_USE_ENCODER=1): 编码器 PID 速度闭环,
 *            Motor_SetSpeed() 设定 mm/s 目标速度，自动调节。
 *
 * @note    硬件引脚（由 SysConfig 生成，见 ti_msp_dl_config.h）:
 *          - CIN1: PA14, CIN2: PB17  (C路/左轮方向)
 *          - DIN1: PB18, DIN2: PB19  (D路/右轮方向)
 *          - PWM_C: PA15 (TIMA1 CCP0), PWM_D: PA16 (TIMA1 CCP1)
 *          - 编码器: TIMG8 QEI, PB21(PHA) + PB22(PHB)
 */

#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ====================================================================
 *           工作模式选择 (用户修改此处即可切换)
 * ==================================================================== */

/**
 * @brief 编码器闭环开关
 *        0 = 开环模式 — 不需要编码器，直接用 Motor(pwm_l, pwm_r) 控制
 *        1 = 闭环模式 — 需要编码器 + 控制定时器, 使用 Motor_SetSpeed(mm/s)
 */
#ifndef MOTOR_USE_ENCODER
#define MOTOR_USE_ENCODER       1
#endif

/* ====================================================================
 *                        公共常量
 * ==================================================================== */

/** @brief PWM 最大值（与 SysConfig 中 TIMG0 period 一致） */
#define PWM_MAX                 1000

/* ====================================================================
 *                        GPIO 方向宏
 * ==================================================================== */

/* C路电机 (左轮): CIN1=PA14, CIN2=PB17 */
#define CIN1_HIGH()             DL_GPIO_setPins(CIN1_PORT, CIN1_PIN_0_PIN)
#define CIN1_LOW()              DL_GPIO_clearPins(CIN1_PORT, CIN1_PIN_0_PIN)
#define CIN2_HIGH()             DL_GPIO_setPins(CIN2_PORT, CIN2_PIN_1_PIN)
#define CIN2_LOW()              DL_GPIO_clearPins(CIN2_PORT, CIN2_PIN_1_PIN)

/* D路电机 (右轮): DIN1=PB18, DIN2=PB19 */
#define DIN1_HIGH()             DL_GPIO_setPins(DIN1_PORT, DIN1_PIN_2_PIN)
#define DIN1_LOW()              DL_GPIO_clearPins(DIN1_PORT, DIN1_PIN_2_PIN)
#define DIN2_HIGH()             DL_GPIO_setPins(DIN2_PORT, DIN2_PIN_3_PIN)
#define DIN2_LOW()              DL_GPIO_clearPins(DIN2_PORT, DIN2_PIN_3_PIN)

/* ====================================================================
 *                        PWM 占空比宏
 * ==================================================================== */

/** @brief 左轮 PWM (TIMA1 CCP0 = PA15) */
#define PWM_C_SET(duty)         DL_TimerA_setCaptureCompareValue(PWM_0_INST, (duty), DL_TIMER_CC_0_INDEX)

/** @brief 右轮 PWM (TIMA1 CCP1 = PA16) */
#define PWM_D_SET(duty)         DL_TimerA_setCaptureCompareValue(PWM_0_INST, (duty), DL_TIMER_CC_1_INDEX)

/* ====================================================================
 *                   公共 API (两种模式均可用)
 * ==================================================================== */

void Motor_Init(void);
void Motor_Stop(void);
void Motor_Set(int16_t left_speed, int16_t right_speed);
void Motor_Forward(uint16_t pwm_l, uint16_t pwm_r);
void Motor_Backward(uint16_t pwm_l, uint16_t pwm_r);
void Motor_Turn_Left(uint16_t pwm);
void Motor_Turn_Right(uint16_t pwm);
void MotorC_Control(uint8_t dir, uint16_t speed);
void MotorD_Control(uint8_t dir, uint16_t speed);

/* ====================================================================
 *              闭环模式 (MOTOR_USE_ENCODER = 1)
 * ==================================================================== */

#if MOTOR_USE_ENCODER

#include "encoder.h"

/* ---------- 类型定义 ---------- */

typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    float out_max, out_min;
} PID_Controller;

typedef struct {
    PID_Controller pid;
    float target_speed;         /* mm/s */
    float current_speed;        /* mm/s */
    int16_t output;
    uint32_t test;              /* ISR 触发次数 */
} SpeedCtrl;

/* ---------- 控制定时器 ---------- */

#ifndef CONTROL_TIMER_INST
#define CONTROL_TIMER_INST      TIMG0
#define CONTROL_TIMER_INST_IRQHandler   TIMG0_IRQHandler
#define CONTROL_TIMER_INST_INT_IRQN     (TIMG0_INT_IRQn)
#endif

/* ---------- 全局变量 ---------- */

extern SpeedCtrl g_speed_left;
extern SpeedCtrl g_speed_right;
extern volatile uint8_t g_motor_control_flag;

/* ---------- 闭环 API ---------- */

void Motor_Control_Init(void);
void Motor_SetSpeed(float left_mm_s, float right_mm_s);
void Motor_Control_Loop(void);
//void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float out_max);
float PID_Calc_Motor(PID_Controller *pid, float setpoint, float measurement);
int32_t Motor_GetDistance(void);

#endif /* MOTOR_USE_ENCODER */

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H__ */

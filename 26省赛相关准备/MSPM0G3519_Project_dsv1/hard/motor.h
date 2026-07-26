/**
 * @file    motor.h
 * @brief   MSPM0G3519 双路直流电机驱动 + 速度闭环 — TB6612
 *
 *          硬件引脚:
 *            CIN1 — PA25 (左电机 IN1)    CIN2 — PA27 (左电机 IN2)
 *            DIN1 — PA22 (右电机 IN1)    DIN2 — PB24 (右电机 IN2)
 *            PWML — PB4  (TIMAx CCP)     PWMR — PB5  (TIMAx CCP)
 *
 *          架构: 级联 PID
 *            外环: 循迹 PID → 目标速度 (mm/s)
 *            内环: 速度 PID → PWM → TB6612
 */

#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include "encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ====================================================================
 *                        公共常量
 * ==================================================================== */

/** @brief PWM 最大值 (与 SysConfig TIMA period 一致) */
#define PWM_MAX                 1000

/* ====================================================================
 *                    GPIO 方向引脚宏 (TB6612)
 * ==================================================================== */

/* CIN1 — PA25 (左电机 IN1) */
#define CIN1_HIGH()             DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_25)
#define CIN1_LOW()              DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_25)

/* CIN2 — PA27 (左电机 IN2) */
#define CIN2_HIGH()             DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_27)
#define CIN2_LOW()              DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_27)

/* DIN1 — PA22 (右电机 IN1) */
#define DIN1_HIGH()             DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_22)
#define DIN1_LOW()              DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_22)

/* DIN2 — PB24 (右电机 IN2) */
#define DIN2_HIGH()             DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_24)
#define DIN2_LOW()              DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_24)

/* ====================================================================
 *                    PWM 占空比宏 (PB4/PB5)
 * ==================================================================== */

/** @brief 左轮 PWM — PB4 (TIMA CCP) */
#define PWM_L_SET(duty)         DL_TimerA_setCaptureCompareValue(PWM_0_INST, (duty), DL_TIMER_CC_0_INDEX)

/** @brief 右轮 PWM — PB5 (TIMA CCP) */
#define PWM_R_SET(duty)         DL_TimerA_setCaptureCompareValue(PWM_0_INST, (duty), DL_TIMER_CC_1_INDEX)

/* ====================================================================
 *                    速度闭环 PID 类型
 * ==================================================================== */

/** @brief PID 控制器 (含一阶低通滤波) */
typedef struct {
    float Kp, Ki, Kd;
    float integral;             /* 积分累加值 (限幅 ±1000)       */
    float prev_error;           /* 上一次误差                    */
    float last_output;          /* 上一次输出 (一阶低通滤波用)    */
    float out_max, out_min;     /* 输出限幅                      */
} PID_Controller;

/** @brief 单轮速度控制 */
typedef struct {
    PID_Controller pid;
    float    target_speed;      /* 目标速度 (mm/s)               */
    float    current_speed;     /* 实测速度 (mm/s)               */
    int16_t  output;            /* PID 输出 PWM 值               */
    int32_t  last_count;        /* 编码器计数值快照              */
} SpeedCtrl;

/* ====================================================================
 *                    控制定时器
 * ==================================================================== */

#ifndef CONTROL_TIMER_INST
#define CONTROL_TIMER_INST                  TIMG0
#define CONTROL_TIMER_INST_IRQHandler       TIMG0_IRQHandler
#define CONTROL_TIMER_INST_INT_IRQN         (TIMG0_INT_IRQn)
#endif

/* ====================================================================
 *                    全局变量
 * ==================================================================== */

extern SpeedCtrl g_speed_left;
extern SpeedCtrl g_speed_right;
extern volatile uint8_t g_motor_control_flag;
extern volatile uint8_t track_flag;        /* 1=循迹模式, 由 Track_Run 控制  */
extern volatile uint8_t turn_flag;         /* 1=转弯模式, 速度环使用硬转弯值 */

/* ====================================================================
 *                    公共 API
 * ==================================================================== */

/* ── 基础电机控制 ── */
void Motor_Init(void);
void Motor_Stop(void);
void Motor(int16_t left_speed, int16_t right_speed);

/* ── 速度闭环 ── */
void Motor_Control_Init(void);
void Motor_Control_Loop(void);
void Motor_SetSpeed(float left_mm_s, float right_mm_s);

/* ── PID ── */
void  PID_Init(PID_Controller *pid, float kp, float ki, float kd, float out_max);
float PID_Calc_Motor(PID_Controller *pid, float setpoint, float measurement);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H__ */

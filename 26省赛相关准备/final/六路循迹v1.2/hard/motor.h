/**
 * @file    motor.h
 * @brief   双路电机驱动 + 速度 PI
 */

#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>

#define PWM_MAX  1000

/* ── 方向引脚 ── */
#define CIN1_HIGH()  DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_25)
#define CIN1_LOW()   DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_25)
#define CIN2_HIGH()  DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_27)
#define CIN2_LOW()   DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_27)
#define DIN1_HIGH()  DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_22)
#define DIN1_LOW()   DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_22)
#define DIN2_HIGH()  DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_24)
#define DIN2_LOW()   DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_24)

#define PWM_L_SET(d) DL_TimerA_setCaptureCompareValue(PWM_0_INST, (d), DL_TIMER_CC_0_INDEX)
#define PWM_R_SET(d) DL_TimerA_setCaptureCompareValue(PWM_0_INST, (d), DL_TIMER_CC_1_INDEX)

#define CONTROL_TIMER_INST             TIMG0
#define CONTROL_TIMER_INST_INT_IRQN    (TIMG0_INT_IRQn)

/* ── PI 控制器 (定点 Q12: 实际值 = 存储值 / 4096) ── */
typedef struct {
    int32_t Kp, Ki;
    int32_t integral;
    int32_t Kd;
    int32_t prev_error;
    int32_t out_max;
} PI_t;

typedef struct {
    PI_t    pi;
    int32_t target;
    int32_t current;
    int16_t out;
} Motor_t;

extern Motor_t g_motor_l;
extern Motor_t g_motor_r;

/* ── API ── */
void    Motor_Init(void);
void    Motor_Stop(void);              /* 立即停止 (不经过缓启动) */
void    Motor(int16_t l, int16_t r);   /* 带缓启动的电机驱动 */
void    Motor_SetRamp(uint16_t step);  /* 设置缓启动步长 (默认 25) */
void    Motor_Loop(void);
void    Motor_SetTarget(int32_t l, int32_t r);
void    PI_Init(PI_t *pi, int32_t kp, int32_t ki, int32_t kd, int32_t omax);
int32_t PI_Calc(PI_t *pi, int32_t setpoint, int32_t measured);

#endif

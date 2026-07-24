#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include "tim.h"
#include "gpio.h"


/* ========== 硬件抽象层 ========== */
#define PWM_MAX 999
#define AIN1_H()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)
#define AIN1_L()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET)
#define AIN2_H()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)
#define AIN2_L()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)
#define BIN1_H()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define BIN1_L()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define BIN2_H()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)
#define BIN2_L()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)

#define MOTOR_PWM_TIM &htim2

#define PWM_L_SET(d)  __HAL_TIM_SET_COMPARE(MOTOR_PWM_TIM, TIM_CHANNEL_1, d)
#define PWM_R_SET(d)  __HAL_TIM_SET_COMPARE(MOTOR_PWM_TIM, TIM_CHANNEL_2, d)

#define LEFT_ENC_DIR   1
#define RIGHT_ENC_DIR  1

#define WHEEL_DIAMETER  65.0f
#define ENCODER_LINES   13
#define GEAR_RATIO      30
#define PULSE_PER_ROUND (ENCODER_LINES * GEAR_RATIO * 4)
#define MM_PER_PULSE    (3.1415926f * WHEEL_DIAMETER / PULSE_PER_ROUND)

#define CONTROL_PERIOD_S  0.01f   // 仅用于速度计算，PID不再依赖



typedef struct {
    float Kp, Ki, Kd;
    float integral;      // 积分累加值 (add)
    float prev_error;    // 上一次误差
    float last_output;   // 上一次PID输出值 (用于滤波)
    float out_max, out_min;
} PID_Controller;

typedef struct {
    float target_speed;   // mm/s
    float current_speed;
    int16_t output;       // PWM 值
    int32_t last_count;
    uint32_t test;
    PID_Controller pid;
} SpeedCtrl;

  

void Motor_Init(void);
void Motor_Stop(void);
void Motor_Forward(uint16_t pwm_l, uint16_t pwm_r);
void Motor_Backward(uint16_t pwm_l, uint16_t pwm_r);
void Motor(int16_t left_speed, int16_t right_speed);
void Motor_Set(int left_speed, int right_speed);

void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float out_max);
void Motor_SetSpeed(float left_mm_s, float right_mm_s);
void Motor_Control_Loop(void);
int32_t Motor_GetDistance(void);

#endif

#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include <stdint.h>

#define PWM_MAX 999

extern TIM_HandleTypeDef htim2;

// 原有 GPIO 控制宏
#define AIN1_H()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)
#define AIN1_L()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET)
#define AIN2_H()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)
#define AIN2_L()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)

#define BIN1_H()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define BIN1_L()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define BIN2_H()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)
#define BIN2_L()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)

#define PWM_L_SET(duty)  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty)
#define PWM_R_SET(duty)  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, duty)

// 初始化与停止
void Motor_Init(void);
void Motor_Stop(void);

// 统一控制接口：参数范围为 -PWM_MAX 到 PWM_MAX
// 正数 => 前进，负数 => 后退，零 => 停止（对应侧）
void Motor(int16_t left_speed, int16_t right_speed);

// 可选保留原有的简单函数（不强制使用）
void Motor_Forward(uint16_t pwm_l, uint16_t pwm_r);
void Motor_Backward(uint16_t pwm_l, uint16_t pwm_r);
void Motor_Turn_Left(uint16_t pwm);
void Motor_Turn_Right(uint16_t pwm);

#endif
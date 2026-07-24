#ifndef __SERVO_H
#define __SERVO_H

#include "tim.h"          // 包含 TIM 句柄声明
#include <stdint.h>

/* 舵机使用的定时器及通道 */
#define SERVO_PWM_TIM       &htim8
#define SERVO_PWM_CHANNEL1  TIM_CHANNEL_1
#define SERVO_PWM_CHANNEL2  TIM_CHANNEL_2
#define SERVO_PWM_CHANNEL3  TIM_CHANNEL_3
#define SERVO_PWM_CHANNEL4  TIM_CHANNEL_4



/**
 * @brief  初始化所有舵机通道
 * @note   启动 PWM 输出，并设置初始比较值
 */
void Servo_Init(void);

/**
 * @brief  直接设置舵机通道的比较值
 * @param  channel  通道宏（如 SERVO_PWM_CHANNEL1）
 * @param  pulse    比较值（典型范围 500 ~ 2500）
 */
void Servo_SetPulse(uint32_t channel, uint16_t pulse);

/**
 * @brief  设置舵机角度（0° ~ 180°）
 * @param  channel  通道宏
 * @param  angle    角度值（0.0 ~ 180.0）
 */
void Servo_SetAngle(uint32_t channel, float angle);

/**
 * @brief  设置四路舵机角度（0° ~ 180°）
 * @param  angle1,angle2,angle3,angle4    四个舵机角度值（0.0 ~ 180.0）
 */
void Servo_Control(float angle1,float angle2,float angle3,float angle4);

#endif /* __SERVO_H */

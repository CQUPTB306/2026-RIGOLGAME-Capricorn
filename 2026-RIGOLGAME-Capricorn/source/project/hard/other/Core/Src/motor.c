#include "motor.h"
#include <stdlib.h>   // for abs()

void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}

void Motor_Stop(void)
{
    AIN1_L(); AIN2_L();
    BIN1_L(); BIN2_L();
    PWM_L_SET(0);
    PWM_R_SET(0);
}

// 核心统一函数
void Motor(int16_t left_speed, int16_t right_speed)
{
    // 限幅处理
    if (left_speed > PWM_MAX) left_speed = PWM_MAX;
    if (left_speed < -PWM_MAX) left_speed = -PWM_MAX;
    if (right_speed > PWM_MAX) right_speed = PWM_MAX;
    if (right_speed < -PWM_MAX) right_speed = -PWM_MAX;

    // 处理左电机
    if (left_speed >= 0) {
        AIN1_H(); AIN2_L();   // 前进
        PWM_L_SET((uint16_t)left_speed);
    } else {
        AIN1_L(); AIN2_H();   // 后退
        PWM_L_SET((uint16_t)(-left_speed));
    }

    // 处理右电机
    if (right_speed >= 0) {
        BIN1_H(); BIN2_L();   // 前进
        PWM_R_SET((uint16_t)right_speed);
    } else {
        BIN1_L(); BIN2_H();   // 后退
        PWM_R_SET((uint16_t)(-right_speed));
    }
}

// 以下为原有函数，保留兼容性（可直接调用 Motor 实现）
void Motor_Forward(uint16_t pwm_l, uint16_t pwm_r)
{
    Motor((int16_t)pwm_l, (int16_t)pwm_r);
}

void Motor_Backward(uint16_t pwm_l, uint16_t pwm_r)
{
    Motor(-(int16_t)pwm_l, -(int16_t)pwm_r);
}

void Motor_Turn_Left(uint16_t pwm)
{
    // 左转：左轮慢，右轮快，均向前
    uint16_t pwm_left = pwm * 60 / 100;
    if (pwm_left > PWM_MAX) pwm_left = PWM_MAX;
    if (pwm > PWM_MAX) pwm = PWM_MAX;
    Motor((int16_t)pwm_left, (int16_t)pwm);
}

void Motor_Turn_Right(uint16_t pwm)
{
    uint16_t pwm_right = pwm * 60 / 100;
    if (pwm_right > PWM_MAX) pwm_right = PWM_MAX;
    if (pwm > PWM_MAX) pwm = PWM_MAX;
    Motor((int16_t)pwm, (int16_t)pwm_right);
}
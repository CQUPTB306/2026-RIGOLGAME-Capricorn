#include "servo.h"
#include "extern.h"


/**
 * @brief  初始化所有舵机通道
 */
void Servo_Init(void)
{
    /* 启动四路 PWM */
    HAL_TIM_PWM_Start(SERVO_PWM_TIM, SERVO_PWM_CHANNEL1);
    HAL_TIM_PWM_Start(SERVO_PWM_TIM, SERVO_PWM_CHANNEL2);
    HAL_TIM_PWM_Start(SERVO_PWM_TIM, SERVO_PWM_CHANNEL3);
    HAL_TIM_PWM_Start(SERVO_PWM_TIM, SERVO_PWM_CHANNEL4);

    /* 设置初始比较值（与原有代码一致） */
    __HAL_TIM_SET_COMPARE(SERVO_PWM_TIM, SERVO_PWM_CHANNEL1, 0);
    __HAL_TIM_SET_COMPARE(SERVO_PWM_TIM, SERVO_PWM_CHANNEL2, 0);
    __HAL_TIM_SET_COMPARE(SERVO_PWM_TIM, SERVO_PWM_CHANNEL3, 0);
    __HAL_TIM_SET_COMPARE(SERVO_PWM_TIM, SERVO_PWM_CHANNEL4, 0);
}

/**
 * @brief  直接设置舵机通道的比较值
 */
void Servo_SetPulse(uint32_t channel, uint16_t pulse)
{
    __HAL_TIM_SET_COMPARE(SERVO_PWM_TIM, channel, pulse);
}

/**
 * @brief  将角度（0° ~ 180°）映射为比较值并设置
 */
void Servo_SetAngle(uint32_t channel, float angle)
{
    uint16_t pulse;

    /* 限幅到 0° ~ 180° */
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;

    /* 线性映射：0° → SERVO_MIN_PULSE，180° → SERVO_MAX_PULSE */
    pulse = (uint16_t)(SERVO_MIN_PULSE + 
                       (angle / 180.0f) * (SERVO_MAX_PULSE - SERVO_MIN_PULSE));

    __HAL_TIM_SET_COMPARE(SERVO_PWM_TIM, channel, pulse);
}


/**
 * @brief  一次性控制四路舵机输出
 */
void Servo_Control(float angle1,float angle2,float angle3,float angle4)
{
    Servo_SetAngle(TIM_CHANNEL_1, angle1);//488,2557
    Servo_SetAngle(TIM_CHANNEL_2, angle2);
    Servo_SetAngle(TIM_CHANNEL_3, angle3);
    Servo_SetAngle(TIM_CHANNEL_4, angle4);
}


/**
 * @brief  带速度限制的舵机角度设置（阻塞式）
 * @param  channel      通道宏，如 TIM_CHANNEL_1
 * @param  target_angle 目标角度 0~180°
 * @param  deg_per_sec  角速度（度/秒），例如 30.0f
 * @note   调用后会一直阻塞，直到到达目标角度
 */
void Servo_SetAngle_Smooth(uint32_t channel, float target_angle, float deg_per_sec)
{
    // 限幅
    if (target_angle < 0.0f) target_angle = 0.0f;
    if (target_angle > 180.0f) target_angle = 180.0f;

    // 由通道宏确定数组索引（简单判断，和原来宏一样功能）
    uint8_t idx;
    if      (channel == TIM_CHANNEL_1) idx = 0;
    else if (channel == TIM_CHANNEL_2) idx = 1;
    else if (channel == TIM_CHANNEL_3) idx = 2;
    else                               idx = 3;

    float current = servo_angle[idx];   // 从全局变量取当前角度

    const float step = 0.5f;            // 每次移动 0.5°，越小越平滑
    uint32_t step_delay_ms = (uint32_t)((step / deg_per_sec) * 1000.0f);

    if (target_angle > current) 
    {
        while (current < target_angle) {
            current += step;
            if (current > target_angle) current = target_angle;
            Servo_SetAngle(channel, current);          // 直接调用原函数
            servo_angle[idx] = current;                // 更新全局记录
            HAL_Delay(step_delay_ms);
        }
    }
    else 
    {
        while (current > target_angle) {
            current -= step;
            if (current < target_angle) current = target_angle;
            Servo_SetAngle(channel, current);
            servo_angle[idx] = current;
            HAL_Delay(step_delay_ms);
        }
    }
}

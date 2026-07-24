 #include "encoder.h"
#include <stdlib.h>
#include "extern.h"
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "track.h"
#include "motor.h"

/* 编码器里程全局变量定义 */
int32_t g_encoder_distance = 0;

static int16_t s_last_left = 0;
static int16_t s_last_right = 0;

void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    g_encoder_distance = 0;
    s_last_left  = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    s_last_right = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
}


void Encoder_Update(void)
{
    /* 读取硬件计数器（不移位、不复位——避免干扰 Motor_Control_Loop） */
    int16_t left = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    int16_t right = -(int16_t)__HAL_TIM_GET_COUNTER(&htim4);

    /* 计算本次增量（int16_t 减法自动处理 16bit 溢出） */
    int16_t delta_l = left - s_last_left;
    int16_t delta_r = right - s_last_right;
    s_last_left  = left;
    s_last_right = right;

    /* 两轮同向时才计入里程（避免原地转向时乱跳） */
    int16_t avg = (delta_l + delta_r) / 2;
    if (g_car_dir == 0)
        g_encoder_distance += avg;
    else
        g_encoder_distance -= avg;
}
int32_t Encoder_GetDistance(void) { return g_encoder_distance; }

void Encoder_ResetDistance(void)
{
    g_encoder_distance = 0;
}

/* ================================================================
 * 定距循迹停车函数（阻塞式）
 * ================================================================
 * 用法：Run_To_Distance(30);   // 循迹开30cm停（向前）
 *       Run_To_Distance(-20);  // 倒车20cm停（向后）
 *
 * 校准方法：
 *   1. 用卷尺量好10cm实际地面距离
 *   2. 程序跑一下，看Encoder_GetDistance()的值
 *   3. CM_TO_RAW = 编码器值 / 10，改到下面宏里
 *
 * 速度调参（写死在此）：
 *   RUN_BASE_PWM   : 循迹时电机基速PWM（原400→280，七成速度）
 *   RUN_KP/_KI/_KD : 循迹PID
 * ================================================================ */
#define CM_TO_RAW        50    // 【校准】1厘米 = 76个编码器计数（来自params.h）

#define PWM_LIMIT  999         // PWM最大值

void Run_To_Distance(float target_cm)
{
    /* 厘米 → 编码器原始计数 */
    float target = target_cm * CM_TO_RAW;
    Encoder_ResetDistance();
    uint8_t dir = (target >= 0) ? 0 : 1;
    g_car_dir = dir;
		track_flag =1;
    while (1)
    {
        int32_t cur = Encoder_GetDistance();

        if ((dir == 0 && cur >= target) ||
            (dir == 1 && cur <= target))
						
            break;
        /* 循迹PID纠偏 */
				Track_Run ();
        Encoder_Update();   
    }
		Motor_Stop();
}

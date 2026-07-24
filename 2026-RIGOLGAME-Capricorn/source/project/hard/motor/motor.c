#include "motor.h"
#include "encoder.h"
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "extern.h"


SpeedCtrl g_speed_left, g_speed_right;



/* ---------- 初始化 ---------- */
void Motor_Init(void)
{
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_PWM_Start(MOTOR_PWM_TIM, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(MOTOR_PWM_TIM, TIM_CHANNEL_2);
    Motor_Stop();

    PID_Init(&g_speed_left.pid,  2.15f, 1.1f, 0.001f, PWM_MAX);
    PID_Init(&g_speed_right.pid, 2.15f, 1.1f, 0.001f, PWM_MAX);

    g_speed_left.last_count  = (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    g_speed_right.last_count = (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(&htim4);
}

/* ---------- 底层PWM控制 ---------- */
void Motor(int16_t left_speed0, int16_t right_speed0)
{
    int16_t left_speed = -left_speed0; 
    int16_t right_speed = -right_speed0;
    if (left_speed > PWM_MAX) left_speed = PWM_MAX;
    if (left_speed < -PWM_MAX) left_speed = -PWM_MAX;
    if (right_speed > PWM_MAX) right_speed = PWM_MAX;
    if (right_speed < -PWM_MAX) right_speed = -PWM_MAX;

    if (left_speed >= 0) {
        AIN1_H(); AIN2_L();
        PWM_L_SET((uint16_t)left_speed);
    } else {
        AIN1_L(); AIN2_H();
        PWM_L_SET((uint16_t)(-left_speed));
    }

    if (right_speed >= 0) {
        BIN1_H(); BIN2_L();
        PWM_R_SET((uint16_t)right_speed);
    } else {
        BIN1_L(); BIN2_H();
        PWM_R_SET((uint16_t)(-right_speed));
    }
}


/* ---------- PID 相关函数 ---------- */
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float out_max)
{
    pid->Kp = kp; 
    pid->Ki = ki; 
    pid->Kd = kd;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->last_output = 0;
    pid->out_max = out_max;
    pid->out_min = -out_max;
}

static float PID_Calculate(PID_Controller *pid, float setpoint, float measurement)
{
    float error = setpoint - measurement;          // 目标 - 实际
    float Pout = pid->Kp * error;

    // 积分累加 (不乘dt，与参考逻辑一致)
    pid->integral += error;
    // 积分限幅 ±1000
    if (pid->integral > 1000.0f) pid->integral = 1000.0f;
    if (pid->integral < -1000.0f) pid->integral = -1000.0f;
    float Iout = pid->Ki * pid->integral;

    // 微分 (不除dt)
    float derivative = error - pid->prev_error;
    float Dout = pid->Kd * derivative;

    // 原始PID输出
    float raw_output = Pout + Iout + Dout;

    // 一阶低通滤波: 0.08 * last_output + 0.92 * raw_output
    float filtered_output = 0.08f * pid->last_output + 0.92f * raw_output;

    // 输出限幅
    if (filtered_output > pid->out_max) filtered_output = pid->out_max;
    if (filtered_output < pid->out_min) filtered_output = pid->out_min;

    // 保存状态
    pid->prev_error = error;
    pid->last_output = filtered_output;

    return filtered_output;
}

/* ---------- 设定目标速度 ---------- */
void Motor_SetSpeed(float left_mm_s, float right_mm_s)
{
    g_speed_left.target_speed  = left_mm_s;
    g_speed_right.target_speed = right_mm_s;
}

/* ---------- 速度控制主循环（在定时器中断中调用） ---------- */
void Motor_Control_Loop(void)
{
    int16_t count_l = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    int16_t count_r = -(int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    int16_t delta_l = (count_l - (int16_t)g_speed_left.last_count) * LEFT_ENC_DIR;
    int16_t delta_r = (count_r - (int16_t)g_speed_right.last_count) * RIGHT_ENC_DIR;
    g_speed_left.last_count  = count_l;
    g_speed_right.last_count = count_r;

    float speed_l = (delta_l * MM_PER_PULSE) / CONTROL_PERIOD_S;
    float speed_r = (delta_r * MM_PER_PULSE) / CONTROL_PERIOD_S;
    g_speed_left.current_speed  = speed_l;
    g_speed_right.current_speed = speed_r;

    // 累加里程（仅当两轮同向时）
    if (speed_l > 0 && speed_r > 0)
        g_encoder_distance += (int32_t)((delta_l + delta_r) / 2 * MM_PER_PULSE);
    else if (speed_l < 0 && speed_r < 0)
        g_encoder_distance -= (int32_t)((delta_l + delta_r) / 2 * MM_PER_PULSE);

    // PID 计算
    float out_l = PID_Calculate(&g_speed_left.pid,  g_speed_left.target_speed,  speed_l);
    float out_r = PID_Calculate(&g_speed_right.pid, g_speed_right.target_speed, speed_r);
    g_speed_left.output  = (int16_t)out_l;
    g_speed_right.output = (int16_t)out_r;
		if(turn_flag == 1)
		{
			Motor(80,-98);
		}
		else if(track_flag ==1)
		{
			Motor(g_speed_left.output, g_speed_right.output);
		}
			
}

int32_t Motor_GetDistance(void) { return g_encoder_distance; }

/* ---------- 定时器中断回调 ---------- */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) 
    {
        Motor_Control_Loop();
       
    }
}

void Motor_Set(int left_speed, int right_speed)
{
		g_speed_left.target_speed=left_speed;
		g_speed_right.target_speed=right_speed;
}

void Motor_Stop(void) 
{
track_flag =0;
Motor_Set(0,0);
Motor(0,0);
}

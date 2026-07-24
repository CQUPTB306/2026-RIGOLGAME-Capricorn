#include "encoder.h"
#include <stdlib.h>

#define WHEEL_DIAMETER  65.0f
#define ENCODER_LINES   13
#define GEAR_RATIO      30
#define PULSE_PER_ROUND (ENCODER_LINES * GEAR_RATIO * 4)
#define MM_PER_PULSE    (3.1415926f * WHEEL_DIAMETER / PULSE_PER_ROUND)

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

static int32_t g_encoder_distance = 0;
static int32_t g_abs_distance = 0;  

void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    g_encoder_distance = 0;
    g_abs_distance = 0;
}

void Encoder_Update(void)
{
    int16_t left = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    int16_t right = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    // ���ݵ�ǰ��ʻ��������з�������
    int16_t avg = (left + right) / 2;
    extern uint8_t g_car_dir;  // 0:ǰ��,1:����
    if(g_car_dir == 0)
        g_encoder_distance += (int32_t)(avg * MM_PER_PULSE);
    else
        g_encoder_distance -= (int32_t)(avg * MM_PER_PULSE);

    g_abs_distance += (uint32_t)((abs(left) + abs(right)) * MM_PER_PULSE / 2);
}

int32_t Encoder_GetDistance(void) { return g_encoder_distance; }
void Encoder_ResetDistance(void) { g_encoder_distance = 0; g_abs_distance = 0; }
int32_t Encoder_GetAbsDistance(void) { return g_abs_distance; }
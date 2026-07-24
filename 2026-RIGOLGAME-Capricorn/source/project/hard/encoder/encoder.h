#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

#define WHEEL_DIAMETER  65.0f
#define ENCODER_LINES   13
#define GEAR_RATIO      30
#define PULSE_PER_ROUND (ENCODER_LINES * GEAR_RATIO * 4)
#define MM_PER_PULSE    (3.1415926f * WHEEL_DIAMETER / PULSE_PER_ROUND)

void Encoder_Init(void);
void Encoder_Update(void);
int32_t Encoder_GetDistance(void);
void Encoder_ResetDistance(void);

/* 定距循迹停车：循迹前进/后退指定厘米后停车（阻塞式）
 * 用法：Run_To_Distance(30);    // 向前30cm停车
 *       Run_To_Distance(-20);   // 向后20cm停车
 * 初始校准：改 encoder.c 中的 CM_TO_RAW 宏 */
void Run_To_Distance(float target_cm);

#endif

/**
 * @file    encoder.c
 * @brief   G3507 空壳 — 开环循迹不需要编码器
 */

#include "encoder.h"

void Encoder_Init(void) {}
void Encoder_GetSpeed(float *speed_l, float *speed_r)
{
    if (speed_l) *speed_l = 0.0f;
    if (speed_r) *speed_r = 0.0f;
}
int32_t Encoder_GetDistance(void) { return 0; }
uint32_t Encoder_GetCallCount(void) { return 0; }
void Encoder_Reset(void) {}

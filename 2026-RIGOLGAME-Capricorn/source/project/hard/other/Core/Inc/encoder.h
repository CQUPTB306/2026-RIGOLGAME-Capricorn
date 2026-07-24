#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

void Encoder_Init(void);
void Encoder_Update(void);
int32_t Encoder_GetDistance(void);
void Encoder_ResetDistance(void);
int32_t Encoder_GetAbsDistance(void); // 总行驶距离（正向累加）

#endif
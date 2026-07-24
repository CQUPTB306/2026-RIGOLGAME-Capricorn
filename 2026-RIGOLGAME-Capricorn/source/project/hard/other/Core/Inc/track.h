#ifndef __TRACK_H
#define __TRACK_H

#include "main.h"

void Track_Sensor_Read(void);
void Track_PID_Calc(float kp, float ki, float kd);
int8_t Get_Track_Error(void);
float Get_Track_PID_Out(void);
void Track_Reset_PID(void);

#endif
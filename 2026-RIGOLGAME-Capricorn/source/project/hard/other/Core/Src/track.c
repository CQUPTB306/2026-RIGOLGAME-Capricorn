#include "track.h"

static uint8_t g_track_raw = 0;
static int8_t  g_track_error = 0;
static float   g_track_pid_out = 0;
static int8_t  g_track_error_last = 0;
static float   g_track_integral = 0;

#define TRACK_READ()  (\
  (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) << 0) |\
  (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) << 1) |\
  (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) << 2) |\
  (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) << 3) |\
  (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)  << 4) |\
  (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)  << 5) |\
  (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6)  << 6) |\
  (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)  << 7) )

void Track_Sensor_Read(void)
{
  g_track_raw = TRACK_READ();
  switch(g_track_raw)
  {
        case 0x01: g_track_error = -7; break;
        case 0x02: g_track_error = -5; break;
        case 0x04: g_track_error = -3; break;
        case 0x08: g_track_error = -1; break;
        case 0x10: g_track_error = 1;  break;
        case 0x20: g_track_error = 3;  break;
        case 0x40: g_track_error = 5;  break;
        case 0x80: g_track_error = 7;  break;
        default:
            g_track_error = (g_track_error < 0) ? -2 : 2;
            break;
  }
}

void Track_PID_Calc(float kp, float ki, float kd)
{
    int e = g_track_error;
    float p = kp * e;
    g_track_integral += e;
    if(g_track_integral > 1000)  g_track_integral = 1000;
    if(g_track_integral < -1000) g_track_integral = -1000;
    float i = ki * g_track_integral;
    float d = kd * (e - g_track_error_last);
    g_track_pid_out = p + i + d;
    g_track_error_last = e;
}

int8_t Get_Track_Error(void) { return g_track_error; }
float Get_Track_PID_Out(void) { return g_track_pid_out; }
void Track_Reset_PID(void) { g_track_integral = 0; g_track_error_last = 0; }
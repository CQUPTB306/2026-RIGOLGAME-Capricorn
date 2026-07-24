#ifndef EXTERN_H
#define EXTERN_H

#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "soft_i2c.h"
#include "oled.h"
#include "soft_i2c_track.h"
#include "track.h"
#include <string.h>
#include "color_uart.h"
#include "mpu_soft_i2c.h"
#include "mpu6050.h"
#include "stdio.h"
#include "motor.h"
#include "encoder.h"
#include "BlueSerial.h"
#include "servo.h"

/*========== 陀螺仪相关变量 =============*/
extern float yaw;
extern float pitch;
extern float roll;

/*========= 循迹相关 ===========*/
extern PID_TypeDef PID;
extern PID_TypeDef PID_MV;
extern int IR_Weight[8];
extern uint8_t BASE_SPEED;
extern float ir;

/*======= 舵机相关 ========*/
/* 脉冲宽度范围（比较值），对应 0° ~ 180° */
extern int SERVO_MIN_PULSE;
extern int SERVO_MAX_PULSE;
extern uint8_t ServoAngle1;
extern uint8_t ServoAngle2;
extern uint8_t ServoAngle3;
extern uint8_t ServoAngle4;
extern float servo_angle[4];
extern float deg_per_sec[4];
/*========== 电机，编码器相关 =============*/
extern SpeedCtrl g_speed_left, g_speed_right;
extern int32_t g_encoder_distance;
extern uint8_t g_car_dir;
extern int32_t start_distance;
/*========== OpenMV相关 ==========*/
extern ColorFrame_t g_color_frame;

extern int temp_count;

extern uint8_t track_flag;

extern uint8_t turn_flag;

extern uint32_t distance;


#endif /* EXTERN_H */

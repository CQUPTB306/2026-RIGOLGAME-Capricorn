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
#include "extern.h"

/*==========陀螺仪相关变量=============*/
float yaw = 0;
float pitch = 0;
float roll = 0;

/*==================== 循迹相关 ====================*/
int IR_Weight[8] = {0, 31, 18, 2, -2, -18, -31, 0};//循迹权重（从左到右 8 路）
PID_TypeDef PID = {2.0, 0, 1.9, 0, 0, 0, 0, 0, 0};
PID_TypeDef PID_MV={2.0, 0, 1.9, 0, 0, 0, 0, 0, 0};
uint8_t BASE_SPEED=80;
float ir=80;

int32_t start_distance=0;
/*============舵机相关=============*/
int SERVO_MIN_PULSE=488;
int SERVO_MAX_PULSE=2557;
uint8_t ServoAngle1=80;
uint8_t ServoAngle2=80;
uint8_t ServoAngle3=80;
uint8_t ServoAngle4=80;
float servo_angle[4] = {0.0f};   // 分别记录四通道的当前角度（0°初始）
float deg_per_sec[4] = {40.0f};   //四个舵机的速度，初始为40
/*============= OpenMV相关 ============*/
ColorFrame_t g_color_frame = {0};


uint8_t track_flag=0;

uint8_t turn_flag=0;


int temp_count=0;

uint32_t distance=0;

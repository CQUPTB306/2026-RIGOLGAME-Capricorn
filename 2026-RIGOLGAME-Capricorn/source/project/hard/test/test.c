#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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
#include "test.h"

void Init_ALL(void)
{
			OLED_Init(); 
			Track_Init();
			BlueSerial_Init();
			MPU6050_Init();
			MPU6050_Calibrate();
			MPU6050_ResetYaw(); 
			MPU6050_SetGyroOffset(0,0,8);
			Color_UART_Init();
			Motor_Init();
			Encoder_Init();
			Servo_Init ();
}

/**
 * @brief  MPU6050采集到的yaw,pitch,roll显示到OLED上
 */
void MPU6050_OLED(void)
{
	MPU6050_UpdateAngles();
	yaw=MPU6050_GetYaw() ;
	pitch = MPU6050_GetPitch();
	roll = MPU6050_GetRoll();
	char buf[20];
	sprintf(buf, "Yaw%.1f deg", yaw);
    OLED_ShowString(1, 1, buf);
	sprintf(buf, "P%.1f", pitch);
	OLED_ShowString(2, 1, buf);
	sprintf(buf, "Roll %.1f", roll);
	OLED_ShowString(3, 1, buf);
}

/**
 * @brief  I2C八路循迹采集到情况，其中转化为8位2进制显示到OLED上
 */
void Track_OLED(void)
{
     uint8_t track=Track_Read_All();
    OLED_ShowBinNum(2,1,track,8);
}

/**
 * @brief  接收OpenMV发送过来的东西，一个使用示例
 */
void Color_uart(void)
{
	if (Color_IsFrameComplete())
	{
    	if (Color_IsDetected(0))
		{
    		int16_t x, y;
			Color_GetCoordinates(0, &x, &y);
			OLED_ShowNum(2,1,0,1);
			OLED_ShowNum(3,1,x,3);
			OLED_ShowNum(4,1,y,3);
	    }
		if (Color_IsDetected(3))
		{
			int16_t x, y;
			Color_GetCoordinates(3, &x, &y);
		    OLED_ShowNum(2,1,1,1);
			OLED_ShowNum(3,1,x,3);
			OLED_ShowNum(4,1,y,3);						
        }
		Color_ClearFrame();
	}
}
/**
 * @brief  循迹调参数时，配合蓝牙调节PID，权重
 */
void tiaocan_OLED(void)
{
    	OLED_ShowFloatNum(1,1,PID.Kp,1,1);
		OLED_ShowFloatNum(1,6,PID.Kd,1,1);
		OLED_ShowNum(3,4,IR_Weight[0],2);
		OLED_ShowNum(3,7,IR_Weight[1],2);
		OLED_ShowNum(3,10,IR_Weight[2],2);
		OLED_ShowNum(3,13,IR_Weight[3],2);
		OLED_ShowFloatNum(4,1,ir,1,2);
}
/**
 * @brief  采集编码器数值，显示在OLED上
 */
void Encoder_OLED(void)
{
	Encoder_Update();

	OLED_ShowSignedNum(1,1,g_encoder_distance,5);
	int xx=-__HAL_TIM_GET_COUNTER(&htim4);
	OLED_ShowSignedNum(2,1, xx,5);
	int yy=__HAL_TIM_GET_COUNTER(&htim3);
	OLED_ShowSignedNum(3,1, yy,5);
	//if(Key_Scan()==1||Key_Scan()==2||Key_Scan()==3||Key_Scan()==4)Encoder_ResetDistance();
}


void Run_distance(int distance)
{
    g_encoder_distance = 0;

    while (1)
    {
        Track_Run();
        HAL_Delay(10);
        if (g_encoder_distance>= distance)
        {
            Motor_Set(0,0);   // 立即停车
            break;
        }
    }
}

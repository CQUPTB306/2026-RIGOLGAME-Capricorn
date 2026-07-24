/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ==================== 电机控制 ====================
#define AIN1_H()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)
#define AIN1_L()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET)
#define AIN2_H()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)
#define AIN2_L()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)

#define BIN1_H()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define BIN1_L()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define BIN2_H()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)
#define BIN2_L()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)

// PWM控制
#define PWM_L_SET(duty)  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, duty)
#define PWM_R_SET(duty)  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, duty)

#define PWM_MAX    999
#define PWM_BASE   400
#define PWM_BACK   350
#define PWM_TURN   300

// ==================== 8路循迹  ====================
#define TRACK_READ()  (\
  (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) << 0) |\
  (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) << 1) |\
  (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) << 2) |\
  (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) << 3) |\
  (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)  << 4) |\
  (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)  << 5) |\
  (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6)  << 6) |\
  (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)  << 7) )

// ==================== 边框检测 ====================
#define BORDER_READ()  ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) << 0) | (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) << 1))
#define BORDER_TRIGGER 0x03

// ==================== 编码器 ====================
#define WHEEL_DIAMETER  65
#define ENCODER_LINES   13
#define GEAR_RATIO      30
#define PULSE_PER_ROUND (ENCODER_LINES * GEAR_RATIO * 4)
#define MM_PER_PULSE    (3.1415926 * WHEEL_DIAMETER / PULSE_PER_ROUND)

// ==================== 按键 ====================
#define KEY1_READ()  HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)
#define KEY2_READ()  HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1)
#define KEY3_READ()  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)
#define KEY_PRESSED  1

// ==================== 循迹PID参数 ====================
#define TRACK_KP  1.2
#define TRACK_KI  0.01
#define TRACK_KD  0.3

// ==================== 任务模式定义 ====================
typedef enum {
    TASK_MODE_FIXED = 0,
    TASK_MODE_CUSTOM
} Task_Mode;

// ==================== 节点/点位定义 ====================
typedef enum {
    NODE_HOME = 0,
    NODE_WAREHOUSE,
    NODE_A,
    NODE_B,
    NODE_C,
    NODE_TRANSITION,
    NODE_MAX
} Node_e;

// ==================== 车辆方向状态 ====================
typedef enum {
    DIR_FORWARD = 0,
    DIR_BACKWARD = 1
} Car_Dir;

// ==================== 车辆状态 ====================
typedef enum {
    STATE_IDLE = 0,
    STATE_START,
    STATE_GO_WAREHOUSE,
    STATE_ALIGN_CENTER,
    STATE_LOAD_ALL_GOODS,
    STATE_GO_UNLOAD_A,
    STATE_UNLOAD_A,
    STATE_GO_UNLOAD_B,
    STATE_UNLOAD_B,
    STATE_GO_UNLOAD_C,
    STATE_UNLOAD_C,
    STATE_RETURN_HOME,
    STATE_FINISH
} Car_State;

// ==================== 货物类型 ====================
typedef enum {
    GOODS_A = 0,
    GOODS_B = 1,
    GOODS_C = 2
} Goods_Type;

// ==================== 机械臂参数 ====================
#define BASE_H 30.0
#define L1 100.0
#define L2 50.0
#define J0_MIN  -90.0
#define J0_MAX   90.0
#define J1_MIN    0.0
#define J1_MAX   90.0
#define J2_MIN  -90.0
#define J2_MAX    0.0
#define RAD_TO_DEG(r) (r / 3.1415926 * 180.0)
#define DEG_TO_RAD(d) (d / 180.0 * 3.1415926)

#define MAGNET_ON()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET)
#define MAGNET_OFF() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET)

// ==================== 机械臂目标位置 ====================
#define ARM_RESET     {0, 45, -80}
#define ARM_ALIGN     {90, 0, -80}
#define ARM_PICK_A 	  {90, 30, -30}
#define ARM_PICK_B 	  {90, 30, -30}
#define ARM_PICK_C    {90, 30, -30}
#define ARM_MOVE      {90, 30, -30}
#define ARM_DROP_A    {90, 80, -30}
#define ARM_DROP_B    {-90, 30, -30}
#define ARM_DROP_C    {-90, 30, -30}
const float arm_point[][3] = {ARM_RESET, ARM_ALIGN, ARM_PICK_A, ARM_PICK_B, ARM_PICK_C, ARM_MOVE, ARM_DROP_A, ARM_DROP_B, ARM_DROP_C};
typedef enum
{
    POINT_RESET=0,
    POINT_ALIGN,
    POINT_PICK_A,
    POINT_PICK_B,
    POINT_PICK_C,
    POINT_MOVE,
    POINT_DROP_A,
    POINT_DROP_B,
    POINT_DROP_C
} Arm_Point_e;

// ====================串口通信缓存规模 ====================
#define UART_BUF_SIZE 50

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// ==================== 小车状态 ====================
Car_State g_car_state = STATE_IDLE;
Task_Mode g_task_mode = TASK_MODE_FIXED;
Car_Dir   g_car_dir = DIR_FORWARD;

Goods_Type g_unload_order[3] = {GOODS_A, GOODS_B, GOODS_C};
uint8_t g_unload_index = 0;
uint8_t g_goods_loaded = 0;
uint8_t g_current_node = NODE_HOME;
uint8_t g_target_node = 0;

uint8_t g_goods_index = 0;

// ==================== 机械臂参数 ====================
uint8_t g_arm_done_flag = 0;
float target_X = 150.0, target_Y = 0.0, target_Z = 100.0;
float delta_X = 0.0, delta_Y = 0.0;
uint8_t line_flag = 0, align_flag = 0;
float center_offset = 0;

// ==================== 串口 ====================
uint8_t uart_error_flag = 0;
uint8_t uart_rx_buf[UART_BUF_SIZE], uart_rx_data, uart_rx_cnt = 0;

// ==================== PID ====================
uint8_t g_track_raw = 0;
int8_t  g_track_error = 0;
float   g_track_pid_out = 0;
int8_t  g_track_error_last = 0;
float   g_track_integral = 0;

int32_t g_total_distance = 0;
int16_t g_encoder_left = 0;
int16_t g_encoder_right = 0;
int32_t g_distance = 0;

// ==================== 按键 ====================
uint8_t g_key_val = 0;
uint8_t g_order_set_step = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// 电机控制
void Motor_Stop(void);
void Motor_Forward(uint16_t pwm_l, uint16_t pwm_r);
void Motor_Backward(uint16_t pwm_l, uint16_t pwm_r);
void Motor_Turn_Left(uint16_t pwm);
void Motor_Turn_Right(uint16_t pwm);

// 传感器
void Track_Sensor_Read(void);
void Encoder_Distance_Calc(void);
uint8_t Border_Check(void);
uint8_t Key_Scan(void);

// PID
void Track_PID_Calc(void);

// 状态机
uint8_t Path_Plan(uint8_t current, uint8_t target);
void Car_State_Machine(void);

// 机械臂
void PCA9685_Init(void);
void PCA9685_Write(uint8_t r, uint8_t d);
void Servo_SetAngle(uint8_t ch, float angle);
void Arm_MovePoint(Arm_Point_e point_num);
void Arm_Pick_A(void);
void Arm_Pick_B(void);
void Arm_Pick_C(void);
void Arm_Drop_A(void);
void Arm_Drop_B(void);
void Arm_Drop_C(void);

// 视觉
void OpenMV_SetMode(uint8_t mode);
void Car_Align_Center(void);
void Block_Align(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, "System Ready!");

    Motor_Stop();
    PCA9685_Init();
    MAGNET_OFF();

    HAL_UART_Receive_IT(&huart1, &uart_rx_data, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		// OLED_ShowString(0,1,"666");
		// PWM_L_SET(500);
		// PWM_R_SET(500);
		// HAL_Delay(1000);

	
		// PWM_L_SET(0);
		// PWM_R_SET(0);
		// HAL_Delay(1000);
		//  OLED_ShowString(0,1,"Angle: 0°   ");
        
        // HAL_I2C_Mem_Write(&hi2c1, 0x80, 0x06, 1, (uint8_t[]){0,0, 127, 1}, 4, 100);
        // HAL_Delay(1000);

        // OLED_ShowString(0,1,"Angle: 90°  ");
        
        // HAL_I2C_Mem_Write(&hi2c1, 0x80, 0x06, 1, (uint8_t[]){0,0, 204, 3}, 4, 100);
        // HAL_Delay(1000);

        // OLED_ShowString(0,1,"Angle: 180° ");
        
        // HAL_I2C_Mem_Write(&hi2c1, 0x80, 0x06, 1, (uint8_t[]){0,0, 255, 7}, 4, 100);
        // HAL_Delay(1000);

	//  OLED_ShowString(0,1,"FMAGNET ON");
	//  MAGNET_ON();
	//  HAL_Delay(5000);

	//  OLED_ShowString(0,1,"FMAGNET OFF");
	//  MAGNET_OFF();
	//  HAL_Delay(5000);


		// Motor_Forward(300, 300);   // 前进
		// HAL_Delay(2000);

		// OLED_ShowString(0,1,"Stop        ");
		// Motor_Stop();            
		// HAL_Delay(1000);

		// OLED_ShowString(0,1,"Backward    ");
		// Motor_Backward(300, 300); 
		// HAL_Delay(2000);

		// OLED_ShowString(0,1,"Stop        ");
		// Motor_Stop();
		// HAL_Delay(1000);

		// OLED_ShowString(0,1,"Left        ");
		// Motor_Turn_Left(250);     
		// HAL_Delay(1000);

		// OLED_ShowString(0,1,"Stop        ");
		// Motor_Stop();
		// HAL_Delay(1000);

		// OLED_ShowString(0,1,"Right  ");
		// Motor_Turn_Right(250);    
		// HAL_Delay(1000);

		// OLED_ShowString(0,1,"Stop        ");
		// Motor_Stop();
		// HAL_Delay(1000);
  	}

    // Car_State_Machine();
    HAL_Delay(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// ====================== 电机控制 ======================
void Motor_Stop(void)
{
    AIN1_L(); AIN2_L();
    BIN1_L(); BIN2_L();
    PWM_L_SET(0);
    PWM_R_SET(0);
}

void Motor_Forward(uint16_t pwm_l, uint16_t pwm_r)
{
    if(pwm_l > PWM_MAX) pwm_l = PWM_MAX;
    if(pwm_l < 0) pwm_l = 0;
    if(pwm_r > PWM_MAX) pwm_r = PWM_MAX;
    if(pwm_r < 0) pwm_r = 0;

    AIN1_H(); AIN2_L();
    BIN1_H(); BIN2_L();

    PWM_L_SET(pwm_l);
    PWM_R_SET(pwm_r);
    g_car_dir = DIR_FORWARD;
}

void Motor_Backward(uint16_t pwm_l, uint16_t pwm_r)
{
    if(pwm_l>PWM_MAX) pwm_l=PWM_MAX;
    if(pwm_l<0) pwm_l=0;
    if(pwm_r>PWM_MAX) pwm_r=PWM_MAX;
    if(pwm_r<0) pwm_r=0;

    AIN1_L(); AIN2_H();
    BIN1_L(); BIN2_H();
    PWM_L_SET(pwm_l);
    PWM_R_SET(pwm_r);
    g_car_dir=DIR_BACKWARD;
}

void Motor_Turn_Left(uint16_t pwm)
{
    if(pwm > PWM_MAX) pwm = PWM_MAX;
    if(pwm < 0) pwm = 0;

    uint16_t pwm_left = pwm * 60 / 100;
    uint16_t pwm_right = pwm;

    AIN1_H(); AIN2_L();
    BIN1_H(); BIN2_L();

    PWM_L_SET(pwm_left);
    PWM_R_SET(pwm_right);
}

void Motor_Turn_Right(uint16_t pwm)
{
    if(pwm > PWM_MAX) pwm = PWM_MAX;
    if(pwm < 0) pwm = 0;

    uint16_t pwm_left = pwm;
    uint16_t pwm_right = pwm * 60 / 100;

    AIN1_H(); AIN2_L();
    BIN1_H(); BIN2_L();

    PWM_L_SET(pwm_left);
    PWM_R_SET(pwm_right);
}

// ====================== 8路循迹 ======================
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

// ====================== 编码器距离计算 ======================
void Encoder_Distance_Calc(void)
{
    g_encoder_left = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    g_encoder_right = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);

    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    g_distance += (uint32_t)(abs(g_encoder_left) + abs(g_encoder_right)) * MM_PER_PULSE / 2;
}

// ====================== 边框检测 ======================
uint8_t Border_Check(void)
{
    if(BORDER_READ() == BORDER_TRIGGER)
        return 1;
    else
        return 0;
}

// ====================== 按键扫描 ======================
uint8_t Key_Scan(void)
{
    static uint8_t key_up = 1;

    if(key_up && (KEY1_READ() == KEY_PRESSED || KEY2_READ() == KEY_PRESSED || KEY3_READ() == KEY_PRESSED))
    {
        HAL_Delay(10);
        key_up = 0;
        if(KEY1_READ() == KEY_PRESSED)     return 1;
        else if(KEY2_READ() == KEY_PRESSED) return 2;
        else if(KEY3_READ() == KEY_PRESSED) return 3;
    }
    else if(KEY1_READ() != KEY_PRESSED && KEY2_READ() != KEY_PRESSED && KEY3_READ() != KEY_PRESSED)
    {
        key_up = 1;
    }
    return 0;
}

// ====================== 循迹PID计算 ======================
void Track_PID_Calc(void)
{
    int e = g_track_error;
    float p = TRACK_KP * e;
    g_track_integral += e;

    if(g_track_integral > 1000)  g_track_integral = 1000;
    if(g_track_integral < -1000) g_track_integral = -1000;

    float i = TRACK_KI * g_track_integral;
    float d = TRACK_KD * (e - g_track_error_last);
    g_track_pid_out = p + i + d;
    g_track_error_last = e;
}

// ====================== 路径规划 ======================
uint8_t Path_Plan(uint8_t cur,uint8_t tar)
{
    if(g_task_mode == TASK_MODE_FIXED)
        return DIR_FORWARD;

    uint8_t f = (tar - cur + NODE_MAX) % NODE_MAX;
    uint8_t b = (cur - tar + NODE_MAX) % NODE_MAX;
    return (f <= b) ? DIR_FORWARD : DIR_BACKWARD;
}

// ==================== 中心对准 ====================
void Car_Align_Center(void)
{
    uint8_t ok = 0;
    uint32_t t = 0;
    OpenMV_SetMode(3);
    Arm_MovePoint(POINT_ALIGN);

    while(!ok && t < 300)
    {
        if(line_flag)
        {
            line_flag = 0;
            t = 0;
            if(fabsf(center_offset) < 0.3)
            {
                ok = 1;
                break;
            }
            if(center_offset > 0.5)
            {
                Motor_Backward(PWM_BACK/6, PWM_BACK/6);
                HAL_Delay(20);
                Motor_Stop();
            }
            if(center_offset < -0.5)
            {
                Motor_Forward(PWM_BASE/6, PWM_BASE/6);
                HAL_Delay(20);
                Motor_Stop();
            }
        }
        else
        {
            t++;
        }
        HAL_Delay(10);
    }
    Arm_MovePoint(POINT_RESET);
    Motor_Stop();
}

// ====================== 视觉模式 ======================
void OpenMV_SetMode(uint8_t m)
{
    char buf[16];
    sprintf(buf,"M,%d\r\n",m);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);
    HAL_Delay(50);
}

// ==================== 机械臂 ====================
void PCA9685_Init(void)
{
    HAL_Delay(100);
    PCA9685_Write(0x00, 0x10);
    PCA9685_Write(0xFE, 121);
    PCA9685_Write(0x00, 0x20);
}

void PCA9685_Write(uint8_t r, uint8_t d)
{
    HAL_I2C_Mem_Write(&hi2c1, 0x80, r, 1, &d, 1, 100);
}

void Servo_SetAngle(uint8_t ch, float a)
{
    if(a > 180) a = 180;
    if(a < 0)   a = 0;

    uint16_t p = 500 + (a / 180.0f) * 2000;
    uint16_t off = (p * 4096) / 20000;
    uint8_t da[4] = {0, 0, off & 0xFF, off >> 8};
    HAL_I2C_Mem_Write(&hi2c1, 0x80, 0x06 + 4*ch, 1, da, 4, 100);
}

void Arm_MovePoint(Arm_Point_e p)
{
    float t0 = arm_point[p][0], t1 = arm_point[p][1], t2 = arm_point[p][2];
    static float n0 = 0, n1 = 45, n2 = -80;

    for(int i=0; i<=50; i++)
    {
        float o0 = n0 + (t0 - n0)/50.0f * i;
        float o1 = n1 + (t1 - n1)/50.0f * i;
        float o2 = n2 + (t2 - n2)/50.0f * i;

        Servo_SetAngle(0, o0 + 90);
        Servo_SetAngle(1, 90 - o1);
        Servo_SetAngle(2, -o2);
        HAL_Delay(10);
    }
    n0 = t0; n1 = t1; n2 = t2;
    HAL_Delay(300);
}

void Arm_Pick_A(void)
{
    Arm_MovePoint(POINT_PICK_A);
    MAGNET_ON();
    HAL_Delay(200);
    Arm_MovePoint(POINT_MOVE);
    g_arm_done_flag = 1;
}

void Arm_Pick_B(void)
{
    Arm_MovePoint(POINT_PICK_B);
    MAGNET_ON();
    HAL_Delay(200);
    Arm_MovePoint(POINT_MOVE);
    g_arm_done_flag = 1;
}

void Arm_Pick_C(void)
{
    Arm_MovePoint(POINT_PICK_C);
    MAGNET_ON();
    HAL_Delay(200);
    Arm_MovePoint(POINT_MOVE);
    g_arm_done_flag = 1;
}

void Arm_Drop_A(void)
{
    Arm_MovePoint(POINT_MOVE);
    MAGNET_ON();
    HAL_Delay(300);

    Arm_MovePoint(POINT_DROP_A);
    HAL_Delay(200);
    MAGNET_OFF();
    HAL_Delay(150);

    for(int k=0;k<2;k++){
        MAGNET_ON(); HAL_Delay(10);
        MAGNET_OFF(); HAL_Delay(30);
    }

    Arm_MovePoint(POINT_RESET);
    g_arm_done_flag = 1;
}

void Arm_Drop_B(void)
{
    Arm_MovePoint(POINT_MOVE);
    MAGNET_ON();
    HAL_Delay(300);

    Arm_MovePoint(POINT_DROP_B);
    HAL_Delay(200);
    MAGNET_OFF();
    HAL_Delay(150);

    for(int k=0;k<2;k++){
        MAGNET_ON(); HAL_Delay(10);
        MAGNET_OFF(); HAL_Delay(30);
    }

    Arm_MovePoint(POINT_RESET);
    g_arm_done_flag = 1;
}

void Arm_Drop_C(void)
{
    Arm_MovePoint(POINT_MOVE);
    MAGNET_ON();
    HAL_Delay(300);

    Arm_MovePoint(POINT_DROP_C);
    HAL_Delay(200);
    MAGNET_OFF();
    HAL_Delay(150);

    for(int k=0;k<2;k++){
        MAGNET_ON(); HAL_Delay(10);
        MAGNET_OFF(); HAL_Delay(30);
    }

    Arm_MovePoint(POINT_RESET);
    g_arm_done_flag = 1;
}

// ====================== 状态机 ======================
void Car_State_Machine(void)
{
    g_key_val = Key_Scan();
    uint16_t l, r, dir;

    switch(g_car_state)
    {
        case STATE_IDLE:
            Motor_Stop();
            OLED_ShowString(0,0,g_task_mode==TASK_MODE_FIXED ? "Mode1:Fixed" : "Mode2:Custom");
            OLED_ShowString(0,1,"K1:Start K3:Mode");

            if(g_key_val == 1)
            {
                g_car_state = STATE_START;
                OLED_Clear();
            }
            if(g_key_val == 3)
            {
                g_task_mode = !g_task_mode;
                OLED_Clear();
                HAL_Delay(200);
            }
            break;

        case STATE_START:
            OLED_ShowString(0,0,"Task Start");
            HAL_Delay(500);
            g_goods_index = 0;
            g_unload_index = 0;
            g_target_node = NODE_WAREHOUSE;
            g_car_state = STATE_GO_WAREHOUSE;
            break;

        case STATE_GO_WAREHOUSE:
            dir = Path_Plan(g_current_node, g_target_node);
            Track_Sensor_Read();
            Track_PID_Calc();
            Encoder_Distance_Calc();

            l = (dir == DIR_FORWARD ? PWM_BASE : PWM_BACK) - g_track_pid_out;
            r = (dir == DIR_FORWARD ? PWM_BASE : PWM_BACK) + g_track_pid_out;

            dir == DIR_FORWARD ? Motor_Forward(l, r) : Motor_Backward(l, r);
            OLED_ShowString(0,0,"Go Warehouse");

            if(Border_Check())
            {
                Motor_Stop();
                g_car_state = STATE_ALIGN_CENTER;
                OLED_Clear();
            }
            break;

        case STATE_ALIGN_CENTER:
            OLED_ShowString(0,0,"Vision Align");
            Car_Align_Center();
            g_current_node = NODE_WAREHOUSE;
            g_car_state = STATE_LOAD_ALL_GOODS;
            OLED_Clear();
            break;

        case STATE_LOAD_ALL_GOODS:
            OLED_ShowString(0,0,"Loading Goods");
            OLED_ShowNum(0,1,g_goods_index+1,1);

            switch(g_goods_index)
            {
                case 0: Arm_Pick_A(); break;
                case 1: Arm_Pick_B(); break;
                case 2: Arm_Pick_C(); break;
            }

            if(g_arm_done_flag)
            {
                g_arm_done_flag = 0;
                g_goods_index++;
                HAL_Delay(500);

                if(g_goods_index >= 3)
                {
                    g_goods_index = 0;
                    g_car_state = STATE_GO_UNLOAD_A;
                    OLED_Clear();
                }
            }
            break;

        case STATE_GO_UNLOAD_A:
            g_target_node = NODE_A;
            dir = Path_Plan(g_current_node, g_target_node);
            Track_Sensor_Read();
            Track_PID_Calc();
            Encoder_Distance_Calc();

            l = (dir == DIR_FORWARD ? PWM_BASE : PWM_BACK) - g_track_pid_out;
            r = (dir == DIR_FORWARD ? PWM_BASE : PWM_BACK) + g_track_pid_out;

            dir == DIR_FORWARD ? Motor_Forward(l, r) : Motor_Backward(l, r);
            OLED_ShowString(0,0,"Go Unload A");

            if(Border_Check())
            {
                Motor_Stop();
                g_current_node = NODE_A;
                g_car_state = STATE_UNLOAD_A;
            }
            break;

        case STATE_UNLOAD_A:
            OLED_ShowString(0,0,"Unload A");
            Arm_Drop_A();
            if(g_arm_done_flag)
            {
                g_arm_done_flag = 0;
                g_car_state = STATE_GO_UNLOAD_B;
                OLED_Clear();
            }
            break;

        case STATE_GO_UNLOAD_B:
            g_target_node = NODE_B;
            dir = Path_Plan(g_current_node, g_target_node);
            Track_Sensor_Read();
            Track_PID_Calc();
            Encoder_Distance_Calc();

            l = (dir == DIR_FORWARD ? PWM_BASE : PWM_BACK) - g_track_pid_out;
            r = (dir == DIR_FORWARD ? PWM_BASE : PWM_BACK) + g_track_pid_out;

            dir == DIR_FORWARD ? Motor_Forward(l, r) : Motor_Backward(l, r);
            OLED_ShowString(0,0,"Go Unload B");

            if(Border_Check())
            {
                Motor_Stop();
                g_current_node = NODE_B;
                g_car_state = STATE_UNLOAD_B;
            }
            break;

        case STATE_UNLOAD_B:
            OLED_ShowString(0,0,"Unload B");
            Arm_Drop_B();
            if(g_arm_done_flag)
            {
                g_arm_done_flag = 0;
                g_car_state = STATE_GO_UNLOAD_C;
                OLED_Clear();
            }
            break;

        case STATE_GO_UNLOAD_C:
            g_target_node = NODE_C;
            dir = Path_Plan(g_current_node, g_target_node);
            Track_Sensor_Read();
            Track_PID_Calc();
            Encoder_Distance_Calc();

            l = (dir == DIR_FORWARD ? PWM_BASE : PWM_BACK) - g_track_pid_out;
            r = (dir == DIR_FORWARD ? PWM_BASE : PWM_BACK) + g_track_pid_out;

            dir == DIR_FORWARD ? Motor_Forward(l, r) : Motor_Backward(l, r);
            OLED_ShowString(0,0,"Go Unload C");

            if(Border_Check())
            {
                Motor_Stop();
                g_current_node = NODE_C;
                g_car_state = STATE_UNLOAD_C;
            }
            break;

        case STATE_UNLOAD_C:
            OLED_ShowString(0,0,"Unload C");
            Arm_Drop_C();
            if(g_arm_done_flag)
            {
                g_arm_done_flag = 0;
                g_car_state = STATE_RETURN_HOME;
                g_target_node = NODE_HOME;
                OLED_Clear();
            }
            break;

        case STATE_RETURN_HOME:
            dir = Path_Plan(g_current_node, g_target_node);
            Track_Sensor_Read();
            Track_PID_Calc();
            Encoder_Distance_Calc();

            l = (dir == DIR_FORWARD ? PWM_BASE : PWM_BACK) - g_track_pid_out;
            r = (dir == DIR_FORWARD ? PWM_BASE : PWM_BACK) + g_track_pid_out;

            dir == DIR_FORWARD ? Motor_Forward(l, r) : Motor_Backward(l, r);
            OLED_ShowString(0,0,"Return Home");

            if(Border_Check())
            {
                Motor_Stop();
                g_current_node = NODE_HOME;
                g_car_state = STATE_FINISH;
            }
            break;

        case STATE_FINISH:
            Motor_Stop();
            OLED_ShowString(0,0,"Task Finish!");
            HAL_Delay(2000);
            g_car_state = STATE_IDLE;
            OLED_Clear();
            break;
    }
}

// // ====================== 串口中断 ======================
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//   if(huart == &huart1)
//   {
//     if(uart_rx_data == '\n')
//     {
//       uart_rx_buf[uart_rx_cnt] = '\0';
//       sscanf((char*)uart_rx_buf, "C,%f", &center_offset);
//       line_flag = 1;
//       uart_rx_cnt = 0;
//     }
//     else
//     {
//       if(uart_rx_cnt < UART_BUF_SIZE-1)
//           uart_rx_buf[uart_rx_cnt++] = uart_rx_data;
//       else
//           uart_rx_cnt = 0;
//     }
//     HAL_UART_Receive_IT(&huart1, &uart_rx_data, 1);
//   }
// }

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif /* USE_FULL_ASSERT */
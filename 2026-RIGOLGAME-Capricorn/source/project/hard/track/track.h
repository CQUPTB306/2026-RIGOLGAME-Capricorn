#ifndef __TRACK_H
#define __TRACK_H

#include "stdint.h"
#include "motor.h"

/*==================== 硬件配置 ====================*/
#define TRACK_I2C_ADDR  0x12   // I2C 从机地址（7位）
#define TRACK_REG_DATA  0x00   // 数据寄存器地址

/*==================== PID 结构体 ====================*/

typedef struct
{
    float Kp, Ki, Kd;
    int error;        // 当前偏差
    int P;            // 比例项
    int I;            // 积分项
    int D;            // 微分项
    int last_error;   // 上一次偏差
    int output;       // PID 输出值
} PID_TypeDef;

extern PID_TypeDef PID;
extern PID_TypeDef PID_MV;

extern int IR_Weight[8];
extern uint8_t BASE_SPEED;
extern float ir;

extern volatile int8_t g_track_error;
/*==================== 外部依赖 ====================*/
// 用户需在外部实现电机控制函数
void Motor_Set(int left_speed, int right_speed);  // 速度范围 0~1000

/*==================== 循迹模块接口 ====================*/
void Track_Init(void);                      // 初始化 I2C 循迹模块
uint8_t Track_Read_All(void);               // 读取全部 8 路状态（bit0~bit7 对应 1~8 路，1=黑线）
uint8_t Track_Read_Channel(uint8_t channel);// 读取指定通道

// 新增循迹控制接口
int  Get_Track_Error(void);                 // 计算当前偏差（加权平均）
int  PID_Calc(int error);                   // PID 计算（需传入偏差）
void Track_Run(void);                       // 循迹主函数（周期性调用）
void Track_Run_MV(void);

#endif

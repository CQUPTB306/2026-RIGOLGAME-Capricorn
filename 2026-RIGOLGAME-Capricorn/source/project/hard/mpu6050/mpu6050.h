#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdint.h>
#include <stdbool.h>

//==================== 用户可配置宏 ====================
// MPU6050 I2C设备地址（7位地址）
#define MPU6050_DEV_ADDR        0x68

// 量程配置（必须与初始化时写入的寄存器值匹配）
#define MPU6050_GYRO_RANGE      0x18    // ±2000dps
#define MPU6050_ACCEL_RANGE     0x18    // ±8g

// 灵敏度（单位：LSB/(°/s) 和 LSB/g）
// ±2000dps → 16.4 LSB/(°/s) , ±8g → 4096 LSB/g
#define MPU6050_GYRO_SCALE      16.4f
#define MPU6050_ACCEL_SCALE     4096.0f

// 采样率分频器（输出频率 = 1kHz / (1+div)）
#define MPU6050_SMPLRT_DIV_VAL  0x09    // 100Hz

// 低通滤波器配置
#define MPU6050_DLPF_CFG        0x06    // 带宽约5Hz

// 互补滤波系数（角度信任陀螺仪的比例，通常0.96~0.99）
#define MPU6050_COMPL_FILTER_ALPHA  0.96f

// 零偏校准采样次数（静止时）
#define MPU6050_CALIB_SAMPLES   200
//======================================================

// 初始化MPU6050
void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);

// 零偏校准（静止水平放置，自动计算加速度计和陀螺仪零偏）
void MPU6050_Calibrate(void);

// 姿态更新（需定期调用，建议采样周期10ms，内部自动计算时间差）
void MPU6050_UpdateAngles(void);

// 获取欧拉角（单位：度）
float MPU6050_GetPitch(void);   // 俯仰角  -90 ~ +90
float MPU6050_GetRoll(void);    // 滚转角  -180 ~ +180
float MPU6050_GetYaw(void);     // 偏航角  0 ~ 360 （仅陀螺仪积分，会漂移）

// 重置偏航角（将当前角度设为0）
void MPU6050_ResetYaw(void);

// 手动设置零偏（可选，不调用校准函数时可直接设置）
void MPU6050_SetAccelOffset(int16_t x, int16_t y, int16_t z);
void MPU6050_SetGyroOffset(int16_t x, int16_t y, int16_t z);

// 获取原始数据（已减去零偏，单位：LSB）
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);

#endif

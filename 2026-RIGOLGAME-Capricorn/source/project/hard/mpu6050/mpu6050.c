/*
校准函数要求传感器静止且水平放置。但每次你手动上电时，传感器放置的微小倾斜角度、桌面是否绝对水平、是否有人碰到桌子等因素不同。
如果校准瞬间传感器有微弱的振动或倾斜，计算出的 gyro_offset[2] 就会偏离真实零偏，导致静止时仍有残留角速度 → 积分后 yaw 自增
也就是上电的时候需要让陀螺仪静止没有扰动，不然会以为有角速度的时候为0而使小车误以为静止时存在角速度
*/

#include "stm32f1xx_hal.h"
#include "mpu6050.h"
#include "mpu6050_Reg.h"
#include "mpu_soft_i2c.h"
#include <math.h>

// 静态变量存储零偏值（原始LSB）
static int16_t accel_offset[3] = {0, 0, 0};
static int16_t gyro_offset[3]  = {0, 0, 0};

// 姿态解算变量
static float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
static float gyro_pitch = 0.0f, gyro_roll = 0.0f, gyro_yaw = 0.0f;
static uint32_t last_update_time = 0;
static bool is_calibrated = false;


// 内部函数：读取原始数据（不减去零偏）
static void MPU6050_ReadRawData(int16_t *ax, int16_t *ay, int16_t *az,
                                int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t Data[14];
    mpu_I2C_Read_Buf(MPU6050_DEV_ADDR, MPU6050_ACCEL_XOUT_H, 14, Data);
    
    *ax = (int16_t)(Data[0] << 8) | Data[1];
    *ay = (int16_t)(Data[2] << 8) | Data[3];
    *az = (int16_t)(Data[4] << 8) | Data[5];
    *gx = (int16_t)(Data[8] << 8) | Data[9];
    *gy = (int16_t)(Data[10] << 8) | Data[11];
    *gz = (int16_t)(Data[12] << 8) | Data[13];
}

// 校准函数（静止水平放置）
void MPU6050_Calibrate(void)
{
    int32_t sum_ax=0, sum_ay=0, sum_az=0;
    int32_t sum_gx=0, sum_gy=0, sum_gz=0;
    int16_t ax, ay, az, gx, gy, gz;
    
    // 采样
    for(int i=0; i<MPU6050_CALIB_SAMPLES; i++)
    {
        MPU6050_ReadRawData(&ax, &ay, &az, &gx, &gy, &gz);
        sum_ax += ax; sum_ay += ay; sum_az += az;
        sum_gx += gx; sum_gy += gy; sum_gz += gz;
        HAL_Delay(5);
    }
    
    // 加速度计零偏：理论静止时X=0, Y=0, Z=+1g = ACCEL_SCALE LSB
    // 因此偏移 = 平均值 - 理论值
    accel_offset[0] = (int16_t)(sum_ax / MPU6050_CALIB_SAMPLES) - 0;
    accel_offset[1] = (int16_t)(sum_ay / MPU6050_CALIB_SAMPLES) - 0;
    accel_offset[2] = (int16_t)(sum_az / MPU6050_CALIB_SAMPLES) - (int16_t)MPU6050_ACCEL_SCALE;
    
    // 陀螺仪零偏：静止时理论输出0，偏移即平均值
    gyro_offset[0] = (int16_t)(sum_gx / MPU6050_CALIB_SAMPLES);
    gyro_offset[1] = (int16_t)(sum_gy / MPU6050_CALIB_SAMPLES);
    gyro_offset[2] = (int16_t)(sum_gz / MPU6050_CALIB_SAMPLES);
    
    is_calibrated = true;
}

// 写寄存器
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    mpu_I2C_Write_One_Byte(MPU6050_DEV_ADDR, RegAddress, Data);
}

// 读单个寄存器
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    return mpu_I2C_Read_One_Byte(MPU6050_DEV_ADDR, RegAddress);
}

// 连续读多个寄存器
void MPU6050_ReadRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count)
{
    mpu_I2C_Read_Buf(MPU6050_DEV_ADDR, RegAddress, Count, DataArray);
}

// 初始化MPU6050
void MPU6050_Init(void)
{
    //mpu_I2C_Init();
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);   // 时钟源选择X轴陀螺仪PLL
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);   // 所有轴使能
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, MPU6050_SMPLRT_DIV_VAL);
    MPU6050_WriteReg(MPU6050_CONFIG, MPU6050_DLPF_CFG);
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, MPU6050_GYRO_RANGE);
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_RANGE);
    
    last_update_time = HAL_GetTick();
}

// 读取ID
uint8_t MPU6050_GetID(void)
{
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

// 读取加速度和陀螺仪原始数据（已减去零偏）
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    uint8_t Data[14];
    MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_H, Data, 14);
    
    *AccX = ((int16_t)(Data[0] << 8) | Data[1]) - accel_offset[0];
    *AccY = ((int16_t)(Data[2] << 8) | Data[3]) - accel_offset[1];
    *AccZ = ((int16_t)(Data[4] << 8) | Data[5]) - accel_offset[2];
    
    *GyroX = ((int16_t)(Data[8] << 8) | Data[9]) - gyro_offset[0];
    *GyroY = ((int16_t)(Data[10] << 8) | Data[11]) - gyro_offset[1];
    *GyroZ = ((int16_t)(Data[12] << 8) | Data[13]) - gyro_offset[2];
}

// 设置加速度计零偏
void MPU6050_SetAccelOffset(int16_t x, int16_t y, int16_t z)
{
    accel_offset[0] = x;
    accel_offset[1] = y;
    accel_offset[2] = z;
}

// 设置陀螺仪零偏
void MPU6050_SetGyroOffset(int16_t x, int16_t y, int16_t z)
{
    gyro_offset[0] = x;
    gyro_offset[1] = y;
    gyro_offset[2] = z;
}

// 重置偏航角
void MPU6050_ResetYaw(void)
{
    yaw = 0.0f;
    gyro_yaw = 0.0f;
}

// 姿态更新（需定期调用，推荐每10ms调用一次）
void MPU6050_UpdateAngles(void)
{
    uint32_t now = HAL_GetTick();
    float dt = (now - last_update_time) / 1000.0f;
    if(dt <= 0.001f || dt > 0.1f)  // 时间差异常则跳过
    {
        last_update_time = now;
        return;
    }
    
    int16_t ax, ay, az, gx, gy, gz;
    MPU6050_GetData(&ax, &ay, &az, &gx, &gy, &gz);
    
    // 将原始数据转换为物理量
    float acc_x_g = ax / MPU6050_ACCEL_SCALE;
    float acc_y_g = ay / MPU6050_ACCEL_SCALE;
    float acc_z_g = az / MPU6050_ACCEL_SCALE;
    
    float gyro_x_rad = (gx / MPU6050_GYRO_SCALE) * 3.14159f / 180.0f;
    float gyro_y_rad = (gy / MPU6050_GYRO_SCALE) * 3.14159f / 180.0f;
    float gyro_z_rad = (gz / MPU6050_GYRO_SCALE) * 3.14159f / 180.0f;
    
    // 加速度计计算的俯仰和滚转（弧度）
    float acc_pitch = atan2f(acc_y_g, sqrtf(acc_x_g*acc_x_g + acc_z_g*acc_z_g));
    float acc_roll  = atan2f(-acc_x_g, sqrtf(acc_y_g*acc_y_g + acc_z_g*acc_z_g));
    
    // 陀螺仪积分（姿态更新）
    gyro_pitch += gyro_y_rad * dt;   // 注意坐标系对应关系
    gyro_roll  += gyro_x_rad * dt;
    gyro_yaw   += gyro_z_rad * dt;
    
    // 互补滤波
    pitch = MPU6050_COMPL_FILTER_ALPHA * gyro_pitch + (1 - MPU6050_COMPL_FILTER_ALPHA) * acc_pitch;
    roll  = MPU6050_COMPL_FILTER_ALPHA * gyro_roll  + (1 - MPU6050_COMPL_FILTER_ALPHA) * acc_roll;
    yaw   = gyro_yaw;   // 偏航角仅由陀螺仪积分，无磁力计校正
    
    // 重置积分值，防止累积误差过大
    gyro_pitch = pitch;
    gyro_roll  = roll;
    
    // 将弧度转换为度，并限制范围
    pitch = pitch * 180.0f / 3.14159f;
    roll  = roll  * 180.0f / 3.14159f;
    yaw   = yaw   * 180.0f / 3.14159f;
    
    // 偏航角规整到 0~360
    if(yaw >= 360.0f) yaw -= 360.0f;
    if(yaw < 0.0f)    yaw += 360.0f;
    
    last_update_time = now;
}

float MPU6050_GetPitch(void) { return pitch; }
float MPU6050_GetRoll(void)  { return roll;  }
float MPU6050_GetYaw(void)   { return yaw;   }

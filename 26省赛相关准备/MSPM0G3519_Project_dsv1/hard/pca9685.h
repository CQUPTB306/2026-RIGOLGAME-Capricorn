/**
 * @file    pca9685.h
 * @brief   PCA9685 16通道 12位 PWM 驱动器 (舵机控制)
 * @author  Ziv-Xu
 * @date    2026-05-22
 *
 * @note    特性:
 *          - 16 路独立 PWM 输出通道
 *          - 12 位分辨率 (4096 级)
 *          - 内置 25MHz 振荡器，可调频率 (典型 50Hz 用于舵机)
 *          - 软件 I2C 通信 (通过宏映射到 soft_i2c_simple)
 *          - 标准舵机脉冲范围: 0.5ms ~ 2.5ms (对应 0° ~ 180°)
 *
 * @warning I2C 地址: 7 位 0x40，写入时左移为 0x80
 */

#ifndef __PCA9685_H
#define __PCA9685_H

#include "soft_i2c_simple.h"

/*=================== 硬件地址 ====================*/

/** @brief PCA9685 I2C 从机写地址 (7位 0x40 << 1) */
#define PCA9685_ADDR    0x80

/*=================== 寄存器地址 ====================*/

#define PCA9685_MODE1       0x00  /**< 模式寄存器 1 (SLEEP/RESTART/AI/ALLCALL 等) */
#define PCA9685_MODE2       0x01  /**< 模式寄存器 2 (推挽/开漏、输出逻辑)        */
#define PCA9685_PRE_SCALE   0xFE  /**< 预分频寄存器 (设置 PWM 频率)              */

/** @name 通道 0 的 PWM 寄存器 (通道 N 偏移 = PCA9685_LED0_ON_L + 4*N) */
/**@{*/
#define PCA9685_LED0_ON_L   0x06  /**< 通道 0 ON  计数低字节 */
#define PCA9685_LED0_ON_H   0x07  /**< 通道 0 ON  计数高字节 */
#define PCA9685_LED0_OFF_L  0x08  /**< 通道 0 OFF 计数低字节 */
#define PCA9685_LED0_OFF_H  0x09  /**< 通道 0 OFF 计数高字节 */
/**@}*/

/**
 * @note  每个通道占用 4 个连续寄存器 (ON_L, ON_H, OFF_L, OFF_H)
 *        通道 N 的基地址 = PCA9685_LED0_ON_L + 4 × N
 */

/*=================== 外部引用 ====================*/

/** @brief PCA9685 使用的软件 I2C 总线对象 (定义在 soft_i2c_simple.c) */
extern SoftI2C_Obj i2c_pca9685;

/*=================== I2C 函数宏映射 ====================*/

/**
 * @brief 将软 I2C 函数调用映射到 PCA9685 专用 I2C 总线对象
 */
#define IIC_Start()       SoftI2C_Start(&i2c_pca9685)         /**< 发送 I2C 起始信号   */
#define IIC_Send_Byte(d)  SoftI2C_SendByte(&i2c_pca9685, (d)) /**< 发送一个字节         */
#define IIC_Wait_Ack()    SoftI2C_WaitAck(&i2c_pca9685)       /**< 等待从机应答         */
#define IIC_Stop()        SoftI2C_Stop(&i2c_pca9685)          /**< 发送 I2C 停止信号   */

/*=================== 函数声明 ====================*/

/**
 * @brief   PCA9685 初始化
 * @note    执行流程: 设置 PWM 频率 50Hz → 配置推挽输出 → 清空所有通道
 * @warning 调用前必须先执行 SoftI2C_Init(&i2c_pca9685)
 */
void pca9685_Init(void);

/**
 * @brief   设置 PWM 频率
 * @param   freq_hz 目标频率 (Hz)，舵机典型值 50.0
 * @note    内部自动计算预分频值: prescale = round(25MHz / (4096 × freq)) - 1
 * @warning 设置频率过程中会进入 Sleep 模式然后唤醒
 */
void pca9685_SetPWMFreq(float freq_hz);

/**
 * @brief   设置指定通道的 PWM 占空比
 * @param   channel 通道号 (0~15)
 * @param   off     OFF 计数值 (0~4095)
 * @note    ON 固定为 0，OFF = off
 *          - off = 0    → 始终低电平 (0% 占空比)
 *          - off = 2048 → 50% 占空比
 *          - off = 4095 → 接近 100% 占空比
 * @warning channel 超过 15 时函数直接返回，不执行操作
 */
void pca9685_SetPWM(uint8_t channel, uint16_t off);

/**
 * @brief   设置舵机角度 (高精度 0.1°)
 * @param   channel 通道号 (0~15)
 * @param   angle   角度 ×10 (0~1800, 对应 0.0°~180.0°)
 * @note    换算公式:
 *          - 脉冲宽度 (μs) = 500 + angle × 2000 / 1800
 *          - PWM 计数值 = 脉冲宽度 × 4096 / 20000
 *          - 0° 对应约 0.5ms (off=102)，180° 对应约 2.5ms (off=512)
 * @warning angle 超过 1800 时自动截断为 1800
 */
void pca9685_SetServoAngle(uint8_t channel, uint16_t angle);

/**
 * @brief   设置270°舵机角度 (高精度 0.1°)
 * @param   channel 通道号 (0~15)
 * @param   angle   角度 ×10 (0~2700, 对应 0.0°~270.0°)
 * @note    换算公式:
 *          - 脉冲宽度 (μs) = 500 + angle × 2000 / 2700
 * @warning angle 超过 2700 时自动截断为 2700
 */
void pca9685_SetServoAngle270(uint8_t channel, uint16_t angle);

/**
 * @brief   软件复位 PCA9685
 * @note    通过 MODE1 寄存器的 RESTART 位 (bit7) 实现
 *          流程: 置位 RESTART → 延时 10ms → 清除 RESTART → 延时 10ms
 */
void pca9685_Reset(void);

/**
 * @brief   关闭所有 16 个通道的 PWM 输出
 * @note    将所有通道的 OFF 计数值设为 0 (舵机回到最小位置)
 */
void pca9685_AllOff(void);

/**
 * @brief   设置所有 16 个通道的 PWM 占空比 (批量操作)
 * @param   off OFF 计数值 (0~4095)，所有通道同步设置
 * @note    等价于依次对通道 0~15 调用 pca9685_SetPWM()
 * @see     pca9685_SetPWM
 */
void pca9685_SetAllPWM(uint16_t off);

/**
 * @brief   设置所有 16 个通道的舵机角度 (批量操作)
 * @param   angle 角度 ×10 (0~1800)，所有舵机同步转动
 * @note    等价于依次对通道 0~15 调用 pca9685_SetServoAngle()
 * @see     pca9685_SetServoAngle
 */
void pca9685_SetAllAngles(uint16_t angle);

/**
 * @brief   读取 PCA9685 内部寄存器
 * @param   reg 寄存器地址 (0x00 ~ 0xFE)
 * @return  寄存器当前值
 * @note    内部使用 I2C 组合读写时序 (写寄存器地址 → 重复起始 → 读数据)
 */
uint8_t pca_read_reg(uint8_t reg);

#endif /* __PCA9685_H */

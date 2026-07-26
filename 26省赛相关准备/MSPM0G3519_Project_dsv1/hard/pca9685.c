#include "pca9685.h"

// 内部函数：写一个寄存器
static void pca_write_reg(uint8_t reg, uint8_t data)
{
    IIC_Start();
    IIC_Send_Byte(PCA9685_ADDR);
    IIC_Wait_Ack();
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    IIC_Send_Byte(data);
    IIC_Wait_Ack();
    IIC_Stop();
}

// 内部函数：读一个寄存器（用于调试，本例未使用，但保留）
uint8_t pca_read_reg(uint8_t reg)
{
    uint8_t val = 0;
    IIC_Start();
    IIC_Send_Byte(PCA9685_ADDR);
    IIC_Wait_Ack();
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(PCA9685_ADDR | 0x01); // 读地址
    IIC_Wait_Ack();
    // 这里需要读一个字节，且最后发 NACK
    // 因为你的软I2C库有 SoftI2C_ReadByte，我们直接调用
    val = SoftI2C_ReadByte(&i2c_pca9685, 0); // 参数0表示最后一个字节发NACK
    IIC_Stop();
    return val;
}

// 初始化 PCA9685：设置频率 50Hz，进入正常工作模式
void pca9685_Init(void)
{
    pca9685_SetPWMFreq(50.0f);
    pca_write_reg(PCA9685_MODE2, 0x04);   // 推挽输出
    pca9685_AllOff();
}

// 设置 PWM 频率（单位 Hz）
void pca9685_SetPWMFreq(float freq_hz)
{
    uint8_t prescale;
    uint8_t old_mode;

    // 1. 读取当前 MODE1
    old_mode = pca_read_reg(PCA9685_MODE1);

    // 2. 进入 Sleep 模式（设置 SLEEP 位）
    pca_write_reg(PCA9685_MODE1, old_mode | 0x10);
    delay_ms(5);

    // 3. 计算预分频值
    prescale = (uint8_t)((25000000.0f / (4096.0f * freq_hz)) + 0.5f) - 1;

    // 4. 写入 PRE_SCALE
    pca_write_reg(PCA9685_PRE_SCALE, prescale);

    // 5. 退出 Sleep 模式（清除 SLEEP 位）★ 关键修改
    pca_write_reg(PCA9685_MODE1, old_mode & ~0x10);   // 清除 bit4
    delay_ms(5);   // 等待振荡器稳定（至少 500us）

    // 6. 触发 RESTART，使新设置生效
    pca_write_reg(PCA9685_MODE1, (old_mode & ~0x10) | 0x80);
    delay_ms(1);
    pca_write_reg(PCA9685_MODE1, old_mode & ~0x90);   // 清除 SLEEP 和 RESTART

    // 7. 设置 MODE2 为推挽输出
    pca_write_reg(PCA9685_MODE2, 0x04);
}
// 设置某个通道的 PWM 占空比（12位，off 范围 0~4095）
// 本函数将 ON 设为 0，OFF 设为 off
void pca9685_SetPWM(uint8_t channel, uint16_t off)
{
    uint8_t reg_base;
    if (channel > 15) return;

    reg_base = PCA9685_LED0_ON_L + 4 * channel;

    // ON = 0
    pca_write_reg(reg_base,     0x00);
    pca_write_reg(reg_base + 1, 0x00);
    // OFF 低 8 位
    pca_write_reg(reg_base + 2, (uint8_t)(off & 0xFF));
    // OFF 高 4 位（只写低4位）
    pca_write_reg(reg_base + 3, (uint8_t)((off >> 8) & 0x0F));
}

// 舵机专用：角度 0~1800 (单位 0.1°) 转换为 PWM 值并设置
// 标准舵机：脉冲宽度 0.5ms~2.5ms，周期 20ms (50Hz)
void pca9685_SetServoAngle(uint8_t channel, uint16_t angle)
{
    uint16_t pwm;
    float pulse_us;

    if (channel > 15) return;
    if (angle > 1800) angle = 1800;

    // 脉冲宽度 (us)：0.5ms + (angle / 1800) * 2ms
    pulse_us = 500.0f + angle * 2000.0f / 1800.0f;
    // 转换成 12 位计数值：pulse_us / 20000us * 4096
    pwm = (uint16_t)(pulse_us * 4096.0f / 20000.0f);

    // 限制范围（防止过界）
    if (pwm < 102) pwm = 102;   // 对应约 0.5ms
    if (pwm > 512) pwm = 512;   // 对应约 2.5ms

    pca9685_SetPWM(channel, pwm);
}

// 270°舵机专用：角度 0~2700 (单位 0.1°) 转换为 PWM 值并设置
// 脉冲范围同样是 0.5ms~2.5ms，只改变角度映射斜率
void pca9685_SetServoAngle270(uint8_t channel, uint16_t angle)
{
    uint16_t pwm;
    float pulse_us;

    if (channel > 15) return;
    if (angle > 2700) angle = 2700;

    // 脉冲宽度 (us)：0.5ms + (angle / 2700) * 2ms
    pulse_us = 500.0f + angle * 2000.0f / 2700.0f;
    pwm = (uint16_t)(pulse_us * 4096.0f / 20000.0f);

    if (pwm < 102) pwm = 102;
    if (pwm > 512) pwm = 512;

    pca9685_SetPWM(channel, pwm);
}

// 软件复位：通过 MODE1 的 RESTART 位
void pca9685_Reset(void)
{
    uint8_t mode1;
    // 读 MODE1
    IIC_Start();
    IIC_Send_Byte(PCA9685_ADDR);
    IIC_Wait_Ack();
    IIC_Send_Byte(PCA9685_MODE1);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(PCA9685_ADDR | 0x01);
    IIC_Wait_Ack();
    mode1 = SoftI2C_ReadByte(&i2c_pca9685, 0);
    IIC_Stop();

    // 置位 RESTART 位 (bit7)
    mode1 |= 0x80;
    pca_write_reg(PCA9685_MODE1, mode1);
    delay_ms(10);
    // 清除 RESTART 位（可选）
    mode1 &= 0x7F;
    pca_write_reg(PCA9685_MODE1, mode1);
    delay_ms(10);
}

// 所有通道输出 0（舵机回到最小位置）
void pca9685_AllOff(void)
{
    for (uint8_t ch = 0; ch < 16; ch++) {
        pca9685_SetPWM(ch, 0);
    }
}

// 批量设置所有 16 个通道的 PWM 占空比
void pca9685_SetAllPWM(uint16_t off)
{
    for (uint8_t ch = 0; ch < 16; ch++) {
        pca9685_SetPWM(ch, off);
    }
}

// 批量设置所有 16 个通道的舵机角度
void pca9685_SetAllAngles(uint16_t angle)
{
    for (uint8_t ch = 0; ch < 16; ch++) {
        pca9685_SetServoAngle(ch, angle);
    }
}
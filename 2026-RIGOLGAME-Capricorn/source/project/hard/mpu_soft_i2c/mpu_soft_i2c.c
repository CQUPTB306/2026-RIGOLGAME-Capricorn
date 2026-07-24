#include "mpu_soft_i2c.h"

// 简单延时（根据主频调整，保证I2C速度<400KHz）
static void I2C_Delay(void)
{
//    uint16_t i = 80;
//    while(i--);
}

// 设置SDA为输出模式
static void SDA_Output_Mode(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = MPU_I2C_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;   // 开漏输出
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MPU_I2C_SDA_PORT, &GPIO_InitStruct);
}

// 设置SDA为输入模式
static void SDA_Input_Mode(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = MPU_I2C_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(MPU_I2C_SDA_PORT, &GPIO_InitStruct);
}

// 初始化I2C引脚（SCL和SDA均为开漏输出，外部上拉）
void mpu_I2C_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // 使能GPIOB时钟
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    // 配置SCL为开漏输出
    GPIO_InitStruct.Pin = MPU_I2C_SCL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MPU_I2C_SCL_PORT, &GPIO_InitStruct);
    
    // 配置SDA为开漏输出（初始输出高电平）
    GPIO_InitStruct.Pin = MPU_I2C_SDA_PIN;
    HAL_GPIO_Init(MPU_I2C_SDA_PORT, &GPIO_InitStruct);
    
    // 总线空闲：SCL=1, SDA=1
    HAL_GPIO_WritePin(MPU_I2C_SCL_PORT, MPU_I2C_SCL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MPU_I2C_SDA_PORT, MPU_I2C_SDA_PIN, GPIO_PIN_SET);
}

// 起始信号
void mpu_I2C_Start(void)
{
    SDA_Output_Mode();
    HAL_GPIO_WritePin(MPU_I2C_SDA_PORT, MPU_I2C_SDA_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MPU_I2C_SCL_PORT, MPU_I2C_SCL_PIN, GPIO_PIN_SET);
    I2C_Delay();
    HAL_GPIO_WritePin(MPU_I2C_SDA_PORT, MPU_I2C_SDA_PIN, GPIO_PIN_RESET);
    I2C_Delay();
    HAL_GPIO_WritePin(MPU_I2C_SCL_PORT, MPU_I2C_SCL_PIN, GPIO_PIN_RESET);
    I2C_Delay();
}

// 停止信号
void mpu_I2C_Stop(void)
{
    SDA_Output_Mode();
    HAL_GPIO_WritePin(MPU_I2C_SCL_PORT, MPU_I2C_SCL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MPU_I2C_SDA_PORT, MPU_I2C_SDA_PIN, GPIO_PIN_RESET);
    I2C_Delay();
    HAL_GPIO_WritePin(MPU_I2C_SCL_PORT, MPU_I2C_SCL_PIN, GPIO_PIN_SET);
    I2C_Delay();
    HAL_GPIO_WritePin(MPU_I2C_SDA_PORT, MPU_I2C_SDA_PIN, GPIO_PIN_SET);
    I2C_Delay();
}

// 等待从机应答，返回0：应答，1：无应答
uint8_t mpu_I2C_Wait_Ack(void)
{
    uint8_t ucErrTime = 0;
    SDA_Input_Mode();
    HAL_GPIO_WritePin(MPU_I2C_SDA_PORT, MPU_I2C_SDA_PIN, GPIO_PIN_SET);
    I2C_Delay();
    HAL_GPIO_WritePin(MPU_I2C_SCL_PORT, MPU_I2C_SCL_PIN, GPIO_PIN_SET);
    I2C_Delay();
    
    while(HAL_GPIO_ReadPin(MPU_I2C_SDA_PORT, MPU_I2C_SDA_PIN))
    {
        ucErrTime++;
        if(ucErrTime > 250)
        {
            mpu_I2C_Stop();
            return 1;
        }
    }
    HAL_GPIO_WritePin(MPU_I2C_SCL_PORT, MPU_I2C_SCL_PIN, GPIO_PIN_RESET);
    I2C_Delay();
    return 0;
}

// 主机产生ACK
void mpu_I2C_Ack(void)
{
    SDA_Output_Mode();
    HAL_GPIO_WritePin(MPU_I2C_SCL_PORT, MPU_I2C_SCL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MPU_I2C_SDA_PORT, MPU_I2C_SDA_PIN, GPIO_PIN_RESET);
    I2C_Delay();
    HAL_GPIO_WritePin(MPU_I2C_SCL_PORT, MPU_I2C_SCL_PIN, GPIO_PIN_SET);
    I2C_Delay();
    HAL_GPIO_WritePin(MPU_I2C_SCL_PORT, MPU_I2C_SCL_PIN, GPIO_PIN_RESET);
    I2C_Delay();
}

// 主机产生NACK
void mpu_I2C_NAck(void)
{
    SDA_Output_Mode();
    HAL_GPIO_WritePin(MPU_I2C_SCL_PORT, MPU_I2C_SCL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MPU_I2C_SDA_PORT, MPU_I2C_SDA_PIN, GPIO_PIN_SET);
    I2C_Delay();
    HAL_GPIO_WritePin(MPU_I2C_SCL_PORT, MPU_I2C_SCL_PIN, GPIO_PIN_SET);
    I2C_Delay();
    HAL_GPIO_WritePin(MPU_I2C_SCL_PORT, MPU_I2C_SCL_PIN, GPIO_PIN_RESET);
    I2C_Delay();
}

// 发送一个字节
void mpu_I2C_Send_Byte(uint8_t txd)
{
    uint8_t t;
    SDA_Output_Mode();
    HAL_GPIO_WritePin(MPU_I2C_SCL_PORT, MPU_I2C_SCL_PIN, GPIO_PIN_RESET);
    for(t=0; t<8; t++)
    {
        if(txd & 0x80)
            HAL_GPIO_WritePin(MPU_I2C_SDA_PORT, MPU_I2C_SDA_PIN, GPIO_PIN_SET);
        else
            HAL_GPIO_WritePin(MPU_I2C_SDA_PORT, MPU_I2C_SDA_PIN, GPIO_PIN_RESET);
        txd <<= 1;
        I2C_Delay();
        HAL_GPIO_WritePin(MPU_I2C_SCL_PORT, MPU_I2C_SCL_PIN, GPIO_PIN_SET);
        I2C_Delay();
        HAL_GPIO_WritePin(MPU_I2C_SCL_PORT, MPU_I2C_SCL_PIN, GPIO_PIN_RESET);
        I2C_Delay();
    }
}

// 读取一个字节，ack=1时发送ACK，ack=0时发送NACK
uint8_t mpu_I2C_Read_Byte(uint8_t ack)
{
    uint8_t i, receive = 0;
    SDA_Input_Mode();
    for(i=0; i<8; i++)
    {
        HAL_GPIO_WritePin(MPU_I2C_SCL_PORT, MPU_I2C_SCL_PIN, GPIO_PIN_RESET);
        I2C_Delay();
        HAL_GPIO_WritePin(MPU_I2C_SCL_PORT, MPU_I2C_SCL_PIN, GPIO_PIN_SET);
        receive <<= 1;
        if(HAL_GPIO_ReadPin(MPU_I2C_SDA_PORT, MPU_I2C_SDA_PIN))
            receive |= 0x01;
        I2C_Delay();
    }
    if(ack)
        mpu_I2C_Ack();
    else
        mpu_I2C_NAck();
    return receive;
}

// 写一个字节到寄存器
uint8_t mpu_I2C_Write_One_Byte(uint8_t dev_addr, uint8_t reg, uint8_t data)
{
    mpu_I2C_Start();
    mpu_I2C_Send_Byte((dev_addr << 1) | MPU_I2C_WRITE);
    if(mpu_I2C_Wait_Ack()) { mpu_I2C_Stop(); return 1; }
    mpu_I2C_Send_Byte(reg);
    if(mpu_I2C_Wait_Ack()) { mpu_I2C_Stop(); return 1; }
    mpu_I2C_Send_Byte(data);
    if(mpu_I2C_Wait_Ack()) { mpu_I2C_Stop(); return 1; }
    mpu_I2C_Stop();
    return 0;
}

// 读一个字节
uint8_t mpu_I2C_Read_One_Byte(uint8_t dev_addr, uint8_t reg)
{
    uint8_t temp = 0;
    mpu_I2C_Start();
    mpu_I2C_Send_Byte((dev_addr << 1) | MPU_I2C_WRITE);
    mpu_I2C_Wait_Ack();
    mpu_I2C_Send_Byte(reg);
    mpu_I2C_Wait_Ack();
    mpu_I2C_Start();
    mpu_I2C_Send_Byte((dev_addr << 1) | MPU_I2C_READ);
    mpu_I2C_Wait_Ack();
    temp = mpu_I2C_Read_Byte(0);  // 最后一个字节后发送NACK
    mpu_I2C_Stop();
    return temp;
}

// 连续写多个字节
void mpu_I2C_Write_Buf(uint8_t dev_addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    uint8_t i;
    mpu_I2C_Start();
    mpu_I2C_Send_Byte((dev_addr << 1) | MPU_I2C_WRITE);
    mpu_I2C_Wait_Ack();
    mpu_I2C_Send_Byte(reg);
    mpu_I2C_Wait_Ack();
    for(i=0; i<len; i++)
    {
        mpu_I2C_Send_Byte(buf[i]);
        mpu_I2C_Wait_Ack();
    }
    mpu_I2C_Stop();
}

// 连续读多个字节
void mpu_I2C_Read_Buf(uint8_t dev_addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    mpu_I2C_Start();
    mpu_I2C_Send_Byte((dev_addr << 1) | MPU_I2C_WRITE);
    mpu_I2C_Wait_Ack();
    mpu_I2C_Send_Byte(reg);
    mpu_I2C_Wait_Ack();
    mpu_I2C_Start();
    mpu_I2C_Send_Byte((dev_addr << 1) | MPU_I2C_READ);
    mpu_I2C_Wait_Ack();
    while(len)
    {
        if(len == 1)
            *buf = mpu_I2C_Read_Byte(0);
        else
            *buf = mpu_I2C_Read_Byte(1);
        buf++;
        len--;
    }
    mpu_I2C_Stop();
}
#include "soft_i2c.h"
#include "gpio.h"

// 简单延时函数（根据主频调整，保证I2C速度<400KHz即可）
static void I2C_Delay(void)
{
    uint8_t i = 10;
    while(i--);
}
/*
模式的选择之后在CubeMX中配置
// SDA 输入模式，也就是此页中把PB6设置为输出上拉模式
static void SDA_Input_Mode(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = I2C_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;  // 上拉
    HAL_GPIO_Init(I2C_SDA_GPIO_PORT, &GPIO_InitStruct);
}

// SDA 输出模式，也就是此页中PB7设置为上拉输入模式
static void SDA_Output_Mode(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = I2C_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(I2C_SDA_GPIO_PORT, &GPIO_InitStruct);
}
*/


// I2C 起始信号
void I2C_Start(void)
{
    //SDA_Output_Mode();
    HAL_GPIO_WritePin(I2C_SDA_GPIO_PORT, I2C_SDA_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(I2C_SCL_GPIO_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
    I2C_Delay();
    HAL_GPIO_WritePin(I2C_SDA_GPIO_PORT, I2C_SDA_PIN, GPIO_PIN_RESET);
    I2C_Delay();
    HAL_GPIO_WritePin(I2C_SCL_GPIO_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
}

// I2C 停止信号
void I2C_Stop(void)
{
    //SDA_Output_Mode();
    HAL_GPIO_WritePin(I2C_SCL_GPIO_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(I2C_SDA_GPIO_PORT, I2C_SDA_PIN, GPIO_PIN_RESET);
    I2C_Delay();
    HAL_GPIO_WritePin(I2C_SCL_GPIO_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(I2C_SDA_GPIO_PORT, I2C_SDA_PIN, GPIO_PIN_SET);
    I2C_Delay();
}

// 等待ACK
uint8_t I2C_Wait_Ack(void)
{
    uint8_t ucErrTime = 0;
    //SDA_Input_Mode();
    HAL_GPIO_WritePin(I2C_SDA_GPIO_PORT, I2C_SDA_PIN, GPIO_PIN_SET);
    I2C_Delay();
    HAL_GPIO_WritePin(I2C_SCL_GPIO_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
    I2C_Delay();

    while(HAL_GPIO_ReadPin(I2C_SDA_GPIO_PORT, I2C_SDA_PIN))
    {
        ucErrTime++;
        if(ucErrTime > 250)
        {
            I2C_Stop();
            return 1;
        }
    }
    HAL_GPIO_WritePin(I2C_SCL_GPIO_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
    return 0;
}

// 产生 ACK
void I2C_Ack(void)
{
    HAL_GPIO_WritePin(I2C_SCL_GPIO_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
    //SDA_Output_Mode();
    HAL_GPIO_WritePin(I2C_SDA_GPIO_PORT, I2C_SDA_PIN, GPIO_PIN_RESET);
    I2C_Delay();
    HAL_GPIO_WritePin(I2C_SCL_GPIO_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
    I2C_Delay();
    HAL_GPIO_WritePin(I2C_SCL_GPIO_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
}

// 产生 NACK
void I2C_NAck(void)
{
    HAL_GPIO_WritePin(I2C_SCL_GPIO_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
    //SDA_Output_Mode();
    HAL_GPIO_WritePin(I2C_SDA_GPIO_PORT, I2C_SDA_PIN, GPIO_PIN_SET);
    I2C_Delay();
    HAL_GPIO_WritePin(I2C_SCL_GPIO_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
    I2C_Delay();
    HAL_GPIO_WritePin(I2C_SCL_GPIO_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
}

// 发送一个字节
void I2C_Send_Byte(uint8_t txd)
{
    uint8_t t;
    //SDA_Output_Mode();
    HAL_GPIO_WritePin(I2C_SCL_GPIO_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
    for(t=0;t<8;t++)
    {
        if((txd&0x80)>>7)
            HAL_GPIO_WritePin(I2C_SDA_GPIO_PORT, I2C_SDA_PIN, GPIO_PIN_SET);
        else
            HAL_GPIO_WritePin(I2C_SDA_GPIO_PORT, I2C_SDA_PIN, GPIO_PIN_RESET);
        txd<<=1;
        I2C_Delay();
        HAL_GPIO_WritePin(I2C_SCL_GPIO_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
        I2C_Delay();
        HAL_GPIO_WritePin(I2C_SCL_GPIO_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
        I2C_Delay();
    }
}

// 读取一个字节
uint8_t I2C_Read_Byte(unsigned char ack)
{
    unsigned char i,receive=0;
    //SDA_Input_Mode();
    for(i=0;i<8;i++)
    {
        HAL_GPIO_WritePin(I2C_SCL_GPIO_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
        I2C_Delay();
        HAL_GPIO_WritePin(I2C_SCL_GPIO_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
        receive<<=1;
        if(HAL_GPIO_ReadPin(I2C_SDA_GPIO_PORT, I2C_SDA_PIN))receive++;
        I2C_Delay();
    }
    if(!ack)
        I2C_NAck();
    else
        I2C_Ack();
    return receive;
}

/************************* 常用应用接口 *************************/
// 写1个字节到指定寄存器
uint8_t I2C_Write_One_Byte(uint8_t addr, uint8_t reg, uint8_t data)
{
    I2C_Start();
    I2C_Send_Byte((addr<<1)|I2C_WRITE);
    if(I2C_Wait_Ack()){I2C_Stop();return 1;}
    I2C_Send_Byte(reg);
    I2C_Wait_Ack();
    I2C_Send_Byte(data);
    I2C_Wait_Ack();
    I2C_Stop();
    return 0;
}

// 从指定寄存器读1个字节
uint8_t I2C_Read_One_Byte(uint8_t addr, uint8_t reg)
{
    uint8_t temp=0;
    I2C_Start();
    I2C_Send_Byte((addr<<1)|I2C_WRITE);
    I2C_Wait_Ack();
    I2C_Send_Byte(reg);
    I2C_Wait_Ack();
    I2C_Start();
    I2C_Send_Byte((addr<<1)|I2C_READ);
    I2C_Wait_Ack();
    temp=I2C_Read_Byte(0);
    I2C_Stop();
    return temp;
}

// 连续写多个字节
void I2C_Write_Buf(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    uint8_t i;
    I2C_Start();
    I2C_Send_Byte((addr<<1)|I2C_WRITE);
    I2C_Wait_Ack();
    I2C_Send_Byte(reg);
    I2C_Wait_Ack();
    for(i=0;i<len;i++)
    {
        I2C_Send_Byte(buf[i]);
        I2C_Wait_Ack();
    }
    I2C_Stop();
}

// 连续读多个字节
void I2C_Read_Buf(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    I2C_Start();
    I2C_Send_Byte((addr<<1)|I2C_WRITE);
    I2C_Wait_Ack();
    I2C_Send_Byte(reg);
    I2C_Wait_Ack();
    I2C_Start();
    I2C_Send_Byte((addr<<1)|I2C_READ);
    I2C_Wait_Ack();
    while(len)
    {
        if(len==1)*buf=I2C_Read_Byte(0);
        else *buf=I2C_Read_Byte(1);
        buf++;
        len--;
    }
    I2C_Stop();
}

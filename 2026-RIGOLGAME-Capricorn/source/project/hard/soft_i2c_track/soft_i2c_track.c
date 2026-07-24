#include "soft_i2c_track.h"
#include "gpio.h"

/* 简单延时函数（可根据主频调整，保证 I2C 速度低于 400kHz） */
static void TRACK_I2C_Delay(void)
{
    uint16_t i =800;   // 可根据实际主频调整，保证时序稳定
    while(i--);
}

/*==================== 引脚模式切换函数 ====================*/
/**
 * @brief  将 SDA 引脚配置为输入模式（用于读取 ACK 或数据）
 */
static void TRACK_SDA_Input_Mode(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = TRACK_I2C_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;          // 开启内部上拉
//		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;  // 开漏输出，配合上拉电阻
//    GPIO_InitStruct.Pull = GPIO_PULLUP;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(TRACK_I2C_SDA_GPIO_PORT, &GPIO_InitStruct);
}

/**
 * @brief  将 SDA 引脚配置为输出模式（开漏输出，用于发送数据）
 */
static void TRACK_SDA_Output_Mode(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = TRACK_I2C_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;  // 开漏输出，配合上拉电阻
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(TRACK_I2C_SDA_GPIO_PORT, &GPIO_InitStruct);
}

/*====================== 基础时序 ========================*/
void TRACK_I2C_Start(void)
{
    TRACK_SDA_Output_Mode();    // 确保 SDA 为输出
    HAL_GPIO_WritePin(TRACK_I2C_SDA_GPIO_PORT, TRACK_I2C_SDA_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TRACK_I2C_SCL_GPIO_PORT, TRACK_I2C_SCL_PIN, GPIO_PIN_SET);
    TRACK_I2C_Delay();
    HAL_GPIO_WritePin(TRACK_I2C_SDA_GPIO_PORT, TRACK_I2C_SDA_PIN, GPIO_PIN_RESET);
    TRACK_I2C_Delay();
    HAL_GPIO_WritePin(TRACK_I2C_SCL_GPIO_PORT, TRACK_I2C_SCL_PIN, GPIO_PIN_RESET);
}

void TRACK_I2C_Stop(void)
{
    TRACK_SDA_Output_Mode();
    HAL_GPIO_WritePin(TRACK_I2C_SCL_GPIO_PORT, TRACK_I2C_SCL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TRACK_I2C_SDA_GPIO_PORT, TRACK_I2C_SDA_PIN, GPIO_PIN_RESET);
    TRACK_I2C_Delay();
    HAL_GPIO_WritePin(TRACK_I2C_SCL_GPIO_PORT, TRACK_I2C_SCL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TRACK_I2C_SDA_GPIO_PORT, TRACK_I2C_SDA_PIN, GPIO_PIN_SET);
    TRACK_I2C_Delay();
}

uint8_t TRACK_I2C_Wait_Ack(void)
{
    uint8_t ucErrTime = 0;

    TRACK_SDA_Input_Mode();     // 切换为输入，准备读取 ACK
    HAL_GPIO_WritePin(TRACK_I2C_SDA_GPIO_PORT, TRACK_I2C_SDA_PIN, GPIO_PIN_SET); // 释放总线
    TRACK_I2C_Delay();
    HAL_GPIO_WritePin(TRACK_I2C_SCL_GPIO_PORT, TRACK_I2C_SCL_PIN, GPIO_PIN_SET);
    TRACK_I2C_Delay();

    while (HAL_GPIO_ReadPin(TRACK_I2C_SDA_GPIO_PORT, TRACK_I2C_SDA_PIN))
    {
        ucErrTime++;
        if (ucErrTime > 250)
        {
            TRACK_SDA_Output_Mode();   // 恢复输出模式
            TRACK_I2C_Stop();
            return 1;
        }
    }
    HAL_GPIO_WritePin(TRACK_I2C_SCL_GPIO_PORT, TRACK_I2C_SCL_PIN, GPIO_PIN_RESET);

    TRACK_SDA_Output_Mode();       // 恢复输出模式
    return 0;
}

void TRACK_I2C_Ack(void)
{
    TRACK_SDA_Output_Mode();
    HAL_GPIO_WritePin(TRACK_I2C_SCL_GPIO_PORT, TRACK_I2C_SCL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TRACK_I2C_SDA_GPIO_PORT, TRACK_I2C_SDA_PIN, GPIO_PIN_RESET);
    TRACK_I2C_Delay();
    HAL_GPIO_WritePin(TRACK_I2C_SCL_GPIO_PORT, TRACK_I2C_SCL_PIN, GPIO_PIN_SET);
    TRACK_I2C_Delay();
    HAL_GPIO_WritePin(TRACK_I2C_SCL_GPIO_PORT, TRACK_I2C_SCL_PIN, GPIO_PIN_RESET);
}

void TRACK_I2C_NAck(void)
{
    TRACK_SDA_Output_Mode();
    HAL_GPIO_WritePin(TRACK_I2C_SCL_GPIO_PORT, TRACK_I2C_SCL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TRACK_I2C_SDA_GPIO_PORT, TRACK_I2C_SDA_PIN, GPIO_PIN_SET);
    TRACK_I2C_Delay();
    HAL_GPIO_WritePin(TRACK_I2C_SCL_GPIO_PORT, TRACK_I2C_SCL_PIN, GPIO_PIN_SET);
    TRACK_I2C_Delay();
    HAL_GPIO_WritePin(TRACK_I2C_SCL_GPIO_PORT, TRACK_I2C_SCL_PIN, GPIO_PIN_RESET);
}

void TRACK_I2C_Send_Byte(uint8_t txd)
{
    uint8_t t;
    TRACK_SDA_Output_Mode();      // 确保为输出模式
    HAL_GPIO_WritePin(TRACK_I2C_SCL_GPIO_PORT, TRACK_I2C_SCL_PIN, GPIO_PIN_RESET);
    for (t = 0; t < 8; t++)
    {
        if ((txd & 0x80) >> 7)
            HAL_GPIO_WritePin(TRACK_I2C_SDA_GPIO_PORT, TRACK_I2C_SDA_PIN, GPIO_PIN_SET);
        else
            HAL_GPIO_WritePin(TRACK_I2C_SDA_GPIO_PORT, TRACK_I2C_SDA_PIN, GPIO_PIN_RESET);
        txd <<= 1;
        TRACK_I2C_Delay();
        HAL_GPIO_WritePin(TRACK_I2C_SCL_GPIO_PORT, TRACK_I2C_SCL_PIN, GPIO_PIN_SET);
        TRACK_I2C_Delay();
        HAL_GPIO_WritePin(TRACK_I2C_SCL_GPIO_PORT, TRACK_I2C_SCL_PIN, GPIO_PIN_RESET);
        TRACK_I2C_Delay();
    }
}

uint8_t TRACK_I2C_Read_Byte(unsigned char ack)
{
    unsigned char i, receive = 0;
    TRACK_SDA_Input_Mode();

    for (i = 0; i < 8; i++)
    {
        HAL_GPIO_WritePin(TRACK_I2C_SCL_GPIO_PORT, TRACK_I2C_SCL_PIN, GPIO_PIN_RESET);
        TRACK_I2C_Delay();
        HAL_GPIO_WritePin(TRACK_I2C_SCL_GPIO_PORT, TRACK_I2C_SCL_PIN, GPIO_PIN_SET);
        
        /* 等待数据稳定 (72MHz 下约 1~2us) */
        uint8_t wait = 20;
        while (wait--);
        
        receive <<= 1;
        if (HAL_GPIO_ReadPin(TRACK_I2C_SDA_GPIO_PORT, TRACK_I2C_SDA_PIN))
            receive |= 0x01;
        TRACK_I2C_Delay();
    }

    TRACK_SDA_Output_Mode();
    if (ack == 0) TRACK_I2C_NAck();
    else          TRACK_I2C_Ack();
    return receive;
}

/*==================== 高级读写接口 ====================*/
uint8_t TRACK_I2C_Write_One_Byte(uint8_t addr, uint8_t reg, uint8_t data)
{
    TRACK_I2C_Start();
    TRACK_I2C_Send_Byte((addr << 1) | TRACK_I2C_WRITE);
    if (TRACK_I2C_Wait_Ack()) { TRACK_I2C_Stop(); return 1; }
    TRACK_I2C_Send_Byte(reg);
    TRACK_I2C_Wait_Ack();
    TRACK_I2C_Send_Byte(data);
    TRACK_I2C_Wait_Ack();
    TRACK_I2C_Stop();
    return 0;
}

uint8_t TRACK_I2C_Read_One_Byte(uint8_t addr, uint8_t reg)
{
    uint8_t temp = 0;
    TRACK_I2C_Start();
    TRACK_I2C_Send_Byte((addr << 1) | TRACK_I2C_WRITE);
    TRACK_I2C_Wait_Ack();
    TRACK_I2C_Send_Byte(reg);
    TRACK_I2C_Wait_Ack();
    TRACK_I2C_Start();
    TRACK_I2C_Send_Byte((addr << 1) | TRACK_I2C_READ);
    TRACK_I2C_Wait_Ack();
    temp = TRACK_I2C_Read_Byte(0);
    TRACK_I2C_Stop();
    return temp;
}

void TRACK_I2C_Write_Buf(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    uint8_t i;
    TRACK_I2C_Start();
    TRACK_I2C_Send_Byte((addr << 1) | TRACK_I2C_WRITE);
    TRACK_I2C_Wait_Ack();
    TRACK_I2C_Send_Byte(reg);
    TRACK_I2C_Wait_Ack();
    for (i = 0; i < len; i++)
    {
        TRACK_I2C_Send_Byte(buf[i]);
        TRACK_I2C_Wait_Ack();
    }
    TRACK_I2C_Stop();
}

void TRACK_I2C_Read_Buf(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    TRACK_I2C_Start();
    TRACK_I2C_Send_Byte((addr << 1) | TRACK_I2C_WRITE);
    TRACK_I2C_Wait_Ack();
    TRACK_I2C_Send_Byte(reg);
    TRACK_I2C_Wait_Ack();
    TRACK_I2C_Start();
    TRACK_I2C_Send_Byte((addr << 1) | TRACK_I2C_READ);
    TRACK_I2C_Wait_Ack();
    while (len)
    {
        if (len == 1)
            *buf = TRACK_I2C_Read_Byte(0);
        else
            *buf = TRACK_I2C_Read_Byte(1);
        buf++;
        len--;
    }
    TRACK_I2C_Stop();
}

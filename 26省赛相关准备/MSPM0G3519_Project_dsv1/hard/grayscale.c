/**
 * @file    grayscale.c
 * @brief   8路灰度传感器 MUX 驱动实现
 * @note    参考 D:\!ziv\Nova\26省赛相关准备\八路灰度模块\源码\1.单片机数据读取\1.MSPM0G3507\Keil\BSP\grayscale_sensor.c
 */

#include "grayscale.h"
#include "board.h"   /* delay_us */

/* ── 内部: 选择 MUX 通道 (0-7) ── */
static void _select_channel(uint8_t channel)
{
    GRAY_AD0_WRITE((channel >> 0) & 0x01);
    GRAY_AD1_WRITE((channel >> 1) & 0x01);
    GRAY_AD2_WRITE((channel >> 2) & 0x01);
}

/* ── 初始化: AD0/AD1/AD2 推挽输出, OUT 输入上拉 ── */
void Grayscale_Init(void)
{
    /* AD0 — PA12, 推挽输出, 初始低电平 */
    DL_GPIO_initDigitalOutput(GRAY_AD0_IOMUX);
    DL_GPIO_clearPins(GRAY_AD0_PORT, GRAY_AD0_PIN);

    /* AD1 — PB23, 推挽输出, 初始低电平 */
    DL_GPIO_initDigitalOutput(GRAY_AD1_IOMUX);
    DL_GPIO_clearPins(GRAY_AD1_PORT, GRAY_AD1_PIN);

    /* AD2 — PB27, 推挽输出, 初始低电平 */
    DL_GPIO_initDigitalOutput(GRAY_AD2_IOMUX);
    DL_GPIO_clearPins(GRAY_AD2_PORT, GRAY_AD2_PIN);

    /* OUT — PB8, 数字输入 + 内部上拉 (高阻时默认高电平=白底) */
    DL_GPIO_initDigitalInputFeatures(GRAY_OUT_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}

/* ── 读取全部 8 路 ── */
void Grayscale_Read_All(uint8_t sensor[8])
{
    uint8_t i;
    for (i = 0; i < GRAYSCALE_CHANNELS; i++)
    {
        _select_channel(i);
        delay_us(50);                    /* 等待 MUX 输出稳定 */
        sensor[i] = GRAY_OUT_READ();     /* 0=黑线, 1=白底 */
    }
}

/* ── 读取单路 ── */
uint8_t Grayscale_Read_Single(uint8_t channel)
{
    if (channel >= GRAYSCALE_CHANNELS)
        return 0;

    _select_channel(channel);
    delay_us(50);
    return GRAY_OUT_READ();
}

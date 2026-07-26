/**
 * @file    track_gpio.c
 * @brief   6路 GPIO 循迹传感器 — 硬件层实现
 * @note    所有 6 路配置为数字输入 + 内部上拉, 全部在 GPIOB
 *          读取时 0=黑线(LOW) 1=白底(HIGH)
 */

#include "track_gpio.h"

void TrackGPIO_Init(void)
{
    /* 使能 GPIOB 电源 (所有 6 路均在 PORTB) */
    DL_GPIO_enablePower(GPIOB);

    /* PIN_0: PB6 — 最右 */
    DL_GPIO_initDigitalInputFeatures(TRACK_PIN_0_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    /* PIN_1: PB7 */
    DL_GPIO_initDigitalInputFeatures(TRACK_PIN_1_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    /* PIN_2: PB8 */
    DL_GPIO_initDigitalInputFeatures(TRACK_PIN_2_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    /* PIN_3: PB9 */
    DL_GPIO_initDigitalInputFeatures(TRACK_PIN_3_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    /* PIN_4: PB10 */
    DL_GPIO_initDigitalInputFeatures(TRACK_PIN_4_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    /* PIN_5: PB11 — 最左 */
    DL_GPIO_initDigitalInputFeatures(TRACK_PIN_5_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}

uint8_t TrackGPIO_Read_All(void)
{
    uint8_t status = 0;

    /* 从右到左逐位读取, bit0=最右(PB6) — 传感器拉低=0(黑线), 高阻+上拉=1(白底) */
    if (DL_GPIO_readPins(TRACK_PIN_0_PORT, TRACK_PIN_0_PIN)) status |= 0x01;  /* PB6 */
    if (DL_GPIO_readPins(TRACK_PIN_1_PORT, TRACK_PIN_1_PIN)) status |= 0x02;  /* PB7 */
    if (DL_GPIO_readPins(TRACK_PIN_2_PORT, TRACK_PIN_2_PIN)) status |= 0x04;  /* PB8 */
    if (DL_GPIO_readPins(TRACK_PIN_3_PORT, TRACK_PIN_3_PIN)) status |= 0x08;  /* PB9 */
    if (DL_GPIO_readPins(TRACK_PIN_4_PORT, TRACK_PIN_4_PIN)) status |= 0x10;  /* PB10 */
    if (DL_GPIO_readPins(TRACK_PIN_5_PORT, TRACK_PIN_5_PIN)) status |= 0x20;  /* PB11 */

    return status;
}

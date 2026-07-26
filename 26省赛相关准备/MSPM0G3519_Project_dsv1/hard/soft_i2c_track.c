/**
 * @file    soft_i2c_track.c
 * @brief   循迹传感器专用软件 I2C — 推挽+模式切换 (对齐 soft_i2c_simple 已验证方案)
 * @note    PA28(SDA) PA31(SCL), ~1us 延迟, 初始化时发9个SCL恢复总线
 */

#include "soft_i2c_track.h"
#include "ti_msp_dl_config.h"

#define DELAY() do { volatile uint16_t d = 267; while (d--); } while (0)

/*==================== SDA 模式切换 ====================*/

static void SDA_Out(void)
{
    DL_GPIO_initDigitalOutput(TRACK_I2C_SDA_IOMUX);
    DL_GPIO_enableOutput(TRACK_I2C_SDA_PORT, TRACK_I2C_SDA_PIN);
}

static void SDA_In(void)
{
    DL_GPIO_initDigitalInputFeatures(TRACK_I2C_SDA_IOMUX,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
}

/*==================== 总线恢复 ====================*/

static void BusRecover(void)
{
    SDA_In();  /* 释放 SDA */
    for (uint8_t i = 0; i < 9; i++) {
        DL_GPIO_setPins(TRACK_I2C_SCL_PORT, TRACK_I2C_SCL_PIN);
        DELAY();
        DL_GPIO_clearPins(TRACK_I2C_SCL_PORT, TRACK_I2C_SCL_PIN);
        DELAY();
    }
    /* STOP */
    SDA_Out();
    DL_GPIO_clearPins(TRACK_I2C_SDA_PORT, TRACK_I2C_SDA_PIN);
    DELAY();
    DL_GPIO_setPins(TRACK_I2C_SCL_PORT, TRACK_I2C_SCL_PIN);
    DELAY();
    DL_GPIO_setPins(TRACK_I2C_SDA_PORT, TRACK_I2C_SDA_PIN);
    DELAY();
}

/*==================== 初始化 ====================*/

void Track_I2C_Init(void)
{
    DL_GPIO_enablePower(TRACK_I2C_SCL_PORT);
    DL_GPIO_enablePower(TRACK_I2C_SDA_PORT);

    DL_GPIO_initDigitalOutput(TRACK_I2C_SCL_IOMUX);
    DL_GPIO_enableOutput(TRACK_I2C_SCL_PORT, TRACK_I2C_SCL_PIN);

    SDA_Out();
    DL_GPIO_setPins(TRACK_I2C_SCL_PORT, TRACK_I2C_SCL_PIN);
    DL_GPIO_setPins(TRACK_I2C_SDA_PORT, TRACK_I2C_SDA_PIN);

    BusRecover();
}

/*==================== 基础时序 ====================*/

void Track_I2C_Start(void)
{
    SDA_Out();
    DL_GPIO_setPins(TRACK_I2C_SDA_PORT, TRACK_I2C_SDA_PIN);
    DL_GPIO_setPins(TRACK_I2C_SCL_PORT, TRACK_I2C_SCL_PIN);
    DELAY();
    DL_GPIO_clearPins(TRACK_I2C_SDA_PORT, TRACK_I2C_SDA_PIN);
    DELAY();
    DL_GPIO_clearPins(TRACK_I2C_SCL_PORT, TRACK_I2C_SCL_PIN);
    DELAY();
}

void Track_I2C_Stop(void)
{
    SDA_Out();
    DL_GPIO_clearPins(TRACK_I2C_SCL_PORT, TRACK_I2C_SCL_PIN);
    DL_GPIO_clearPins(TRACK_I2C_SDA_PORT, TRACK_I2C_SDA_PIN);
    DELAY();
    DL_GPIO_setPins(TRACK_I2C_SCL_PORT, TRACK_I2C_SCL_PIN);
    DELAY();
    DL_GPIO_setPins(TRACK_I2C_SDA_PORT, TRACK_I2C_SDA_PIN);
    DELAY();
}

uint8_t Track_I2C_WaitAck(void)
{
    uint8_t err = 0;
    SDA_In();
    DELAY();
    DL_GPIO_setPins(TRACK_I2C_SCL_PORT, TRACK_I2C_SCL_PIN);
    DELAY();

    while (DL_GPIO_readPins(TRACK_I2C_SDA_PORT, TRACK_I2C_SDA_PIN)) {
        if (++err > 250) {
            Track_I2C_Stop();
            return 1;
        }
    }
    DL_GPIO_clearPins(TRACK_I2C_SCL_PORT, TRACK_I2C_SCL_PIN);
    DELAY();
    SDA_Out();
    return 0;
}

void Track_I2C_Ack(void)
{
    SDA_Out();
    DL_GPIO_clearPins(TRACK_I2C_SCL_PORT, TRACK_I2C_SCL_PIN);
    DL_GPIO_clearPins(TRACK_I2C_SDA_PORT, TRACK_I2C_SDA_PIN);
    DELAY();
    DL_GPIO_setPins(TRACK_I2C_SCL_PORT, TRACK_I2C_SCL_PIN);
    DELAY();
    DL_GPIO_clearPins(TRACK_I2C_SCL_PORT, TRACK_I2C_SCL_PIN);
    DELAY();
}

void Track_I2C_NAck(void)
{
    SDA_Out();
    DL_GPIO_clearPins(TRACK_I2C_SCL_PORT, TRACK_I2C_SCL_PIN);
    DL_GPIO_setPins(TRACK_I2C_SDA_PORT, TRACK_I2C_SDA_PIN);
    DELAY();
    DL_GPIO_setPins(TRACK_I2C_SCL_PORT, TRACK_I2C_SCL_PIN);
    DELAY();
    DL_GPIO_clearPins(TRACK_I2C_SCL_PORT, TRACK_I2C_SCL_PIN);
    DELAY();
}

void Track_I2C_SendByte(uint8_t data)
{
    uint8_t i;
    SDA_Out();
    DL_GPIO_clearPins(TRACK_I2C_SCL_PORT, TRACK_I2C_SCL_PIN);
    for (i = 0; i < 8; i++) {
        if (data & 0x80)
            DL_GPIO_setPins(TRACK_I2C_SDA_PORT, TRACK_I2C_SDA_PIN);
        else
            DL_GPIO_clearPins(TRACK_I2C_SDA_PORT, TRACK_I2C_SDA_PIN);
        data <<= 1;
        DELAY();
        DL_GPIO_setPins(TRACK_I2C_SCL_PORT, TRACK_I2C_SCL_PIN);
        DELAY();
        DL_GPIO_clearPins(TRACK_I2C_SCL_PORT, TRACK_I2C_SCL_PIN);
        DELAY();
    }
}

uint8_t Track_I2C_ReadByte(uint8_t ack)
{
    uint8_t i, val = 0;
    SDA_In();
    for (i = 0; i < 8; i++) {
        DL_GPIO_clearPins(TRACK_I2C_SCL_PORT, TRACK_I2C_SCL_PIN);
        DELAY();
        DL_GPIO_setPins(TRACK_I2C_SCL_PORT, TRACK_I2C_SCL_PIN);
        DELAY();
        val <<= 1;
        if (DL_GPIO_readPins(TRACK_I2C_SDA_PORT, TRACK_I2C_SDA_PIN))
            val |= 0x01;
        DELAY();
    }
    SDA_Out();
    if (ack) Track_I2C_Ack();
    else     Track_I2C_NAck();
    return val;
}

/*==================== 高级接口 ====================*/

uint8_t Track_I2C_WriteOneByte(uint8_t addr, uint8_t reg, uint8_t data)
{
    Track_I2C_Start();
    Track_I2C_SendByte((addr << 1) | TRACK_I2C_WRITE);
    if (Track_I2C_WaitAck()) { Track_I2C_Stop(); return 1; }
    Track_I2C_SendByte(reg);
    Track_I2C_WaitAck();
    Track_I2C_SendByte(data);
    Track_I2C_WaitAck();
    Track_I2C_Stop();
    return 0;
}

uint8_t Track_I2C_ReadOneByte(uint8_t addr, uint8_t reg)
{
    uint8_t val = 0;
    Track_I2C_Start();
    Track_I2C_SendByte((addr << 1) | TRACK_I2C_WRITE);
    if (Track_I2C_WaitAck()) { Track_I2C_Stop(); return 0; }
    Track_I2C_SendByte(reg);
    if (Track_I2C_WaitAck()) { Track_I2C_Stop(); return 0; }
    Track_I2C_Start();
    Track_I2C_SendByte((addr << 1) | TRACK_I2C_READ);
    if (Track_I2C_WaitAck()) { Track_I2C_Stop(); return 0; }
    val = Track_I2C_ReadByte(0);
    Track_I2C_Stop();
    return val;
}

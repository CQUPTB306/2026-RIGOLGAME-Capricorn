/**
 * @file    soft_i2c_track.c
 * @brief   软件 I2C — MSPM0, PA0=SDA(开漏), PA1=SCL
 */

#include "soft_i2c_track.h"
#include "board.h"

/* 位时钟延时: 50µs≈6.7kHz 太慢, 控制循环被拖到 ~16Hz;
 * 5µs≈66kHz (标准 100kHz 内), 读取 ~0.6ms, 刹车检查才能跟上 */
#define TRACK_I2C_BIT_DELAY_US  50
static void _delay(void) { delay_us(TRACK_I2C_BIT_DELAY_US); }

/* ── SDA 状态 ──
 *   idle: PC=1(输出), DOUT=0, 关使能 → 上拉=高, 开使能=拉低
 *   out:  开使能 → 拉低
 *   in:   关使能 → 释放(上拉)
 *   read: PC=0(输入), 正确读 DIN → 用完回 idle
 */
static void _sda_idle(void) {
    DL_GPIO_initDigitalOutputFeatures(IOMUX_PINCM1,
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, 0);
    DL_GPIO_clearPins(TRACK_I2C_SDA_PORT, TRACK_I2C_SDA_PIN);
    DL_GPIO_disableOutput(TRACK_I2C_SDA_PORT, TRACK_I2C_SDA_PIN);
}
static void _sda_out(void) { DL_GPIO_enableOutput (TRACK_I2C_SDA_PORT, TRACK_I2C_SDA_PIN); }
static void _sda_in (void) { DL_GPIO_disableOutput(TRACK_I2C_SDA_PORT, TRACK_I2C_SDA_PIN); }
static void _sda_read(void) {
    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM1,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP, 0, 0);
}

#define SDA_H()  _sda_in()
#define SDA_L()  do { DL_GPIO_clearPins(TRACK_I2C_SDA_PORT, TRACK_I2C_SDA_PIN); _sda_out(); } while(0)
#define SCL_H()  DL_GPIO_setPins(TRACK_I2C_SCL_PORT, TRACK_I2C_SCL_PIN)
#define SCL_L()  DL_GPIO_clearPins(TRACK_I2C_SCL_PORT, TRACK_I2C_SCL_PIN)
#define SDA_RD() (!!DL_GPIO_readPins(TRACK_I2C_SDA_PORT, TRACK_I2C_SDA_PIN))

/* ==================== 基础时序 ==================== */
void TRACK_I2C_Start(void)
{
    SDA_H(); SCL_H(); _delay();
    _sda_out(); SDA_L(); _delay();
    SCL_L();
}
void TRACK_I2C_Stop(void)
{
    _sda_out(); SDA_L(); _delay();
    SCL_H(); _delay();
    SDA_H(); _delay();
}

uint8_t TRACK_I2C_Wait_Ack(void)
{
    uint8_t to = 0;
    _sda_read();                          /* 切输入读 DIN */
    SCL_L(); _delay();
    SCL_H(); _delay();
    while (SDA_RD()) { if (++to > 250) { _sda_idle(); TRACK_I2C_Stop(); return 1; } }
    SCL_L();
    _sda_idle();                          /* 恢复输出模式 */
    return 0;
}

static void _ack(void)  { _sda_out(); SDA_L(); _delay(); SCL_H(); _delay(); SCL_L(); }
static void _nack(void) { SDA_H(); SCL_L(); _delay(); SCL_H(); _delay(); SCL_L(); }

void TRACK_I2C_Send_Byte(uint8_t txd)
{
    uint8_t t;
    for (t = 0; t < 8; t++) {
        if (txd & 0x80) SDA_H(); else SDA_L();
        txd <<= 1;
        _delay(); SCL_H(); _delay();
        SCL_L(); _delay();
    }
}

uint8_t TRACK_I2C_Read_Byte(unsigned char ack)
{
    uint8_t i, recv = 0;
    _sda_read();                          /* 切输入读 DIN */
    for (i = 0; i < 8; i++) {
        SCL_L(); _delay();
        SCL_H(); _delay();
        recv <<= 1;
        if (SDA_RD()) recv |= 0x01;
        _delay();
    }
    _sda_idle();                          /* 恢复输出模式 */
    if (ack) _ack(); else _nack();
    return recv;
}

/* ==================== 高级接口 ==================== */
uint8_t TRACK_I2C_Read_One_Byte(uint8_t addr, uint8_t reg)
{
    uint8_t temp;
    TRACK_I2C_Start();
    TRACK_I2C_Send_Byte((addr << 1) | TRACK_I2C_WRITE);
    if (TRACK_I2C_Wait_Ack()) { TRACK_I2C_Stop(); return 0xFF; }
    TRACK_I2C_Send_Byte(reg);
    if (TRACK_I2C_Wait_Ack()) { TRACK_I2C_Stop(); return 0xFF; }
    TRACK_I2C_Start();
    TRACK_I2C_Send_Byte((addr << 1) | TRACK_I2C_READ);
    if (TRACK_I2C_Wait_Ack()) { TRACK_I2C_Stop(); return 0xFF; }
    temp = TRACK_I2C_Read_Byte(0);
    TRACK_I2C_Stop();
    return temp;
}

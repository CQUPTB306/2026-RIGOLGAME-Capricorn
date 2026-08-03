/**
 * @file    soft_i2c_simple.c
 * @brief   软件 I2C 驱动 (MSPM0G3507) — 多总线版本
 * @note    GPIO API 映射 (STM32 HAL -> MSPM0 DL):
 *          HAL_GPIO_WritePin(p, pin, SET)   -> DL_GPIO_setPins(p, pin)
 *          HAL_GPIO_WritePin(p, pin, RESET) -> DL_GPIO_clearPins(p, pin)
 *          HAL_GPIO_ReadPin(p, pin)         -> DL_GPIO_readPins(p, pin) & pin
 */

#include "soft_i2c_simple.h"

/*-------------------- 内部辅助 --------------------*/

/** @brief 默认延时 (~5us @80MHz, 对应 100kHz I2C) */
static void SoftI2C_DefaultDelay(void)
{
    volatile uint16_t i = 400;  /* volatile 防止编译器优化掉延时 */
    while (i--);
}

/** @brief 循迹 I2C 延时 — 无延迟跑满 GPIO 速度 (对齐 trace_i2c) */
static void SoftI2C_TrackDelay(void)
{
}

/*=================== SDA 模式切换 ====================*/

static void SDA_OutputMode(SoftI2C_Obj *obj)
{
    /* 强驱动 + 上拉, DOUT=0 → 开使能拉低, 关使能释放 */
    DL_GPIO_initDigitalOutputFeatures(obj->sda_iomux,
        DL_GPIO_DRIVE_STRENGTH_HIGH, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, 0);
    DL_GPIO_clearPins(obj->sda_port, obj->sda_pin);
    DL_GPIO_enableOutput(obj->sda_port, obj->sda_pin);
}

static void SDA_InputMode(SoftI2C_Obj *obj)
{
    DL_GPIO_initDigitalInputFeatures(obj->sda_iomux,
                                     DL_GPIO_INVERSION_DISABLE,
                                     DL_GPIO_RESISTOR_PULL_UP,
                                     DL_GPIO_HYSTERESIS_DISABLE,
                                     DL_GPIO_WAKEUP_DISABLE);
}

/*====================== 初始化 ======================*/

void SoftI2C_Init(SoftI2C_Obj *obj)
{
    DL_GPIO_enablePower(obj->scl_port);
    DL_GPIO_enablePower(obj->sda_port);

    /* SCL: 推挽 + 强驱动 */
    DL_GPIO_initDigitalOutputFeatures(obj->scl_iomux,
        DL_GPIO_DRIVE_STRENGTH_HIGH, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, 0);
    DL_GPIO_enableOutput(obj->scl_port, obj->scl_pin);

    /* SDA: 开漏风格 (DOUT=0 + 关使能 = 上拉高; 开使能 = 拉低) */
    DL_GPIO_initDigitalOutputFeatures(obj->sda_iomux,
        DL_GPIO_DRIVE_STRENGTH_HIGH, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, 0);
    DL_GPIO_clearPins(obj->sda_port, obj->sda_pin);
    DL_GPIO_disableOutput(obj->sda_port, obj->sda_pin);

    DL_GPIO_setPins(obj->scl_port, obj->scl_pin);

    if (obj->delay == NULL)
        obj->delay = SoftI2C_DefaultDelay;
}

/*====================== 基础时序 ======================*/

void SoftI2C_Start(SoftI2C_Obj *obj)
{
    /* 先将 SCL 拉低, 再操作 SDA, 防止虚假 START/STOP 脉冲 */
    DL_GPIO_clearPins(obj->scl_port, obj->scl_pin);
    SDA_OutputMode(obj);
    DL_GPIO_setPins(obj->sda_port, obj->sda_pin);     /* SDA HIGH (SCL=LOW, 安全) */
    obj->delay();
    DL_GPIO_setPins(obj->scl_port, obj->scl_pin);     /* SCL HIGH */
    obj->delay();
    DL_GPIO_clearPins(obj->sda_port, obj->sda_pin);   /* SDA LOW → START */
    obj->delay();
    DL_GPIO_clearPins(obj->scl_port, obj->scl_pin);   /* SCL LOW */
    obj->delay();
}

void SoftI2C_Stop(SoftI2C_Obj *obj)
{
    DL_GPIO_clearPins(obj->scl_port, obj->scl_pin);   /* SCL LOW first */
    SDA_OutputMode(obj);
    DL_GPIO_clearPins(obj->sda_port, obj->sda_pin);   /* SDA LOW (SCL=LOW, 安全) */
    obj->delay();
    DL_GPIO_setPins(obj->scl_port, obj->scl_pin);     /* SCL HIGH */
    obj->delay();
    DL_GPIO_setPins(obj->sda_port, obj->sda_pin);     /* SDA HIGH → STOP */
    obj->delay();
}

uint8_t SoftI2C_WaitAck(SoftI2C_Obj *obj)
{
    uint8_t ucErrTime = 0;

    SDA_InputMode(obj);
    DL_GPIO_setPins(obj->sda_port, obj->sda_pin);
    obj->delay();
    DL_GPIO_setPins(obj->scl_port, obj->scl_pin);
    obj->delay();

    while (DL_GPIO_readPins(obj->sda_port, obj->sda_pin))
    {
        if (++ucErrTime > 250)
        {
            SoftI2C_Stop(obj);
            return 1;
        }
    }

    DL_GPIO_clearPins(obj->scl_port, obj->scl_pin);
    obj->delay();
    SDA_OutputMode(obj);
    return 0;
}

void SoftI2C_Ack(SoftI2C_Obj *obj)
{
    SDA_OutputMode(obj);
    DL_GPIO_clearPins(obj->scl_port, obj->scl_pin);
    DL_GPIO_clearPins(obj->sda_port, obj->sda_pin);
    obj->delay();
    DL_GPIO_setPins(obj->scl_port, obj->scl_pin);
    obj->delay();
    DL_GPIO_clearPins(obj->scl_port, obj->scl_pin);
    obj->delay();
}

void SoftI2C_NAck(SoftI2C_Obj *obj)
{
    SDA_OutputMode(obj);
    DL_GPIO_clearPins(obj->scl_port, obj->scl_pin);
    DL_GPIO_setPins(obj->sda_port, obj->sda_pin);
    obj->delay();
    DL_GPIO_setPins(obj->scl_port, obj->scl_pin);
    obj->delay();
    DL_GPIO_clearPins(obj->scl_port, obj->scl_pin);
    obj->delay();
}

void SoftI2C_SendByte(SoftI2C_Obj *obj, uint8_t data)
{
    uint8_t i;
    SDA_OutputMode(obj);
    DL_GPIO_clearPins(obj->scl_port, obj->scl_pin);
    for (i = 0; i < 8; i++)
    {
        if (data & 0x80)
            DL_GPIO_setPins(obj->sda_port, obj->sda_pin);
        else
            DL_GPIO_clearPins(obj->sda_port, obj->sda_pin);
        data <<= 1;
        obj->delay();
        DL_GPIO_setPins(obj->scl_port, obj->scl_pin);
        obj->delay();
        DL_GPIO_clearPins(obj->scl_port, obj->scl_pin);
        obj->delay();
    }
}

uint8_t SoftI2C_ReadByte(SoftI2C_Obj *obj, uint8_t ack)
{
    uint8_t i, receive = 0;
    SDA_InputMode(obj);

    for (i = 0; i < 8; i++)
    {
        DL_GPIO_clearPins(obj->scl_port, obj->scl_pin);
        obj->delay();
        DL_GPIO_setPins(obj->scl_port, obj->scl_pin);
        receive <<= 1;
        if (DL_GPIO_readPins(obj->sda_port, obj->sda_pin))
            receive |= 0x01;
        obj->delay();
    }

    SDA_OutputMode(obj);
    if (ack)
        SoftI2C_Ack(obj);
    else
        SoftI2C_NAck(obj);
    return receive;
}

/*=================== 高级读写接口 ====================*/

uint8_t SoftI2C_WriteOneByte(SoftI2C_Obj *obj, uint8_t dev_addr, uint8_t reg, uint8_t data)
{
    SoftI2C_Start(obj);
    SoftI2C_SendByte(obj, (dev_addr << 1) | SOFT_I2C_WRITE);
    if (SoftI2C_WaitAck(obj)) { SoftI2C_Stop(obj); return 1; }
    SoftI2C_SendByte(obj, reg);
    if (SoftI2C_WaitAck(obj)) { SoftI2C_Stop(obj); return 1; }
    SoftI2C_SendByte(obj, data);
    if (SoftI2C_WaitAck(obj)) { SoftI2C_Stop(obj); return 1; }
    SoftI2C_Stop(obj);
    return 0;
}

uint8_t SoftI2C_ReadOneByte(SoftI2C_Obj *obj, uint8_t dev_addr, uint8_t reg)
{
    uint8_t temp = 0;
    SoftI2C_Start(obj);
    SoftI2C_SendByte(obj, (dev_addr << 1) | SOFT_I2C_WRITE);
    SoftI2C_WaitAck(obj);
    SoftI2C_SendByte(obj, reg);
    SoftI2C_WaitAck(obj);
    SoftI2C_Start(obj);
    SoftI2C_SendByte(obj, (dev_addr << 1) | SOFT_I2C_READ);
    SoftI2C_WaitAck(obj);
    temp = SoftI2C_ReadByte(obj, 0);
    SoftI2C_Stop(obj);
    return temp;
}

void SoftI2C_WriteBuf(SoftI2C_Obj *obj, uint8_t dev_addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    SoftI2C_Start(obj);
    SoftI2C_SendByte(obj, (dev_addr << 1) | SOFT_I2C_WRITE);
    SoftI2C_WaitAck(obj);
    SoftI2C_SendByte(obj, reg);
    SoftI2C_WaitAck(obj);
    while (len--)
    {
        SoftI2C_SendByte(obj, *buf++);
        SoftI2C_WaitAck(obj);
    }
    SoftI2C_Stop(obj);
}

void SoftI2C_ReadBuf(SoftI2C_Obj *obj, uint8_t dev_addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    SoftI2C_Start(obj);
    SoftI2C_SendByte(obj, (dev_addr << 1) | SOFT_I2C_WRITE);
    SoftI2C_WaitAck(obj);
    SoftI2C_SendByte(obj, reg);
    SoftI2C_WaitAck(obj);
    SoftI2C_Start(obj);
    SoftI2C_SendByte(obj, (dev_addr << 1) | SOFT_I2C_READ);
    SoftI2C_WaitAck(obj);
    while (len)
    {
        if (len == 1)
            *buf = SoftI2C_ReadByte(obj, 0);
        else
            *buf = SoftI2C_ReadByte(obj, 1);
        buf++;
        len--;
    }
    SoftI2C_Stop(obj);
}

/* delay_ms 由 board.c 提供 (SysTick 硬件延时) */

/*=================== 三个总线实例 ====================*/

/** @brief OLED I2C 总线: PA15(SDA=PINCM37), PA16(SCL=PINCM38)
 *         OLED: 0x3C
 *         ⚠️ PA15/PA16 与 TM1637 数码管共享物理引脚 (不可同时使用) */
SoftI2C_Obj i2c_pca9685 = {
    .scl_port  = GPIOA,
    .scl_pin   = DL_GPIO_PIN_16,
    .scl_iomux = IOMUX_PINCM38,
    .sda_port  = GPIOA,
    .sda_pin   = DL_GPIO_PIN_15,
    .sda_iomux = IOMUX_PINCM37,
    .delay     = NULL
};

/** @brief MPU6050 I2C 总线: PA28(SDA=PINCM3), PA31(SCL=PINCM6) */
SoftI2C_Obj i2c_mpu6050 = {
    .scl_port  = GPIOA,
    .scl_pin   = DL_GPIO_PIN_31,
    .scl_iomux = IOMUX_PINCM6,      /* PA31 -> PINCM6 */
    .sda_port  = GPIOA,
    .sda_pin   = DL_GPIO_PIN_28,
    .sda_iomux = IOMUX_PINCM3,      /* PA28 -> PINCM3 */
    .delay     = NULL
};

/** @brief 循迹传感器 I2C 总线: PB03(SDA=PINCM16), PB02(SCL=PINCM15)
 *         I2C1_SDA + I2C1_SCL 硬件引脚, 零延迟对齐 trace_i2c */
SoftI2C_Obj i2c_track = {
    .scl_port  = GPIOB,
    .scl_pin   = DL_GPIO_PIN_2,
    .scl_iomux = IOMUX_PINCM15,     /* PB02 -> PINCM15 */
    .sda_port  = GPIOB,
    .sda_pin   = DL_GPIO_PIN_3,
    .sda_iomux = IOMUX_PINCM16,     /* PB03 -> PINCM16 */
    .delay     = SoftI2C_TrackDelay  /* 零延迟, 对齐 trace_i2c */
};

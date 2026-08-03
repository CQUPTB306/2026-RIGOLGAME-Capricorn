/**
 * @file    tm1637.c
 * @brief   TM1637 数码管驱动 — MSPM0, PA15=DIO, PA16=CLK
 * @note    两线串行协议 (类 I2C, 但数据 LSB 先发)
 *
 *          时序 (参考 datasheet v2.1):
 *          - CLK 高时 DIO 必须稳定
 *          - Start: CLK=H, DIO H→L
 *          - Stop:  CLK=H, DIO L→H
 *          - 写字节: LSB 先, 每 bit 在 CLK 低时放数据, CLK 上升沿锁存
 *          - ACK: 第 8 个 CLK 下降沿芯片拉低 DIO → 第 9 个 CLK 释放
 *
 *          GPIO 双向 DIO 实现 (参考 soft_i2c_track.c):
 *          - idle: DOUT=0, 关使能 → 上拉电阻拉高 (释放总线)
 *          - 写 0: 开使能 → DOUT=0 拉低
 *          - 写 1: 关使能 → 释放, 上拉电阻拉高
 *          - 读:   切输入模式 → 读 DIN
 */

#include "tm1637.h"
#include "board.h"

/* ── 段码查找表 ──
 *   bit0=a, bit1=b, bit2=c, bit3=d, bit4=e, bit5=f, bit6=g, bit7=dp
 *   共阳数码管: 写 1 → SEG 拉低 → 段亮
 *   索引: 0-9=数字, 10-15=A~F, 16='-', 17=' ', 18='_' */
const uint8_t TM1637_SEG_CODE[] = {
    0x3F,   /* 0: a b c d e f */
    0x06,   /* 1: b c */
    0x5B,   /* 2: a b d e g */
    0x4F,   /* 3: a b c d g */
    0x66,   /* 4: b c f g */
    0x6D,   /* 5: a c d f g */
    0x7D,   /* 6: a c d e f g */
    0x07,   /* 7: a b c */
    0x7F,   /* 8: a b c d e f g */
    0x6F,   /* 9: a b c d f g */
    0x77,   /* A: a b c e f g */
    0x7C,   /* b: c d e f g */
    0x39,   /* C: a d e f */
    0x5E,   /* d: b c d e g */
    0x79,   /* E: a d e f g */
    0x71,   /* F: a e f g */
    0x40,   /* -: g (中横) */
    0x00,   /* ' ': 全灭 */
    0x08,   /* _: d (下横) */
};

/* ── 内部延时 (约 50us @80MHz) ── */
#define TM_DELAY()   delay_us(50)

/* 当前亮度 (0~7), 用于写数据后恢复亮度 */
static uint8_t _current_brightness = TM1637_BRIGHTNESS_MAX;

/* ═══════════════════════════════════════════════════════════════
 *  GPIO 原子操作 — DIO 双向
 * ═══════════════════════════════════════════════════════════════ */

/* DIO 释放 → 上拉电阻拉高 (输出关, 开漏) */
static void _dio_release(void) {
    DL_GPIO_disableOutput(TM1637_DIO_PORT, TM1637_DIO_PIN);
}

/* DIO 拉低 (使能输出, DOUT=0) */
static void _dio_low(void) {
    DL_GPIO_clearPins(TM1637_DIO_PORT, TM1637_DIO_PIN);
    DL_GPIO_enableOutput(TM1637_DIO_PORT, TM1637_DIO_PIN);
}

/* 设置 DIO 输出电平 (1=释放, 0=拉低) */
static void _dio_write(uint8_t val) {
    if (val) _dio_release();
    else     _dio_low();
}

/* 读 DIO 输入电平 */
static uint8_t _dio_read(void) {
    return !!DL_GPIO_readPins(TM1637_DIO_PORT, TM1637_DIO_PIN);
}

/* CLK 操作 (推挽输出) */
#define _clk_high()  DL_GPIO_setPins(TM1637_CLK_PORT, TM1637_CLK_PIN)
#define _clk_low()   DL_GPIO_clearPins(TM1637_CLK_PORT, TM1637_CLK_PIN)

/* ═══════════════════════════════════════════════════════════════
 *  TM1637 协议层
 * ═══════════════════════════════════════════════════════════════ */

/**
 * @brief  通信起始条件: CLK=H 时 DIO H→L
 */
static void _tm1637_start(void)
{
    _clk_high();
    _dio_release();                     /* DIO = H */
    TM_DELAY();
    _dio_low();                         /* DIO → L */
    TM_DELAY();
    _clk_low();
}

/**
 * @brief  通信停止条件: CLK=H 时 DIO L→H
 */
static void _tm1637_stop(void)
{
    _clk_low();
    _dio_low();                         /* 准备: DIO = L */
    TM_DELAY();
    _clk_high();
    TM_DELAY();
    _dio_release();                     /* DIO → H */
    TM_DELAY();
}

/**
 * @brief  等待 ACK — 第 8 个时钟下降沿后芯片拉低 DIO
 * @return 0=ACK 成功, 1=超时/无应答
 */
static uint8_t _tm1637_wait_ack(void)
{
    uint16_t timeout = 0;

    _dio_release();                     /* 释放 DIO 给芯片应答 */

    /* 切换到输入模式读 DIO */
    DL_GPIO_initDigitalInputFeatures(TM1637_DIO_IOMUX,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);

    /*
     *  ACK 时序 (参考 datasheet v2.1 和 8051 参考代码):
     *  写完 8bit 数据后 CLK=H, 然后:
     *  (1) CLK→L  ← 第 8 个下降沿 → 芯片拉低 DIO
     *  (2) 等待 DIO=L  (ACK 应答)
     *  (3) CLK→H  ← 第 9 个时钟 → 芯片释放 DIO
     *  (4) CLK→L  ← 准备下个字节
     */
    _clk_low();                         /* (1) 第 8 个下降沿: 触发 ACK */
    TM_DELAY();

    /* (2) 等待芯片拉低 DIO */
    while (_dio_read()) {
        if (++timeout > 5000) {
            /* 超时: 恢复 DIO 为输出+释放模式 */
            DL_GPIO_initDigitalOutputFeatures(TM1637_DIO_IOMUX,
                DL_GPIO_DRIVE_STRENGTH_HIGH, DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR_PULL_UP, 0);
            DL_GPIO_clearPins(TM1637_DIO_PORT, TM1637_DIO_PIN);
            DL_GPIO_disableOutput(TM1637_DIO_PORT, TM1637_DIO_PIN);
            _tm1637_stop();
            return 1;
        }
    }

    _clk_high();                        /* (3) 第 9 个时钟: 芯片释放 DIO */
    TM_DELAY();
    _clk_low();                         /* (4) 准备下个字节 */

    /* 恢复 DIO 为输出+释放模式 */
    DL_GPIO_initDigitalOutputFeatures(TM1637_DIO_IOMUX,
        DL_GPIO_DRIVE_STRENGTH_HIGH, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, 0);
    DL_GPIO_clearPins(TM1637_DIO_PORT, TM1637_DIO_PIN);
    DL_GPIO_disableOutput(TM1637_DIO_PORT, TM1637_DIO_PIN);

    return 0;
}

/**
 * @brief  写一个字节 (LSB 先发)
 * @param  data  待发送字节
 */
static void _tm1637_write_byte(uint8_t data)
{
    uint8_t i;

    for (i = 0; i < 8; i++) {
        _clk_low();
        _dio_write(data & 0x01);        /* LSB 先发 */
        TM_DELAY();
        _clk_high();
        TM_DELAY();
        data >>= 1;
    }
}

/* ═══════════════════════════════════════════════════════════════
 *  TM1637 命令层
 * ═══════════════════════════════════════════════════════════════ */

/**
 * @brief  发送指令+数据块 (自动地址递增模式)
 * @param  cmd     指令字节 (0x40 或 0x44)
 * @param  addr    起始地址 (0xC0~0xC5)
 * @param  data    数据缓冲区
 * @param  len     数据长度
 * @return 0=成功, 1=ACK 失败
 */
static uint8_t _tm1637_send_data(uint8_t cmd, uint8_t addr,
                                  const uint8_t *data, uint8_t len)
{
    uint8_t i;

    /* 1. 发送数据命令 */
    _tm1637_start();
    _tm1637_write_byte(cmd);
    if (_tm1637_wait_ack()) return 1;
    _tm1637_stop();

    /* 2. 发送地址 + 连续数据 */
    _tm1637_start();
    _tm1637_write_byte(addr);
    if (_tm1637_wait_ack()) return 1;

    for (i = 0; i < len; i++) {
        _tm1637_write_byte(data[i]);
        if (_tm1637_wait_ack()) return 1;
    }
    _tm1637_stop();

    /* 3. 发送显示控制 (开显示+当前亮度) */
    _tm1637_start();
    _tm1637_write_byte(TM1637_DISPLAY_ON | _current_brightness);
    if (_tm1637_wait_ack()) return 1;
    _tm1637_stop();

    return 0;
}

/* ═══════════════════════════════════════════════════════════════
 *  API 实现
 * ═══════════════════════════════════════════════════════════════ */

/**
 * @brief  TM1637 初始化
 * @note   配置 PA0(DIO) 开漏+强驱动+上拉, PA1(CLK) 推挽+强驱动
 *         HIGH 驱动强度提升 3.3V→5V 信号电平
 */
void TM1637_Init(void)
{
    /* CLK = PA1: 推挽输出+强驱动, 初始低电平 */
    DL_GPIO_initDigitalOutputFeatures(TM1637_CLK_IOMUX,
        DL_GPIO_DRIVE_STRENGTH_HIGH, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, 0);
    DL_GPIO_clearPins(TM1637_CLK_PORT, TM1637_CLK_PIN);
    DL_GPIO_enableOutput(TM1637_CLK_PORT, TM1637_CLK_PIN);

    /* DIO = PA0: 开漏+强驱动+上拉, DOUT=0, 关使能 (释放/高电平) */
    DL_GPIO_initDigitalOutputFeatures(TM1637_DIO_IOMUX,
        DL_GPIO_DRIVE_STRENGTH_HIGH, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, 0);
    DL_GPIO_clearPins(TM1637_DIO_PORT, TM1637_DIO_PIN);
    DL_GPIO_disableOutput(TM1637_DIO_PORT, TM1637_DIO_PIN);

    /* 上电稳定延时 */
    delay_ms(10);

    /* 全部熄灭 — 发送 6 字节 0x00 */
    TM1637_Clear();
}

void TM1637_SetBrightness(uint8_t level)
{
    if (level > TM1637_BRIGHTNESS_MAX) level = TM1637_BRIGHTNESS_MAX;
    _current_brightness = level;

    _tm1637_start();
    _tm1637_write_byte(TM1637_DISPLAY_ON | (level & TM1637_BRIGHTNESS_MASK));
    _tm1637_wait_ack();
    _tm1637_stop();
}

void TM1637_WriteAll(const uint8_t data[6])
{
    /* 自动地址递增模式: 0x40 + 地址 0xC0 + 6 字节 */
    _tm1637_send_data(TM1637_DATA_AUTO, TM1637_ADDR_BASE, data, 6);
}

void TM1637_Clear(void)
{
    uint8_t blank[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    TM1637_WriteAll(blank);
}

/**
 * @brief  显示无符号整数 (右对齐, 6 位)
 * @param  num  0~999999
 */
void TM1637_ShowNum(uint32_t num)
{
    uint8_t buf[6];
    uint8_t i;
    uint8_t show = 0;   /* 是否遇到第一个非零数字 */

    if (num > 999999) num = 999999;

    for (i = 0; i < 6; i++) {
        uint32_t div = 1;
        uint8_t j;
        for (j = 0; j < (5 - i); j++) div *= 10;

        uint8_t digit = (uint8_t)(num / div);
        num %= div;

        if (digit > 0 || i == 5) show = 1;  /* 最后一位总是显示 */

        buf[i] = show ? TM1637_SEG_CODE[digit] : TM1637_SEG_CODE[TM1637_SEG_SPACE];
    }

    TM1637_WriteAll(buf);
}

/**
 * @brief  显示分:秒格式 (MM.SS), 右对齐占 4 位
 *         第 2 位显示小数点作为分隔符
 */
void TM1637_ShowMinSec(uint8_t minutes, uint8_t seconds)
{
    uint8_t buf[6];

    if (minutes > 99)  minutes = 99;
    if (seconds > 59)  seconds = 59;

    /* 空 空 M M . S S → 左对齐或右对齐 */
    /* 格式: "  M  M.S  S" — DP 在 digit 2 (从左数第 3 位) */
    buf[0] = (minutes >= 10) ? TM1637_SEG_CODE[minutes / 10] : TM1637_SEG_CODE[TM1637_SEG_SPACE];
    buf[1] = TM1637_SEG_CODE[minutes % 10];
    buf[2] = TM1637_SEG_CODE[seconds / 10] | TM1637_SEG_DP;  /* 秒十位 + 小数点 */
    buf[3] = TM1637_SEG_CODE[seconds % 10];
    buf[4] = TM1637_SEG_CODE[TM1637_SEG_SPACE];
    buf[5] = TM1637_SEG_CODE[TM1637_SEG_SPACE];

    TM1637_WriteAll(buf);
}

/**
 * @brief  全 6 位时-分-秒格式 (HH-MM-SS 或 H-MM-SS)
 *         使用横杠作分隔符, 自动消前导零
 */
void TM1637_ShowHMS(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
    uint8_t buf[6];

    if (hours > 99)     hours   = 99;
    if (minutes > 59)   minutes = 59;
    if (seconds > 59)   seconds = 59;

    if (hours >= 10) {
        buf[0] = TM1637_SEG_CODE[hours / 10];
    } else {
        buf[0] = TM1637_SEG_CODE[TM1637_SEG_SPACE];
    }
    buf[1] = TM1637_SEG_CODE[hours % 10];
    buf[2] = TM1637_SEG_CODE[TM1637_SEG_DASH];
    buf[3] = TM1637_SEG_CODE[minutes / 10];
    buf[4] = TM1637_SEG_CODE[minutes % 10];
    buf[5] = TM1637_SEG_CODE[seconds / 10];
    /* 注意: 仅 6 位数码管, 秒个位放不下, buf[5] 显示秒十位 */

    TM1637_WriteAll(buf);
}

/**
 * @brief  倒计时显示 — 自适应格式
 *         total_seconds < 60:  显示秒数, 带 1 位小数
 *         total_seconds < 3600: MM.SS 格式
 *         total_seconds >= 3600: H-MM-SS (小时-分-秒, 秒只显示十位)
 */
void TM1637_ShowCountdown(uint16_t total_seconds, uint8_t tenths)
{
    uint8_t buf[6];

    if (total_seconds < 60) {
        /* ── 秒数 + 十分位: "  XX.X" 或 "  X.XX" ── */
        uint8_t sec = (uint8_t)total_seconds;
        if (tenths > 9) tenths = 9;

        buf[0] = TM1637_SEG_CODE[TM1637_SEG_SPACE];
        buf[1] = TM1637_SEG_CODE[TM1637_SEG_SPACE];
        buf[2] = (sec >= 10) ? TM1637_SEG_CODE[sec / 10] : TM1637_SEG_CODE[TM1637_SEG_SPACE];
        buf[3] = TM1637_SEG_CODE[sec % 10] | TM1637_SEG_DP;  /* 整数秒 + 小数点 */
        buf[4] = TM1637_SEG_CODE[tenths];
        buf[5] = TM1637_SEG_CODE[TM1637_SEG_SPACE];
    }
    else if (total_seconds < 3600) {
        /* ── MM.SS ── */
        uint8_t min = total_seconds / 60;
        uint8_t sec = total_seconds % 60;

        buf[0] = TM1637_SEG_CODE[TM1637_SEG_SPACE];
        buf[1] = (min >= 10) ? TM1637_SEG_CODE[min / 10] : TM1637_SEG_CODE[TM1637_SEG_SPACE];
        buf[2] = TM1637_SEG_CODE[min % 10] | TM1637_SEG_DP;
        buf[3] = TM1637_SEG_CODE[sec / 10];
        buf[4] = TM1637_SEG_CODE[sec % 10];
        buf[5] = TM1637_SEG_CODE[TM1637_SEG_SPACE];
    }
    else {
        /* ── HH.MM (≥1 小时, 仅 6 位时省略秒) ── */
        uint16_t h = total_seconds / 3600;
        uint8_t  m = (total_seconds % 3600) / 60;

        if (h > 99) h = 99;

        buf[0] = TM1637_SEG_CODE[TM1637_SEG_SPACE];
        buf[1] = (h >= 10) ? TM1637_SEG_CODE[h / 10] : TM1637_SEG_CODE[TM1637_SEG_SPACE];
        buf[2] = TM1637_SEG_CODE[h % 10] | TM1637_SEG_DP;
        buf[3] = TM1637_SEG_CODE[m / 10];
        buf[4] = TM1637_SEG_CODE[m % 10];
        buf[5] = TM1637_SEG_CODE[TM1637_SEG_SPACE];
    }

    TM1637_WriteAll(buf);
}


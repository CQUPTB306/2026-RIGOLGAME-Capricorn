/**
 * @file    tm1637.h
 * @brief   TM1637 数码管驱动 — MSPM0, PA15=DIO, PA16=CLK
 * @note    驱动 6 位共阳数码管, 两线串行接口 (类 I2C, 但 LSB 先发)
 *
 *          TM1637 段码映射 (共阳, SEG=阴极):
 *            --a--       bit0=a, bit1=b, bit2=c, bit3=d,
 *           f   b       bit4=e, bit5=f, bit6=g, bit7=dp
 *            --g--       写 1 = SEG 拉低 = 段亮
 *           e   c
 *            --d--  •dp
 */

#ifndef __TM1637_H__
#define __TM1637_H__

#include "ti_msp_dl_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════════════════
 *  引脚定义 — PA15=DIO, PA16=CLK
 *  ⚠️ IOMUX 值来自 MSPM0G350X SDK (mspm0g350x.h):
 *     PA15→PINCM37 (GPIOA_DIO15), PA16→PINCM38 (GPIOA_DIO16)
 * ═══════════════════════════════════════════════════════════════ */
#define TM1637_CLK_PORT         GPIOA
#define TM1637_CLK_PIN          DL_GPIO_PIN_16
#define TM1637_CLK_IOMUX        (IOMUX_PINCM38)

#define TM1637_DIO_PORT         GPIOA
#define TM1637_DIO_PIN          DL_GPIO_PIN_15
#define TM1637_DIO_IOMUX        (IOMUX_PINCM37)

/* ═══════════════════════════════════════════════════════════════
 *  TM1637 指令集
 * ═══════════════════════════════════════════════════════════════ */

/* 数据命令 (01xxxxxx) */
#define TM1637_DATA_AUTO        0x40    /* 写显示寄存器, 地址自动递增 */
#define TM1637_DATA_FIXED       0x44    /* 写显示寄存器, 固定地址 */
#define TM1637_DATA_READ_KEY    0x42    /* 读取按键扫描数据 */

/* 地址命令 (11xxxxxx) — 显示寄存器 00H~05H 对应 GRID1~GRID6 */
#define TM1637_ADDR_BASE        0xC0    /* 起始地址 00H */
#define TM1637_ADDR_MASK        0x07    /* 地址低 3 位有效 (0~5) */

/* 显示控制 (10xxxxxx) — B4=开/关, B3~B0=亮度 */
#define TM1637_DISPLAY_OFF      0x80    /* 关闭显示 */
#define TM1637_DISPLAY_ON       0x88    /* 开启显示 (亮度 0) */
#define TM1637_BRIGHTNESS_MIN   0       /* 最小亮度 (1/16) */
#define TM1637_BRIGHTNESS_MAX   7       /* 最大亮度 (14/16) */
#define TM1637_BRIGHTNESS_MASK  0x07

/* ═══════════════════════════════════════════════════════════════
 *  段码查找表 (16 进制字符 0~F + 特殊符号)
 *  索引: 0-9=数字, 10-15=A~F, 16='-', 17=' ', 18='_'
 * ═══════════════════════════════════════════════════════════════ */
extern const uint8_t TM1637_SEG_CODE[];

/* 段码表索引 */
#define TM1637_SEG_DASH         16      /* '-' 中间横杠 */
#define TM1637_SEG_SPACE        17      /* ' ' 全灭 */
#define TM1637_SEG_UNDERSCORE   18      /* '_' 下划线 (d段) */

#define TM1637_SEG_DP           0x80    /* 小数点 OR 掩码 */

/* ═══════════════════════════════════════════════════════════════
 *  API 函数
 * ═══════════════════════════════════════════════════════════════ */

/**
 * @brief  TM1637 初始化 — GPIO 推挽输出 + 上电复位 + 开显示
 * @note   初始化后所有 6 位显示全灭, 亮度最大
 */
void TM1637_Init(void);

/**
 * @brief  设置亮度
 * @param  level  0~7 (0=最暗 1/16, 7=最亮 14/16)
 */
void TM1637_SetBrightness(uint8_t level);

/**
 * @brief  直接写入 6 位段码 (自动地址递增模式)
 * @param  data  6 字节数组, data[0]=GRID1(最左), data[5]=GRID6(最右)
 */
void TM1637_WriteAll(const uint8_t data[6]);

/**
 * @brief  清零所有位
 */
void TM1637_Clear(void);

/**
 * @brief  显示一个无符号整数 (右对齐, 不足补空)
 * @param  num   0~999999
 */
void TM1637_ShowNum(uint32_t num);

/**
 * @brief  显示倒计时分秒格式 (MM.SS, 占 4 位), 右侧 2 位灭
 *         例: 5分30秒 → " 5.30"
 * @param  minutes  0~99
 * @param  seconds  0~59
 */
void TM1637_ShowMinSec(uint8_t minutes, uint8_t seconds);

/**
 * @brief  全 6 位时分秒格式 (HH-MM-SS, 每位中间加横杠)
 *         例: 1小时23分45秒 → "1-23-45"
 * @param  hours    0~99
 * @param  minutes  0~59
 * @param  seconds  0~59
 */
void TM1637_ShowHMS(uint8_t hours, uint8_t minutes, uint8_t seconds);

/**
 * @brief  显示倒计时 (秒为单位) — 自适应格式
 *         <60秒:  秒数+小数点 → "  XX.X" 或 "  X.XX" (保留1位小数)
 *         <60分:  MM.SS 格式 → "  MM.SS"
 *         ≥60分:  显示超出范围提示
 * @param  total_seconds  剩余总秒数 (0~3599)
 * @param  tenths         十分之一秒 (0~9, 仅 total_seconds<60 时有效)
 */
void TM1637_ShowCountdown(uint16_t total_seconds, uint8_t tenths);

#ifdef __cplusplus
}
#endif

#endif /* __TM1637_H__ */

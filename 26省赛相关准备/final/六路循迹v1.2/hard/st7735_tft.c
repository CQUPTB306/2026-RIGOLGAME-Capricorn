/**
 * @file    st7735_tft.c
 * @brief   ST7735 TFT LCD 驱动实现 (MSPM0G3507 + TI DriverLib)
 * @note    使用软件模拟 SPI 驱动 ST7735, 128x160 分辨率
 *          所有 GPIO 操作使用 TI MSPM0 DriverLib API
 */

#include "st7735_tft.h"
#include "font_asc.h"
#include  "board.h"

/*===========================================================================
 * 外部字库声明 (由用户提供, 在 font_asc.h 中定义)
 *===========================================================================*/
extern const unsigned char asc[];              /* 8x16 ASCII 字库 */

/*===========================================================================
 * 基本 GPIO 初始化 (使用 DriverLib API)
 *===========================================================================*/
void TFT_GPIO_Init(void)
{
    /* 使能 GPIO 模块电源 (不调用 DL_GPIO_reset — 避免破坏 I2C/Track 等已初始化的引脚) */
    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);

    /* ---- 初始化 TFT 控制引脚 ---- */

    /* PB18 - SCLK (SPI 时钟) */
    DL_GPIO_initDigitalOutput(TFT_SCLK_IOMUX);

    /* PB19 - MOSI (SPI 数据) */
    DL_GPIO_initDigitalOutput(TFT_MOSI_IOMUX);

    /* PB17 - RES (复位) */
    DL_GPIO_initDigitalOutput(TFT_RES_IOMUX);

    /* PA14 - DC (数据/命令) */
    DL_GPIO_initDigitalOutput(TFT_DC_IOMUX);

    /* PB25 - CS (片选) */
    DL_GPIO_initDigitalOutput(TFT_CS_IOMUX);

    /* ---- 设置初始电平 ---- */
    TFT_CS_H;       /* 片选高 (未选中) */
    TFT_SCLK_L;     /* 时钟低 */
    TFT_MOSI_L;     /* 数据低 */
    TFT_DC_H;       /* DC 高 */
    TFT_RES_H;      /* 复位高 */
    /* BLK 不使用 — 背光通过硬件接 VCC 常亮或外部 PWM 控制 */

    /* 使能所有输出 (按端口分组) */
    DL_GPIO_enableOutput(GPIOA, TFT_DC_PIN);
    DL_GPIO_enableOutput(GPIOB, TFT_SCLK_PIN | TFT_MOSI_PIN | TFT_RES_PIN | TFT_CS_PIN);
}

/* SPI 延时 (可调整: 值越大速率越低, 越稳定)
 * 80MHz 下 ~50 ≈ 2μs/bit → ~500kHz SCLK */
#define TFT_SPI_DELAY()  do { volatile uint16_t _d = 3; while (_d--); } while (0)

/*===========================================================================
 * 软件模拟 SPI 发送一个字节 (CPOL=0, CPHA=0, MSB First)
 *===========================================================================*/
void TFT_SPI_SendByte(uint8_t data)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        TFT_SCLK_L;                         /* 时钟拉低 */
        if (data & 0x80)
            TFT_MOSI_H;                     /* 数据线拉高 */
        else
            TFT_MOSI_L;                     /* 数据线拉低 */
        data <<= 1;
        TFT_SPI_DELAY();                    /* 数据建立时间 */
        TFT_SCLK_H;                         /* 时钟拉高, 上升沿采样 */
        TFT_SPI_DELAY();                    /* 时钟高电平保持 */
    }
}

/*===========================================================================
 * ST7735 命令/数据发送
 *===========================================================================*/

/**
 * @brief 发送命令 (寄存器地址)
 */
void TFT_SendCommand(uint8_t cmd)
{
    TFT_CS_L;           /* 选中 */
    TFT_DC_L;           /* 命令模式 (DC=0) */
    TFT_SPI_SendByte(cmd);
    TFT_CS_H;           /* 释放 */
}

/**
 * @brief 发送单字节数据
 */
void TFT_SendData(uint8_t data)
{
    TFT_CS_L;
    TFT_DC_H;           /* 数据模式 (DC=1) */
    TFT_SPI_SendByte(data);
    TFT_CS_H;
}

/**
 * @brief 发送 16 位数据 (颜色值, RGB565)
 */
void TFT_SendData16(uint16_t data)
{
    TFT_CS_L;
    TFT_DC_H;
    TFT_SPI_SendByte((uint8_t)(data >> 8));   /* 高字节在前 */
    TFT_SPI_SendByte((uint8_t)(data));
    TFT_CS_H;
}

/**
 * @brief 发送命令 + 参数 (快捷方式)
 */
void TFT_SendCommandData(uint8_t cmd, uint8_t data)
{
    TFT_SendCommand(cmd);
    TFT_SendData(data);
}

/*===========================================================================
 * 硬件复位
 *===========================================================================*/
void TFT_Reset(void)
{
    TFT_RES_L;
    delay_ms(100);          /* 延时 100ms, 等待电源稳定 */
    TFT_RES_H;
    delay_ms(50);
}

/*===========================================================================
 * 设置屏幕显示方向
 * @param rotation: 0=0°, 1=90°, 2=180°, 3=270°
 *===========================================================================*/
void TFT_SetRotation(uint8_t rotation)
{
    TFT_SendCommand(0x36);      /* MADCTL 寄存器 */
    switch (rotation)
    {
        case 0: TFT_SendData(0xC0); break;   /* 竖屏 0° */
        case 1: TFT_SendData(0xA0); break;   /* 横屏 90° */
        case 2: TFT_SendData(0x00); break;   /* 竖屏 180° */
        case 3: TFT_SendData(0x60); break;   /* 横屏 270° */
        default: TFT_SendData(0xC0); break;
    }
}

/*===========================================================================
 * ST7735 初始化序列
 *===========================================================================*/
void TFT_Init(void)
{
    /* 1. GPIO 初始化 */
    TFT_GPIO_Init();

    /* 2. 硬件复位 */
    TFT_Reset();

    /* 3. 软件初始化序列 (与 STM32 原版一致) */

    TFT_SendCommand(0x11);      /* Sleep Out */
    delay_ms(120);

    TFT_SendCommandData(0x36, 0x00);     /* MADCTL */
    TFT_SendCommandData(0x3A, 0x05);     /* COLMOD: 16-bit RGB565 */

    /* Frame Rate Control */
    TFT_SendCommand(0xB1);
    TFT_SendData(0x05); TFT_SendData(0x3C); TFT_SendData(0x3C);

    TFT_SendCommand(0xB2);
    TFT_SendData(0x05); TFT_SendData(0x3C); TFT_SendData(0x3C);

    TFT_SendCommand(0xB3);
    TFT_SendData(0x05); TFT_SendData(0x3C); TFT_SendData(0x3C);
    TFT_SendData(0x05); TFT_SendData(0x3C); TFT_SendData(0x3C);

    TFT_SendCommandData(0xB4, 0x03);     /* Dot Inversion */

    /* Power Control */
    TFT_SendCommand(0xC0);
    TFT_SendData(0x2E); TFT_SendData(0x06); TFT_SendData(0x04);

    TFT_SendCommand(0xC1);
    TFT_SendData(0xC0); TFT_SendData(0xC2);

    TFT_SendCommand(0xC2);
    TFT_SendData(0x0D); TFT_SendData(0x0D);

    TFT_SendCommand(0xC3);
    TFT_SendData(0x8D); TFT_SendData(0xEE);

    TFT_SendCommand(0xC4);
    TFT_SendData(0x8D); TFT_SendData(0xEE);

    TFT_SendCommandData(0xC5, 0x00);     /* VCOM Control */

    /* MADCTL: 设置方向 */
    TFT_SendCommandData(0x36, 0xC0);

    /* Gamma 校正 (正极性) */
    TFT_SendCommand(0xE0);
    TFT_SendData(0x1B); TFT_SendData(0x21); TFT_SendData(0x10);
    TFT_SendData(0x15); TFT_SendData(0x2B); TFT_SendData(0x25);
    TFT_SendData(0x1F); TFT_SendData(0x23); TFT_SendData(0x22);
    TFT_SendData(0x22); TFT_SendData(0x2B); TFT_SendData(0x37);
    TFT_SendData(0x00); TFT_SendData(0x15); TFT_SendData(0x02);
    TFT_SendData(0x3F);

    /* Gamma 校正 (负极性) */
    TFT_SendCommand(0xE1);
    TFT_SendData(0x1A); TFT_SendData(0x20); TFT_SendData(0x0F);
    TFT_SendData(0x15); TFT_SendData(0x2A); TFT_SendData(0x25);
    TFT_SendData(0x1E); TFT_SendData(0x23); TFT_SendData(0x23);
    TFT_SendData(0x22); TFT_SendData(0x2B); TFT_SendData(0x37);
    TFT_SendData(0x00); TFT_SendData(0x15); TFT_SendData(0x02);
    TFT_SendData(0x3F);

    TFT_SendCommand(0x2C);      /* Memory Write (空) */
    TFT_SendCommand(0x29);      /* Display ON */

    /* 清屏为黑色 */
    TFT_Clear(TFT_BLACK);

    /* BLK 不使用, 背光由硬件控制 (接 VCC 常亮或外部 PWM) */
}

/*===========================================================================
 * 区域设置 (列地址 + 行地址)
 *===========================================================================*/
void TFT_SetRegion(uint16_t x_start, uint16_t y_start,
                   uint16_t x_end, uint16_t y_end)
{
    TFT_SendCommand(0x2A);          /* CASET: 列地址 */
    TFT_SendData(0x00);
    TFT_SendData((uint8_t)(x_start + 2));
    TFT_SendData(0x00);
    TFT_SendData((uint8_t)(x_end));

    TFT_SendCommand(0x2B);          /* RASET: 行地址 */
    TFT_SendData(0x00);
    TFT_SendData((uint8_t)(y_start + 1));
    TFT_SendData(0x00);
    TFT_SendData((uint8_t)(y_end));

    TFT_SendCommand(0x2C);          /* RAMWR: 内存写 */
}

/*===========================================================================
 * 设置光标位置 (单个像素点)
 *===========================================================================*/
void TFT_SetCursor(uint16_t x, uint16_t y)
{
    TFT_SetRegion(x, y, x, y);
}

/*===========================================================================
 * 全屏清屏
 *===========================================================================*/
void TFT_Clear(uint16_t color)
{
    TFT_SetRegion(0, 0, TFT_WIDTH - 1, TFT_HEIGHT - 1);
    for (uint16_t i = 0; i < TFT_WIDTH; i++)
    {
        for (uint16_t j = 0; j < TFT_HEIGHT; j++)
        {
            TFT_SendData16(color);
        }
    }
}

/*===========================================================================
 * 区域填充
 *===========================================================================*/
void TFT_Fill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
              uint16_t color)
{
    uint32_t total = (uint32_t)(x2 - x1 + 1) * (y2 - y1 + 1);

    TFT_SetRegion(x1, y1, x2, y2);
    while (total--)
    {
        TFT_SendData16(color);
    }
    /* 恢复全屏区域 */
    TFT_SetRegion(0, 0, TFT_WIDTH - 1, TFT_HEIGHT - 1);
}

/*===========================================================================
 * 画点
 *===========================================================================*/
void TFT_DrawPoint(uint16_t x, uint16_t y, uint16_t color)
{
    TFT_SetCursor(x, y);
    TFT_SendData16(color);
}

/*===========================================================================
 * 画线 (Bresenham 算法)
 *===========================================================================*/
void TFT_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
                  uint16_t color)
{
    int16_t dx, dy;
    int16_t dx2, dy2;
    int16_t x_inc, y_inc;
    int16_t error;
    int16_t index;

    TFT_SetCursor(x0, y0);

    dx = (int16_t)x1 - (int16_t)x0;
    dy = (int16_t)y1 - (int16_t)y0;

    /* x 方向 */
    if (dx >= 0)
    {
        x_inc = 1;
    }
    else
    {
        x_inc = -1;
        dx = -dx;
    }

    /* y 方向 */
    if (dy >= 0)
    {
        y_inc = 1;
    }
    else
    {
        y_inc = -1;
        dy = -dy;
    }

    dx2 = dx << 1;
    dy2 = dy << 1;

    if (dx > dy)
    {
        /* |斜率| < 1: x 轴为步进轴 */
        error = dy2 - dx;
        for (index = 0; index <= dx; index++)
        {
            TFT_DrawPoint(x0, y0, color);

            if (error >= 0)
            {
                error -= dx2;
                y0 = (uint16_t)((int16_t)y0 + y_inc);
            }
            error += dy2;
            x0 = (uint16_t)((int16_t)x0 + x_inc);
        }
    }
    else
    {
        /* |斜率| >= 1: y 轴为步进轴 */
        error = dx2 - dy;
        for (index = 0; index <= dy; index++)
        {
            TFT_DrawPoint(x0, y0, color);

            if (error >= 0)
            {
                error -= dy2;
                x0 = (uint16_t)((int16_t)x0 + x_inc);
            }
            error += dx2;
            y0 = (uint16_t)((int16_t)y0 + y_inc);
        }
    }
}

/*===========================================================================
 * 画圆 (Bresenham 算法)
 *===========================================================================*/
void TFT_DrawCircle(uint16_t cx, uint16_t cy, uint16_t r, uint16_t color)
{
    int16_t a = 0;
    int16_t b = (int16_t)r;
    int16_t c = 3 - 2 * (int16_t)r;

    while (a < b)
    {
        /* 利用八对称性画点 */
        TFT_DrawPoint((uint16_t)(cx + a), (uint16_t)(cy + b), color);
        TFT_DrawPoint((uint16_t)(cx - a), (uint16_t)(cy + b), color);
        TFT_DrawPoint((uint16_t)(cx + a), (uint16_t)(cy - b), color);
        TFT_DrawPoint((uint16_t)(cx - a), (uint16_t)(cy - b), color);
        TFT_DrawPoint((uint16_t)(cx + b), (uint16_t)(cy + a), color);
        TFT_DrawPoint((uint16_t)(cx - b), (uint16_t)(cy + a), color);
        TFT_DrawPoint((uint16_t)(cx + b), (uint16_t)(cy - a), color);
        TFT_DrawPoint((uint16_t)(cx - b), (uint16_t)(cy - a), color);

        if (c < 0)
        {
            c = c + 4 * a + 6;
        }
        else
        {
            c = c + 4 * (a - b) + 10;
            b -= 1;
        }
        a += 1;
    }

    /* a == b 时补画 */
    if (a == b)
    {
        TFT_DrawPoint((uint16_t)(cx + a), (uint16_t)(cy + b), color);
        TFT_DrawPoint((uint16_t)(cx - a), (uint16_t)(cy + b), color);
        TFT_DrawPoint((uint16_t)(cx + a), (uint16_t)(cy - b), color);
        TFT_DrawPoint((uint16_t)(cx - a), (uint16_t)(cy - b), color);
        TFT_DrawPoint((uint16_t)(cx + b), (uint16_t)(cy + a), color);
        TFT_DrawPoint((uint16_t)(cx - b), (uint16_t)(cy + a), color);
        TFT_DrawPoint((uint16_t)(cx + b), (uint16_t)(cy - a), color);
        TFT_DrawPoint((uint16_t)(cx - b), (uint16_t)(cy - a), color);
    }
}

/*===========================================================================
 * 画矩形边框
 *===========================================================================*/
void TFT_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                  uint16_t color)
{
    TFT_DrawLine(x, y, x + w, y, color);                 /* 上边 */
    TFT_DrawLine(x + w, y, x + w, y + h, color);         /* 右边 */
    TFT_DrawLine(x, y + h, x + w, y + h, color);         /* 下边 */
    TFT_DrawLine(x, y, x, y + h, color);                 /* 左边 */
}

/*===========================================================================
 * 画填充矩形
 *===========================================================================*/
void TFT_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                  uint16_t color)
{
    TFT_Fill(x, y, x + w, y + h, color);
}

/*===========================================================================
 * 显示 ASCII 字符 (8x16 点阵)
 * @param x,y: 起始坐标
 * @param fc:   前景色
 * @param bc:   背景色
 * @param c:    待显示字符 (ASCII 32-126)
 *===========================================================================*/
void TFT_ShowChar(uint8_t x, uint8_t y, uint16_t fc, uint16_t bc, char c)
{
    uint16_t k = (uint16_t)(c - 32) * 16;

    for (uint8_t i = 0; i < 16; i++)
    {
        for (uint8_t j = 0; j < 8; j++)
        {
            if (asc[k + i] & (0x80 >> j))
                TFT_DrawPoint(x + j, y + i, fc);
            else
                TFT_DrawPoint(x + j, y + i, bc);
        }
    }
}

/*===========================================================================
 * 显示字符串 (自动换行)
 *===========================================================================*/
void TFT_ShowString(uint8_t x, uint8_t y, uint16_t fc, uint16_t bc, char *str)
{
    uint8_t cur_x = x;
    uint8_t cur_y = y;
    uint16_t len = (uint16_t)strlen(str);

    for (uint16_t i = 0; i < len; i++)
    {
        /* 换行判断 */
        if (cur_x + 8 > TFT_WIDTH)
        {
            cur_x = 0;
            cur_y += 16;
        }
        /* 超出屏幕底部则停止 */
        if (cur_y + 16 > TFT_HEIGHT)
        {
            break;
        }
        TFT_ShowChar(cur_x, cur_y, fc, bc, str[i]);
        cur_x += 8;
    }
}

/*===========================================================================
 * 显示整数 (支持负数)
 *===========================================================================*/
void TFT_ShowNumber(uint8_t x, uint8_t y, uint16_t fc, uint16_t bc,
                    int32_t num)
{
    char buf[16];
    uint8_t idx = 0;
    uint8_t is_neg = 0;

    if (num < 0)
    {
        is_neg = 1;
        num = -num;
    }

    /* 特殊情况: num == 0 */
    if (num == 0)
    {
        buf[idx++] = '0';
    }
    else
    {
        while (num > 0)
        {
            buf[idx++] = (char)('0' + (num % 10));
            num /= 10;
        }
    }

    if (is_neg)
    {
        buf[idx++] = '-';
    }

    /* 反转字符串 */
    for (uint8_t i = 0; i < idx / 2; i++)
    {
        char tmp = buf[i];
        buf[i] = buf[idx - 1 - i];
        buf[idx - 1 - i] = tmp;
    }

    buf[idx] = '\0';
    TFT_ShowString(x, y, fc, bc, buf);
}

/*===========================================================================
 * 显示图片 (RGB565 格式)
 * @param x,y: 起始坐标
 * @param w:   图片宽度
 * @param h:   图片高度
 * @param p:   图片数据指针 (每个像素占 2 字节, 低字节在前)
 *===========================================================================*/
void TFT_ShowImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                   const unsigned char *p)
{
    TFT_SetRegion(x, y, x + w - 1, y + h - 1);

    uint32_t total = (uint32_t)w * h;
    for (uint32_t i = 0; i < total; i++)
    {
        uint8_t  picL = p[2 * i];
        uint8_t  picH = p[2 * i + 1];
        uint16_t color = (uint16_t)((picH << 8) | picL);
        TFT_SendData16(color);
    }

    /* 恢复全屏区域 */
    TFT_SetRegion(0, 0, TFT_WIDTH - 1, TFT_HEIGHT - 1);
}

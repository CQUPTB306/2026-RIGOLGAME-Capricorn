/**
 * @file    oled.c
 * @brief   0.96寸 OLED 驱动 (SSD1306) — MSPM0G3507 适配版
 * @note    使用软件 I2C (soft_i2c_simple.h) 驱动
 *          OLED 从机地址: 0x3C (写 0x78)
 *          分辨率: 128 × 64, 8x16 字体
 *          改编自 STM32 最终代码 — 适配 MSPM0 多总线软件 I2C
 */

#include "oled.h"
#include "oled_font.h"
#include "../board.h"

/**
 * @brief  OLED 写命令
 * @param  obj     软件 I2C 总线对象指针
 * @param  Command 要写入的命令字节
 */
static void OLED_WriteCommand(SoftI2C_Obj *obj, uint8_t Command)
{
    SoftI2C_Start(obj);
    SoftI2C_SendByte(obj, 0x78);        /* OLED 从机地址 (0x3C << 1) */
    SoftI2C_WaitAck(obj);
    SoftI2C_SendByte(obj, 0x00);        /* 控制字节: 命令模式 */
    SoftI2C_WaitAck(obj);
    SoftI2C_SendByte(obj, Command);
    SoftI2C_WaitAck(obj);
    SoftI2C_Stop(obj);
}

/**
 * @brief  OLED 写数据
 * @param  obj  软件 I2C 总线对象指针
 * @param  Data 要写入的数据字节
 */
void OLED_WriteData(SoftI2C_Obj *obj, uint8_t Data)
{
    SoftI2C_Start(obj);
    SoftI2C_SendByte(obj, 0x78);        /* OLED 从机地址 (0x3C << 1) */
    SoftI2C_WaitAck(obj);
    SoftI2C_SendByte(obj, 0x40);        /* 控制字节: 数据模式 */
    SoftI2C_WaitAck(obj);
    SoftI2C_SendByte(obj, Data);
    SoftI2C_WaitAck(obj);
    SoftI2C_Stop(obj);
}

/**
 * @brief  OLED 设置光标位置
 * @param  obj 软件 I2C 总线对象指针
 * @param  Y   页地址 (0~7)
 * @param  X   列地址 (0~127)
 */
void OLED_SetCursor(SoftI2C_Obj *obj, uint8_t Y, uint8_t X)
{
    OLED_WriteCommand(obj, 0xB0 | Y);                     /* 页地址 */
    OLED_WriteCommand(obj, 0x10 | ((X & 0xF0) >> 4));     /* 列地址高 4 位 */
    OLED_WriteCommand(obj, 0x00 | (X & 0x0F));             /* 列地址低 4 位 */
}

/**
 * @brief  OLED 初始化 (SSD1306 标准初始化序列)
 * @param  obj 软件 I2C 总线对象指针
 */
void OLED_Init(SoftI2C_Obj *obj)
{
    uint8_t retry = 3;

    /* 上电等待 OLED 稳定 */
    delay_ms(100);

    /* 软件 I2C 初始化 (GPIO 已在 SoftI2C_Init 中配置) */
    /* obj 的 init 应由调用方在 SoftI2C_Init(&i2c_oled) 中完成 */

    /* SSD1306 初始化序列 (带重试) */
    while (retry--)
    {
        OLED_WriteCommand(obj, 0xAE);  /* 关闭显示 (Display OFF) */

        OLED_WriteCommand(obj, 0xD5);  /* 设置时钟分频比/振荡器频率 */
        OLED_WriteCommand(obj, 0x80);  /* 默认值: 分频比=1, 频率≈370kHz */

        OLED_WriteCommand(obj, 0xA8);  /* 设置多路复用比 */
        OLED_WriteCommand(obj, 0x3F);  /* 64 COM (128×64) */

        OLED_WriteCommand(obj, 0xD3);  /* 设置显示偏移 */
        OLED_WriteCommand(obj, 0x00);  /* 偏移 = 0 */

        OLED_WriteCommand(obj, 0x40);  /* 设置显示起始行 = 0 */

        OLED_WriteCommand(obj, 0xA1);  /* 段重映射: 列 127 → SEG0 (左右正常) */
        OLED_WriteCommand(obj, 0xC8);  /* COM 扫描方向: COM[N-1] → COM0 (上下正常) */

        OLED_WriteCommand(obj, 0xDA);  /* COM 引脚硬件配置 */
        OLED_WriteCommand(obj, 0x12);  /* 备选 COM 引脚配置 (128×64) */

        OLED_WriteCommand(obj, 0x81);  /* 设置对比度 */
        OLED_WriteCommand(obj, 0xCF);  /* 对比度值 (0~255, 默认 0x7F) */

        OLED_WriteCommand(obj, 0xD9);  /* 设置预充电周期 */
        OLED_WriteCommand(obj, 0xF1);  /* Phase1=1, Phase2=15 */

        OLED_WriteCommand(obj, 0xDB);  /* 设置 VCOMH 电压 */
        OLED_WriteCommand(obj, 0x30);  /* ~0.77×VCC */

        OLED_WriteCommand(obj, 0xA4);  /* 全屏点亮: 跟随 GDDRAM 内容 */

        OLED_WriteCommand(obj, 0xA6);  /* 正常显示 (非反色) */

        OLED_WriteCommand(obj, 0x8D);  /* 启用电荷泵 */
        OLED_WriteCommand(obj, 0x14);  /* 电荷泵开启 (3.3V 供电必需) */

        OLED_WriteCommand(obj, 0xAF);  /* 开启显示 (Display ON) */

        break;  /* 成功则跳出 */
    }

    OLED_Clear(obj);  /* 上电后清屏 */
}

/**
 * @brief  OLED 清屏 (全填充 0x00)
 * @param  obj 软件 I2C 总线对象指针
 */
void OLED_Clear(SoftI2C_Obj *obj)
{
    uint8_t i, j;
    for (j = 0; j < 8; j++)
    {
        OLED_SetCursor(obj, j, 0);
        for (i = 0; i < 128; i++)
        {
            OLED_WriteData(obj, 0x00);
        }
    }
}

/*==================== 字符 / 字符串显示 ====================*/

/**
 * @brief  OLED 显示单个 8×16 ASCII 字符
 * @param  obj    软件 I2C 总线对象指针
 * @param  Line   行号 (1~4)
 * @param  Column 列号 (1~16)
 * @param  Char   要显示的 ASCII 字符
 */
void OLED_ShowChar(SoftI2C_Obj *obj, uint8_t Line, uint8_t Column, char Char)
{
    uint8_t i;
    /* 上半部分 (8 像素) */
    OLED_SetCursor(obj, (Line - 1) * 2, (Column - 1) * 8);
    for (i = 0; i < 8; i++)
    {
        OLED_WriteData(obj, OLED_F8x16[Char - ' '][i]);
    }
    /* 下半部分 (8 像素) */
    OLED_SetCursor(obj, (Line - 1) * 2 + 1, (Column - 1) * 8);
    for (i = 0; i < 8; i++)
    {
        OLED_WriteData(obj, OLED_F8x16[Char - ' '][i + 8]);
    }
}

/**
 * @brief  OLED 显示字符串
 * @param  obj    软件 I2C 总线对象指针
 * @param  Line   起始行 (1~4)
 * @param  Column 起始列 (1~16)
 * @param  String 字符串 (以 '\0' 结尾)
 */
void OLED_ShowString(SoftI2C_Obj *obj, uint8_t Line, uint8_t Column, char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++)
    {
        OLED_ShowChar(obj, Line, Column + i, String[i]);
    }
}

/*==================== 数字显示 ====================*/

/**
 * @brief  OLED 内部幂运算
 * @param  X 底数
 * @param  Y 指数
 * @retval X^Y
 */
static uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--)
    {
        Result *= X;
    }
    return Result;
}

/**
 * @brief  OLED 显示无符号十进制数字
 * @param  obj    软件 I2C 总线对象指针
 * @param  Line   行号 (1~4)
 * @param  Column 起始列 (1~16)
 * @param  Number 数字 (0~4294967295)
 * @param  Length 显示位数 (1~10)
 */
void OLED_ShowNum(SoftI2C_Obj *obj, uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
    {
        OLED_ShowChar(obj, Line, Column + i,
                      Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/**
 * @brief  OLED 显示带符号十进制数字
 * @param  obj    软件 I2C 总线对象指针
 * @param  Line   行号 (1~4)
 * @param  Column 起始列 (1~16)
 * @param  Number 数字 (-2147483648~2147483647)
 * @param  Length 显示位数 (不含符号位, 1~10)
 */
void OLED_ShowSignedNum(SoftI2C_Obj *obj, uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
    uint8_t i;
    uint32_t Number1;
    if (Number >= 0)
    {
        OLED_ShowChar(obj, Line, Column, '+');
        Number1 = Number;
    }
    else
    {
        OLED_ShowChar(obj, Line, Column, '-');
        Number1 = -Number;
    }
    for (i = 0; i < Length; i++)
    {
        OLED_ShowChar(obj, Line, Column + i + 1,
                      Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/**
 * @brief  OLED 显示十六进制数字
 * @param  obj    软件 I2C 总线对象指针
 * @param  Line   行号 (1~4)
 * @param  Column 起始列 (1~16)
 * @param  Number 数字 (0~0xFFFFFFFF)
 * @param  Length 显示位数 (1~8)
 */
void OLED_ShowHexNum(SoftI2C_Obj *obj, uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i, SingleNumber;
    for (i = 0; i < Length; i++)
    {
        SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
        if (SingleNumber < 10)
        {
            OLED_ShowChar(obj, Line, Column + i, SingleNumber + '0');
        }
        else
        {
            OLED_ShowChar(obj, Line, Column + i, SingleNumber - 10 + 'A');
        }
    }
}

/**
 * @brief  OLED 显示二进制数字
 * @param  obj    软件 I2C 总线对象指针
 * @param  Line   行号 (1~4)
 * @param  Column 起始列 (1~16)
 * @param  Number 数字 (0~65535)
 * @param  Length 显示位数 (1~16)
 */
void OLED_ShowBinNum(SoftI2C_Obj *obj, uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
    {
        OLED_ShowChar(obj, Line, Column + i,
                      Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
    }
}

/**
 * @brief  OLED 显示浮点数 (支持四舍五入)
 * @param  obj       软件 I2C 总线对象指针
 * @param  Line      行号 (1~4)
 * @param  Column    起始列 (1~16)
 * @param  Number    浮点数 (支持正负)
 * @param  IntLength 整数部分位数 (1~9)
 * @param  DecLength 小数部分位数 (1~6)
 */
void OLED_ShowFloatNum(SoftI2C_Obj *obj, uint8_t Line, uint8_t Column,
                        float Number, uint8_t IntLength, uint8_t DecLength)
{
    uint32_t IntPart;
    uint32_t DecPart;
    float    DecTemp;
    uint8_t  i;

    /* 1. 符号位 */
    if (Number < 0)
    {
        OLED_ShowChar(obj, Line, Column, '-');
        Number = -Number;
    }
    else
    {
        OLED_ShowChar(obj, Line, Column, '+');
    }
    Column++;

    /* 2. 分离整数 + 小数 (四舍五入) */
    IntPart = (uint32_t)Number;
    DecTemp = (Number - IntPart) * OLED_Pow(10, DecLength) + 0.5f;
    DecPart = (uint32_t)DecTemp;

    /* 3. 小数进位处理 */
    if (DecPart >= OLED_Pow(10, DecLength))
    {
        IntPart += 1;
        DecPart = 0;
    }

    /* 4. 整数部分 */
    for (i = 0; i < IntLength; i++)
    {
        OLED_ShowChar(obj, Line, Column + i,
                      IntPart / OLED_Pow(10, IntLength - i - 1) % 10 + '0');
    }
    Column += IntLength;

    /* 5. 小数点 */
    OLED_ShowChar(obj, Line, Column, '.');
    Column++;

    /* 6. 小数部分 */
    for (i = 0; i < DecLength; i++)
    {
        OLED_ShowChar(obj, Line, Column + i,
                      DecPart / OLED_Pow(10, DecLength - i - 1) % 10 + '0');
    }
}

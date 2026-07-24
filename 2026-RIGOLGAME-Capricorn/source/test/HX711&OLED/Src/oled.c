#include "oled.h"
#include "oled_font.h"

/**
 * @brief  OLED写命令
 * @param  Command 要写入的命令
 * @retval 无
 */
static void OLED_WriteCommand(uint8_t Command)
{
    Soft_I2C_Start();
    Soft_I2C_SendByte(0x78);        // OLED从机地址（0x3C << 1）
    Soft_I2C_WaitAck();             // 补充：等待地址应答
    Soft_I2C_SendByte(0x00);        // 写命令标志
    Soft_I2C_WaitAck();             // 补充：等待控制字节应答
    Soft_I2C_SendByte(Command);
    Soft_I2C_WaitAck();             // 补充：等待命令应答
    Soft_I2C_Stop();
}

/**
 * @brief  OLED写数据
 * @param  Data 要写入的数据
 * @retval 无
 */
static void OLED_WriteData(uint8_t Data)
{
    Soft_I2C_Start();
    Soft_I2C_SendByte(0x78);        // OLED从机地址
    Soft_I2C_WaitAck();             // 补充：等待地址应答
    Soft_I2C_SendByte(0x40);        // 写数据标志
    Soft_I2C_WaitAck();             // 补充：等待控制字节应答
    Soft_I2C_SendByte(Data);
    Soft_I2C_WaitAck();             // 补充：等待数据应答
    Soft_I2C_Stop();
}

/**
 * @brief  OLED设置光标位置
 * @param  Y 页地址（0~7）
 * @param  X 列地址（0~127）
 * @retval 无
 */
static void OLED_SetCursor(uint8_t Y, uint8_t X)
{
    OLED_WriteCommand(0xB0 | Y);                    // 设置Y位置（页）
    OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));    // 设置X位置高4位
    OLED_WriteCommand(0x00 | (X & 0x0F));            // 设置X位置低4位
}

/**
 * @brief  OLED初始化
 * @param  无
 * @retval 无
 */
void OLED_Init(void)
{
    /* 上电延时 */
    HAL_Delay(100);

    /* 初始化软件I2C（依赖CubeMX配置，仅保留函数调用） */
    // Soft_I2C_Init();  // 注释：CubeMX已配置GPIO，无需重复初始化

    /* OLED寄存器初始化 */
    OLED_WriteCommand(0xAE);  // 关闭显示
    OLED_WriteCommand(0xD5);  // 设置显示时钟分频比/振荡器频率
    OLED_WriteCommand(0x80);
    OLED_WriteCommand(0xA8);  // 设置多路复用率
    OLED_WriteCommand(0x3F);
    OLED_WriteCommand(0xD3);  // 设置显示偏移
    OLED_WriteCommand(0x00);
    OLED_WriteCommand(0x40);  // 设置显示开始行
    OLED_WriteCommand(0xA1);  // 设置左右方向，0xA1正常 0xA0反置
    OLED_WriteCommand(0xC8);  // 设置上下方向，0xC8正常 0xC0反置
    OLED_WriteCommand(0xDA);  // 设置COM引脚硬件配置
    OLED_WriteCommand(0x12);
    OLED_WriteCommand(0x81);  // 设置对比度
    OLED_WriteCommand(0xCF);
    OLED_WriteCommand(0xD9);  // 设置预充电周期
    OLED_WriteCommand(0xF1);
    OLED_WriteCommand(0xDB);  // 设置VCOMH
    OLED_WriteCommand(0x30);
    OLED_WriteCommand(0xA4);  // 全局显示开启
    OLED_WriteCommand(0xA6);  // 正常显示
    OLED_WriteCommand(0x8D);  // 开启充电泵
    OLED_WriteCommand(0x14);
    OLED_WriteCommand(0xAF);  // 开启显示

    OLED_Clear();  // 清屏
}

/**
 * @brief  OLED清屏
 * @param  无
 * @retval 无
 */
void OLED_Clear(void)
{
    uint8_t i, j;
    for (j = 0; j < 8; j++)
    {
        OLED_SetCursor(j, 0);
        for (i = 0; i < 128; i++)
        {
            OLED_WriteData(0x00);
        }
    }
}

/**
 * @brief  OLED显示单个字符
 * @param  Line 行（1~4）
 * @param  Column 列（1~16）
 * @param  Char 要显示的字符（ASCII）
 * @retval 无
 */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
    uint8_t i;
    /* 设置光标到字符上半部分 */
    OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);
    for (i = 0; i < 8; i++)
    {
        OLED_WriteData(OLED_F8x16[Char - ' '][i]);
    }
    /* 设置光标到字符下半部分 */
    OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);
    for (i = 0; i < 8; i++)
    {
        OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]);
    }
}

/**
 * @brief  OLED显示字符串
 * @param  Line 起始行（1~4）
 * @param  Column 起始列（1~16）
 * @param  String 要显示的字符串
 * @retval 无
 */
void OLED_ShowString(uint8_t Line, uint8_t Column, char* String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++)
    {
        OLED_ShowChar(Line, Column + i, String[i]);
    }
}

/**
 * @brief  次方函数（内部使用）
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
 * @brief  OLED显示十进制数字（无符号）
 * @param  Line 起始行（1~4）
 * @param  Column 起始列（1~16）
 * @param  Number 要显示的数字（0~4294967295）
 * @param  Length 显示长度（1~10）
 * @retval 无
 */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
    {
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/**
 * @brief  OLED显示十进制数字（带符号）
 * @param  Line 起始行（1~4）
 * @param  Column 起始列（1~16）
 * @param  Number 要显示的数字（-2147483648~2147483647）
 * @param  Length 显示长度（1~10）
 * @retval 无
 */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
    uint8_t i;
    uint32_t Number1;
    if (Number >= 0)
    {
        OLED_ShowChar(Line, Column, '+');
        Number1 = Number;
    }
    else
    {
        OLED_ShowChar(Line, Column, '-');
        Number1 = -Number;
    }
    for (i = 0; i < Length; i++)
    {
        OLED_ShowChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/**
 * @brief  OLED显示十六进制数字
 * @param  Line 起始行（1~4）
 * @param  Column 起始列（1~16）
 * @param  Number 要显示的数字（0~0xFFFFFFFF）
 * @param  Length 显示长度（1~8）
 * @retval 无
 */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i, SingleNumber;
    for (i = 0; i < Length; i++)
    {
        SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
        if (SingleNumber < 10)
        {
            OLED_ShowChar(Line, Column + i, SingleNumber + '0');
        }
        else
        {
            OLED_ShowChar(Line, Column + i, SingleNumber - 10 + 'A');
        }
    }
}

/**
 * @brief  OLED显示二进制数字
 * @param  Line 起始行（1~4）
 * @param  Column 起始列（1~16）
 * @param  Number 要显示的数字（0~0xFFFF）
 * @param  Length 显示长度（1~16）
 * @retval 无
 */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
    {
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
    }
}

/**
 * @brief  OLED显示浮点型数字
 * @param  Line 起始行（1~4）
 * @param  Column 起始列（1~16）
 * @param  Number 要显示的浮点数（支持正负）
 * @param  IntLength 整数部分显示长度（1~9）
 * @param  DecLength 小数部分显示长度（1~6，建议不超过6避免精度丢失）
 * @retval 无
 * @note   1. 小数部分采用四舍五入处理；2. 超出长度的整数位会截断高位；
 *         3. 浮点数范围建议：-999999999.999999 ~ 999999999.999999
 */
void OLED_ShowFloatNum(uint8_t Line, uint8_t Column, float Number, uint8_t IntLength, uint8_t DecLength)
{
    uint32_t IntPart;    // 整数部分
    uint32_t DecPart;    // 小数部分
    float DecTemp;       // 小数部分临时变量
    uint8_t i;

    // 1. 处理负数：显示负号，取绝对值
    if (Number < 0)
    {
        OLED_ShowChar(Line, Column, '-');
        Number = -Number;  // 转为正数处理
    }
    else
    {
        OLED_ShowChar(Line, Column, '+');  // 正数显示正号（也可改为空格）
    }
    Column++;  // 符号位占1列，列数后移

    // 2. 分离整数部分和小数部分（四舍五入）
    IntPart = (uint32_t)Number;                     // 提取整数部分（截断）
    DecTemp = (Number - IntPart) * OLED_Pow(10, DecLength) + 0.5;  // 小数部分四舍五入
    DecPart = (uint32_t)DecTemp;

    // 3. 处理小数部分进位（如 9.999 保留2位小数时，需进1到整数部分）
    if (DecPart >= OLED_Pow(10, DecLength))
    {
        IntPart += 1;
        DecPart = 0;
    }

    // 4. 显示整数部分
    for (i = 0; i < IntLength; i++)
    {
        OLED_ShowChar(Line, Column + i, IntPart / OLED_Pow(10, IntLength - i - 1) % 10 + '0');
    }
    Column += IntLength;  // 整数位占IntLength列，列数后移

    // 5. 显示小数点
    OLED_ShowChar(Line, Column, '.');
    Column++;  // 小数点占1列，列数后移

    // 6. 显示小数部分
    for (i = 0; i < DecLength; i++)
    {
        OLED_ShowChar(Line, Column + i, DecPart / OLED_Pow(10, DecLength - i - 1) % 10 + '0');
    }
}

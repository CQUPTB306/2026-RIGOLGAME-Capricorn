/**
 * @file    track_oled_demo.c
 * @brief   循迹传感器 OLED 监测 Demo 实现
 */

#include "track_oled_demo.h"
#include "track.h"
#include "oled.h"

/**
 * @brief  刷新一帧循迹状态到 OLED
 * @param  oled_bus OLED 所在的 I2C 总线对象
 */
void TrackOLED_ShowStatus(SoftI2C_Obj *oled_bus)
{
    static uint16_t frame = 0;          /* 帧计数, 不断增长 = 没卡住 */
    uint8_t status = Track_Read_All();  /* 8 位, 0=检测到黑线, 1=白线 */
    char    visual[9];                   /* 可视化 (8 字符 + '\0') */
    uint8_t count  = 0;                 /* 检测到黑线的传感器数量 */
    int     sum    = 0;                 /* 加权求和 */
    int     ch_cnt = 0;                 /* 检测到黑线的通道数 */
    int     error  = 0;                 /* 加权偏差 */

    frame++;

    /* 构造可视化条形图 + 加权偏差 */
    /* visual[0]=bit7(左), visual[7]=bit0(右) — 与Bin显示方向一致 */
    /* IR_Weight[0]对应bit0, IR_Weight[7]对应bit7 — 与track.c一致 */
    for (int i = 0; i < 8; i++)
    {
        uint8_t bit = 7 - i;
        if (status & (1 << bit))
        {
            visual[i] = '.';
        }
        else
        {
            visual[i] = '#';
            count++;
            sum    += IR_Weight[bit];
            ch_cnt++;
        }
    }
    visual[8] = '\0';
    error = (ch_cnt > 0) ? (sum / ch_cnt) : 0;

    /* 第1行: 帧计数 + 活跃通道 + 闪烁点 */
    OLED_ShowString(oled_bus, 1, 1, "F:");
    OLED_ShowNum(oled_bus,    1, 3, frame, 4);
    OLED_ShowString(oled_bus, 1, 8, "T:");
    OLED_ShowNum(oled_bus,    1, 10, count, 1);
    OLED_ShowString(oled_bus, 1, 11, (frame & 0x04) ? "*" : " ");

    /* 第2行: 二进制 */
    OLED_ShowString(oled_bus, 2, 1, "Bin:");
    OLED_ShowBinNum(oled_bus,  2, 5, status, 8);

    /* 第3行: 可视化 */
    OLED_ShowString(oled_bus, 3, 1, "Vis:");
    OLED_ShowString(oled_bus, 3, 5, visual);

    /* 第4行: 偏差 */
    OLED_ShowString(oled_bus,    4, 1, "Err:");
    OLED_ShowSignedNum(oled_bus, 4, 5, error, 3);
}

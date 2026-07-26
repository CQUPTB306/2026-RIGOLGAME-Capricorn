/**
 * @file    track_oled_demo.h
 * @brief   循迹传感器 OLED 监测 Demo — 在 OLED 上实时显示 8 路状态
 */

#ifndef __TRACK_OLED_DEMO_H
#define __TRACK_OLED_DEMO_H

#include "soft_i2c_simple.h"

/**
 * @brief  刷新一帧循迹状态到 OLED
 * @param  oled_bus  OLED 所在的 I2C 总线对象 (&i2c_pca9685)
 * @note   调用频率建议 10~50Hz, 内部有静态帧计数器用于判断是否卡住
 *
 *         显示布局:
 *         Line 1: F:0123 T:2 *    帧计数 / 活跃通道 / 闪烁点
 *         Line 2: Bin:00011000    8路二进制 (MSB左)
 *         Line 3: Vis:..####..    可视化 ('.'=白 '#'=黑)
 *         Line 4: Err: -3         加权偏差
 */
void TrackOLED_ShowStatus(SoftI2C_Obj *oled_bus);

#endif

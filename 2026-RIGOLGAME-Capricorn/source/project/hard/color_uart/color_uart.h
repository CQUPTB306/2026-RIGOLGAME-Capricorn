#ifndef COLOR_UART_H
#define COLOR_UART_H

#include "stdint.h"

#define COLOR_ID_MAX   4   // 颜色ID 0~3

typedef struct {
    uint8_t id;
    int16_t x;
    int16_t y;
    uint8_t detected;       // 当前帧是否检测到该颜色
} ColorInfo_t;

typedef struct {
    ColorInfo_t colors[COLOR_ID_MAX];
    uint8_t frame_complete; // 一帧接收完成标志（由中断置1，用户读取后清零）
} ColorFrame_t;



void Color_UART_Init(void);
void Color_ClearFrame(void);                     // 清除所有检测标志及帧完成标志
uint8_t Color_IsFrameComplete(void);             // 查询是否收到完整一帧
uint8_t Color_IsDetected(uint8_t id);
uint8_t Color_GetCoordinates(uint8_t id, int16_t *x, int16_t *y);

#endif

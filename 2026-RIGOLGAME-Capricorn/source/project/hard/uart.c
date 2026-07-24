/**
 * @file    uart.c
 * @brief   UART/OpenMV 视觉对准存根
 * @note    Vision_GetAlignFlag 返回0（未对准），
 *          状态机超时后以 HAL_Delay(500ms) 降级
 */
#include "uart.h"
#include <stdio.h>
#include <string.h>

static uint8_t rx_data = 0;                    /* 单字节接收缓冲 */
static volatile uint8_t vision_align_flag = 0; /* 0=未对准, 1=偏移数据, 2=OK */
static volatile float vision_offset = 0;       /* 偏移量 */
static uint8_t rx_buf[50];
static uint8_t rx_cnt = 0;

void UART_Init(void)
{
    /* 默认由 Core/Src/main.c 的 Init_ALL 初始化 UART4 */
    /* 此处预留，当前不做额外操作 */
}

uint8_t Vision_GetAlignFlag(void) { return vision_align_flag; }
void Vision_ClearAlignFlag(void) { vision_align_flag = 0; }
float Vision_GetOffset(void) { return vision_offset; }

void OpenMV_SetMode(uint8_t mode)
{
    /* 预留：向 OpenMV 发送模式切换指令 */
    /* char buf[10]; */
    /* sprintf(buf, "M,%d\r\n", mode); */
    /* HAL_UART_Transmit(&huart4, (uint8_t*)buf, strlen(buf), 100); */
}

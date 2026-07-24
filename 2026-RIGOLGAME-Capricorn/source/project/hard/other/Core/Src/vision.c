#include "vision.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart1;

static uint8_t rx_data;                    // 接收单字节缓存
static volatile uint8_t vision_align_flag = 0;
static volatile float vision_offset = 0;
static uint8_t rx_buf[50];
static uint8_t rx_cnt = 0;

void UART_Init(void)
{
    // 启动第一个字节的接收
    HAL_UART_Receive_IT(&huart1, &rx_data, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1)
    {
        if(rx_data == '\n')
        {
            rx_buf[rx_cnt] = '\0';
            if(strncmp((char*)rx_buf, "C,", 2) == 0)
            {
                sscanf((char*)rx_buf, "C,%f", &vision_offset);
                vision_align_flag = 1;
            }
            else if(strncmp((char*)rx_buf, "OK", 2) == 0)
            {
                vision_align_flag = 2;
            }
            rx_cnt = 0;
        }
        else
        {
            if(rx_cnt < sizeof(rx_buf)-1)
                rx_buf[rx_cnt++] = rx_data;
            else
                rx_cnt = 0;
        }
        // 继续接收下一个字节
        HAL_UART_Receive_IT(&huart1, &rx_data, 1);
    }
}

void OpenMV_SetMode(uint8_t mode)
{
    char buf[10];
    sprintf(buf, "M,%d\r\n", mode);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);
    HAL_Delay(50);
}

uint8_t Vision_GetAlignFlag(void) { return vision_align_flag; }
void Vision_ClearAlignFlag(void) { vision_align_flag = 0; }
float Vision_GetOffset(void) { return vision_offset; }
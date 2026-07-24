#include "color_uart.h"
#include "string.h"
#include "stdio.h"
#include "stm32f1xx_hal.h"
#include "extern.h"

extern UART_HandleTypeDef huart4;



#define RX_BUF_SIZE  64
static char rx_buf[RX_BUF_SIZE];
static uint8_t rx_index = 0;
static uint8_t rx_byte;

static void Parse_Color_String(char *str)
{
    int id, x, y;
    if (sscanf(str, "C,%d,%d,%d", &id, &x, &y) == 3)
    {
        if (id >= 0 && id < COLOR_ID_MAX)
        {
            g_color_frame.colors[id].id = (uint8_t)id;
            g_color_frame.colors[id].x = (int16_t)x;
            g_color_frame.colors[id].y = (int16_t)y;
            g_color_frame.colors[id].detected = 1;
        }
    }
}

void Color_UART_Init(void)
{
    HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
}

void Color_ClearFrame(void)
{
    for (int i = 0; i < COLOR_ID_MAX; i++)
    {
        g_color_frame.colors[i].detected = 0;
    }
    g_color_frame.frame_complete = 0;
}

uint8_t Color_IsFrameComplete(void)
{
    return g_color_frame.frame_complete;
}

uint8_t Color_IsDetected(uint8_t id)
{
    if (id >= COLOR_ID_MAX) return 0;
    return g_color_frame.colors[id].detected;
}

uint8_t Color_GetCoordinates(uint8_t id, int16_t *x, int16_t *y)
{
    if (id >= COLOR_ID_MAX || !g_color_frame.colors[id].detected)
        return 0;
    if (x) *x = g_color_frame.colors[id].x;
    if (y) *y = g_color_frame.colors[id].y;
    return 1;
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if (huart->Instance == UART4)
//    {
//        if (rx_byte == '\n')
//        {
//            rx_buf[rx_index] = '\0';
//            if (rx_index > 0)
//            {
//                // 判断是否为帧结束标记 "E"
//                if (rx_buf[0] == 'E' && rx_index == 1)
//                {
//                    g_color_frame.frame_complete = 1;
//                }
//                else
//                {
//                    Parse_Color_String(rx_buf);
//                }
//            }
//            rx_index = 0;
//        }
//        else
//        {
//            if (rx_index < RX_BUF_SIZE - 1)
//            {
//                rx_buf[rx_index++] = rx_byte;
//            }
//            else
//            {
//                rx_index = 0;   // 溢出重置
//            }
//        }
//        HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
//    }
//}
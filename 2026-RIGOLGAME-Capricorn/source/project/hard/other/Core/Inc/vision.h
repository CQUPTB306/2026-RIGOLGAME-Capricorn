#ifndef VISION_H
#define VISION_H

#include <stdint.h>

void UART_Init(void);
void OpenMV_SetMode(uint8_t mode);
uint8_t Vision_GetAlignFlag(void);
void Vision_ClearAlignFlag(void);
float Vision_GetOffset(void);

#endif
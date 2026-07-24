#ifndef __UART_H
#define __UART_H

#include "main.h"

/* OpenMV visual alignment functions (stubs for now) */
void UART_Init(void);
void OpenMV_SetMode(uint8_t mode);
uint8_t Vision_GetAlignFlag(void);
void Vision_ClearAlignFlag(void);
float Vision_GetOffset(void);

#endif

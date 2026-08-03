/**
 * @file    board.h
 * @brief   MSPM0G3519 板级支持包 — Keil MDK5
 */

#ifndef __BOARD_H__
#define __BOARD_H__

#include "ti_msp_dl_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── 系统滴答 ── */
extern volatile uint32_t g_sys_tick_ms;

/* ── 初始化 ── */
void board_init(void);

/* ── 延时 ── */
void delay_us(unsigned long __us);
void delay_ms(unsigned long ms);
void delay_1us(unsigned long __us);
void delay_1ms(unsigned long ms);

/* ── UART ── */
void uart0_send_char(char ch);
void uart0_send_string(char *str);

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H__ */

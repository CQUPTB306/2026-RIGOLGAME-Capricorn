/**
 * @file    board.c
 * @brief   MSPM0G3519 板级支持包 — Keil MDK5
 * @note    基于 MSPM0G3507 CCS 版本移植:
 *          - SysTick 1ms 滴答计时器 (DL API)
 *          - delay_us / delay_ms 精确定时
 *          - UART0 printf 重定向 (Keil MicroLIB 兼容)
 *          - UART0 中断接收
 */

#include "board.h"
#include "stdio.h"

#define RE_0_BUFF_LEN_MAX   128

volatile uint8_t  recv0_buff[RE_0_BUFF_LEN_MAX] = {0};
volatile uint16_t recv0_length = 0;
volatile uint8_t  recv0_flag = 0;

/* 系统滴答计时器 — 由 SysTick_Handler 每 1ms 递增 */
volatile uint32_t g_sys_tick_ms = 0;

/* ── 板级初始化 ── */

void board_init(void)
{
    /* SYSCFG 初始化 (时钟+GPIO+外设) */
    SYSCFG_DL_init();

    /* 配置 SysTick 产生 1ms 定时中断
     * CPUCLK_FREQ = 80000000, 所以 ticks = 80000000 / 1000 = 80000 */
    DL_SYSTICK_init(80000);
    DL_SYSTICK_enableInterrupt();
    DL_SYSTICK_enable();

    printf("Board Init [[ MSPM0G3519 Keil5 ]]\r\n");
}

/* ── SysTick 中断 (1ms) ── */

void SysTick_Handler(void)
{
    g_sys_tick_ms++;
}

/* ── 微秒延时 (基于 SysTick 硬件计数器) ── */

void delay_us(unsigned long __us)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 38;

    ticks = __us * (80000000 / 1000000);  /* 80 ticks/us @ 80MHz */

    told = SysTick->VAL;

    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
                tcnt += told - tnow;
            else
                tcnt += SysTick->LOAD - tnow + told;
            told = tnow;
            if (tcnt >= ticks)
                break;
        }
    }
}

/* ── 毫秒延时 ── */

void delay_ms(unsigned long ms)
{
    delay_us(ms * 1000);
}

void delay_1us(unsigned long __us) { delay_us(__us); }
void delay_1ms(unsigned long ms)   { delay_ms(ms); }

/* ── UART0 发送 ── */

void uart0_send_char(char ch)
{
    while (DL_UART_isBusy(UART_0_INST) == true);
    DL_UART_Main_transmitData(UART_0_INST, ch);
}

void uart0_send_string(char *str)
{
    while (*str != 0 && str != 0)
    {
        uart0_send_char(*str++);
    }
}

/* ── printf 重定向 (Keil MicroLIB / 标准库兼容) ── */

#if defined(__CC_ARM) || defined(__ARMCLANG_VERSION)
/* Keil ARMCLANG / ARMCC */

#if defined(__MICROLIB)
/* MicroLIB: fputc() 即可 */
int fputc(int ch, FILE *stream)
{
    (void)stream;
    while (DL_UART_isBusy(UART_0_INST) == true);
    DL_UART_Main_transmitData(UART_0_INST, ch);
    return ch;
}
#else
/* 标准库: 需要重定向 __stdout */
#include <rt_misc.h>
/* 禁用半主机模式 */
#pragma import(__use_no_semihosting)

struct __FILE { int handle; };
FILE __stdout;

int fputc(int ch, FILE *stream)
{
    (void)stream;
    while (DL_UART_isBusy(UART_0_INST) == true);
    DL_UART_Main_transmitData(UART_0_INST, ch);
    return ch;
}

void _sys_exit(int x)
{
    (void)x;
    while (1);  /* 无限循环, 避免半主机调用 */
}
#endif

#else
/* GCC / TI 编译器 */
#if !defined(__MICROLIB)

#if (__ARMCLIB_VERSION <= 6000000)
struct __FILE
{
    int handle;
};
#endif

FILE __stdout;

void _sys_exit(int x)
{
    (void)x;
}

#endif

int fputc(int ch, FILE *stream)
{
    (void)stream;
    while (DL_UART_isBusy(UART_0_INST) == true);
    DL_UART_Main_transmitData(UART_0_INST, ch);
    return ch;
}
#endif

/* ── UART0 中断接收 ── */

void UART_0_INST_IRQHandler(void)
{
    uint8_t receivedData = 0;

    switch (DL_UART_getPendingInterrupt(UART_0_INST))
    {
        case DL_UART_IIDX_RX:
            receivedData = DL_UART_Main_receiveData(UART_0_INST);
            if (recv0_length < RE_0_BUFF_LEN_MAX - 1)
            {
                recv0_buff[recv0_length++] = receivedData;
                uart0_send_char(receivedData);  /* 回显 */
            }
            else
            {
                recv0_length = 0;
            }
            recv0_flag = 1;
            break;

        default:
            break;
    }
}

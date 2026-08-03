/**
 * @file    key.c
 * @brief   按键驱动 — MSPM0G3519, 2 路开关量输入
 * @note    外围上拉 + 内部上拉, 按下 = 低电平
 *          KEY1: PB11 (PINCM28)  KEY2: PA30 (PINCM5)
 *          去抖基于 g_sys_tick_ms (SysTick 1ms)
 */

#include "key.h"
#include "board.h"

/* ── 按键状态 ── */
static uint32_t g_key1_last_ms;
static uint32_t g_key2_last_ms;
static uint8_t  g_key1_locked;
static uint8_t  g_key2_locked;

/* ── 初始化 ── */
void Key_Init(void)
{
    /* KEY1 = PB11: 数字输入 + 内部上拉 */
    DL_GPIO_initDigitalInputFeatures(KEY1_IOMUX,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);

    /* KEY2 = PA30: 数字输入 + 内部上拉 */
    DL_GPIO_initDigitalInputFeatures(KEY2_IOMUX,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);

    g_key1_last_ms = g_sys_tick_ms;
    g_key2_last_ms = g_sys_tick_ms;
    g_key1_locked  = 0;
    g_key2_locked  = 0;
}

/* ── 原始电平 (0=按下, 1=释放) ── */
uint8_t Key_GetState(uint8_t key_id)
{
    if (key_id == KEY1_PRESS)
        return !!(DL_GPIO_readPins(KEY1_PORT, KEY1_PIN));
    else if (key_id == KEY2_PRESS)
        return !!(DL_GPIO_readPins(KEY2_PORT, KEY2_PIN));
    return 1;
}

/* ── 带去抖的按键读取 (下降沿触发, 每次按下只报一次) ── */
uint8_t Key_Read(void)
{
    uint8_t  result = KEY_NONE;
    uint32_t now    = g_sys_tick_ms;

    /* KEY1 (PB11) */
    if (!Key_GetState(KEY1_PRESS))
    {
        if (!g_key1_locked && (now - g_key1_last_ms >= KEY_DEBOUNCE_MS))
        {
            g_key1_locked  = 1;
            g_key1_last_ms = now;
            result = KEY1_PRESS;
        }
    }
    else
    {
        if (g_key1_locked)
        {
            g_key1_locked  = 0;
            g_key1_last_ms = now;
        }
    }

    /* KEY2 (PA30) */
    if (!Key_GetState(KEY2_PRESS))
    {
        if (!g_key2_locked && (now - g_key2_last_ms >= KEY_DEBOUNCE_MS))
        {
            g_key2_locked  = 1;
            g_key2_last_ms = now;
            if (result == KEY_NONE)
                result = KEY2_PRESS;
        }
    }
    else
    {
        if (g_key2_locked)
        {
            g_key2_locked  = 0;
            g_key2_last_ms = now;
        }
    }

    return result;
}
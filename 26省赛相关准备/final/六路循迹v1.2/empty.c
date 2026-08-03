/**
 * @file    empty.c
 * @brief   循迹 + TFT/数码管计时 — MSPM0G3519
 * @note    启动: KEY1 切换模式, KEY2 确认
 *          确认后: 开始计时 + 启动电机循迹
 *          刹车后: 停止电机, 冻结最终时间
 */

#include "ti_msp_dl_config.h"
#include "board.h"
#include "motor.h"
#include "track.h"
#include "tm1637.h"
#include "soft_i2c_track.h"
#include "st7735_tft.h"
#include "key.h"

/* ── 全局状态 ── */
static volatile uint8_t  g_tick;
static volatile uint8_t  g_disp_update;
static          uint32_t g_last_second_ms;
uint16_t        g_total_seconds;
static volatile uint8_t  g_running = 0;      /* 1=已确认模式, 运行中 */

/* ── TIMG0 中断 ── */
void TIMG0_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO)
    {
        g_tick = 1;

        /* 仅运行中计时, 刹车后停止 */
        if (g_running && !Track_IsBraking())
        {
            if ((g_sys_tick_ms - g_last_second_ms) >= 1000)
            {
                g_last_second_ms = g_sys_tick_ms;
                if (g_total_seconds < 3599) g_total_seconds++;
                g_disp_update = 1;
            }
        }
    }
}

/* ── TFT 显示模式名 ── */
static void TFT_ShowMode(void)
{
    TFT_ShowString(0, 40, TFT_CYAN, TFT_BLACK, (char *)g_mode_names[g_track_mode]);
}

/* ── 启动前模式选择: KEY1 切换, KEY2 确认 ── */
static void ModeSelect(void)
{
    uint8_t confirmed = 0;
    uint8_t last_k1 = 0, last_k2 = 0;

    TFT_Fill(0, 0, 128, 64, TFT_BLACK);
    TFT_ShowString(0, 0, TFT_YELLOW, TFT_BLACK, "Select Mode:");
    TFT_ShowString(0, 20, TFT_WHITE, TFT_BLACK, (char *)g_mode_names[g_track_mode]);
    TFT_ShowString(0, 40, TFT_WHITE, TFT_BLACK, "K1:Switch K2:OK");

    while (!confirmed)
    {
        uint8_t k1 = Key_GetState(KEY1_PRESS);
        uint8_t k2 = Key_GetState(KEY2_PRESS);

        /* KEY1 按下 (下降沿) → 切换模式 */
        if (k1 == 0 && last_k1 != 0)
        {
            uint8_t m = (g_track_mode == TRACK_MODE_FAST)
                        ? TRACK_MODE_SLOW : TRACK_MODE_FAST;
            Track_SetMode(m);
            TFT_Fill(0, 20, 128, 36, TFT_BLACK);
            TFT_ShowString(0, 20, TFT_WHITE, TFT_BLACK,
                           (char *)g_mode_names[g_track_mode]);
            delay_ms(200);
        }

        /* KEY2 按下 (下降沿) → 确认 */
        if (k2 == 0 && last_k2 != 0)
        {
            confirmed = 1;
            delay_ms(200);
        }

        last_k1 = k1;
        last_k2 = k2;
    }

    /* 确认后显示 */
    TFT_Fill(0, 0, 128, 64, TFT_BLACK);
    TFT_ShowString(0, 0, TFT_GREEN, TFT_BLACK, "Mode Confirmed!");
    TFT_ShowString(0, 20, TFT_YELLOW, TFT_BLACK, (char *)g_mode_names[g_track_mode]);
    TFT_ShowString(0, 40, TFT_WHITE, TFT_BLACK, "Starting...");
    delay_ms(500);
}

/* ── 主函数 ── */
int main(void)
{
    board_init();

    /* ── 红外 I2C ── */
    DL_GPIO_initDigitalOutput(IOMUX_PINCM2);
    DL_GPIO_enableOutput(TRACK_I2C_SCL_PORT, TRACK_I2C_SCL_PIN);
    DL_GPIO_initDigitalOutputFeatures(IOMUX_PINCM1,
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, 0);
    DL_GPIO_clearPins(TRACK_I2C_SDA_PORT, TRACK_I2C_SDA_PIN);
    DL_GPIO_disableOutput(TRACK_I2C_SDA_PORT, TRACK_I2C_SDA_PIN);

    Motor_Init();
    Track_Init();
    // TM1637_Init();
    Key_Init();
    TFT_Init();
    TFT_Clear(TFT_BLACK);

    /* ── 1. 模式选择 (确认前电机不动, 不计时) ── */
    ModeSelect();

    /* ── 2. 确认后启动: 清屏 + 开始计时 ── */
    TFT_Clear(TFT_BLACK);
    TFT_ShowString(0, 0, TFT_WHITE, TFT_BLACK, "Track Timer");
    TFT_ShowMode();
    // TM1637_ShowCountdown(0, 0);

    g_last_second_ms = g_sys_tick_ms;
    g_total_seconds  = 0;
    g_running = 1;                     /* 开始计时 */
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    while (1)
    { 
		
        /* ── 循迹 (每 tick, 确认后启动电机) ── */
        if (g_tick)
        {
            g_tick = 0;
            Track_Run();               /* 电机开始循迹 */
        }

        /* ── 显示刷新 (每秒) ── */
        if (g_disp_update)
        {
            g_disp_update = 0;

            if (Track_IsBraking())
            {
                /* 刹车后: 冻结最终时间 */
                // TM1637_ShowCountdown(g_total_seconds, 0);
                TFT_Fill(0, 20, 128, 36, TFT_BLACK);
                TFT_ShowString(0, 20, TFT_WHITE, TFT_BLACK, "Final:");
                TFT_ShowNumber(48, 20, TFT_YELLOW, TFT_BLACK, g_total_seconds);
                TFT_ShowString(72, 20, TFT_WHITE, TFT_BLACK, "s");
                TFT_ShowMode();
            }
            else
            {
                // TM1637_ShowCountdown(g_total_seconds, 0);
                TFT_ShowNumber(0, 20, TFT_WHITE, TFT_BLACK, g_total_seconds);
            }
        }
    }
}
	
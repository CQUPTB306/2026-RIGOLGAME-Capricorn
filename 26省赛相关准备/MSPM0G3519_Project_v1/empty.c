/**
 * @file    empty.c
 * @brief   MSPM0G3519 主程序 — Keil MDK5
 * @note    基于 MSPM0G3507 CCS 版本移植:
 *          - 循迹小车主控制循环
 *          - 6路 GPIO 循迹传感器 + PID 控制
 *          - 双路电机 PWM 驱动 (TB6612)
 *          - ST7735 TFT 状态显示 (100Hz 刷新)
 *          - 编码器速度闭环 (MOTOR_USE_ENCODER=1)
 */

#include "ti_msp_dl_config.h"
#include "motor.h"
#include "soft_i2c_simple.h"
#include "pca9685.h"
#include "track.h"
#include "oled.h"
#include "board.h"
#include <stdio.h>
#include "st7735_tft.h"
#include "image1.h"


/* ── 控制定时器中断 (TIMG0, 100Hz) ── */

void TIMG0_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(CONTROL_TIMER_INST))
    {
        case DL_TIMER_IIDX_ZERO:
            g_motor_control_flag = 1;
            break;
        default:
            break;
    }
}


/* ── 主函数 ── */

int main(void)
{
    /* 板级初始化: 时钟 + GPIO + 外设 (SYSCFG_DL_init) + SysTick */
    board_init();

    /* 使能控制定时器中断 (TIMG0, 100Hz) */
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    /* 外设初始化 */
    TFT_Init();                     /* ST7735 TFT, 默认黑屏 */
    SoftI2C_Init(&i2c_pca9685);     /* PA28/PA31: PCA9685 + OLED 共享 I2C */
    delay_ms(10);
    pca9685_Init();                 /* PCA9685 舵机驱动 */
    Motor_Init();                   /* 电机 PWM + 方向引脚 */
    Track_Init();                   /* 6路循迹传感器 GPIO */
    PID_Init(&PID, 1.0f, 0, 0);    /* Kp=5.0, Ki=0, Kd=0 */

    /* 局部变量 */
    uint8_t  Data       = 0;
    int16_t  left_Speed  = PWM_MAX;
    int16_t  right_Speed = PWM_MAX;

    /* ── 主循环 ── */
    while (1)
    {
        /* 循迹控制 (PID + 状态机 → Motor_Set) */
        Track_Run();

        /* 编码器速度闭环: 如需, 取消注释 Motor_Control_Loop()
         * 当前使用纯开环 PWM, 直接由 Track_Run 控制 */
         //Motor_Set(191, 209);  // 调试: 直接测试电机

//        /* TFT 显示刷新 (100Hz, 由 TIMG0 控制定时器触发) */
//        if (g_motor_control_flag == 1)
//        {
//            /* 读取传感器状态 */
//            Data = Track_Read_All();

//            /* 读取当前 PWM 占空比 (用于调试显示) */
//            left_Speed  = PWM_MAX -
//                DL_TimerA_getCaptureCompareValue(PWM_0_INST, DL_TIMER_CC_0_INDEX);
//            right_Speed = PWM_MAX -
//                DL_TimerA_getCaptureCompareValue(PWM_0_INST, DL_TIMER_CC_1_INDEX);

//            /* TFT 显示: 6路传感器状态 */
//            TFT_ShowNumber(10, 10, TFT_BLUE, TFT_BLACK, (Data >> 0) & 0x01);
//            TFT_ShowNumber(10 * 2, 10, TFT_BLUE, TFT_BLACK, (Data >> 1) & 0x01);
//            TFT_ShowNumber(10 * 3, 10, TFT_BLUE, TFT_BLACK, (Data >> 2) & 0x01);
//            TFT_ShowNumber(10 * 4, 10, TFT_BLUE, TFT_BLACK, (Data >> 3) & 0x01);
//            TFT_ShowNumber(10 * 5, 10, TFT_BLUE, TFT_BLACK, (Data >> 4) & 0x01);
//            TFT_ShowNumber(10 * 6, 10, TFT_BLUE, TFT_BLACK, (Data >> 5) & 0x01);

//            /* TFT 显示: 状态机 + PWM 速度 */
//            TFT_ShowNumber(10, 30, TFT_BLUE2, TFT_BLACK, g_track_state);
//            TFT_ShowNumber(10, 50, TFT_BLUE2, TFT_BLACK, left_Speed);
//            TFT_ShowNumber(10, 70, TFT_BLUE2, TFT_BLACK, right_Speed);

//            /* TFT 显示: 转弯计数 / 最大次数 */
//            TFT_ShowNumber(10, 90, TFT_YELLOW, TFT_BLACK, g_turn_count);
//            TFT_ShowNumber(40, 90, TFT_YELLOW, TFT_BLACK, TURN_MAX_COUNT);

//            /* 清除标志 */
//            g_motor_control_flag = 0;
//        }
    }
}

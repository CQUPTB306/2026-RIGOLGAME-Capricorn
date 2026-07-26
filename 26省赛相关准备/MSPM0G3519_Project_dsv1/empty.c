/**
 * @file    empty.c
 * @brief   MSPM0G3519 主程序 — 级联 PID 循迹小车 (Keil MDK5)
 *
 *          架构:
 *          - 100Hz TIMG0 中断 -> g_motor_control_flag -> 主循环轮询
 *          - Motor_Control_Loop() 内含完整感知+策略+执行:
 *            读灰度 -> 循迹PID -> 目标速度 -> 读编码器 -> 速度PID -> PWM
 *          - 8路灰度 MUX 循迹传感器
 *          - 双路编码器速度闭环 (硬件 QEI)
 *          - TB6612 电机驱动 (PA25/PA27/PA22/PB24 + PB4/PB5 PWM)
 */

#include "ti_msp_dl_config.h"
#include "motor.h"
#include "grayscale.h"
#include "track.h"
#include "board.h"

/* -- 控制定时器中断 (TIMG0, 100Hz) -- */

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

/* -- 主函数 -- */

int main(void)
{
    /* 板级初始化: 时钟 + GPIO + 外设 (SYSCFG_DL_init) + SysTick */
    board_init();

    /* 使能控制定时器中断 (TIMG0, 100Hz) */
    NVIC_EnableIRQ(CONTROL_TIMER_INST_INT_IRQN);

    /* 外设初始化 */
    Motor_Init();           /* 电机 GPIO + PWM 启动            */
    Motor_Control_Init();   /* 双编码器 + 速度 PID 初始化      */
    Track_Init();           /* 8路灰度 + 循迹 PID + 状态机     */
    Grayscale_Init();       /* 灰度 MUX 引脚初始化 (Track_Init 已调用, 此处冗余安全) */

    /* -- 主循环 -- */
    while (1)
    {
        /* 100Hz 控制周期: TIMG0 中断标志位触发 */
        if (g_motor_control_flag == 1)
        {
            g_motor_control_flag = 0;

            /* 循迹控制 (8路灰度 -> 偏差 -> TrackPID -> Motor_SetSpeed) */
            Track_Run();

            /* 速度闭环 (读编码器 -> 速度 PID -> Motor PWM 输出)
             * 内部根据 track_flag/turn_flag 选择 PID 输出或硬转弯值 */
            Motor_Control_Loop();
        }

        /* 其他低优先级任务可在此处添加 (如 TFT 显示刷新等) */
    }
}

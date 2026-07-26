/**
 * @file    track.c
 * @brief   6路 GPIO 循迹传感器模块 — MSPM0G3507
 * @note    基于 STM32 track.c 移植, 改用 GPIO 直读替代 I2C
 *          GPIO 读取 → 加权偏差 → PID → Motor_Set
 */

#include "track.h"
#include "track_gpio.h"
#include "motor.h"
#include "board.h"

/*==================== 全局变量 ====================*/

PID_TypeDef PID;

int  IR_Weight[6] = {-90, -39, -26, 23, 38, 90};
uint16_t BASE_SPEED = 240;  // 修复: uint8_t→uint16_t, 避免溢出 (原uint8_t最大值255, 400溢出=144)
float ir = 1.0f;
volatile int8_t g_track_error = 0;

/*==================== 直角转弯状态 ====================*/

Track_State g_track_state = TRACK_STATE_FOLLOW;
volatile uint8_t g_turn_count = 0;      /* 转弯次数计数器 */
static int    g_last_error_sign = 0;
static uint32_t g_all_white_start_ms = 0;
static uint32_t g_turn_start_ms = 0;
static uint32_t g_turn_exit_ms = 0;     /* 上次转弯结束时刻 (计数去重用) */
static uint32_t g_last_count_ms = 0;   /* 上次成功计数的时刻 (与 g_turn_exit_ms 解耦, 虚假退出不污染) */

/*==================== 初始化 ====================*/

void Track_Init(void)
{
    TrackGPIO_Init();
    g_turn_count = 0;           /* 复位转弯计数器 */
    g_turn_exit_ms = 0;         /* 复位去重时间戳 (0=尚未转过弯, 首个弯必计数) */
    g_last_count_ms = 0;        /* 复位计数时刻 */
    delay_ms(10);
}

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd)
{
    // 1. 设置 PID 参数
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    // 2. 清零历史数据，确保从零开始
    pid->error = 0;
    pid->last_error = 0;
    pid->P = 0;
    pid->I = 0;
    pid->D = 0;
    pid->output = 0;
}
/*==================== 传感器读取 ====================*/

uint8_t Track_Read_All(void)
{
    return TrackGPIO_Read_All();
}

uint8_t Track_Read_Channel(uint8_t channel)
{
    uint8_t data;
    if (channel > 5) return 0;
    data = Track_Read_All();
    return (data >> channel) & 0x01;
}

/*==================== 加权偏差 ====================*/

int Get_Track_Error(void)
{
    uint8_t status = Track_Read_All();   /* 0=黑线, 1=白底 */
    int sum = 0, count = 0;

    for (int i = 0; i < 6; i++)
    {
        if ((status & (1 << i)) == 0)    /* 检测到黑线 */
        {
            sum += IR_Weight[i];
            count++;
        }
    }

    if (count == 0) return 0;   /* 全白 → 无偏差 */
    return sum / count;
}

/*==================== PID 计算 ====================*/

int PID_Calc(int error)
{
    PID.error = error;
    PID.P = (int)(PID.Kp * PID.error);
    PID.I += (int)(PID.Ki * PID.error);
    PID.D = (int)(PID.Kd * (PID.error - PID.last_error));
    PID.output = PID.P + PID.I + PID.D;
    PID.last_error = PID.error;
    return PID.output;
}


/*==================== 转弯触发 ====================*/

/* 触发一次转弯: 计数去重 + 状态切换
 * 同一物理弯可能因 FOLLOW/TURN 状态抖动被多次触发,
 * 距上次成功计数不足 TURN_COOLDOWN_MS 的触发只转弯、不重复计数
 * 注: 使用 g_last_count_ms (上次计数时刻) 而非 g_turn_exit_ms, 避免虚假退出污染去重窗口 */
static void Track_Enter_Turn(void)
{
    g_all_white_start_ms = 0;   /* 无论转弯还是停车, 全白计时都重新开始 */
#if TRACK_TURN_COUNT_ENABLE
    if (g_last_count_ms == 0 ||
        (g_sys_tick_ms - g_last_count_ms) >= TURN_COOLDOWN_MS)
    {
        g_turn_count++;              /* 计一次转弯 */
        g_last_count_ms = g_sys_tick_ms;  /* 记录计数时刻 */
        if (g_turn_count >= TURN_MAX_COUNT)
        {
            Motor_Set(0, 0);
            g_track_state = TRACK_STATE_STOP;
            return;
        }
    }
#endif
    g_track_state = TRACK_STATE_TURN;
    g_turn_start_ms = g_sys_tick_ms;
}


/*==================== 循迹主控 ====================*/

void Track_Run(void)
{
    #define SPEED_MAX  999
    #define SPEED_MIN  0

    /*==================== 读取传感器 ====================*/
    uint8_t status = Track_Read_All();
    int count = 0, sum = 0, ch_cnt = 0;
    int error = 0;
    
    /* 计算黑线数量和加权偏差 (0=黑线, 1=白底) */
    for (int i = 0; i < 6; i++)
    {
        if ((status & (1 << i)) == 0)
        {
            count++;
            sum    += IR_Weight[i];
            ch_cnt++;
        }
    }
    error = (ch_cnt > 0) ? (sum / ch_cnt) : 0;

    /* 记住上次偏差方向 (有黑线时更新) */
    if (count > 0)
    {
        if (error > 0) g_last_error_sign =  1;
        if (error < 0) g_last_error_sign = -1;
    }

    /*==================== 状态机 ====================*/

    switch (g_track_state)
    {
    case TRACK_STATE_FOLLOW:
        if (count == 0)
        {
           g_track_state = TRACK_STATE_TURN;
            g_turn_start_ms = g_sys_tick_ms;
            /* 全白 → 冲过了直角弯, 开始计时 */
            if (g_all_white_start_ms == 0)
                g_all_white_start_ms = g_sys_tick_ms;

            if ((g_sys_tick_ms - g_all_white_start_ms) >= TURN_ALL_WHITE_MS)
            {
                Track_Enter_Turn();
            }
        }
        else
        {
            /* 有部分黑线 → 清计时, 正常循迹 */
            g_all_white_start_ms = 0;

            int pwm   = PID_Calc(error);
						int left  = 190 + pwm;
            int right = 211 - pwm*1.1;
            if (left  > SPEED_MAX) left  = SPEED_MAX;
            if (left  < SPEED_MIN) left  = SPEED_MIN;
            if (right > SPEED_MAX) right = SPEED_MAX;
            if (right < SPEED_MIN) right = SPEED_MIN;

            Motor_Set(left, right);
        }
        break;

    case TRACK_STATE_TURN:
        /* 超时保护 */
        if ((g_sys_tick_ms - g_turn_start_ms) > TURN_TIMEOUT_MS)
        {
#if TRACK_TURN_COUNT_ENABLE
            /* 补计数: 若转弯入口未计数(全白噪声直入TURN), 在超时出口补上 */
            if (g_last_count_ms < g_turn_start_ms
                && (g_sys_tick_ms - g_turn_start_ms) >= TURN_MIN_DURATION_MS)
            {
                if (g_last_count_ms == 0 ||
                    (g_sys_tick_ms - g_last_count_ms) >= TURN_COOLDOWN_MS)
                {
                    g_turn_count++;
                    g_last_count_ms = g_sys_tick_ms;
                    if (g_turn_count >= TURN_MAX_COUNT)
                    {
                        Motor_Set(0, 0);
                        g_track_state = TRACK_STATE_STOP;
                        break;
                    }
                }
            }
#endif
            Motor_Set(0, 0);      /* 超时强制停止 */
            g_track_state = TRACK_STATE_FOLLOW;
            g_all_white_start_ms = 0;
            g_turn_exit_ms = g_sys_tick_ms;
            break;
        }

        /* 重新检测到"线"(1~2路) → 恢复循迹 (直接在此计算并设置电机, 避免空转一个周期)
         * 注意: count>=3 说明仍压在横向黑线上, 不退出、继续旋转 —
         * 否则回到 FOLLOW 会立即再次触发转弯, 同一个弯被反复计数 */
        if (count > 0)
        {
#if TRACK_TURN_COUNT_ENABLE
            /* 补计数: 若转弯入口未计数(全白噪声直入TURN), 在找线出口补上 */
            if (g_last_count_ms < g_turn_start_ms
                && (g_sys_tick_ms - g_turn_start_ms) >= TURN_MIN_DURATION_MS)
            {
                if (g_last_count_ms == 0 ||
                    (g_sys_tick_ms - g_last_count_ms) >= TURN_COOLDOWN_MS)
                {
                    g_turn_count++;
                    g_last_count_ms = g_sys_tick_ms;
                    if (g_turn_count >= TURN_MAX_COUNT)
                    {
                        Motor_Set(0, 0);
                        g_track_state = TRACK_STATE_STOP;
                        break;
                    }
                }
            }
#endif
            g_track_state = TRACK_STATE_FOLLOW;
            g_all_white_start_ms = 0;
            g_turn_exit_ms = g_sys_tick_ms;

            /* 立即计算循迹电机值, 避免本周期无 Motor_Set 调用 */
            {
                int pwm   = PID_Calc(error);
                int left  = 190 + pwm;
                int right = 211 - pwm*1.1;

                if (left  > SPEED_MAX) left  = SPEED_MAX;
                if (left  < SPEED_MIN) left  = SPEED_MIN;
                if (right > SPEED_MAX) right = SPEED_MAX;
                if (right < SPEED_MIN) right = SPEED_MIN;

                Motor_Set(left, right);
            }
            break;
        }

        /* 按上次偏差方向硬转 */
        // if (g_last_error_sign > 0)
        // {
        //     /* 线在右侧消失 → 右转 (左轮正转, 右轮反转) */
        //     Motor_Set(TURN_SPEED, -TURN_SPEED);
        // }
        // else
        // {
        //     /* 线在左侧消失 → 左转 (左轮反转, 右轮正转) */
        //     Motor_Set(-TURN_SPEED, TURN_SPEED);
        // }
        Motor_Set(-TURN_SPEED, TURN_SPEED);
        break;

#if TRACK_TURN_COUNT_ENABLE
    case TRACK_STATE_STOP:
        /* 转弯次数已达标, 停止循迹 */
        Motor_Set(0, 0);
        break;
#endif
    }
}

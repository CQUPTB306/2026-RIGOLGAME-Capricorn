/**
 * @file    track.c
 * @brief   8路灰度循迹控制 — 级联 PID 外环
 *
 *          架构:
 *            1. Grayscale_Read_All() → 8路数字量
 *            2. 加权偏差 → TrackPID → 速度偏差 (mm/s)
 *            3. Motor_SetSpeed(BASE ± track_out) → 设定目标速度
 *
 *          直角转弯状态机逻辑保持不变 (参考原始 track.c)
 */

#include "track.h"
#include "grayscale.h"
#include "motor.h"
#include "board.h"

/* ==================== 全局变量 ==================== */

TrackPID_TypeDef TrackPID;

/* 8路加权值: 初始线性内插原始 6路值, 实际需赛道标定
 * 通道 0 (最左) 到 通道 7 (最右) */
int IR_Weight[8] = {-90, -60, -39, -26, 0, 23, 38, 90};

float BASE_SPEED_MM_S = 200.0f;    /* 基础目标速度 ~200mm/s (替代原 PWM 240) */

Track_State g_track_state = TRACK_STATE_FOLLOW;
volatile uint8_t g_turn_count = 0;

/* ==================== 内部状态 ==================== */

static int      g_last_error_sign    = 0;
static uint32_t g_all_white_start_ms = 0;
static uint32_t g_turn_start_ms      = 0;
static uint32_t g_turn_exit_ms       = 0;
static uint32_t g_last_count_ms      = 0;

/* ==================== 初始化 ==================== */

void TrackPID_Init(TrackPID_TypeDef *pid, float kp, float ki, float kd)
{
    pid->Kp         = kp;
    pid->Ki         = ki;
    pid->Kd         = kd;
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->output     = 0;
}

void Track_Init(void)
{
    Grayscale_Init();
    TrackPID_Init(&TrackPID, 1.0f, 0.0f, 0.0f);  /* 初始仅 P, I/D 后续调参加入 */

    g_track_state  = TRACK_STATE_FOLLOW;
    g_turn_count   = 0;
    g_turn_exit_ms = 0;
    g_last_count_ms = 0;

    track_flag = 1;   /* 使能循迹模式 */
    turn_flag  = 0;

    delay_ms(10);
}

/* ==================== 加权偏差 ==================== */

int Get_Track_Error(void)
{
    uint8_t sensor[8];
    int sum = 0, count = 0;

    Grayscale_Read_All(sensor);

    for (int i = 0; i < 8; i++)
    {
        if (sensor[i] == 0)          /* 检测到黑线 */
        {
            sum += IR_Weight[i];
            count++;
        }
    }

    if (count == 0) return 0;        /* 全白 → 无偏差 */
    return sum / count;
}

/* ==================== 循迹 PID 计算 ==================== */

/**
 * @brief 循迹 PID — 输出速度偏差 (mm/s), 非 PWM
 *
 * 公式: output = Kp*e + Ki*∫e*dt + Kd*de/dt
 * 不做输出限幅 (限幅在 Motor() 层做)
 */
int TrackPID_Calc(int error)
{
    TrackPID_TypeDef *pid = &TrackPID;

    /* 比例 */
    float Pout = pid->Kp * error;

    /* 积分 */
    pid->integral += error;
    float Iout = pid->Ki * pid->integral;

    /* 微分 */
    float derivative = error - pid->prev_error;
    float Dout = pid->Kd * derivative;

    pid->prev_error = error;
    pid->output = (int)(Pout + Iout + Dout);

    return pid->output;
}

/* ==================== 转弯触发 (保持原有去重逻辑) ==================== */

static void Track_Enter_Turn(void)
{
    g_all_white_start_ms = 0;

#if TRACK_TURN_COUNT_ENABLE
    if (g_last_count_ms == 0 ||
        (g_sys_tick_ms - g_last_count_ms) >= TURN_COOLDOWN_MS)
    {
        g_turn_count++;
        g_last_count_ms = g_sys_tick_ms;

        if (g_turn_count >= TURN_MAX_COUNT)
        {
            Motor_Stop();
            g_track_state = TRACK_STATE_STOP;
            track_flag = 0;
            turn_flag  = 0;
            return;
        }
    }
#endif
    g_track_state = TRACK_STATE_TURN;
    g_turn_start_ms = g_sys_tick_ms;

    /* 切换为转弯模式: 速度闭环使用硬转弯值 */
    track_flag = 0;
    turn_flag  = 1;
    /* 设定硬转弯目标速度: 左轮反转, 右轮正转 (原地左转) */
    Motor_SetSpeed(-TURN_SPEED_MM_S, TURN_SPEED_MM_S);
}

/* ==================== 循迹主控 ==================== */

void Track_Run(void)
{
    uint8_t sensor[8];
    int count = 0, sum = 0, ch_cnt = 0;
    int error = 0;

    /* 1. 读 8 路灰度 */
    Grayscale_Read_All(sensor);

    /* 2. 计算黑线检测数和加权偏差 */
    for (int i = 0; i < 8; i++)
    {
        if (sensor[i] == 0)          /* 0=黑线 */
        {
            count++;
            sum    += IR_Weight[i];
            ch_cnt++;
        }
    }
    error = (ch_cnt > 0) ? (sum / ch_cnt) : 0;

    /* 记住上次偏差方向 */
    if (count > 0)
    {
        if (error > 0) g_last_error_sign =  1;
        if (error < 0) g_last_error_sign = -1;
    }

    /* 3. 状态机 */
    switch (g_track_state)
    {
    case TRACK_STATE_FOLLOW:
        if (count == 0)
        {
            /* 全白: 开始计时 */
            if (g_all_white_start_ms == 0)
                g_all_white_start_ms = g_sys_tick_ms;

            if ((g_sys_tick_ms - g_all_white_start_ms) >= TURN_ALL_WHITE_MS)
            {
                Track_Enter_Turn();
            }
        }
        else
        {
            /* 有黑线: 正常循迹 */
            if (count == 8)
            {
                /* 全黑 (十字路口): 用上次偏差方向维持 */
                error = g_last_error_sign * 30;  /* 模拟中等偏差 */
            }
            g_all_white_start_ms = 0;
            track_flag = 1;
            turn_flag  = 0;
            int track_out = TrackPID_Calc(error);
            float left_target  = BASE_SPEED_MM_S + track_out;
            float right_target = BASE_SPEED_MM_S - track_out * 1.1f;
            Motor_SetSpeed(left_target, right_target);
        }
        break;

    case TRACK_STATE_TURN:
        /* 超时保护 */
        if ((g_sys_tick_ms - g_turn_start_ms) > TURN_TIMEOUT_MS)
        {
#if TRACK_TURN_COUNT_ENABLE
            /* 补计数 */
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
                        Motor_Stop();
                        g_track_state = TRACK_STATE_STOP;
                        track_flag = 0;
                        turn_flag  = 0;
                        break;
                    }
                }
            }
#endif
            Motor_Stop();
            g_track_state = TRACK_STATE_FOLLOW;
            TrackPID.integral = 0.0f;   /* 转弯结束 → 清积分防止积分饱和 */
            g_all_white_start_ms = 0;
            g_turn_exit_ms = g_sys_tick_ms;
            turn_flag = 0;
            track_flag = 1;
            break;
        }

        /* 重新检测到线 (1~2路) → 恢复循迹 */
        if (count > 0 && count <= 2)
        {
#if TRACK_TURN_COUNT_ENABLE
            /* 补计数 */
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
                        Motor_Stop();
                        g_track_state = TRACK_STATE_STOP;
                        track_flag = 0;
                        turn_flag  = 0;
                        break;
                    }
                }
            }
#endif
            g_track_state = TRACK_STATE_FOLLOW;
            TrackPID.integral = 0.0f;   /* 转弯结束 → 清积分防止积分饱和 */
            g_all_white_start_ms = 0;
            g_turn_exit_ms = g_sys_tick_ms;

            /* 立即恢复正常循迹 */
            turn_flag = 0;
            track_flag = 1;
            int track_out = TrackPID_Calc(error);
            float left_target  = BASE_SPEED_MM_S + track_out;
            float right_target = BASE_SPEED_MM_S - track_out * 1.1f;
            Motor_SetSpeed(left_target, right_target);
            break;
        }

        /* 否则: 继续原地旋转 (hard turn) —
         * Motor_Control_Loop 中 turn_flag==1 已处理 */
        break;

#if TRACK_TURN_COUNT_ENABLE
    case TRACK_STATE_STOP:
        Motor_Stop();
        track_flag = 0;
        turn_flag  = 0;
        break;
#endif
    }
}

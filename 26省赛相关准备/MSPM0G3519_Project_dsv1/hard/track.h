/**
 * @file    track.h
 * @brief   8路灰度循迹控制 — MSPM0G3519
 * @note    级联 PID 外环: 8路灰度 MUX → 加权偏差 → 循迹 PID → 目标速度 (mm/s)
 *          转弯状态机 + 计数保持原有逻辑
 *
 *          IR_Weight[8]: 8路线性内插原始 6路权值
 *          原始: [-90, -39, -26, 23, 38, 90]
 *          扩展: [-90, -60, -39, -26, 0, 23, 38, 90]  (实际需赛道标定)
 */

#ifndef __TRACK_H
#define __TRACK_H

#include <stdint.h>

/* ==================== 循迹 PID 结构 ==================== */

typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    int   output;               /* 速度偏差 (mm/s), 非 PWM! */
} TrackPID_TypeDef;

/* ==================== 全局变量 ==================== */

extern TrackPID_TypeDef TrackPID;
extern int     IR_Weight[8];         /* 8路加权值 */
extern float   BASE_SPEED_MM_S;      /* 基础速度 (mm/s), 替代原 BASE_SPEED */

/* ==================== 直角转弯状态机 ==================== */

typedef enum {
    TRACK_STATE_FOLLOW,
    TRACK_STATE_TURN,
    TRACK_STATE_STOP
} Track_State;

#define TURN_SPEED_MM_S         150.0f  /* 转弯时目标速度 (mm/s)    */
#define TURN_ALL_WHITE_MS       20      /* 全白持续触发的阈值 (ms)  */
#define TURN_TIMEOUT_MS         2000    /* 转弯超时保护 (ms)        */

#define TRACK_TURN_COUNT_ENABLE  1      /* 1=达上限停车; 0=无限     */
#define TURN_MAX_COUNT           12     /* 最大转弯次数              */
#define TURN_COOLDOWN_MS         1500   /* 同弯去重间隔 (ms)        */
#define TURN_MIN_DURATION_MS     80     /* 最短转弯时长 (ms)        */

extern Track_State g_track_state;
extern volatile uint8_t g_turn_count;

/* 系统滴答 (1ms) */
extern volatile uint32_t g_sys_tick_ms;

/* ==================== API ==================== */

void    Track_Init(void);
void    Track_Run(void);
int     Get_Track_Error(void);
int     TrackPID_Calc(int error);
void    TrackPID_Init(TrackPID_TypeDef *pid, float kp, float ki, float kd);

#endif

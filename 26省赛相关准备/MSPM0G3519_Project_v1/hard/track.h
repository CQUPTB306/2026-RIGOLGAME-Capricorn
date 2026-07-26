/**
 * @file    track.h
 * @brief   6路 GPIO 循迹传感器模块 — MSPM0G3507
 * @note    基于 STM32 track.c 移植, 改用 GPIO 直读替代 I2C
 *          GPIO: PB6 PB7 PB8 PB9 PB10 PB11
 *          加权偏差 + PID 控制
 */

#ifndef __TRACK_H
#define __TRACK_H

#include <stdint.h>

/*==================== 硬件配置 ====================*/

#define TRACK_I2C_ADDR    0x12
#define TRACK_REG_DATA    0x00

/*==================== PID 结构 ====================*/

typedef struct {
    float Kp, Ki, Kd;
    int   error;
    int   P, I, D;
    int   last_error;
    int   output;
} PID_TypeDef;

extern PID_TypeDef PID;

extern int     IR_Weight[6];
extern uint16_t BASE_SPEED;
extern float   ir;
extern volatile int8_t g_track_error;

/*==================== 直角转弯 ====================*/

typedef enum {
    TRACK_STATE_FOLLOW,
    TRACK_STATE_TURN,
    TRACK_STATE_STOP        /* 转弯次数达标, 停止循迹 */
} Track_State;

#define TURN_SPEED          300   /* 转弯时基准速度 */
#define TURN_ALL_WHITE_MS   20   /* 全白持续超过此时间才触发转弯 (ms) */
#define TURN_TIMEOUT_MS     2000  /* 转弯超时, 防止无限旋转 */

#define TRACK_TURN_COUNT_ENABLE  1   /* 1=达TURN_MAX_COUNT后停车; 0=不限次数, 永远循迹 */
#define TURN_MAX_COUNT           12   /* 最大转弯次数 (仅TRACK_TURN_COUNT_ENABLE=1时生效) */
#define TURN_COOLDOWN_MS         1500 /* 距上次成功计数不足此时间则只转弯不计数 (同一个弯去重, 1.5秒间隔) */
#define TURN_MIN_DURATION_MS       80  /* 转弯最短持续时间, 低于此时长视为噪声, 不计数 */

extern Track_State g_track_state;
extern volatile uint8_t g_turn_count;   /* 当前转弯计数 */

/* 系统滴答计时器 (1ms 递增, 由 SysTick_Handler 维护) */
extern volatile uint32_t g_sys_tick_ms;

/*==================== API ====================*/

void    Track_Init(void);
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd);
uint8_t Track_Read_All(void);
uint8_t Track_Read_Channel(uint8_t channel);
int     Get_Track_Error(void);
int     PID_Calc(int error);

void    Track_Run(void);

#endif

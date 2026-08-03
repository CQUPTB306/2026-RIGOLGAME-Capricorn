/**
 * @file    track.h
 * @brief   8路 I2C 循迹 — PD 差速, 无状态机
 *
 *          I2C: PA0=SDA, PA1=SCL, 从机地址 0x12
 *          0=黑线, 1=白底
 */

#ifndef __TRACK_H
#define __TRACK_H

#include <stdint.h>

/* ── I2C 硬件 ── */
#define TRACK_I2C_ADDR  0x12
#define TRACK_REG_DATA  0x00
#define TRACK_CH_NUM    8

/* ── PID 结构体 ── */
typedef struct {
    float Kp, Ki, Kd;
    int error, P, I, D, last_error, output;
} PID_TypeDef;

extern PID_TypeDef TrackPID;
extern int IR_Weight[TRACK_CH_NUM];
extern uint8_t BASE_SPEED;

/* ── 模式切换 ── */
#define TRACK_MODE_FAST   0    /* 高速模式 */
#define TRACK_MODE_SLOW   1    /* 低速模式 */
#define TRACK_MODE_COUNT  2
extern uint8_t  g_track_mode;
extern const char *g_mode_names[];

/* ── API ── */
void    Track_Init(void);
void    Track_SetMode(uint8_t mode);
void    Track_Run(void);
int     Get_Track_Error(void);
int     PID_Calc(int error);
uint8_t Track_CountBlack(void);
uint8_t Track_IsBraking(void);

/* ── 计时器 (由 empty.c 维护) ── */
extern uint16_t g_total_seconds;

#endif

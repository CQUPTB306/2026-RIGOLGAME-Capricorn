/**
 * @file    track.c
 * @brief   8路 I2C 循迹 — PD 差速, 无状态机
 *
 *          架构: I2C(0x12) → 8位 → 加权偏差 → PD → Motor(228+pwm, 211-pwm*1.1)
 */

#include "track.h"
#include "soft_i2c_track.h"
#include "motor.h"
#include "board.h"

/* ── 全局 ── */
PID_TypeDef TrackPID = {2.0f, 0.0f, 3.9f, 0, 0, 0, 0, 0, 0};
uint8_t BASE_SPEED = 228;

/* ── 模式参数 ── */
uint8_t  g_track_mode = TRACK_MODE_FAST;
const char *g_mode_names[TRACK_MODE_COUNT] = {
    "Fast",
    "Slow",
};

/* 高速模式参数 (当前工程) */
static const int IR_Weight_Fast[TRACK_CH_NUM] =
    {-120, -146, -123, -27, 27, 123, 146, 120};
#define SPEED_L_FAST  343
#define SPEED_R_FAST  323

/* 低速模式参数 (30s循迹工程) */
static const int IR_Weight_Slow[TRACK_CH_NUM] =
    {-120, -126, -93, -27, 27, 93, 126, 120};
#define SPEED_L_SLOW  258
#define SPEED_R_SLOW  236

/* 模式 PID (高速不变, 低速 D 项=3.4) */
static const PID_TypeDef PID_Fast = {2.0f, 0.0f, 3.9f, 0, 0, 0, 0, 0, 0};
static const PID_TypeDef PID_Slow = {2.0f, 0.0f, 3.9f, 0, 0, 0, 0, 0, 0};

/* ── 停车区降速: 计时≥此秒数后, 基数与 pwm 都 ×NUM/DEN, 让 6 线区更容易被抓到 ── */
#define TRACK_SLOWDOWN_SEC    17    /* 刹车武装后即开始降速, 可按赛道调 */
#define TRACK_SLOWDOWN_NUM    55    /* 降速系数: 50% */
#define TRACK_SLOWDOWN_DEN    100

/* ── 快速模式超时: 计时≥此秒数直接停 (不依赖黑线) ── */
#define TRACK_FAST_TIMEOUT_SEC  19
#define TRACK_SLOW_TIMEOUT_SEC  30

/* 当前使用的权重 (指向模式对应数组) */
int IR_Weight[TRACK_CH_NUM];

/* ── 刹车状态 ── */
static uint8_t g_brake = 0;        /* 0=未刹车, 1=已触发刹车 */

/* ── 去抖参数 ── */
#define TRACK_DEBOUNCE_SAMPLES   5
#define TRACK_DEBOUNCE_DELAY_MS  1

/* ==================== I2C 读取 (带去抖) ==================== */

static uint8_t _read_all(void)
{
    uint8_t last, cur, stable = 0;
    int timeout = 100;

    last = TRACK_I2C_Read_One_Byte(TRACK_I2C_ADDR, TRACK_REG_DATA);
    stable = 1;

    while (stable < TRACK_DEBOUNCE_SAMPLES && timeout > 0) {
        delay_ms(TRACK_DEBOUNCE_DELAY_MS);
        cur = TRACK_I2C_Read_One_Byte(TRACK_I2C_ADDR, TRACK_REG_DATA);
        if (cur == last) stable++;
        else { last = cur; stable = 1; }
        timeout--;
    }
    return last;
}

/* ==================== 初始化 ==================== */

void Track_Init(void)
{
    delay_ms(10);
    /* 加载默认(高速)权重 + 步长 */
    for (uint8_t i = 0; i < TRACK_CH_NUM; i++)
        IR_Weight[i] = IR_Weight_Fast[i];
    Motor_SetRamp(PWM_MAX);   /* 高速默认不缓启动 */
    Motor(0, 0);
}

/* ── 模式切换 ── */
void Track_SetMode(uint8_t mode)
{
    const int *src;
    const PID_TypeDef *pid;
    if (mode >= TRACK_MODE_COUNT) return;
    g_track_mode = mode;
    src = (mode == TRACK_MODE_SLOW) ? IR_Weight_Slow : IR_Weight_Fast;
    for (uint8_t i = 0; i < TRACK_CH_NUM; i++)
        IR_Weight[i] = src[i];

    /* 切换到对应模式 PID, 并清空误差历史 */
    pid = (mode == TRACK_MODE_SLOW) ? &PID_Slow : &PID_Fast;
    TrackPID.Kp = pid->Kp;
    TrackPID.Ki = pid->Ki;
    TrackPID.Kd = pid->Kd;
    TrackPID.error = TrackPID.P = TrackPID.I = 0;
    TrackPID.D = TrackPID.last_error = TrackPID.output = 0;

    /* 缓启动步长: 低速=25 (缓启), 高速=接近PWM_MAX (一步到位) */
    if (mode == TRACK_MODE_SLOW)
        Motor_SetRamp(10);
    else
        Motor_SetRamp(PWM_MAX);
}

/* ==================== 加权偏差 ==================== */

int Get_Track_Error(void)
{
    uint8_t status = _read_all();
    int sum = 0, count = 0;

    for (int i = 0; i < 8; i++) {
        if ((status & (1 << i)) == 0) {  /* 0=黑线 */
            sum += IR_Weight[i];
            count++;
        }
    }
    if (count == 0) return 0;
    return sum / count;
}

/* ==================== PID ==================== */

int PID_Calc(int error)
{
    PID_TypeDef *p = &TrackPID;
    p->error = error;
    p->P = (int)(p->Kp * error);
    p->I += (int)(p->Ki * error);
    p->D = (int)(p->Kd * (error - p->last_error));
    p->output = p->P + p->I + p->D;
    p->last_error = error;
    return p->output;
}

/* ==================== 黑线计数 ==================== */

uint8_t Track_CountBlack(void)
{
    uint8_t status = _read_all();
    uint8_t count  = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if ((status & (1 << i)) == 0) count++;  /* 0=黑线 */
    }
    return count;
}

uint8_t Track_IsBraking(void)
{
    return g_brake;
}

/* ==================== 循迹主控 ==================== */

void Track_Run(void)
{
    #define SPEED_MAX  999
    #define SPEED_MIN    0

    /* ── 刹车检查: 15 秒后, ≥6 条黑线 → 立即停车 ── */
    if (!g_brake && g_total_seconds >= 15 && g_track_mode == TRACK_MODE_FAST)
    {
        if (Track_CountBlack() >= 6)
        {
            g_brake = 1;
            Motor_Stop();
            return;
        }
    }

    /* ── 快速模式超时: 到点直接停 (仅高速, 低速无此限制) ── */
    if (!g_brake && g_track_mode == TRACK_MODE_FAST &&
		g_total_seconds >= TRACK_FAST_TIMEOUT_SEC)
	{
        g_brake = 1;
        Motor_Stop();
        return;
	}
    /* ── 低速模式超时: 到点直接停 (仅低速, 高速无此限制) ── */
    if (!g_brake && g_track_mode == TRACK_MODE_SLOW &&
		g_total_seconds >= TRACK_SLOW_TIMEOUT_SEC)
	{
        g_brake = 1;
        Motor_Stop();
        return;
	}

    /* 已刹车则不再驱动电机 */
    if (g_brake) return;

    int error = Get_Track_Error();
    int pwm   = PID_Calc(error);
    int left, right, speedL, speedR;

    if (g_track_mode == TRACK_MODE_SLOW) {
        speedL = SPEED_L_SLOW;
        speedR = SPEED_R_SLOW;
    } else {
        speedL = SPEED_L_FAST;
        speedR = SPEED_R_FAST;
    }

    /* 停车区降速: 计时≥17秒后减速, 基数与 pwm 等比缩小 (仅高速模式) */
    if (!g_brake && g_track_mode == TRACK_MODE_FAST &&
        g_total_seconds >= TRACK_SLOWDOWN_SEC)
    {
        speedL = speedL * TRACK_SLOWDOWN_NUM / TRACK_SLOWDOWN_DEN;
        speedR = speedR * TRACK_SLOWDOWN_NUM / TRACK_SLOWDOWN_DEN;
        pwm    = pwm    * TRACK_SLOWDOWN_NUM / TRACK_SLOWDOWN_DEN;
    }

    


    left  = speedL + pwm;
    right = speedR - pwm;

    if (left  > SPEED_MAX) left  = SPEED_MAX;
    if (left  < SPEED_MIN) left  = SPEED_MIN;
    if (right > SPEED_MAX) right = SPEED_MAX;
    if (right < SPEED_MIN) right = SPEED_MIN;

    Motor(-left, right);
}

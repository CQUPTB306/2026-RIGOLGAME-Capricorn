# MSPM0G3519 电机+循迹+速度闭环升级 — 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在现有 MSPM0G3519 竞赛小车代码上完成：电机引脚迁移、8路灰度循迹切换、双编码器速度闭环、中断驱动架构重构。

**Architecture:** 级联 PID — 循迹 PID (外环) 输出目标速度 (mm/s) → 速度 PID (内环) 输出 PWM → 驱动 TB6612。所有控制逻辑由 TIMG0 100Hz 中断触发标志位，主循环轮询执行浮点运算。双编码器硬件 QEI 独立测速。

**Tech Stack:** C (Keil MDK5), MSPM0G3519 (Cortex-M0+), TI DriverLib, SysConfig 生成外设初始化

---

## Global Constraints

- 所有引脚映射必须与 `引脚.txt` 一致: CIN1=PA25, CIN2=PA27, DIN1=PA22, DIN2=PB24, PWML=PB4, PWMR=PB5
- 编码器: ENC1_PHA=PA24, ENC1_PHB=PA26, ENC2_PHA=PA2, ENC2_PHB=PB7
- 8路灰度: AD0=PA12, AD1=PB23, AD2=PB27, OUT=PB8
- 速度 PID 参数: Kp=2.15, Ki=1.1, Kd=0.001, 一阶低通 0.08/0.92, 积分限幅 ±1000
- 控制周期: 100Hz (10ms), 由 TIMG0 中断驱动
- SysConfig 生成的符号 (`ti_msp_dl_config.h`) 为权威来源；代码宏仅做别名
- 保留现有 OLED/TFT/PCA9685/servo 功能不动

---

## File Structure

```
MSPM0G3519_Project_dsv1/
├── empty.c                    ← 修改: 简化主循环
├── board.c/h                  ← 不动
├── hard/
│   ├── grayscale.h            ← 新增: 8路灰度 MUX 驱动
│   ├── grayscale.c            ← 新增: MUX 切换 + 8路扫描
│   ├── encoder.h              ← 修改: 双编码器参数宏
│   ├── encoder.c              ← 重写: 双编码器 Init/Speed/Distance
│   ├── motor.h                ← 修改: 引脚宏, PWM 宏, SpeedCtrl, track_flag
│   ├── motor.c                ← 修改: 新引脚, Motor(), 速度 PID, Motor_Control_Loop()
│   ├── track.h                ← 修改: IR_Weight[8], TrackPID, BASE_SPEED
│   ├── track.c                ← 修改: 8路灰度 + TrackPID(输出速度) + 状态机
│   ├── track_gpio.h           ← 删除
│   ├── track_gpio.c           ← 删除
│   └── (其他文件不动)
└── docs/superpowers/
    ├── specs/2026-07-26-...-design.md
    └── plans/2026-07-26-...-plan.md  ← 本文件
```

**依赖关系**:
```
grayscale.h/c  (无依赖)
       ↓
track.h/c  (依赖 grayscale.h, motor.h)
       ↓
encoder.h/c  (无依赖, 独立模块)
       ↓
motor.h/c  (依赖 encoder.h)
       ↓
empty.c  (依赖所有以上)
```

---

### Task 1: 创建 8 路灰度 MUX 驱动 (grayscale.h + grayscale.c)

**Files:**
- Create: `hard/grayscale.h`
- Create: `hard/grayscale.c`
- Modify: `hard/track_gpio.h` — 后续 Task 6 删除
- Modify: `hard/track_gpio.c` — 后续 Task 6 删除

**Interfaces:**
- Consumes: `ti_msp_dl_config.h` (SysConfig GPIO 符号), `board.h` (delay_us)
- Produces:
  - `void Grayscale_Init(void)` — 初始化 AD0/AD1/AD2 为推挽输出, OUT 为输入上拉
  - `void Grayscale_Read_All(uint8_t sensor[8])` — 扫描 8 路, 存结果
  - `uint8_t Grayscale_Read_Single(uint8_t channel)` — 读单路 (0-7)

- [ ] **Step 1: 创建 `hard/grayscale.h`**

```c
/**
 * @file    grayscale.h
 * @brief   8路灰度传感器 MUX 驱动 — MSPM0G3519
 * @note    通过 AD0/AD1/AD2 三根地址线选择通道 (0-7),
 *          读取 OUT 引脚获得数字量 (0=黑线, 1=白底)
 *
 *          引脚:
 *            AD0  — PA12 (通道选择 bit0)
 *            AD1  — PB23 (通道选择 bit1)
 *            AD2  — PB27 (通道选择 bit2)
 *            OUT  — PB8  (传感器数据输出)
 */

#ifndef __GRAYSCALE_H__
#define __GRAYSCALE_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 通道数 ── */
#define GRAYSCALE_CHANNELS  8

/* ── 引脚宏 (与 SysConfig 生成的 GrayS_PORT/GrayS_PIN_x 符号对应) ── */

/* AD0 — PA12 */
#define GRAY_AD0_PORT       GPIOA
#define GRAY_AD0_PIN        DL_GPIO_PIN_12
#define GRAY_AD0_IOMUX      IOMUX_PINCM37

/* AD1 — PB23 */
#define GRAY_AD1_PORT       GPIOB
#define GRAY_AD1_PIN        DL_GPIO_PIN_23
#define GRAY_AD1_IOMUX      IOMUX_PINCM56

/* AD2 — PB27 */
#define GRAY_AD2_PORT       GPIOB
#define GRAY_AD2_PIN        DL_GPIO_PIN_27
#define GRAY_AD2_IOMUX      IOMUX_PINCM60

/* OUT — PB8 (数字输入, 内部上拉) */
#define GRAY_OUT_PORT       GPIOB
#define GRAY_OUT_PIN        DL_GPIO_PIN_8
#define GRAY_OUT_IOMUX      IOMUX_PINCM25

/* ── MUX 控制宏 (兼容参考代码风格) ── */

#define GRAY_AD0_WRITE(state)  do { \
    if (state) DL_GPIO_setPins(GRAY_AD0_PORT, GRAY_AD0_PIN); \
    else       DL_GPIO_clearPins(GRAY_AD0_PORT, GRAY_AD0_PIN); \
} while(0)

#define GRAY_AD1_WRITE(state)  do { \
    if (state) DL_GPIO_setPins(GRAY_AD1_PORT, GRAY_AD1_PIN); \
    else       DL_GPIO_clearPins(GRAY_AD1_PORT, GRAY_AD1_PIN); \
} while(0)

#define GRAY_AD2_WRITE(state)  do { \
    if (state) DL_GPIO_setPins(GRAY_AD2_PORT, GRAY_AD2_PIN); \
    else       DL_GPIO_clearPins(GRAY_AD2_PORT, GRAY_AD2_PIN); \
} while(0)

#define GRAY_OUT_READ()     (!!(DL_GPIO_readPins(GRAY_OUT_PORT, GRAY_OUT_PIN)))

/* ── API ── */

void    Grayscale_Init(void);
void    Grayscale_Read_All(uint8_t sensor[8]);
uint8_t Grayscale_Read_Single(uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif /* __GRAYSCALE_H__ */
```

- [ ] **Step 2: 创建 `hard/grayscale.c`**

```c
/**
 * @file    grayscale.c
 * @brief   8路灰度传感器 MUX 驱动实现
 * @note    参考 D:\!ziv\Nova\26省赛相关准备\八路灰度模块\源码\1.单片机数据读取\1.MSPM0G3507\Keil\BSP\grayscale_sensor.c
 */

#include "grayscale.h"
#include "board.h"   /* delay_us */

/* ── 内部: 选择 MUX 通道 (0-7) ── */
static void _select_channel(uint8_t channel)
{
    GRAY_AD0_WRITE((channel >> 0) & 0x01);
    GRAY_AD1_WRITE((channel >> 1) & 0x01);
    GRAY_AD2_WRITE((channel >> 2) & 0x01);
}

/* ── 初始化: AD0/AD1/AD2 推挽输出, OUT 输入上拉 ── */
void Grayscale_Init(void)
{
    /* AD0 — PA12, 推挽输出, 初始低电平 */
    DL_GPIO_initDigitalOutput(GRAY_AD0_IOMUX);
    DL_GPIO_clearPins(GRAY_AD0_PORT, GRAY_AD0_PIN);

    /* AD1 — PB23, 推挽输出, 初始低电平 */
    DL_GPIO_initDigitalOutput(GRAY_AD1_IOMUX);
    DL_GPIO_clearPins(GRAY_AD1_PORT, GRAY_AD1_PIN);

    /* AD2 — PB27, 推挽输出, 初始低电平 */
    DL_GPIO_initDigitalOutput(GRAY_AD2_IOMUX);
    DL_GPIO_clearPins(GRAY_AD2_PORT, GRAY_AD2_PIN);

    /* OUT — PB8, 数字输入 + 内部上拉 (高阻时默认高电平=白底) */
    DL_GPIO_initDigitalInputFeatures(GRAY_OUT_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}

/* ── 读取全部 8 路 ── */
void Grayscale_Read_All(uint8_t sensor[8])
{
    uint8_t i;
    for (i = 0; i < GRAYSCALE_CHANNELS; i++)
    {
        _select_channel(i);
        delay_us(50);                    /* 等待 MUX 输出稳定 */
        sensor[i] = GRAY_OUT_READ();     /* 0=黑线, 1=白底 */
    }
}

/* ── 读取单路 ── */
uint8_t Grayscale_Read_Single(uint8_t channel)
{
    if (channel >= GRAYSCALE_CHANNELS)
        return 0;

    _select_channel(channel);
    delay_us(50);
    return GRAY_OUT_READ();
}
```

- [ ] **Step 3: 验证** — 检查 `grayscale.h` 的 IOMUX 索引与 MSPM0G3519 数据手册一致 (PA12=IOMUX_PINCM37, PB23=IOMUX_PINCM56, PB27=IOMUX_PINCM60, PB8=IOMUX_PINCM25)。确认 `delay_us` 在 `board.h` 中已声明。

- [ ] **Step 4: Commit**

```bash
git add hard/grayscale.h hard/grayscale.c
git commit -m "feat: add 8-channel grayscale MUX driver (grayscale.h/c)

Replaces 6-channel GPIO direct-read track_gpio with 8-channel
multiplexed grayscale sensor. AD0/AD1/AD2 select channel 0-7,
OUT pin returns digital value (0=black, 1=white).

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 2: 重写双编码器驱动 (encoder.h + encoder.c)

**Files:**
- Modify: `hard/encoder.h`
- Modify: `hard/encoder.c`

**Interfaces:**
- Consumes: `ti_msp_dl_config.h` (QEI 实例符号: QEI_0_INST, QEI_1_INST 等)
- Produces:
  - `void Encoder_Init(void)` — 初始化双路 QEI, 清零累计值
  - `void Encoder_GetSpeed(float *speed_l, float *speed_r)` — 同时读取左右轮速度 (mm/s)
  - `float Encoder_GetSpeedLeft(void)` — 单独读左轮速度
  - `float Encoder_GetSpeedRight(void)` — 单独读右轮速度
  - `int32_t Encoder_GetDistance(void)` — 累计距离 (两轮平均, mm)
  - `void Encoder_Reset(void)` — 重置累计值

- [ ] **Step 1: 重写 `hard/encoder.h`**

```c
/**
 * @file    encoder.h
 * @brief   MSPM0G3519 双路硬件正交编码器 — TIMG QEI
 *
 *          左编码器: PA24(PHA) + PA26(PHB) → QEI_LEFT  (TIMGx)
 *          右编码器: PA2(PHA)  + PB7(PHB)  → QEI_RIGHT (TIMGy)
 *
 *          SysConfig 需配置两个 TIMG 为 QEI 模式 (4倍频).
 *          若 ENC2 跨端口无法配硬件 QEI, 则软件 GPIO 中断解码.
 */

#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ====================================================================
 *   编码器物理参数
 * ==================================================================== */

#define ENCODER_LINES           13          /* 编码器线数 (每圈脉冲)     */
#define GEAR_RATIO              30          /* 减速比                    */
#define WHEEL_DIAMETER          65.0f       /* 车轮直径 (mm)             */
#define PULSE_PER_ROUND         ((uint16_t)(ENCODER_LINES * GEAR_RATIO * 4))
#define MM_PER_PULSE            (3.1415926f * WHEEL_DIAMETER / PULSE_PER_ROUND)

/* 编码器方向修正 (+1 或 -1), 若实际方向相反则改符号 */
#define ENC_LEFT_DIR            1
#define ENC_RIGHT_DIR           1

/* 控制周期 (s), 必须与 TIMG0 定时器周期一致 */
#define CONTROL_PERIOD_S        0.01f

/* ====================================================================
 *   硬件 QEI 实例 (由 SysConfig 生成, 见 ti_msp_dl_config.h)
 * ==================================================================== */

/* 左编码器 — PA24(PHA) + PA26(PHB) */
#define QEI_LEFT_INST           QEI_0_INST
#define QEI_LEFT_PHA_PORT       GPIOA
#define QEI_LEFT_PHA_PIN        DL_GPIO_PIN_24
#define QEI_LEFT_PHB_PORT       GPIOA
#define QEI_LEFT_PHB_PIN        DL_GPIO_PIN_26

/* 右编码器 — PA2(PHA) + PB7(PHB)
 * 若 SysConfig 无法配置跨端口 QEI, 改 ENC2_USE_SOFTWARE 为 1 */
#define ENC2_USE_SOFTWARE       0   /* 0=硬件QEI, 1=GPIO中断软件解码 */

#if ENC2_USE_SOFTWARE
#define QEI_RIGHT_INST          0   /* 不使用硬件实例 */
#else
#define QEI_RIGHT_INST          QEI_1_INST
#endif
#define QEI_RIGHT_PHA_PORT      GPIOA
#define QEI_RIGHT_PHA_PIN       DL_GPIO_PIN_2
#define QEI_RIGHT_PHB_PORT      GPIOB
#define QEI_RIGHT_PHB_PIN       DL_GPIO_PIN_7

/* ====================================================================
 *                        公共 API
 * ==================================================================== */

void    Encoder_Init(void);
void    Encoder_GetSpeed(float *speed_l, float *speed_r);
float   Encoder_GetSpeedLeft(void);
float   Encoder_GetSpeedRight(void);
int32_t Encoder_GetDistance(void);
void    Encoder_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H__ */
```

- [ ] **Step 2: 重写 `hard/encoder.c`**

```c
/**
 * @file    encoder.c
 * @brief   MSPM0G3519 双路硬件正交编码器实现
 *
 *          左编码器 (ENC1): TIMGx QEI, PA24(PHA)+PA26(PHB)
 *          右编码器 (ENC2): TIMGy QEI, PA2(PHA)+PB7(PHB)
 *                         或 GPIO 中断软件解码 (ENC2_USE_SOFTWARE=1)
 */

#include "encoder.h"

/* ====================================================================
 *           内部状态
 * ==================================================================== */

static int16_t  s_last_count_l  = 0;
static int16_t  s_last_count_r  = 0;
static float    s_speed_l       = 0.0f;
static float    s_speed_r       = 0.0f;
static float    s_distance      = 0.0f;
static uint32_t s_call_count    = 0;

/* ====================================================================
 *           初始化
 * ==================================================================== */

void Encoder_Init(void)
{
    /* 左编码器 QEI — 启动计数器 */
    DL_TimerG_startCounter(QEI_LEFT_INST);
    s_last_count_l = (int16_t)DL_TimerG_getTimerCount(QEI_LEFT_INST);

    /* 右编码器 — 硬件 QEI 或软件中断 */
#if ENC2_USE_SOFTWARE
    /* 软件解码: 配置 PA2/PB7 为双边沿中断输入 (由 SysConfig 处理) */
    /* 中断处理在 stm32f1xx_it.c 对应文件中实现 */
    s_last_count_r = 0;
#else
    DL_TimerG_startCounter(QEI_RIGHT_INST);
    s_last_count_r = (int16_t)DL_TimerG_getTimerCount(QEI_RIGHT_INST);
#endif

    s_speed_l    = 0.0f;
    s_speed_r    = 0.0f;
    s_distance   = 0.0f;
    s_call_count = 0;
}

/* ====================================================================
 *           速度读取 (mm/s)
 * ==================================================================== */

/**
 * @brief 同时读取左右轮速度
 * @param[out] speed_l  左轮速度 (mm/s), 正=前进; 可为 NULL
 * @param[out] speed_r  右轮速度 (mm/s), 正=前进; 可为 NULL
 *
 * 公式: speed = delta * MM_PER_PULSE * ENC_DIR / CONTROL_PERIOD_S
 * delta = 当前计数值 - 上次计数值 (int16_t 减法自动处理 16位翻转)
 */
void Encoder_GetSpeed(float *speed_l, float *speed_r)
{
    /* ── 左编码器 ── */
    int16_t count_l = (int16_t)DL_TimerG_getTimerCount(QEI_LEFT_INST);
    int16_t delta_l = (count_l - s_last_count_l) * ENC_LEFT_DIR;
    s_last_count_l  = count_l;
    float spd_l     = (delta_l * MM_PER_PULSE) / CONTROL_PERIOD_S;

    /* ── 右编码器 ── */
#if ENC2_USE_SOFTWARE
    /* 软件解码: delta 由 GPIO 中断累加, 此处读取并清零 */
    int16_t delta_r = 0;  /* TODO: 从中断累加变量读取 */
    float spd_r     = (delta_r * MM_PER_PULSE) / CONTROL_PERIOD_S;
#else
    int16_t count_r = (int16_t)DL_TimerG_getTimerCount(QEI_RIGHT_INST);
    int16_t delta_r = (count_r - s_last_count_r) * ENC_RIGHT_DIR;
    s_last_count_r  = count_r;
    float spd_r     = (delta_r * MM_PER_PULSE) / CONTROL_PERIOD_S;
#endif

    /* 保存 */
    s_speed_l = spd_l;
    s_speed_r = spd_r;

    /* 累计距离 (两轮同向时累加平均) */
    if (spd_l > 0.0f && spd_r > 0.0f)
        s_distance += (float)((delta_l + delta_r) / 2) * MM_PER_PULSE;
    else if (spd_l < 0.0f && spd_r < 0.0f)
        s_distance -= (float)((delta_l + delta_r) / 2) * MM_PER_PULSE;

    s_call_count++;

    if (speed_l != NULL) *speed_l = spd_l;
    if (speed_r != NULL) *speed_r = spd_r;
}

/* ── 单独读取 ── */

float Encoder_GetSpeedLeft(void)
{
    float spd;
    Encoder_GetSpeed(&spd, NULL);
    return spd;
}

float Encoder_GetSpeedRight(void)
{
    float spd;
    Encoder_GetSpeed(NULL, &spd);
    return spd;
}

/* ====================================================================
 *           累计距离
 * ==================================================================== */

int32_t Encoder_GetDistance(void)
{
    return (int32_t)s_distance;
}

/* ====================================================================
 *           重置
 * ==================================================================== */

void Encoder_Reset(void)
{
    s_last_count_l = (int16_t)DL_TimerG_getTimerCount(QEI_LEFT_INST);
#if !ENC2_USE_SOFTWARE
    s_last_count_r = (int16_t)DL_TimerG_getTimerCount(QEI_RIGHT_INST);
#endif
    s_speed_l    = 0.0f;
    s_speed_r    = 0.0f;
    s_distance   = 0.0f;
    s_call_count = 0;
}
```

- [ ] **Step 3: 验证** — 确认 `QEI_0_INST` 和 `QEI_1_INST` 符号在 `ti_msp_dl_config.h` 中由 SysConfig 生成；确认左编码器 PA24/PA26 在同一端口 (PORTA) 满足硬件 QEI 要求。右编码器若 PA2/PB7 跨端口无法配 QEI，后续通过 SysConfig 配置 GPIO 中断 fallback。

- [ ] **Step 4: Commit**

```bash
git add hard/encoder.h hard/encoder.c
git commit -m "feat: rewrite encoder driver for dual QEI (ENC1 left + ENC2 right)

ENC1: PA24(PHA)+PA26(PHB) hardware QEI
ENC2: PA2(PHA)+PB7(PHB) hardware QEI (fallback: GPIO interrupt)
ENC2_USE_SOFTWARE macro controls fallback mode.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 3: 重写电机驱动 — 引脚 + 速度闭环 (motor.h + motor.c)

**Files:**
- Modify: `hard/motor.h`
- Modify: `hard/motor.c`

**Interfaces:**
- Consumes: `encoder.h` (Encoder_GetSpeed), `ti_msp_dl_config.h` (PWM/GPIO 符号)
- Produces:
  - `void Motor_Init(void)` — 初始化 GPIO 方向引脚 + PWM + 速度 PID
  - `void Motor_Stop(void)` — 停止所有电机
  - `void Motor(int16_t left_speed, int16_t right_speed)` — 底层 PWM 驱动 (-PWM_MAX ~ +PWM_MAX)
  - `void Motor_SetSpeed(float left_mm_s, float right_mm_s)` — 设定目标速度
  - `void Motor_Control_Loop(void)` — 完整控制循环 (读编码器→速度PID→Motor)
  - `void Motor_Control_Init(void)` — 初始化速度闭环数据
  - `float PID_Calc_Motor(PID_Controller *pid, float setpoint, float measurement)` — 速度 PID
  - 全局变量: `SpeedCtrl g_speed_left, g_speed_right`, `volatile uint8_t g_motor_control_flag`
  - 全局变量: `volatile uint8_t track_flag, turn_flag` (循迹/转弯标志)

- [ ] **Step 1: 重写 `hard/motor.h`**

```c
/**
 * @file    motor.h
 * @brief   MSPM0G3519 双路直流电机驱动 + 速度闭环 — TB6612
 *
 *          硬件引脚:
 *            CIN1 — PA25 (左电机 IN1)    CIN2 — PA27 (左电机 IN2)
 *            DIN1 — PA22 (右电机 IN1)    DIN2 — PB24 (右电机 IN2)
 *            PWML — PB4  (TIMAx CCP)     PWMR — PB5  (TIMAx CCP)
 *
 *          架构: 级联 PID
 *            外环: 循迹 PID → 目标速度 (mm/s)
 *            内环: 速度 PID → PWM → TB6612
 */

#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include "encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ====================================================================
 *                        公共常量
 * ==================================================================== */

/** @brief PWM 最大值 (与 SysConfig TIMA period 一致) */
#define PWM_MAX                 1000

/* ====================================================================
 *                    GPIO 方向引脚宏 (TB6612)
 * ==================================================================== */

/* CIN1 — PA25 (左电机 IN1) */
#define CIN1_HIGH()             DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_25)
#define CIN1_LOW()              DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_25)

/* CIN2 — PA27 (左电机 IN2) */
#define CIN2_HIGH()             DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_27)
#define CIN2_LOW()              DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_27)

/* DIN1 — PA22 (右电机 IN1) */
#define DIN1_HIGH()             DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_22)
#define DIN1_LOW()              DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_22)

/* DIN2 — PB24 (右电机 IN2) */
#define DIN2_HIGH()             DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_24)
#define DIN2_LOW()              DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_24)

/* ====================================================================
 *                    PWM 占空比宏 (PB4/PB5)
 * ==================================================================== */

/** @brief 左轮 PWM — PB4 (TIMA CCP) */
#define PWM_L_SET(duty)         DL_TimerA_setCaptureCompareValue(PWM_0_INST, (duty), DL_TIMER_CC_0_INDEX)

/** @brief 右轮 PWM — PB5 (TIMA CCP) */
#define PWM_R_SET(duty)         DL_TimerA_setCaptureCompareValue(PWM_0_INST, (duty), DL_TIMER_CC_1_INDEX)

/* ====================================================================
 *                    速度闭环 PID 类型
 * ==================================================================== */

/** @brief PID 控制器 (含一阶低通滤波) */
typedef struct {
    float Kp, Ki, Kd;
    float integral;             /* 积分累加值 (限幅 ±1000)       */
    float prev_error;           /* 上一次误差                    */
    float last_output;          /* 上一次输出 (一阶低通滤波用)    */
    float out_max, out_min;     /* 输出限幅                      */
} PID_Controller;

/** @brief 单轮速度控制 */
typedef struct {
    PID_Controller pid;
    float    target_speed;      /* 目标速度 (mm/s)               */
    float    current_speed;     /* 实测速度 (mm/s)               */
    int16_t  output;            /* PID 输出 PWM 值               */
    int32_t  last_count;        /* 编码器计数值快照              */
} SpeedCtrl;

/* ====================================================================
 *                    控制定时器
 * ==================================================================== */

#ifndef CONTROL_TIMER_INST
#define CONTROL_TIMER_INST                  TIMG0
#define CONTROL_TIMER_INST_IRQHandler       TIMG0_IRQHandler
#define CONTROL_TIMER_INST_INT_IRQN         (TIMG0_INT_IRQn)
#endif

/* ====================================================================
 *                    全局变量
 * ==================================================================== */

extern SpeedCtrl g_speed_left;
extern SpeedCtrl g_speed_right;
extern volatile uint8_t g_motor_control_flag;
extern volatile uint8_t track_flag;        /* 1=循迹模式, 由 Track_Run 控制  */
extern volatile uint8_t turn_flag;         /* 1=转弯模式, 速度环使用硬转弯值 */

/* ====================================================================
 *                    公共 API
 * ==================================================================== */

/* ── 基础电机控制 ── */
void Motor_Init(void);
void Motor_Stop(void);
void Motor(int16_t left_speed, int16_t right_speed);

/* ── 速度闭环 ── */
void Motor_Control_Init(void);
void Motor_Control_Loop(void);
void Motor_SetSpeed(float left_mm_s, float right_mm_s);

/* ── PID ── */
void  PID_Init(PID_Controller *pid, float kp, float ki, float kd, float out_max);
float PID_Calc_Motor(PID_Controller *pid, float setpoint, float measurement);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H__ */
```

- [ ] **Step 2: 重写 `hard/motor.c`**

```c
/**
 * @file    motor.c
 * @brief   MSPM0G3519 双路直流电机驱动 + 速度闭环实现
 *
 *          级联 PID 架构:
 *            外环循迹 PID → Motor_SetSpeed(target_l, target_r) → 设定目标速度
 *            内环速度 PID → Motor_Control_Loop() → Motor(pwm_l, pwm_r) → TB6612
 *
 *          参考: D:\!ziv\Nova\2026-RIGOLGAME-Capricorn\source\project\hard\motor\motor.c
 */

#include "motor.h"

/* ====================================================================
 *                        全局变量
 * ==================================================================== */

SpeedCtrl g_speed_left;
SpeedCtrl g_speed_right;
volatile uint8_t g_motor_control_flag = 0;
volatile uint8_t track_flag = 0;
volatile uint8_t turn_flag  = 0;

/* ====================================================================
 *                        PID 控制器
 * ==================================================================== */

/**
 * @brief 初始化 PID 控制器
 * @param out_max  输出上限 (同时也是下限的绝对值)
 */
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float out_max)
{
    pid->Kp          = kp;
    pid->Ki          = ki;
    pid->Kd          = kd;
    pid->integral    = 0.0f;
    pid->prev_error  = 0.0f;
    pid->last_output = 0.0f;
    pid->out_max     = out_max;
    pid->out_min     = -out_max;
}

/**
 * @brief PID 计算 (位置式 + 积分限幅 + 一阶低通滤波)
 *
 *  公式:
 *    raw = Kp*e + Ki*∫e + Kd*de
 *    output = 0.08 * last_output + 0.92 * raw   (一阶低通)
 *
 *  参考 Capricorn motor.c 的 PID_Calculate()
 */
float PID_Calc_Motor(PID_Controller *pid, float setpoint, float measurement)
{
    float error = setpoint - measurement;

    /* 比例 */
    float Pout = pid->Kp * error;

    /* 积分 (累加, 不乘 dt — 与 Capricorn 一致) */
    pid->integral += error;

    /* 积分限幅 ±1000, 防止编码器故障飞车 */
    if (pid->integral > 1000.0f)  pid->integral = 1000.0f;
    if (pid->integral < -1000.0f) pid->integral = -1000.0f;

    float Iout = pid->Ki * pid->integral;

    /* 微分 (不除 dt — 与 Capricorn 一致) */
    float derivative = error - pid->prev_error;
    float Dout = pid->Kd * derivative;

    /* 原始 PID 输出 */
    float raw_output = Pout + Iout + Dout;

    /* 一阶低通滤波: 0.08*last + 0.92*raw */
    float filtered_output = 0.08f * pid->last_output + 0.92f * raw_output;

    /* 输出限幅 */
    if (filtered_output > pid->out_max) filtered_output = pid->out_max;
    if (filtered_output < pid->out_min) filtered_output = pid->out_min;

    /* 保存状态 */
    pid->prev_error  = error;
    pid->last_output = filtered_output;

    return filtered_output;
}

/* ====================================================================
 *                        底层电机控制
 * ==================================================================== */

/**
 * @brief 电机初始化 — GPIO 方向引脚 + PWM 启动
 */
void Motor_Init(void)
{
    /* 方向引脚初始低电平 (H 桥关断) */
    CIN1_LOW();
    CIN2_LOW();
    DIN1_LOW();
    DIN2_LOW();

    /* PWM 输出最大占空比 → CCP=0, 但方向引脚全低, 电机不转 */
    PWM_L_SET(PWM_MAX);
    PWM_R_SET(PWM_MAX);

    DL_TimerA_startCounter(PWM_0_INST);
}

/**
 * @brief 停止所有电机
 */
void Motor_Stop(void)
{
    CIN1_LOW();
    CIN2_LOW();
    DIN1_LOW();
    DIN2_LOW();

    PWM_L_SET(PWM_MAX);
    PWM_R_SET(PWM_MAX);
}

/**
 * @brief 底层电机 PWM 驱动 (参考 Capricorn Motor())
 *
 * @param left_speed   正=前进, 负=后退, 范围 -PWM_MAX ~ +PWM_MAX
 * @param right_speed  正=前进, 负=后退, 范围 -PWM_MAX ~ +PWM_MAX
 */
void Motor(int16_t left_speed, int16_t right_speed)
{
    /* 限幅 */
    if (left_speed > PWM_MAX)   left_speed = PWM_MAX;
    if (left_speed < -PWM_MAX)  left_speed = -PWM_MAX;
    if (right_speed > PWM_MAX)  right_speed = PWM_MAX;
    if (right_speed < -PWM_MAX) right_speed = -PWM_MAX;

    /* 左轮 */
    if (left_speed >= 0) {
        CIN1_HIGH(); CIN2_LOW();
        PWM_L_SET((uint16_t)left_speed);
    } else {
        CIN1_LOW(); CIN2_HIGH();
        PWM_L_SET((uint16_t)(-left_speed));
    }

    /* 右轮 */
    if (right_speed >= 0) {
        DIN1_HIGH(); DIN2_LOW();
        PWM_R_SET((uint16_t)right_speed);
    } else {
        DIN1_LOW(); DIN2_HIGH();
        PWM_R_SET((uint16_t)(-right_speed));
    }
}

/* ====================================================================
 *                        速度闭环
 * ==================================================================== */

/**
 * @brief 初始化速度闭环控制
 *
 * 调用顺序: SYSCFG_DL_init() → Motor_Init() → Motor_Control_Init() → 主循环
 */
void Motor_Control_Init(void)
{
    /* 初始化编码器 (双路 QEI) */
    Encoder_Init();

    /* 初始化左右速度 PID — 参数继承 Capricorn 调试值 */
    PID_Init(&g_speed_left.pid,  2.15f, 1.1f, 0.001f, (float)PWM_MAX);
    PID_Init(&g_speed_right.pid, 2.15f, 1.1f, 0.001f, (float)PWM_MAX);

    /* 清零状态 */
    g_speed_left.target_speed   = 0.0f;
    g_speed_right.target_speed  = 0.0f;
    g_speed_left.current_speed  = 0.0f;
    g_speed_right.current_speed = 0.0f;
    g_speed_left.output         = 0;
    g_speed_right.output        = 0;
    g_speed_left.last_count     = 0;
    g_speed_right.last_count    = 0;

    g_motor_control_flag = 0;
    track_flag = 0;
    turn_flag  = 0;
}

/**
 * @brief 设定目标速度 (mm/s), 由循迹 PID 外环调用
 */
void Motor_SetSpeed(float left_mm_s, float right_mm_s)
{
    g_speed_left.target_speed  = left_mm_s;
    g_speed_right.target_speed = right_mm_s;
}

/**
 * @brief 速度控制主循环 — 每个控制周期 (100Hz) 调用一次
 *
 * 调用链:
 *   1. Encoder_GetSpeed() → 读取左右轮实际速度
 *   2. PID_Calc_Motor()   → 速度 PID 计算
 *   3. Motor()            → PWM 输出
 *
 * track_flag/turn_flag 控制输出模式 (参考 Capricorn):
 *   - turn_flag==1: 使用硬转弯值, 忽略 PID
 *   - track_flag==1: 使用 PID 输出
 */
void Motor_Control_Loop(void)
{
    float speed_l, speed_r;

    /* 1. 读取双编码器速度 */
    Encoder_GetSpeed(&speed_l, &speed_r);
    g_speed_left.current_speed  = speed_l;
    g_speed_right.current_speed = speed_r;

    /* 2. 速度 PID 计算 */
    float out_l = PID_Calc_Motor(&g_speed_left.pid,
                                  g_speed_left.target_speed, speed_l);
    float out_r = PID_Calc_Motor(&g_speed_right.pid,
                                  g_speed_right.target_speed, speed_r);

    g_speed_left.output  = (int16_t)out_l;
    g_speed_right.output = (int16_t)out_r;

    /* 3. 电机输出 (根据模式标志选择) */
    if (turn_flag == 1)
    {
        /* 转弯模式: 使用硬编码转弯值 (原地旋转) —
         * 具体值由 Track_Run 在进入 TURN 状态时设定 */
        Motor(g_speed_left.output, g_speed_right.output);
    }
    else if (track_flag == 1)
    {
        /* 循迹模式: 使用 PID 输出 */
        Motor(g_speed_left.output, g_speed_right.output);
    }
    /* else: 空闲, 不输出 (Motor_Stop 由调用者处理) */
}
```

- [ ] **Step 3: 验证** — 检查 PB4/PB5 的 TIMA CCP 通道索引是否与 SysConfig 一致 (可能需要 `DL_TIMER_CC_0_INDEX` vs `DL_TIMER_CC_1_INDEX` 或其他索引)；确认 `PWM_0_INST` 在 `ti_msp_dl_config.h` 中定义；确认 PA25/PA27/PA22/PB24 的 IOMUX 已在 SysConfig 中配置为 GPIO 输出。

- [ ] **Step 4: Commit**

```bash
git add hard/motor.h hard/motor.c
git commit -m "feat: rewrite motor driver with new pins and cascaded speed PID

- Update TB6612 pins: CIN1=PA25, CIN2=PA27, DIN1=PA22, DIN2=PB24
- Update PWM pins: PWML=PB4, PWMR=PB5
- Add speed PID with low-pass filter (0.08/0.92) from Capricorn reference
- Add Motor_Control_Loop() with track_flag/turn_flag mode switch
- Integral anti-windup ±1000

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 4: 更新循迹控制 — 8 路 + 级联输出 (track.h + track.c)

**Files:**
- Modify: `hard/track.h`
- Modify: `hard/track.c`

**Interfaces:**
- Consumes: `grayscale.h` (Grayscale_Read_All), `motor.h` (Motor_SetSpeed, track_flag, turn_flag, BASE_SPEED), `board.h` (g_sys_tick_ms)
- Produces:
  - `void Track_Init(void)` — 初始化灰度 + PID + 状态机
  - `void Track_Run(void)` — 循迹主控 (读灰度→TrackPID→Motor_SetSpeed + 状态机)
  - `int Get_Track_Error(void)` — 加权偏差
  - `int TrackPID_Calc(int error)` — 循迹 PID → 速度偏差

- [ ] **Step 1: 重写 `hard/track.h`**

```c
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
extern float   ir;                   /* 速度倍率 */

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
```

- [ ] **Step 2: 重写 `hard/track.c`**

```c
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
float ir = 1.0f;

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

/* ==================== 传感器读取 ==================== */

/**
 * @brief 读 8 路灰度, 返回 8 位位图 (bit0=通道0=最左, bit7=通道7=最右, 0=黑线)
 *
 * 适配原始代码的位序: bit0 最右 → 新代码需注意传感器物理排列.
 * 此处统一: sensor[0]=CH0(物理最左), sensor[7]=CH7(物理最右)
 * 返回: bit0=CH0 ... bit7=CH7 (0=检测到黑线)
 */
static uint8_t Track_Read_All(void)
{
    uint8_t sensor[8];
    uint8_t status = 0;
    Grayscale_Read_All(sensor);

    for (int i = 0; i < 8; i++)
    {
        if (sensor[i] == 0)      /* 黑线 → 对应位清零 */
        {
            /* status 对应位保持 0 */
        }
        else
        {
            status |= (1 << i);  /* 白底 → 对应位置 1 */
        }
    }
    return status;
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
        else if (count == 8)
        {
            /* 全黑 (十字路口): 用上次偏差方向维持 */
            error = g_last_error_sign * 30;  /* 模拟中等偏差 */
            g_all_white_start_ms = 0;

            /* 正常循迹: 循迹 PID → 目标速度 → 设定 */
            track_flag = 1;
            turn_flag  = 0;
            int track_out = TrackPID_Calc(error);
            float left_target  = BASE_SPEED_MM_S + track_out;
            float right_target = BASE_SPEED_MM_S - track_out * 1.1f;
            Motor_SetSpeed(left_target, right_target);
        }
        else
        {
            /* 有黑线: 正常循迹 */
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
```

- [ ] **Step 3: 验证** — 确认 `BASE_SPEED_MM_S = 200.0f` 与原始 `BASE_SPEED = 240` (PWM 值) 语义不同但量级合理 (200mm/s ≈ 20cm/s 基础速度)；确认转弯状态 `Motor_SetSpeed(-TURN_SPEED_MM_S, TURN_SPEED_MM_S)` 不会触发 Motor_Control_Loop 中 `turn_flag==1` 的重复设定。

- [ ] **Step 4: Commit**

```bash
git add hard/track.h hard/track.c
git commit -m "feat: adapt tracking to 8-channel grayscale + cascaded PID output

- IR_Weight[6] → IR_Weight[8] (linear interpolation of original values)
- Track_Run() now outputs target speed (mm/s) via Motor_SetSpeed()
  instead of direct PWM
- BASE_SPEED changed from PWM value to mm/s (200 mm/s default)
- Turn state machine logic preserved; uses Motor_SetSpeed with
  hard turn values + turn_flag for velocity loop bypass
- Added all-black (cross intersection) handling with last_error_sign

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 5: 简化主文件 (empty.c)

**Files:**
- Modify: `empty.c`

**Interfaces:**
- Consumes: all module headers (`motor.h`, `track.h`, `grayscale.h`, `board.h`)
- Produces: `main()` — 初始化 + 控制循环

- [ ] **Step 1: 重写 `empty.c`**

```c
/**
 * @file    empty.c
 * @brief   MSPM0G3519 主程序 — 级联 PID 循迹小车 (Keil MDK5)
 *
 *          架构:
 *          - 100Hz TIMG0 中断 → g_motor_control_flag → 主循环轮询
 *          - Motor_Control_Loop() 内含完整感知+策略+执行:
 *            读灰度 → 循迹PID → 目标速度 → 读编码器 → 速度PID → PWM
 *          - 8路灰度 MUX 循迹传感器
 *          - 双路编码器速度闭环 (硬件 QEI)
 *          - TB6612 电机驱动 (PA25/PA27/PA22/PB24 + PB4/PB5 PWM)
 */

#include "ti_msp_dl_config.h"
#include "motor.h"
#include "grayscale.h"
#include "track.h"
#include "board.h"

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
    NVIC_EnableIRQ(CONTROL_TIMER_INST_INT_IRQN);

    /* 外设初始化 */
    Motor_Init();           /* 电机 GPIO + PWM 启动            */
    Motor_Control_Init();   /* 双编码器 + 速度 PID 初始化      */
    Track_Init();           /* 8路灰度 + 循迹 PID + 状态机     */
    Grayscale_Init();       /* 灰度 MUX 引脚初始化 (Track_Init 已调用, 此处冗余安全) */

    /* ── 主循环 ── */
    while (1)
    {
        /* 100Hz 控制周期: TIMG0 中断标志位触发 */
        if (g_motor_control_flag == 1)
        {
            g_motor_control_flag = 0;

            /* 循迹控制 (8路灰度 → 偏差 → TrackPID → Motor_SetSpeed) */
            Track_Run();

            /* 速度闭环 (读编码器 → 速度 PID → Motor PWM 输出)
             * 内部根据 track_flag/turn_flag 选择 PID 输出或硬转弯值 */
            Motor_Control_Loop();
        }

        /* 其他低优先级任务可在此处添加 (如 TFT 显示刷新等) */
    }
}
```

- [ ] **Step 2: 验证** — 确认 `CONTROL_TIMER_INST_INT_IRQN` 在 `motor.h` 中已定义 (来自 SysConfig 的 `TIMG0_INT_IRQn`)；确认 `TIMG0_IRQHandler` 函数名与启动文件中的中断向量表一致 (当前代码已使用此名称)；确认 `Grayscale_Init()` 被 `Track_Init()` 调用，main 中的重复调用是安全的冗余。

- [ ] **Step 3: Commit**

```bash
git add empty.c
git commit -m "feat: simplify main loop for interrupt-driven cascaded PID

- Remove direct PWM/TFT display code from main loop
- Main loop polls g_motor_control_flag (100Hz from TIMG0 ISR)
- Each cycle: Track_Run() → Motor_Control_Loop()
- Track_Run outputs target speed; Motor_Control_Loop handles
  encoder reading + speed PID + PWM output

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 6: 清理旧文件

**Files:**
- Delete: `hard/track_gpio.h`
- Delete: `hard/track_gpio.c`

- [ ] **Step 1: 删除被替换的旧循迹 GPIO 驱动**

```bash
git rm hard/track_gpio.h hard/track_gpio.c
```

- [ ] **Step 2: 验证** — 全局搜索 `track_gpio` 引用，确认无残留:

```bash
grep -r "track_gpio" --include="*.c" --include="*.h" .
```

预期输出为空 (无引用)。

- [ ] **Step 3: Commit**

```bash
git commit -m "chore: remove obsolete 6-channel GPIO track driver

Replaced by grayscale.h/c (8-channel MUX-based grayscale sensor).

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 7: 编译验证

- [ ] **Step 1: 编译检查** — 在 Keil MDK5 中打开项目并编译 (Build Target)，确保 0 errors。

- [ ] **Step 2: 符号检查** — 查看 map 文件确认所有函数和全局变量正确链接:
  - `Grayscale_Init`, `Grayscale_Read_All`
  - `Encoder_Init`, `Encoder_GetSpeed`
  - `Motor_Init`, `Motor_Control_Init`, `Motor_Control_Loop`
  - `Track_Init`, `Track_Run`, `TrackPID_Calc`
  - `g_motor_control_flag`, `track_flag`, `turn_flag`

- [ ] **Step 3: 固件烧录** — 烧录到 MSPM0G3519 开发板，确认:
  - 系统正常启动 (通过 UART printf 确认)
  - 灰度传感器能正确读取 8 路数值
  - 编码器能读取速度值 (手动转动轮子看速度变化)
  - 电机能响应方向控制 (低 PWM 值测试，防止飞车)

- [ ] **Step 4: Commit (如有微调)**

```bash
git add -A
git commit -m "fix: compile and runtime verification adjustments"
```

---

## 验证检查清单 (全任务完成后)

- [ ] 所有 `.c/.h` 文件通过 Keil MDK5 编译 (0 errors, 0 warnings)
- [ ] 引脚映射与 `引脚.txt` 100% 一致
- [ ] `IR_Weight[8]` 权值合理 (后续赛道标定)
- [ ] 速度 PID 参数可安全启动 (积分限幅 ±1000 防止飞车)
- [ ] 转弯计数去重逻辑完整 (COOLDOWN + MIN_DURATION)
- [ ] 全白/全黑异常处理就绪
- [ ] 旧 `track_gpio` 文件已删除，无残留引用

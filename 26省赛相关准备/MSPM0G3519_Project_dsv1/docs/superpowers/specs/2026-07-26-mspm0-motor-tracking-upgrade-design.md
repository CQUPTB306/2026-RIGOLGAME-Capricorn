# MSPM0G3519 竞赛小车 — 电机+循迹+速度闭环升级设计

**日期**: 2026-07-26
**项目**: MSPM0G3519_Project_dsv1 (Keil MDK5)
**目标**: 电机引脚修改、8路灰度循迹切换、双编码器速度闭环、中断驱动架构

---

## 1. 概述

在现有 MSPM0G3519 竞赛小车代码基础上进行升级，实现：

1. **电机引脚迁移** — TB6612 方向引脚和 PWM 引脚按新硬件布局重映射
2. **循迹模块切换** — 从 6 路 GPIO 数字量切换到 8 路灰度多路复用模块
3. **速度闭环** — 双编码器硬件 QEI + 级联 PID (循迹 PID → 目标速度 → 速度 PID → PWM)
4. **中断驱动** — 所有控制逻辑由 100Hz 定时器中断触发，主循环仅轮询标志位

### 参考来源

| 参考 | 来源 |
|------|------|
| 8路灰度驱动 | `D:\!ziv\Nova\26省赛相关准备\八路灰度模块\源码\1.单片机数据读取\1.MSPM0G3507\Keil` |
| 电机中断+PID+级联 | `D:\!ziv\Nova\2026-RIGOLGAME-Capricorn\source` |
| 引脚定义 | `D:\!ziv\Nova\26省赛相关准备\引脚.txt` |

---

## 2. 引脚映射

### 2.1 电机驱动 (TB6612)

| 信号 | 旧引脚 | 新引脚 | 说明 |
|------|--------|--------|------|
| CIN1 | PA14 | **PA25** | 左电机 IN1 |
| CIN2 | PB17 | **PA27** | 左电机 IN2 |
| DIN1 | PB18 | **PA22** | 右电机 IN1 |
| DIN2 | PB19 | **PB24** | 右电机 IN2 |
| PWML | PA15 (TIMA1 CCP0) | **PB4** (TIMAx CCP) | 左轮 PWM |
| PWMR | PA16 (TIMA1 CCP1) | **PB5** (TIMAx CCP) | 右轮 PWM |

> **注意**: PB4/PB5 需在 SysConfig 中配置为同一 TIMA 实例的两个 CCP 通道，否则需两个 TIMA 实例并手动同步周期。

### 2.2 双编码器 (硬件 QEI)

| 信号 | 引脚 | 说明 |
|------|------|------|
| ENC1_PHA | **PA24** | 左编码器 A 相 |
| ENC1_PHB | **PA26** | 左编码器 B 相 |
| ENC2_PHA | **PA2** | 右编码器 A 相 |
| ENC2_PHB | **PB7** | 右编码器 B 相 |

> **注意**: ENC2 跨 PORTA/PORTB，需确认 SysConfig 能否配置为硬件 QEI。若不能，ENC2 改用 GPIO 双边沿中断做软件解码。

### 2.3 8路灰度循迹

| 信号 | 引脚 | 说明 |
|------|------|------|
| AD0 | **PA12** | 通道选择 bit0 |
| AD1 | **PB23** | 通道选择 bit1 |
| AD2 | **PB27** | 通道选择 bit2 |
| OUT | **PB8** | 传感器输出 (0=黑, 1=白) |

MUX 原理：AD[2:0] 编码 0-7 选择 8 路中的一路，`delay_us(50)` 等信号稳定后读 OUT 引脚。

---

## 3. 软件架构

### 3.1 总体架构 — 级联 PID

```
┌──────────────────────────────────────────────────┐
│  100Hz 控制周期 (TIMG0 中断 → 置标志位)           │
│                                                    │
│  1. Grayscale_Read_All(sensor[8])                  │
│       ↓                                            │
│  2. error = WeightedSum(sensor, IR_Weight[8])      │
│       ↓                                            │
│  3. track_out = TrackPID_Calc(error)               │
│       → 输出: 速度偏差 (非 PWM)                     │
│       ↓                                            │
│  4. Motor_SetSpeed(BASE+track_out, BASE-track_out) │
│       → 设定目标速度 (mm/s)                         │
│       ↓                                            │
│  5. Encoder_ReadBoth(&enc_l, &enc_r)               │
│       → 双编码器硬件 QEI → 实际速度 (mm/s)          │
│       ↓                                            │
│  6. pwm_l = SpeedPID_L(target_l - actual_l)        │
│     pwm_r = SpeedPID_R(target_r - actual_r)        │
│       → 速度 PID → PWM 占空比                       │
│       ↓                                            │
│  7. Motor(pwm_l, pwm_r) → TB6612                   │
└──────────────────────────────────────────────────┘
```

### 3.2 核心数据结构

```c
// 编码器通道 (独立管理)
typedef struct {
    int16_t  last_count;
    float    speed_mm_s;
    float    distance_mm;
} EncoderChannel;

// 速度闭环 PID (参考 Capricorn, 含低通滤波)
typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    float last_output;     // 一阶低通滤波
    float out_max, out_min;
} PID_Controller;

// 单轮速度控制
typedef struct {
    PID_Controller pid;
    float    target_speed;        // mm/s
    float    current_speed;       // mm/s
    int16_t  output;              // PWM
    int32_t  last_count;          // 编码器快照
} SpeedCtrl;

// 循迹 PID (输出速度偏差)
typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    int   output;                 // 速度偏差, 非 PWM
} TrackPID;
```

### 3.3 控制流

**中断侧 (TIMG0 ISR @ 100Hz)** — 仅置标志位：

```c
void TIMG0_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(CONTROL_TIMER_INST)) {
        case DL_TIMER_IIDX_ZERO:
            g_motor_control_flag = 1;
            break;
        default: break;
    }
}
```

**主循环侧** — 轮询标志位，做浮点运算：

```c
while (1) {
    if (g_motor_control_flag) {
        g_motor_control_flag = 0;
        Motor_Control_Loop();   // 包含全部感知+策略+执行
    }
}
```

> 不在 ISR 中做浮点 PID 运算，避免中断阻塞过长。

### 3.4 关键函数调用链

```
Motor_Control_Loop()
  ├── Grayscale_Read_All(sensor[8])    // grayscale.c
  ├── Track_CalcError(sensor)           // track.c — 加权偏差
  ├── TrackPID_Calc(error)              // track.c — 循迹 PID → 速度偏差
  ├── Motor_SetSpeed(target_l, target_r)// motor.c — 设定目标
  ├── Encoder_GetSpeed(&enc_l, &enc_r)  // encoder.c — 双编码器读速度
  ├── PID_Calc_Motor(&pid_l, target_l, actual_l)  // 速度 PID
  ├── PID_Calc_Motor(&pid_r, target_r, actual_r)
  └── Motor(output_l, output_r)         // motor.c — PWM 输出
```

---

## 4. 模块设计

### 4.1 motor.h / motor.c — 电机驱动

**变更**:
- 所有 `CIN1/CIN2/DIN1/DIN2` 宏更新为新引脚
- PWM 宏 `PWM_C_SET/PWM_D_SET` 更新为 PB4/PB5
- 新增 `track_flag` 和 `turn_flag` 全局标志 (参考 Capricorn)
- `Motor_Control_Loop()` 内含：读编码器 → 速度 PID → Motor() 输出
- 速度 PID 采用一阶低通滤波：`output = 0.08*last + 0.92*raw`
- 积分限幅 ±1000，防止编码器故障飞车
- PID 初始化参数：Kp=2.15, Ki=1.1, Kd=0.001 (继承 Capricorn 调试值)

### 4.2 encoder.h / encoder.c — 双编码器

**变更**:
- 从单路 (TIMG8) 扩展为双路 (TIMGx + TIMGy)
- `Encoder_Init()` 初始化两个 QEI 实例
- `Encoder_GetSpeed(EncoderChannel *enc_l, EncoderChannel *enc_r)` 同时读取
- 硬件 QEI 自动处理方向，计算公式不变：`speed = delta * MM_PER_PULSE / CONTROL_PERIOD_S`
- 编码器参数：线数=13, 减速比=30, 轮径=65mm, 控制周期=0.01s

### 4.3 grayscale.h / grayscale.c — 8路灰度驱动 (新增)

**替代 track_gpio.c/h**:

```c
#define GRAY_AD0_PORT  GPIOA
#define GRAY_AD0_PIN   DL_GPIO_PIN_12   // PA12
#define GRAY_AD1_PORT  GPIOB
#define GRAY_AD1_PIN   DL_GPIO_PIN_23   // PB23
#define GRAY_AD2_PORT  GPIOB
#define GRAY_AD2_PIN   DL_GPIO_PIN_27   // PB27
#define GRAY_OUT_PORT  GPIOB
#define GRAY_OUT_PIN   DL_GPIO_PIN_8    // PB8
```

API:
- `Grayscale_Init()` — 初始化 AD0-AD2 为输出，OUT 为输入上拉
- `Grayscale_Read_All(uint8_t sensor[8])` — 循环切换 0-7 通道，每路 delay_us(50) 后读 OUT
- `Grayscale_Read_Single(uint8_t ch)` — 读单路

### 4.4 track.h / track.c — 循迹控制

**变更**:
- `IR_Weight[6]` → `IR_Weight[8]`，权值根据实际 8 路布局重新标定
- `Get_Track_Error()` 适配 8 位数据
- `Track_Run()` 改为输出目标速度 `{track_out_l, track_out_r}` 而非 PWM
- 转弯/停车状态机逻辑保持不变
- `BASE_SPEED` 含义改为目标速度 (mm/s)，而非 PWM 值

### 4.5 empty.c — 主文件

**变更**:
- 简化：移除直接 PWM 控制代码
- `Motor_Control_Init()` 在初始化阶段调用
- `NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN)` 使能控制定时器
- 主循环仅轮询 `g_motor_control_flag`

---

## 5. 错误处理

| 故障场景 | 检测方式 | 处理 |
|----------|---------|------|
| 编码器断线飞车 | 积分限幅 ±1000 + 输出限幅 ±PWM_MAX | `PID.out_max` 钳位 |
| 全白 (冲出轨道) | `count == 0` 持续 > TURN_ALL_WHITE_MS | 进入 TURN 状态原地旋转找线 |
| 全黑 (十字路口) | `count == 8` 持续 | 用 `g_last_error_sign` 维持方向 |
| 转弯超时 | `(tick - turn_start) > TURN_TIMEOUT_MS` | Motor_Stop + revert to FOLLOW |
| MUX 切换噪声 | `delay_us(50)` 等待稳定 | 过滤单次毛刺 |
| 转弯重复计数 | `TURN_COOLDOWN_MS` + `TURN_MIN_DURATION_MS` | 同弯只计 1 次 |

---

## 6. 文件变更清单

| 操作 | 文件 | 说明 |
|------|------|------|
| ✏️ 修改 | `hard/motor.h` | 引脚宏, PWM 宏, 双 SpeedCtrl, track_flag |
| ✏️ 修改 | `hard/motor.c` | 新引脚, Motor(), 速度 PID + 低通滤波, Motor_Control_Loop() |
| ✏️ 修改 | `hard/encoder.h` | 双编码器独立参数宏 |
| ✏️ 重写 | `hard/encoder.c` | 双编码器 Init/Speed/Distance |
| ✏️ 修改 | `hard/track.h` | IR_Weight[8], TrackPID 结构, BASE_SPEED 语义变更 |
| ✏️ 修改 | `hard/track.c` | 8路 MUX 读取 + TrackPID(输出目标速度) + 状态机保持 |
| ❌ 删除 | `hard/track_gpio.h` | 被 grayscale.h 替代 |
| ❌ 删除 | `hard/track_gpio.c` | 被 grayscale.c 替代 |
| ✨ 新增 | `hard/grayscale.h` | AD0-AD2/OUT 引脚宏 + API |
| ✨ 新增 | `hard/grayscale.c` | MUX 切换 + 8路扫描 |
| ✏️ 修改 | `empty.c` | Motor_Control_Init, 简化主循环 |

---

## 7. 实现注意事项

1. **SysConfig 依赖**: 引脚映射 (IOMUX) 最终由 SysConfig 生成到 `ti_msp_dl_config.h`；如果代码宏与 SysConfig 生成符号冲突，以 SysConfig 为准，代码宏仅做别名
2. **PB4/PB5 PWM**: 必须确认它们能映射到同一 TIMA 的两个 CCP；如果不行，拆分为两个 TIMA 实例
3. **PA2/PB7 跨端口 QEI**: MSPM0 硬件 QEI 要求 A/B 相在同一端口相邻通道；若 SysConfig 无法配置，右编码器 fallback 为 GPIO 双边沿中断软件解码
4. **PID 参数**: 初始值继承 Capricorn (Kp=2.15, Ki=1.1, Kd=0.001)，实际需在赛道上重新调参
5. **IR_Weight[8]**: 8 路权值需要根据传感器物理间距重新标定；初始可线性内插原始 6 路权值

---

## 8. 未覆盖 / 后续扩展

- 舵机控制 (PCA9685) 保持不动
- OLED / TFT 显示保持不动
- BNO085 IMU 不在此次改动范围内
- 定距停车 (Run_To_Distance) 可在速度闭环稳定后追加

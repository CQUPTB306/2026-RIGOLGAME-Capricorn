# 简易自行瞄准装置 — 底层驱动项目

## 项目概述

基于 MSPM0G3507（Yahboom 开发板），为 E 题"简易自行瞄准装置"实现底层硬件驱动。核心竞赛控制逻辑由用户自行编写。

| 项目 | 说明 |
|------|------|
| **芯片** | MSPM0G3507 (Cortex-M0+, 80MHz, 128KB Flash, 32KB SRAM) |
| **开发板** | Yahboom MSPM0G3507 Core Board |
| **工具链** | Keil MDK-ARM V5 (ARM Compiler 6.18) |
| **SDK** | TI MSPM0 SDK v2.10.00.04 |
| **时钟** | MCLK 80MHz (SYSPLL: 32MHz SYSOSC /2 *5) |
| **模板来源** | `工程模版` (Keil 基础模板) |

---

## 引脚分配

### 通信总线

| 功能 | 引脚 | PINCM | 外设 | 参数 |
|------|------|-------|------|------|
| 视觉模块 + printf | PA10(TX) PA11(RX) | PINCM21/22 | UART0 | 9600-8N1, MFCLK 4MHz |
| PCA9685 舵机驱动 | PA0(SDA) PA1(SCL) | PINCM1/2 | 软件 I2C 总线1 (共享) | ~100kHz, 地址 0x40 |
| OLED 128×64 显示屏 | PA0(SDA) PA1(SCL) | PINCM1/2 | 软件 I2C 总线1 (共享) | ~100kHz, 地址 0x3C |
| 8路循迹传感器 | PA28(SDA) PA31(SCL) | PINCM3/6 | 软件 I2C 独立驱动 | ~30kHz, 地址 0x12 |

### 电机驱动 (TB6612)

| 功能 | 引脚 | PINCM | 说明 |
|------|------|-------|------|
| 左轮 PWM | PB08 | PINCM25 | TIMA0_C0, 32kHz |
| 右轮 PWM | PB09 | PINCM26 | TIMA0_C1, 32kHz |
| AIN1 (左方向1) | PA24 | PINCM54 | GPIO 输出 |
| AIN2 (左方向2) | PA26 | PINCM59 | GPIO 输出 |
| BIN1 (右方向1) | PB02 | PINCM15 | GPIO 输出 |
| BIN2 (右方向2) | PB03 | PINCM16 | GPIO 输出 |

### 编码器

| 信号 | 引脚 | PINCM | 外设 | 模式 |
|------|------|-------|------|------|
| A 相 | PB06 | PINCM23 | TIMG6 QEI | 4倍频 |
| B 相 | PB07 | PINCM24 | TIMG6 QEI | 4倍频 |

### 其他

| 功能 | 引脚 | PINCM | 说明 |
|------|------|-------|------|
| 用户 LED | PA15 | PINCM37 | GPIO 输出 |
| 用户 KEY | PA18 | PINCM40 | GPIOA 中断, 上升沿 |

---

## 文件结构

```
MSPM0G3507_Project/
├── README.md               ← 本文档
├── empty.c                 ← 主程序 (main)
├── board.c/h               ← 板级支持 (delay/printf/UART0 ISR)
├── ti_msp_dl_config.c/h    ← 外设初始化 (SysConfig 风格)
├── empty.syscfg            ← SysConfig 配置文件
├── keil/                   ← Keil 工程文件
│   ├── *.uvprojx/*.uvoptx
│   └── startup_*.s         ← 启动文件 (向量表)
├── ti/                     ← TI DriverLib 源码 (SDK v2.10)
└── hard/                   ← 硬件驱动模块
    ├── soft_i2c_simple.c/h ← 共享 I2C 驱动 (PCA9685+OLED, 推挽+切换)
    ├── soft_i2c_track.c/h  ← 循迹独立 I2C 驱动 (推挽+切换, 总线恢复)
    ├── pca9685.c/h         ← PCA9685 16路舵机驱动
    ├── motor.c/h           ← 电机驱动 + 单编码器 PID
    ├── track.c/h           ← 8路循迹传感器 + PID + 直角转弯
    ├── oled.c/h            ← OLED 128x64 显示驱动
    └── oled_font.h         ← OLED 8x16 ASCII 字库
```

---

## 中断分配

| 中断 | 频率 | 优先级 | 用途 |
|------|------|--------|------|
| SysTick | 1MHz | — | delay_us/delay_ms 基准 (80 周期 = 1us @80MHz) |
| TIMG7 | 100Hz | 1 | 电机 PID 控制循环 + g_sys_tick_ms 计数器 |
| UART0 RX | 按需 | — | 视觉模块数据 ISR 接收 → recv0_buff[] |
| GPIOA (KEY) | 按需 | — | PA18 按键 |

---

## 模块 API

### 电机 (motor.h)

单编码器 (TIMG6 QEI, 4倍频), 左右轮共享编码器反馈做 PID 速度闭环。

```c
void Motor_Init(void);                          // 初始化 PWM+方向+编码器+控制定时器
void Motor(int16_t left, int16_t right);        // 直接 PWM (-999 ~ +999)
void Motor_Set(int left, int right);            // 设目标速度 (供循迹调用)
void Motor_Stop(void);                          // 急停
int32_t Motor_GetDistance(void);                // 返回累计里程 (mm)

extern SpeedCtrl  g_speed_left, g_speed_right;  // 速度控制结构体
extern volatile uint32_t g_sys_tick_ms;          // ms 计数器 (10ms 分辨率)
```

**SpeedCtrl 结构体:**
```c
typedef struct {
    float    target_speed;   // 目标速度 mm/s
    float    current_speed;  // 当前速度 mm/s
    int16_t  output;         // PWM 输出值
    PID_Controller pid;      // PID 控制器
} SpeedCtrl;
```

### 循迹 (track.h)

```c
void    Track_Init(void);            // 初始化
uint8_t Track_Read_All(void);        // 读 8 路状态 (带防抖)
int     Get_Track_Error(void);       // 加权偏差
int     PID_Calc(int error);         // PID 计算
void    Track_Run(void);             // 循迹主控 (PID + 直角转弯状态机)

extern PID_TypeDef PID;              // 循迹 PID (可在线调参)
extern uint8_t BASE_SPEED;           // 基准速度 (默认 120)
extern int IR_Weight[8];             // 传感器权重 {-4,-3,-2,-1,1,2,3,4}
```

**直角转弯状态机:**

```
TRACK_FOLLOW ──(≥6路黑线)──────▶ TRACK_TURN  (撞上直角弯横向黑线, 立即转)
TRACK_FOLLOW ──(全白 >30ms)─────▶ TRACK_TURN  (冲过弯道, 延迟确认)
TRACK_TURN ──(重新检测到黑线)────▶ TRACK_FOLLOW
```

转弯时根据上次偏差方向原地旋转 (`TURN_SPEED=180`), 超时 2s 强制停止。

```c
extern Track_State g_track_state;     // 当前状态 (TRACK_STATE_FOLLOW / TRACK_STATE_TURN)
#define TURN_SPEED         180        // 转弯基准速度
#define TURN_ALL_WHITE_MS  30         // 全白触发阈值 (ms)
#define TURN_TIMEOUT_MS    2000       // 转弯超时保护 (ms)
```

### PCA9685 (pca9685.h)

```c
void pca9685_Init(void);                              // 初始化 (50Hz, 推挽)
void pca9685_SetServoAngle(uint8_t ch, uint16_t a);   // 0~1800 (0.0°~180.0°)
void pca9685_SetServoAngle270(uint8_t ch, uint16_t a); // 0~2700 (0.0°~270.0°)
void pca9685_SetPWM(uint8_t ch, uint16_t off);         // 直接设 PWM (0~4095)
void pca9685_SetAllAngles(uint16_t angle);             // 批量设所有通道
```

### 软件 I2C (soft_i2c_simple.h)

```c
void    SoftI2C_Init(SoftI2C_Obj *obj);
uint8_t SoftI2C_WriteOneByte(SoftI2C_Obj *obj, uint8_t addr, uint8_t reg, uint8_t data);
uint8_t SoftI2C_ReadOneByte(SoftI2C_Obj *obj, uint8_t addr, uint8_t reg);
void    SoftI2C_ReadBuf(SoftI2C_Obj *obj, uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

extern SoftI2C_Obj i2c_pca9685;   // PA0(SDA) PA1(SCL) — PCA9685(0x40)+OLED(0x3C)
// 循迹使用独立驱动 soft_i2c_track.c, 不走 SoftI2C_Obj
```

### OLED (oled.h)

基于 SSD1306 驱动芯片, 128×64 分辨率, I2C 通信 (地址 0x3C)。

```c
void OLED_Init(SoftI2C_Obj *obj);          // OLED 初始化 (上电延时 → SSD1306 寄存器配置 → 清屏)
void OLED_Clear(SoftI2C_Obj *obj);         // 全屏清空

void OLED_ShowChar(SoftI2C_Obj *obj, uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(SoftI2C_Obj *obj, uint8_t Line, uint8_t Column, char *String);

void OLED_ShowNum(SoftI2C_Obj *obj, uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(SoftI2C_Obj *obj, uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum(SoftI2C_Obj *obj, uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(SoftI2C_Obj *obj, uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowFloatNum(SoftI2C_Obj *obj, uint8_t Line, uint8_t Column, float Number, uint8_t IntLength, uint8_t DecLength);

/* 底层接口 (通常不需要直接调用) */
void OLED_WriteData(SoftI2C_Obj *obj, uint8_t Data);
void OLED_SetCursor(SoftI2C_Obj *obj, uint8_t Y, uint8_t X);
```

**参数说明:**
| 参数 | 范围 | 说明 |
|------|------|------|
| Line | 1~4 | 行号 (8×16 字体, 共 4 行) |
| Column | 1~16 | 列号 (8 像素宽, 共 16 列) |
| Number (整数) | 0~4294967295 | 无符号整数范围 |
| Number (浮点) | ±999999999.999999 | 推荐浮点数范围 |
| Length | 1~10 (整数) / 1~8 (十六进制) / 1~16 (二进制) | 显示位数 |
| IntLength | 1~9 | 浮点数整数部分位数 |
| DecLength | 1~6 | 浮点数小数部分位数 |

**使用示例:**
```c
// OLED 与 PCA9685 共享 PA0/PA1 的 I2C 总线, 使用同一个 i2c_pca9685 对象
OLED_Init(&i2c_pca9685);       // 初始化 SSD1306

OLED_ShowString(&i2c_pca9685, 1, 1, "Hello MSPM0!");
OLED_ShowNum(&i2c_pca9685, 2, 1, 12345, 5);
OLED_ShowFloatNum(&i2c_pca9685, 3, 1, -3.14f, 2, 2);   // 显示 "-3.14"
OLED_ShowHexNum(&i2c_pca9685, 4, 1, 0xABCD, 4);        // 显示 "ABCD"
```

**empty.c 主程序:**

上电后初始化所有外设并显示启动画面, 主循环预留控制逻辑入口。调用 `Track_Run()` 即可启动循迹（含直角转弯）。

### 板级支持 (board.h)

```c
void board_init(void);                     // 外设初始化
void delay_us(unsigned long us);           // 微秒延时 (SysTick)
void delay_ms(unsigned long ms);           // 毫秒延时
void uart0_send_char(char ch);             // UART0 发送单字符
void uart0_send_string(char *str);         // UART0 发送字符串
// printf 已通过 fputc 重定向到 UART0

extern volatile uint8_t  recv0_buff[128];  // UART0 ISR 接收缓冲
extern volatile uint16_t recv0_length;      // 接收数据长度
extern volatile uint8_t  recv0_flag;        // 接收完成标志
```

---

## 编译配置

### 必须的 Include 路径

Keil → Options → C/C++ → Include Paths:

```
..\..\MSPM0G3507_Project
C:\ti\mspm0_sdk_2_10_00_04\source
C:\ti\mspm0_sdk_2_10_00_04\source\ti\devices\msp\m0p
C:\ti\mspm0_sdk_2_10_00_04\source\third_party\CMSIS\Core\Include
```

### 预编译步骤

**关闭 SysConfig 预编译** (Options → Output → Before Build, 取消勾选 Run #1)。

手动编辑 `ti_msp_dl_config.c/h` 后，SysConfig 会在每次编译前尝试重新生成并覆盖修改。关掉它即可。

### 工程文件分组

| 分组 | 文件 |
|------|------|
| Source | empty.c, board.c, ti_msp_dl_config.c, empty.syscfg, startup_*.s |
| hard | soft_i2c_simple.c, soft_i2c_track.c, pca9685.c, motor.c, track.c, oled.c |
| Driverlib | ti/*.c |

---

## 编码器参数

| 参数 | 值 |
|------|-----|
| 轮径 | 65.0 mm |
| 编码器 PPR | 13 |
| 减速比 | 30:1 |
| 每转脉冲 (4倍频) | 13 × 30 × 4 = **1560** |
| mm/脉冲 | π × 65 / 1560 ≈ **0.131 mm** |
| 控制频率 | 100Hz (10ms 周期) |

---

## 已知限制

1. **单编码器** — MSPM0G3507 仅 1 个硬件 QEI 模块 (TIMG6), 左右轮共享编码器反馈, 右轮无独立速度闭环
2. **I2C 速率 ~30kHz** — 软件 I2C 实际约 30kHz, 16 路舵机全刷约 5ms
3. **printf 和视觉模块共用 UART0** — 视觉指令占主导时 printf 会阻塞
4. **SysConfig 已关闭** — 修改外设配置需手动编辑 `ti_msp_dl_config.c/h`
5. **OLED 与 PCA9685 共享 I2C 总线** — PA0/PA1 同时挂载 PCA9685(0x40) + OLED(0x3C), 地址不冲突

# MSPM0G3507 OLED + 循迹驱动集成报告

## 1. 项目概述

在 MSPM0G3507 (Yahboom) 工程模版基础上新增 OLED 128x64 显示和循迹传感器驱动, 并实现循迹状态在 OLED 上的实时可视化监测。

| 项目 | 值 |
|------|-----|
| 芯片 | MSPM0G3507 (Cortex-M0+, 32MHz SYSOSC) |
| 工程路径 | `工程模版 - 副本/MSPM0G3507_Project/` |
| 参考项目 | `最终代码` (STM32F103), `trace_i2c` (STM32F103) |
| 验证状态 | OLED 正常显示, 循迹 I2C 通信稳定, 8 路数据实时刷新 |

---

## 2. 最终文件清单

### 2.1 新增文件

| 文件 | 用途 |
|------|------|
| `hard/oled.h` | OLED SSD1306 驱动头文件 |
| `hard/oled.c` | OLED 驱动实现, 基于 `soft_i2c_simple` 的 `SoftI2C_Obj` |
| `hard/oled_font.h` | 8x16 ASCII 字库 (95 字符) |
| `hard/soft_i2c_track.h` | 循迹专用独立 I2C 驱动头文件 |
| `hard/soft_i2c_track.c` | 循迹专用独立 I2C 驱动实现 (推挽+模式切换, 含总线恢复) |

### 2.2 修改文件

| 文件 | 改动 |
|------|------|
| `hard/soft_i2c_simple.h` | 总线注释更新 (PCA9685+OLED 共享 Bus1) |
| `hard/soft_i2c_simple.c` | 默认延迟 355@32MHz, 对齐参考代码 |
| `hard/track.c` | 改用独立 `Track_I2C` 驱动, `Track_Init` 内含 I2C 初始化+总线恢复 |
| `empty.c` | OLED 初始化 + 循迹监测主循环 + `TrackOLED_ShowStatus()` |
| `board.c` | `delay_us` 使用 32MHz 参数 |
| `ti_msp_dl_config.h` | `CPUCLK_FREQ` = 32000000 |
| `README.md` | 引脚表/API/文件结构 同步更新 |

---

## 3. 最终引脚分配

### 3.1 I2C 总线

| 总线 | SDA | SCL | 驱动 | 挂载设备 (地址) |
|------|-----|-----|------|----------------|
| Bus1 | PA0 | PA1 | `soft_i2c_simple` (共享) | PCA9685 (0x40) + OLED (0x3C) |
| Bus2 | PA28 | PA31 | `soft_i2c_track` (独立) | 循迹传感器 (0x12) |

### 3.2 其他外设

| 外设 | 引脚 | 备注 |
|------|------|------|
| 电机 PWM | PB08 (左) / PB09 (右) | TIMA0, 32kHz |
| 电机方向 | PA24/PA26 (左) / PB02/PB03 (右) | GPIO |
| 编码器 QEI | PB06 (A) / PB07 (B) | TIMG6, 4 倍频 |
| 用户 LED | PA14 | GPIO |
| 用户 KEY | PA18 | GPIO 中断 |
| UART0 | PA10 (TX) / PA11 (RX) | 9600-8N1 |

---

## 4. I2C 驱动架构

```
  soft_i2c_simple.c                   soft_i2c_track.c
  (共享驱动, 推挽+模式切换)            (独立驱动, 推挽+模式切换)
       |                                    |
  Bus1 (PA0/PA1)                      Bus2 (PA28/PA31)
  PCA9685 + OLED                      循迹传感器 (0x12)
```

**设计原则**:
- Bus1 使用 `soft_i2c_simple.c`, 通过 `SoftI2C_Obj` 参数区分引脚和延迟
- Bus2 使用独立的 `soft_i2c_track.c`, 完全隔离, 避免从机异常时影响其他总线
- `Track_Init()` 自动调用 `Track_I2C_Init()`, 内含总线恢复序列

---

## 5. 核心技术细节

### 5.1 循迹 I2C 为何必须独立驱动

共享 `soft_i2c_simple` 时, 循迹从机状态机跑偏后会把 SDA 拉低。主机侧推挽输出强驱 HIGH 去 "抢" 总线 — 形成短路, 从机彻底锁死。独立驱动虽然无法杜绝从机跑偏, 但把影响限制在循迹总线内, PCA9685 和 OLED 不受牵连。

### 5.2 总线恢复机制

```
Track_I2C_Init()
  → 发 9 个 SCL 脉冲 (SDA 释放, 仅翻转 SCL)
  → 发 STOP 条件
  → 从机状态机强制复位到 IDLE
```

每次上电/重启时执行, 确保从机从干净状态开始通信。

### 5.3 HiZ 开漏方案为何失败 (关键)

**方案 A (HiZ 开漏, 失败)**: SDA 始终保持在 OUTPUT 模式, 通过 IOMUX 寄存器的 HiZ 位切换:

```
输出 '1' (释放): IOMUX->PINCM[x] |= HIZ1_ENABLE    → 引脚浮空
输出 '0' (拉低): IOMUX->PINCM[x] &= ~HIZ1_ENABLE   → 引脚驱动
                  DL_GPIO_clearPins(SDA)            → 输出 LOW
```

**致命缺陷 — 毛刺**: HiZ 关闭瞬间, 输出数据寄存器里残留的是上一次的 HIGH, 引脚瞬间驱 HIGH 再被 `clearPins` 改 LOW。从机视角:

```
HiZ=ON, 输出寄存器=1   引脚浮空 (被上拉拉高)   ✓
HiZ=OFF               引脚瞬间驱 HIGH           ← 假 STOP/START!
clearPins(SDA)        引脚驱 LOW                ← 正常
```

从机把那个短暂的高脉冲当成 I2C 总线事件 (STOP/START), 状态机跳飞, 后续通信全乱。

**方案 B (推挽+模式切换, 成功)**: SDA 不复用 OUTPUT 模式, 输出 '1' 直接切到 INPUT 模式:

```
输出 '1' (释放): DL_GPIO_initDigitalInputFeatures(...PULL_UP)  → 切输入, 上拉拉高
输出 '0' (拉低): DL_GPIO_initDigitalOutput(IOMUX)              → 切输出
                  DL_GPIO_clearPins(SDA)                        → 驱 LOW
```

**为什么没有毛刺**: 每次切回输出模式, `DL_GPIO_initDigitalOutput` 用整值**覆盖写入** IOMUX 寄存器, 所有位同时生效。紧跟 `clearPins` 清零数据寄存器。从机看到 SDA 从浮空 → LOW, 干净无毛刺。

```
INPUT 模式, PULL_UP   引脚浮空 (高)            ✓
OUTPUT 模式, clearPins 引脚驱 LOW               ✓ (正常的 bit 0)
```

**总结**: 方案 A 让一个引脚在两个驱动状态间切换 (驱高阻↔驱低), 切换瞬间有毛刺。方案 B 每次重新初始化引脚模式, 状态转换干净。方案 B 与 `soft_i2c_simple` 在 PCA9685/OLED 上已验证的方案完全相同。

### 5.4 I2C 时序 — 延迟值的选择

| 阶段 | 延迟值 | 效果 |
|------|--------|------|
| 初始尝试 | `i=800` @32MHz (~25µs) | I2C 太慢, 循迹从机超时断开 |
| 第二次 | `i=0` (无延迟) | 太快, 从机来不及响应 |
| 第三次 | `i=30` (~1µs) | 数据能读到但 bit 错位 (从机 SDA 建立时间不够) |
| **最终** | **`i=355` (~11µs)** | **与已验证的 soft_i2c_simple 一致, 稳定** |

355 @32MHz = 800 @72MHz ≈ 11µs/half-period → I2C 时钟 ≈ 30kHz, 循迹从机在此速度下稳定工作。

### 5.5 每帧只读一次 I2C

`Track_Read_All()` 内有防抖循环 (5 次采样 × 1ms 间隔), 本身就是昂贵的 I2C 操作。原 `Get_Track_Error()` 内部又调一次, 导致每帧两次完整防抖读 = 双倍总线负载。改为:

```c
status = Track_Read_All();  // 只读一次
// 偏差在 TrackOLED_ShowStatus 内直接计算, 复用同一次 status
```

---

## 6. 踩过的坑 (时间线)

| # | 问题 | 根因 | 解决 |
|---|------|------|------|
| 1 | 移 I2C 引脚后 OLED 不显示 | PA14 不在排针上; PB02/PB03 是板载 LED 引脚 | Bus1 保持 PA0/PA1, Bus3 改用 PA28/PA31 |
| 2 | 80MHz SYSPLL 不工作 | `DL_SYSCTL_configSYSPLL` 未链接; 直接寄存器写 PLL 锁不定 | 回退 32MHz SYSOSC |
| 3 | 循迹 I2C 运行后卡死 | 共享总线推挽 SDA 争用 + 无从机恢复 | 独立 I2C 驱动 + 总线恢复 |
| 4 | HiZ 开漏不通 | HiZ 切换毛刺 → 从机误判 STOP/START | 放弃 HiZ, 改用推挽+模式切换 (见 §5.3) |
| 5 | 数据 bit 错位 | 30-cycle 延迟太短, 从机 SDA 建立时间不足 | 改为 355-cycle, 对齐已验证驱动 |
| 6 | Vis/Bin 方向相反 | `visual[0]` 对应 LSB 但 `ShowBinNum` 显示 MSB 在左 | `visual[0]=bit7`, `visual[7]=bit0` |
| 7 | 每帧两次 I2C 读 | `Track_Read_All` + `Get_Track_Error` 各读一次 | 只读一次, 偏差内联计算 |

---

## 7. OLED 循迹监测显示布局

```
Line 1: F:0123 T:2 *    帧计数 / 活跃通道数 / 闪烁点 (~800ms)
Line 2: Bin:00111000     8路原始二进制 (MSB=左, LSB=右)
Line 3: Vis:..###...     可视化 ('.'=白线  '#'=黑线)
Line 4: Err: -3          加权偏差值 (PID 输入)
```

- **帧计数** `F:XXXX` 持续增长 → OLED 和主程序未卡死
- **闪烁点** `*` 每 ~800ms 切换 → 肉眼直观判断刷新
- **Bin/Vis** 同方向 (MSB 左, LSB 右) → 对照检查不混乱

---




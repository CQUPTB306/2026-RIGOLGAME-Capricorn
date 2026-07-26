/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  适配 MSPM0G3519 + Keil MDK5 (基于 MSPM0G3507 SysConfig 导出手动转换)
 *
 *  ⚠️ IOMUX 值未经验证 — 需对照 MSPM0G3519 数据手册 Table 6-1 确认!
 *  当前值来自 G3507 SysConfig + G3519 项目实际调试。
 *  编译通过后, 用 SysConfig / 数据手册核对每个 IOMUX_PINCM 和 PF 函数值。
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G351X
#define CONFIG_MSPM0G3519

/* SYSCONFIG_WEAK — 适配多编译器 (CCS / IAR / GCC / Keil ARMCLANG) */
#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__CC_ARM)          /* Keil ARMCC v5 */
#define SYSCONFIG_WEAK __weak
#elif defined(__clang__) || defined(__ARMCLANG_VERSION)  /* Keil ARMCLANG v6 */
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#else
#define SYSCONFIG_WEAK
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 */

/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)

#define CPUCLK_FREQ                                                     80000000
/* Defines for SYSPLL_ERR_01 Workaround */
#define FLOAT_TO_INT_SCALE                                               (1000U)
#define FCC_EXPECTED_RATIO                                                  2500
#define FCC_UPPER_BOUND                       (FCC_EXPECTED_RATIO * (1 + 0.003))
#define FCC_LOWER_BOUND                       (FCC_EXPECTED_RATIO * (1 - 0.003))

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);


/* ====================================================================
 *  PWM_0 (TIMA1): CCP0=PB4 左轮, CCP1=PB5 右轮
 *  ⚠️ IOMUX 需核对: PB4→PINCM?, PB5→PINCM?
 * ==================================================================== */
#define PWM_0_INST                                                         TIMA1
#define PWM_0_INST_IRQHandler                                   TIMA1_IRQHandler
#define PWM_0_INST_INT_IRQN                                     (TIMA1_INT_IRQn)
#define PWM_0_INST_CLK_FREQ                                             80000000
/* GPIO defines for channel 0: PB4 → IOMUX_PINCM17, TIMA1_CCP0 */
#define GPIO_PWM_0_C0_PORT                                                 GPIOB
#define GPIO_PWM_0_C0_PIN                                         DL_GPIO_PIN_4
#define GPIO_PWM_0_C0_IOMUX                                      (IOMUX_PINCM17)
#define GPIO_PWM_0_C0_IOMUX_FUNC                     IOMUX_PINCM17_PF_TIMA1_CCP0
#define GPIO_PWM_0_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1: PB5 → IOMUX_PINCM18, TIMA1_CCP1 */
#define GPIO_PWM_0_C1_PORT                                                 GPIOB
#define GPIO_PWM_0_C1_PIN                                         DL_GPIO_PIN_5
#define GPIO_PWM_0_C1_IOMUX                                      (IOMUX_PINCM18)
#define GPIO_PWM_0_C1_IOMUX_FUNC                     IOMUX_PINCM18_PF_TIMA1_CCP1
#define GPIO_PWM_0_C1_IDX                                    DL_TIMER_CC_1_INDEX


/* ====================================================================
 *  QEI_0 (TIMG8): 左编码器 PA24(PHA) + PA26(PHB)
 *  ⚠️ 原 TIMG8 接 PB21/PB22, 改为 PA24/PA26. IOMUX/PF 需核对!
 * ==================================================================== */
#define QEI_0_INST                                                         TIMG8
#define QEI_0_INST_IRQHandler                                   TIMG8_IRQHandler
#define QEI_0_INST_INT_IRQN                                     (TIMG8_INT_IRQn)
/* PHA Pin: PA26 → IOMUX_PINCM59, TIMG8_CCP0 (硬件QEI CCP0=编码器A相) */
#define GPIO_QEI_0_PHA_PORT                                                GPIOA
#define GPIO_QEI_0_PHA_PIN                                        DL_GPIO_PIN_26
#define GPIO_QEI_0_PHA_IOMUX                                     (IOMUX_PINCM59)
#define GPIO_QEI_0_PHA_IOMUX_FUNC                    IOMUX_PINCM59_PF_TIMG8_CCP0
/* PHB Pin: PA24 → IOMUX_PINCM54, TIMG8_CCP1 (硬件QEI CCP1=编码器B相) */
#define GPIO_QEI_0_PHB_PORT                                                GPIOA
#define GPIO_QEI_0_PHB_PIN                                        DL_GPIO_PIN_24
#define GPIO_QEI_0_PHB_IOMUX                                     (IOMUX_PINCM54)
#define GPIO_QEI_0_PHB_IOMUX_FUNC                    IOMUX_PINCM54_PF_TIMG8_CCP1


/* ====================================================================
 *  TIMER_0 (TIMG0): 100Hz 控制循环, period=62499
 * ==================================================================== */
#define TIMER_0_INST                                                     (TIMG0)
#define TIMER_0_INST_IRQHandler                                 TIMG0_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMG0_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                         (62499U)


/* ====================================================================
 *  UART_0: PA10(TX) + PA11(RX), 9600-8-N-1
 * ==================================================================== */
#define UART_0_INST                                                        UART0
#define UART_0_INST_FREQUENCY                                           40000000
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM22)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM21)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM21_PF_UART0_TX
#define UART_0_BAUD_RATE                                                  (9600)
#define UART_0_IBRD_40_MHZ_9600_BAUD                                       (260)
#define UART_0_FBRD_40_MHZ_9600_BAUD                                        (27)


/* ====================================================================
 *  电机方向引脚 (TB6612, GPIO 推挽输出)
 *  CIN1=PA25, CIN2=PA27, DIN1=PA22, DIN2=PB24
 * ==================================================================== */
/* CIN1: PA25 (左电机 IN1) */
#define CIN1_PORT                                                        (GPIOA)
#define CIN1_PIN_0_PIN                                          (DL_GPIO_PIN_25)
#define CIN1_PIN_0_IOMUX                                         (IOMUX_PINCM55)

/* CIN2: PA27 (左电机 IN2)
 * ⚠️ PA27 原先用作 TFT SCL, TFT 已移除 */
#define CIN2_PORT                                                        (GPIOA)
#define CIN2_PIN_1_PIN                                          (DL_GPIO_PIN_27)
#define CIN2_PIN_1_IOMUX                                         (IOMUX_PINCM60)

/* DIN1: PA22 (右电机 IN1) */
#define DIN1_PORT                                                        (GPIOA)
#define DIN1_PIN_2_PIN                                          (DL_GPIO_PIN_22)
#define DIN1_PIN_2_IOMUX                                         (IOMUX_PINCM47)

/* DIN2: PB24 (右电机 IN2)
 * ⚠️ PB24 原先用作 TFT DC, TFT 已移除 */
#define DIN2_PORT                                                        (GPIOB)
#define DIN2_PIN_3_PIN                                          (DL_GPIO_PIN_24)
#define DIN2_PIN_3_IOMUX                                         (IOMUX_PINCM52)


/* ====================================================================
 *  I2C 软 I2C 引脚 (PCA9685 舵机 + OLED 共享)
 *  SDA=PA28, SCL=PA31
 * ==================================================================== */
#define I2C1_SDA_PORT                                                    (GPIOA)
#define I2C1_SDA_PIN_4_PIN                                      (DL_GPIO_PIN_28)
#define I2C1_SDA_PIN_4_IOMUX                                      (IOMUX_PINCM3)

#define I2C1_SCL_PORT                                                    (GPIOA)
#define I2C1_SCL_PIN_5_PIN                                      (DL_GPIO_PIN_31)
#define I2C1_SCL_PIN_5_IOMUX                                      (IOMUX_PINCM6)


/* ====================================================================
 *  8路灰度 MUX 引脚
 *  AD0=PA12, AD1=PB23, AD2=PB27, OUT=PB8
 *  ⚠️ IOMUX 值需核对 — 以下值可能与 G3519 数据手册不一致!
 *     当前使用与 hard/grayscale.h 一致的占位值.
 * ==================================================================== */
/* AD0: PA12 (MUX 通道选择 bit0) */
#define GRAY_AD0_PORT                                                    (GPIOA)
#define GRAY_AD0_PIN                                             (DL_GPIO_PIN_12)
#define GRAY_AD0_IOMUX                                          (IOMUX_PINCM34)

/* AD1: PB23 (MUX 通道选择 bit1) */
#define GRAY_AD1_PORT                                                    (GPIOB)
#define GRAY_AD1_PIN                                             (DL_GPIO_PIN_23)
#define GRAY_AD1_IOMUX                                          (IOMUX_PINCM51)

/* AD2: PB27 (MUX 通道选择 bit2) — 原 TFT RST 引脚, TFT 已移除 */
#define GRAY_AD2_PORT                                                    (GPIOB)
#define GRAY_AD2_PIN                                             (DL_GPIO_PIN_27)
#define GRAY_AD2_IOMUX                                          (IOMUX_PINCM58)

/* OUT: PB8 (传感器输出, 数字输入+上拉) — 原 6路循迹 TRACK3 */
#define GRAY_OUT_PORT                                                    (GPIOB)
#define GRAY_OUT_PIN                                             (DL_GPIO_PIN_8)
#define GRAY_OUT_IOMUX                                          (IOMUX_PINCM25)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);
void SYSCFG_DL_PWM_0_init(void);
void SYSCFG_DL_QEI_0_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_UART_0_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */

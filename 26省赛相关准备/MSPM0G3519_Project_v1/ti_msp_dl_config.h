/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  适配 MSPM0G3519 + Keil MDK5 (基于 MSPM0G3507 SysConfig 导出手动转换)
 *  原始文件由 SysConfig 为 MSPM0G350X 生成，手动修改为 MSPM0G3519。
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


/* Defines for PWM_0 (TIMA1: CCP0=PA15 左轮, CCP1=PA16 右轮) */
#define PWM_0_INST                                                         TIMA1
#define PWM_0_INST_IRQHandler                                   TIMA1_IRQHandler
#define PWM_0_INST_INT_IRQN                                     (TIMA1_INT_IRQn)
#define PWM_0_INST_CLK_FREQ                                             80000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_0_C0_PORT                                                 GPIOA
#define GPIO_PWM_0_C0_PIN                                         DL_GPIO_PIN_15
#define GPIO_PWM_0_C0_IOMUX                                      (IOMUX_PINCM37)
#define GPIO_PWM_0_C0_IOMUX_FUNC                     IOMUX_PINCM37_PF_TIMA1_CCP0
#define GPIO_PWM_0_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_0_C1_PORT                                                 GPIOA
#define GPIO_PWM_0_C1_PIN                                         DL_GPIO_PIN_16
#define GPIO_PWM_0_C1_IOMUX                                      (IOMUX_PINCM38)
#define GPIO_PWM_0_C1_IOMUX_FUNC                     IOMUX_PINCM38_PF_TIMA1_CCP1
#define GPIO_PWM_0_C1_IDX                                    DL_TIMER_CC_1_INDEX


/* Defines for QEI_0 (TIMG8: PHA=PB21, PHB=PB22) */
#define QEI_0_INST                                                         TIMG8
#define QEI_0_INST_IRQHandler                                   TIMG8_IRQHandler
#define QEI_0_INST_INT_IRQN                                     (TIMG8_INT_IRQn)
/* PHA Pin */
#define GPIO_QEI_0_PHA_PORT                                                GPIOB
#define GPIO_QEI_0_PHA_PIN                                        DL_GPIO_PIN_21
#define GPIO_QEI_0_PHA_IOMUX                                     (IOMUX_PINCM49)
#define GPIO_QEI_0_PHA_IOMUX_FUNC                    IOMUX_PINCM49_PF_TIMG8_CCP0
/* PHB Pin */
#define GPIO_QEI_0_PHB_PORT                                                GPIOB
#define GPIO_QEI_0_PHB_PIN                                        DL_GPIO_PIN_22
#define GPIO_QEI_0_PHB_IOMUX                                     (IOMUX_PINCM50)
#define GPIO_QEI_0_PHB_IOMUX_FUNC                    IOMUX_PINCM50_PF_TIMG8_CCP1


/* Defines for TIMER_0 (TIMG0: 100ms 周期定时器, 100Hz 控制循环) */
#define TIMER_0_INST                                                     (TIMG0)
#define TIMER_0_INST_IRQHandler                                 TIMG0_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMG0_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                         (62499U)


/* Defines for UART_0 */
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


/* ── 电机方向引脚 ── */
/* CIN1: PA14 (左轮方向1), IOMUX_PINCM36 */
#define CIN1_PORT                                                        (GPIOA)
#define CIN1_PIN_0_PIN                                          (DL_GPIO_PIN_14)
#define CIN1_PIN_0_IOMUX                                         (IOMUX_PINCM36)

/* CIN2: PB17 (左轮方向2), IOMUX_PINCM43 */
#define CIN2_PORT                                                        (GPIOB)
#define CIN2_PIN_1_PIN                                          (DL_GPIO_PIN_17)
#define CIN2_PIN_1_IOMUX                                         (IOMUX_PINCM43)

/* DIN1: PB18 (右轮方向1), IOMUX_PINCM44 */
#define DIN1_PORT                                                        (GPIOB)
#define DIN1_PIN_2_PIN                                          (DL_GPIO_PIN_18)
#define DIN1_PIN_2_IOMUX                                         (IOMUX_PINCM44)

/* DIN2: PB19 (右轮方向2), IOMUX_PINCM45 */
#define DIN2_PORT                                                        (GPIOB)
#define DIN2_PIN_3_PIN                                          (DL_GPIO_PIN_19)
#define DIN2_PIN_3_IOMUX                                         (IOMUX_PINCM45)

/* ── I2C1 软 I2C 引脚 (PCA9685 + OLED) ── */
/* SDA: PA28, IOMUX_PINCM3 */
#define I2C1_SDA_PORT                                                    (GPIOA)
#define I2C1_SDA_PIN_4_PIN                                      (DL_GPIO_PIN_28)
#define I2C1_SDA_PIN_4_IOMUX                                      (IOMUX_PINCM3)

/* SCL: PA31, IOMUX_PINCM6 */
#define I2C1_SCL_PORT                                                    (GPIOA)
#define I2C1_SCL_PIN_5_PIN                                      (DL_GPIO_PIN_31)
#define I2C1_SCL_PIN_5_IOMUX                                      (IOMUX_PINCM6)

/* ── TFT SPI 软 SPI 引脚 ── */
/* SCL (SPI Clock): PA27, IOMUX_PINCM60 */
#define SCL_PORT                                                         (GPIOA)
#define SCL_PIN_6_PIN                                           (DL_GPIO_PIN_27)
#define SCL_PIN_6_IOMUX                                          (IOMUX_PINCM60)

/* SDA (SPI MOSI): PA26, IOMUX_PINCM59 */
#define SDA_PORT                                                         (GPIOA)
#define SDA_PIN_7_PIN                                           (DL_GPIO_PIN_26)
#define SDA_PIN_7_IOMUX                                          (IOMUX_PINCM59)

/* RST (TFT Reset): PB27, IOMUX_PINCM58 */
#define RST_PORT                                                         (GPIOB)
#define RST_PIN_8_PIN                                           (DL_GPIO_PIN_27)
#define RST_PIN_8_IOMUX                                          (IOMUX_PINCM58)

/* DC (TFT Data/Command): PB26, IOMUX_PINCM57 */
#define DC_PORT                                                          (GPIOB)
#define DC_PIN_9_PIN                                            (DL_GPIO_PIN_26)
#define DC_PIN_9_IOMUX                                           (IOMUX_PINCM57)

/* CS (TFT Chip Select): PB25, IOMUX_PINCM56 */
#define CS_PORT                                                          (GPIOB)
#define CS_PIN_10_PIN                                           (DL_GPIO_PIN_25)
#define CS_PIN_10_IOMUX                                          (IOMUX_PINCM56)

/* ── 6路循迹传感器 (GPIO 输入, 内部上拉) ── */
/* TRACK1: PB6, IOMUX_PINCM23 */
#define TRACK1_PORT                                                      (GPIOB)
#define TRACK1_PIN_11_PIN                                        (DL_GPIO_PIN_6)
#define TRACK1_PIN_11_IOMUX                                      (IOMUX_PINCM23)

/* TRACK2: PB7, IOMUX_PINCM24 */
#define TRACK2_PORT                                                      (GPIOB)
#define TRACK2_PIN_12_PIN                                        (DL_GPIO_PIN_7)
#define TRACK2_PIN_12_IOMUX                                      (IOMUX_PINCM24)

/* TRACK3: PB8, IOMUX_PINCM25 */
#define TRACK3_PORT                                                      (GPIOB)
#define TRACK3_PIN_13_PIN                                        (DL_GPIO_PIN_8)
#define TRACK3_PIN_13_IOMUX                                      (IOMUX_PINCM25)

/* TRACK4: PB9, IOMUX_PINCM26 */
#define TRACK4_PORT                                                      (GPIOB)
#define TRACK4_PIN_14_PIN                                        (DL_GPIO_PIN_9)
#define TRACK4_PIN_14_IOMUX                                      (IOMUX_PINCM26)

/* TRACK5: PB10, IOMUX_PINCM27 */
#define TRACK5_PORT                                                      (GPIOB)
#define TRACK5_PIN_15_PIN                                       (DL_GPIO_PIN_10)
#define TRACK5_PIN_15_IOMUX                                      (IOMUX_PINCM27)

/* TRACK6: PB11, IOMUX_PINCM28 */
#define TRACK6_PORT                                                      (GPIOB)
#define TRACK6_PIN_16_PIN                                       (DL_GPIO_PIN_11)
#define TRACK6_PIN_16_IOMUX                                      (IOMUX_PINCM28)


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

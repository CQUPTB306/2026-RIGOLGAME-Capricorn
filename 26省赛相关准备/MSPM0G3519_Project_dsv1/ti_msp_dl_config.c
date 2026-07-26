/*
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  适配 MSPM0G3519 + Keil MDK5 (基于 MSPM0G3507 SysConfig 导出手动转换)
 *  DO NOT EDIT — 此文件由 SysConfig 生成后手动转换为 MSPM0G3519。
 */

#include "ti_msp_dl_config.h"

DL_TimerA_backupConfig gPWM_0Backup;
DL_TimerG_backupConfig gQEI_0Backup;

/*
 *  ======== SYSCFG_DL_init ========
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations */
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_PWM_0_init();
    SYSCFG_DL_QEI_0_init();
    SYSCFG_DL_TIMER_0_init();
    SYSCFG_DL_UART_0_init();
    /* Ensure backup structures have no valid state */
    gPWM_0Backup.backupRdy  = false;
    gQEI_0Backup.backupRdy  = false;
}

SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    bool retStatus = true;
    retStatus &= DL_TimerA_saveConfiguration(PWM_0_INST, &gPWM_0Backup);
    retStatus &= DL_TimerG_saveConfiguration(QEI_0_INST, &gQEI_0Backup);
    return retStatus;
}

SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    bool retStatus = true;
    retStatus &= DL_TimerA_restoreConfiguration(PWM_0_INST, &gPWM_0Backup, false);
    retStatus &= DL_TimerG_restoreConfiguration(QEI_0_INST, &gQEI_0Backup, false);
    return retStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_TimerA_reset(PWM_0_INST);
    DL_TimerG_reset(QEI_0_INST);
    DL_TimerG_reset(TIMER_0_INST);
    DL_UART_Main_reset(UART_0_INST);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_TimerA_enablePower(PWM_0_INST);
    DL_TimerG_enablePower(QEI_0_INST);
    DL_TimerG_enablePower(TIMER_0_INST);
    DL_UART_Main_enablePower(UART_0_INST);
    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{
    /* PWM 输出引脚 */
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_0_C0_IOMUX, GPIO_PWM_0_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_0_C0_PORT, GPIO_PWM_0_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_0_C1_IOMUX, GPIO_PWM_0_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_0_C1_PORT, GPIO_PWM_0_C1_PIN);

    /* QEI 编码器输入 */
    DL_GPIO_initPeripheralInputFunction(GPIO_QEI_0_PHA_IOMUX, GPIO_QEI_0_PHA_IOMUX_FUNC);
    DL_GPIO_initPeripheralInputFunction(GPIO_QEI_0_PHB_IOMUX, GPIO_QEI_0_PHB_IOMUX_FUNC);

    /* UART 引脚 */
    DL_GPIO_initPeripheralOutputFunction(GPIO_UART_0_IOMUX_TX, GPIO_UART_0_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(GPIO_UART_0_IOMUX_RX, GPIO_UART_0_IOMUX_RX_FUNC);

    /* 电机方向控制 (GPIO 输出) */
    DL_GPIO_initDigitalOutput(CIN1_PIN_0_IOMUX);
    DL_GPIO_initDigitalOutput(CIN2_PIN_1_IOMUX);
    DL_GPIO_initDigitalOutput(DIN1_PIN_2_IOMUX);
    DL_GPIO_initDigitalOutput(DIN2_PIN_3_IOMUX);

    /* I2C1 引脚 (PCA9685+OLED 用, 开漏+上拉) */
    DL_GPIO_initDigitalOutputFeatures(I2C1_SDA_PIN_4_IOMUX,
         DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
         DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_ENABLE);
    DL_GPIO_initDigitalOutputFeatures(I2C1_SCL_PIN_5_IOMUX,
         DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
         DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_ENABLE);

    /* TFT SPI 控制引脚 */
    DL_GPIO_initDigitalOutput(SCL_PIN_6_IOMUX);
    DL_GPIO_initDigitalOutput(SDA_PIN_7_IOMUX);
    DL_GPIO_initDigitalOutput(RST_PIN_8_IOMUX);
    DL_GPIO_initDigitalOutput(DC_PIN_9_IOMUX);
    DL_GPIO_initDigitalOutput(CS_PIN_10_IOMUX);

    /* 6路循迹传感器 (输入+上拉) */
    DL_GPIO_initDigitalInputFeatures(TRACK1_PIN_11_IOMUX,
         DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
         DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(TRACK2_PIN_12_IOMUX,
         DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
         DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(TRACK3_PIN_13_IOMUX,
         DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
         DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(TRACK4_PIN_14_IOMUX,
         DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
         DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(TRACK5_PIN_15_IOMUX,
         DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
         DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(TRACK6_PIN_16_IOMUX,
         DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
         DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    /* 初始化输出为低电平 */
    DL_GPIO_clearPins(GPIOA, CIN1_PIN_0_PIN |
        I2C1_SDA_PIN_4_PIN |
        I2C1_SCL_PIN_5_PIN |
        SCL_PIN_6_PIN |
        SDA_PIN_7_PIN);
    DL_GPIO_enableOutput(GPIOA, CIN1_PIN_0_PIN |
        I2C1_SDA_PIN_4_PIN |
        I2C1_SCL_PIN_5_PIN |
        SCL_PIN_6_PIN |
        SDA_PIN_7_PIN);
    DL_GPIO_clearPins(GPIOB, CIN2_PIN_1_PIN |
        DIN1_PIN_2_PIN |
        DIN2_PIN_3_PIN |
        RST_PIN_8_PIN |
        DC_PIN_9_PIN |
        CS_PIN_10_PIN);
    DL_GPIO_enableOutput(GPIOB, CIN2_PIN_1_PIN |
        DIN1_PIN_2_PIN |
        DIN2_PIN_3_PIN |
        RST_PIN_8_PIN |
        DC_PIN_9_PIN |
        CS_PIN_10_PIN);
}


/* ── SYSPLL 配置: 32MHz SYSOSC → PLL → 80MHz MCLK ── */

static const DL_SYSCTL_SYSPLLConfig gSYSPLLConfig = {
    .inputFreq   = DL_SYSCTL_SYSPLL_INPUT_FREQ_16_32_MHZ,
    .rDivClk2x   = 3,
    .rDivClk1    = 0,
    .rDivClk0    = 0,
    .enableCLK2x = DL_SYSCTL_SYSPLL_CLK2X_ENABLE,
    .enableCLK1  = DL_SYSCTL_SYSPLL_CLK1_DISABLE,
    .enableCLK0  = DL_SYSCTL_SYSPLL_CLK0_DISABLE,
    .sysPLLMCLK  = DL_SYSCTL_SYSPLL_MCLK_CLK2X,
    .sysPLLRef   = DL_SYSCTL_SYSPLL_REF_SYSOSC,
    .qDiv        = 9,
    .pDiv        = DL_SYSCTL_SYSPLL_PDIV_2
};

SYSCONFIG_WEAK bool SYSCFG_DL_SYSCTL_SYSPLL_init(void)
{
    bool fFCCRatioStatus = false;
    uint32_t fFCCSysoscCount;
    uint32_t fFCCPllCount;
    uint32_t fFCCRatio;
    uint32_t fccTimeOutCounter;

    DL_SYSCTL_setFCCPeriods(DL_SYSCTL_FCC_TRIG_CNT_01);

    /* Measuring PLL */
    DL_SYSCTL_configFCC(DL_SYSCTL_FCC_TRIG_TYPE_RISE_RISE,
                        DL_SYSCTL_FCC_TRIG_SOURCE_LFCLK,
                        DL_SYSCTL_FCC_CLOCK_SOURCE_SYSPLLCLK2X);
    fccTimeOutCounter = 0;
    DL_SYSCTL_startFCC();
    while (DL_SYSCTL_isFCCDone() == 0) {
        delay_cycles(977);
        fccTimeOutCounter++;
        if (fccTimeOutCounter > 65) break;
    }
    fFCCPllCount = DL_SYSCTL_readFCC();

    /* Measuring SYSPLL Source (SYSOSC) */
    DL_SYSCTL_configFCC(DL_SYSCTL_FCC_TRIG_TYPE_RISE_RISE,
                        DL_SYSCTL_FCC_TRIG_SOURCE_LFCLK,
                        DL_SYSCTL_FCC_CLOCK_SOURCE_SYSOSC);
    fccTimeOutCounter = 0;
    DL_SYSCTL_startFCC();
    while (DL_SYSCTL_isFCCDone() == 0) {
        delay_cycles(977);
        fccTimeOutCounter++;
        if (fccTimeOutCounter > 65) break;
    }
    fFCCSysoscCount = DL_SYSCTL_readFCC();

    fFCCRatio = (fFCCPllCount * FLOAT_TO_INT_SCALE) / fFCCSysoscCount;
    if ((FCC_LOWER_BOUND < fFCCRatio) && (fFCCRatio < FCC_UPPER_BOUND)) {
        fFCCRatioStatus = true;
    }
    return fFCCRatioStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);
    DL_SYSCTL_setFlashWaitState(DL_SYSCTL_FLASH_WAIT_STATE_2);

    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    DL_SYSCTL_disableHFXT();
    DL_SYSCTL_disableSYSPLL();
    DL_SYSCTL_configSYSPLL((DL_SYSCTL_SYSPLLConfig *) &gSYSPLLConfig);

    /* [SYSPLL_ERR_01] PLL Incorrect locking workaround */
    while (SYSCFG_DL_SYSCTL_SYSPLL_init() == false) {
        DL_SYSCTL_disableSYSPLL();
        DL_SYSCTL_enableSYSPLL();
        while ((DL_SYSCTL_getClockStatus() & SYSCTL_CLKSTATUS_SYSPLLGOOD_MASK)
               != DL_SYSCTL_CLK_STATUS_SYSPLL_GOOD) {}
    }
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_2);
    DL_SYSCTL_setMCLKSource(SYSOSC, HSCLK, DL_SYSCTL_HSCLK_SOURCE_SYSPLL);
}


/* ── PWM_0 (TIMA1): 双路 PWM, period=1000, 80MHz ── */

static const DL_TimerA_ClockConfig gPWM_0ClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale    = 0U
};

static const DL_TimerA_PWMConfig gPWM_0Config = {
    .pwmMode           = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period            = 1000,
    .isTimerWithFourCC = false,
    .startTimer        = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_PWM_0_init(void)
{
    DL_TimerA_setClockConfig(PWM_0_INST, (DL_TimerA_ClockConfig *) &gPWM_0ClockConfig);
    DL_TimerA_initPWMMode(PWM_0_INST, (DL_TimerA_PWMConfig *) &gPWM_0Config);

    DL_TimerA_setCounterControl(PWM_0_INST,
        DL_TIMER_CZC_CCCTL0_ZCOND, DL_TIMER_CAC_CCCTL0_ACOND,
        DL_TIMER_CLC_CCCTL0_LCOND);

    /* Channel 0 (CCP0 = PA15 左轮) */
    DL_TimerA_setCaptureCompareOutCtl(PWM_0_INST,
        DL_TIMER_CC_OCTL_INIT_VAL_HIGH,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptCompUpdateMethod(PWM_0_INST,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_0_INST, 1000, DL_TIMER_CC_0_INDEX);

    /* Channel 1 (CCP1 = PA16 右轮) */
    DL_TimerA_setCaptureCompareOutCtl(PWM_0_INST,
        DL_TIMER_CC_OCTL_INIT_VAL_HIGH,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    DL_TimerA_setCaptCompUpdateMethod(PWM_0_INST,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_0_INST, 1000, DL_TIMER_CC_1_INDEX);

    DL_TimerA_enableClock(PWM_0_INST);
    DL_TimerA_setCCPDirection(PWM_0_INST, DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT);
}


/* ── QEI_0 (TIMG8): 硬件正交编码器 ── */

static const DL_TimerG_ClockConfig gQEI_0ClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale    = 0U
};

SYSCONFIG_WEAK void SYSCFG_DL_QEI_0_init(void)
{
    DL_TimerG_setClockConfig(QEI_0_INST, (DL_TimerG_ClockConfig *) &gQEI_0ClockConfig);
    DL_TimerG_configQEI(QEI_0_INST, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_0_INDEX);
    DL_TimerG_configQEI(QEI_0_INST, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_1_INDEX);
    DL_TimerG_setLoadValue(QEI_0_INST, 65535);
    DL_TimerG_enableClock(QEI_0_INST);
}


/* ── TIMER_0 (TIMG0): 100ms 周期定时器 (100Hz 控制循环) ── */

static const DL_TimerG_ClockConfig gTIMER_0ClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_2,
    .prescale    = 31U,
};

static const DL_TimerG_TimerConfig gTIMER_0TimerConfig = {
    .period     = TIMER_0_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_TIMER_0_init(void)
{
    DL_TimerG_setClockConfig(TIMER_0_INST,
        (DL_TimerG_ClockConfig *) &gTIMER_0ClockConfig);
    DL_TimerG_initTimerMode(TIMER_0_INST,
        (DL_TimerG_TimerConfig *) &gTIMER_0TimerConfig);
    DL_TimerG_enableInterrupt(TIMER_0_INST, DL_TIMERG_INTERRUPT_ZERO_EVENT);
    DL_TimerG_enableClock(TIMER_0_INST);
}


/* ── UART_0: 9600-8-N-1 ── */

static const DL_UART_Main_ClockConfig gUART_0ClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_0Config = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART_0_init(void)
{
    DL_UART_Main_setClockConfig(UART_0_INST,
        (DL_UART_Main_ClockConfig *) &gUART_0ClockConfig);
    DL_UART_Main_init(UART_0_INST, (DL_UART_Main_Config *) &gUART_0Config);
    DL_UART_Main_setOversampling(UART_0_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_0_INST,
        UART_0_IBRD_40_MHZ_9600_BAUD, UART_0_FBRD_40_MHZ_9600_BAUD);
    DL_UART_Main_enable(UART_0_INST);
}

/*
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  适配 MSPM0G3519 + Keil MDK5 (基于 MSPM0G3507 SysConfig 导出手动转换)
 *
 *  ⚠️ IOMUX 值和 PF (Peripheral Function) 代码需要在 SysConfig 或
 *  MSPM0G3519 数据手册 Table 6-1 中验证后, 硬件才能正常工作.
 *  当前代码保证编译通过, 外设功能可能需要调整.
 */

#include "ti_msp_dl_config.h"

DL_TimerA_backupConfig gPWM_0Backup;

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
    SYSCFG_DL_TIMER_0_init();
    SYSCFG_DL_UART_0_init();
    /* Ensure backup structures have no valid state */
    gPWM_0Backup.backupRdy  = false;
}

SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    bool retStatus = true;
    retStatus &= DL_TimerA_saveConfiguration(PWM_0_INST, &gPWM_0Backup);
    return retStatus;
}

SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    bool retStatus = true;
    retStatus &= DL_TimerA_restoreConfiguration(PWM_0_INST, &gPWM_0Backup, false);
    return retStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_TimerA_reset(PWM_0_INST);
    DL_TimerG_reset(TIMER_0_INST);
    DL_UART_Main_reset(UART_0_INST);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_TimerA_enablePower(PWM_0_INST);
    DL_TimerG_enablePower(TIMER_0_INST);
    DL_UART_Main_enablePower(UART_0_INST);
    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{
    /* ── PWM 输出引脚: PB4(CCP0 左轮), PB5(CCP1 右轮) ── */
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_0_C0_IOMUX, GPIO_PWM_0_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_0_C0_PORT, GPIO_PWM_0_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_0_C1_IOMUX, GPIO_PWM_0_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_0_C1_PORT, GPIO_PWM_0_C1_PIN);

    /* ── UART 引脚: PA10(TX), PA11(RX) ── */
    DL_GPIO_initPeripheralOutputFunction(GPIO_UART_0_IOMUX_TX, GPIO_UART_0_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(GPIO_UART_0_IOMUX_RX, GPIO_UART_0_IOMUX_RX_FUNC);

    /* ── 电机方向控制 (GPIO 输出) ──
     * CIN1=PA25, CIN2=PA27, DIN1=PA22, DIN2=PB24 */
    DL_GPIO_initDigitalOutput(CIN1_PIN_0_IOMUX);   /* PA25 */
    DL_GPIO_initDigitalOutput(CIN2_PIN_1_IOMUX);   /* PA27 */
    DL_GPIO_initDigitalOutput(DIN1_PIN_2_IOMUX);   /* PA22 */
    DL_GPIO_initDigitalOutput(DIN2_PIN_3_IOMUX);   /* PB24 */

    /* ── 初始化电机方向 GPIO 输出为低电平 ── */
    DL_GPIO_clearPins(GPIOA,
        CIN1_PIN_0_PIN | CIN2_PIN_1_PIN | DIN1_PIN_2_PIN);
    DL_GPIO_enableOutput(GPIOA,
        CIN1_PIN_0_PIN | CIN2_PIN_1_PIN | DIN1_PIN_2_PIN);

    DL_GPIO_clearPins(GPIOB, DIN2_PIN_3_PIN);
    DL_GPIO_enableOutput(GPIOB, DIN2_PIN_3_PIN);
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


/* ── PWM_0 (TIMA1): 双路 PWM, period=1000, 80MHz ──
 *  CCP0=PB4(左轮), CCP1=PB5(右轮) */

static const DL_TimerA_ClockConfig gPWM_0ClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale    = 0U
};

/*
 * ⚠️ TIMA1 是 4-CCP 定时器, isTimerWithFourCC 必须为 true!
 *   写 false 会导致 CCP2/3 未配置, PWM 输出异常.
 */
static const DL_TimerA_PWMConfig gPWM_0Config = {
    .pwmMode           = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period            = 1000,
    .isTimerWithFourCC = true,
    .startTimer        = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_PWM_0_init(void)
{
    DL_TimerA_setClockConfig(PWM_0_INST, (DL_TimerA_ClockConfig *) &gPWM_0ClockConfig);
    DL_TimerA_initPWMMode(PWM_0_INST, (DL_TimerA_PWMConfig *) &gPWM_0Config);

    DL_TimerA_setCounterControl(PWM_0_INST,
        DL_TIMER_CZC_CCCTL0_ZCOND, DL_TIMER_CAC_CCCTL0_ACOND,
        DL_TIMER_CLC_CCCTL0_LCOND);

    /* Channel 0: CCP0 = PB4 左轮 */
    DL_TimerA_setCaptureCompareOutCtl(PWM_0_INST,
        DL_TIMER_CC_OCTL_INIT_VAL_HIGH,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptCompUpdateMethod(PWM_0_INST,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_0_INST, 1000, DL_TIMER_CC_0_INDEX);

    /* Channel 1: CCP1 = PB5 右轮 */
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


/* ── TIMER_0 (TIMG0): 100Hz 控制循环 (period=62499) ── */

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


/* ── UART_0: PA10(TX)+PA11(RX), 9600-8-N-1 ── */

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

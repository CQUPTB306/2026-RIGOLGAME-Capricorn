/**
 * @file    track_gpio.h
 * @brief   6路 GPIO 循迹传感器硬件层
 * @note    6 个 GPIO 数字输入, 内部上拉
 *          从左到右: PB11 PB10 PB9 PB8 PB7 PB6
 *          读取返回 6 位: bit0=最右(PB6) ... bit5=最左(PB11)
 */

#ifndef __TRACK_GPIO_H
#define __TRACK_GPIO_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/*==================== 通道数 ====================*/

#define TRACK_CHANNELS  6

/*==================== GPIO 引脚定义 (bit0=最右 → bit5=最左) ====================*/

/* PIN_0: PB6 — 最右 */
#define TRACK_PIN_0_PORT    GPIOB
#define TRACK_PIN_0_PIN     DL_GPIO_PIN_6
#define TRACK_PIN_0_IOMUX   IOMUX_PINCM23

/* PIN_1: PB7 — 右中 */
#define TRACK_PIN_1_PORT    GPIOB
#define TRACK_PIN_1_PIN     DL_GPIO_PIN_7
#define TRACK_PIN_1_IOMUX   IOMUX_PINCM24

/* PIN_2: PB8 — 中右 */
#define TRACK_PIN_2_PORT    GPIOB
#define TRACK_PIN_2_PIN     DL_GPIO_PIN_8
#define TRACK_PIN_2_IOMUX   IOMUX_PINCM25

/* PIN_3: PB9 — 中左 */
#define TRACK_PIN_3_PORT    GPIOB
#define TRACK_PIN_3_PIN     DL_GPIO_PIN_9
#define TRACK_PIN_3_IOMUX   IOMUX_PINCM26

/* PIN_4: PB10 — 左中 */
#define TRACK_PIN_4_PORT    GPIOB
#define TRACK_PIN_4_PIN     DL_GPIO_PIN_10
#define TRACK_PIN_4_IOMUX   IOMUX_PINCM27

/* PIN_5: PB11 — 最左 */
#define TRACK_PIN_5_PORT    GPIOB
#define TRACK_PIN_5_PIN     DL_GPIO_PIN_11
#define TRACK_PIN_5_IOMUX   IOMUX_PINCM28

/*==================== API ====================*/

void    TrackGPIO_Init(void);
uint8_t TrackGPIO_Read_All(void);

#endif

#ifndef __HX711_H
#define __HX711_H

#include "stm32f1xx_hal.h"  // 根据你的芯片型号修改：f4xx、g0、l4等

#ifdef __cplusplus
extern "C" {
#endif

	//==================== HX711 引脚配置 ====================
#define HX711_DOUT_PIN     GPIO_PIN_0
#define HX711_DOUT_PORT    GPIOB

#define HX711_SCK_PIN      GPIO_PIN_1
#define HX711_SCK_PORT     GPIOB

//==================== 函数声明 ====================
/**
 * @brief  读取HX711原始24位数据
 * @param  gain: 增益参数 1(128倍)/2(32倍)/3(64倍)
 * @retval 转换后的32位有符号数
 */
	long HX711_Read(uint8_t gain);

	/**
	 * @brief  多次采样取平均值
	 * @param  times: 采样次数
	 * @retval 平均原始值
	 */
	long HX711_Read_Average(uint8_t times);

	/**
	 * @brief  去皮校准（清零）
	 * @param  无
	 * @retval 无
	 */
	void HX711_Tare(void);

	/**
	 * @brief  获取计算后的重量(克)
	 * @param  无
	 * @retval 重量值(单位：g)
	 */
	float HX711_Get_Weight(void);

	//==================== 外部全局变量声明 ====================
	extern long  hx711_offset;  // 去皮偏移值
	extern float hx711_coef;    // 重量校准系数

#ifdef __cplusplus
}
#endif

#endif

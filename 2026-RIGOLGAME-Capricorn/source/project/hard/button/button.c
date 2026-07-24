#include "button.h"

#define KEY1_READ()  HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)
#define KEY2_READ()  HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1)
#define KEY3_READ()  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)
#define KEY4_READ()  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)

#define KEY_PRESSED  0

uint8_t Key_Scan(void)
{
    static uint8_t key_up = 1;
    if(key_up && (KEY1_READ() == KEY_PRESSED || KEY2_READ() == KEY_PRESSED || KEY3_READ() == KEY_PRESSED || KEY4_READ() == KEY_PRESSED))
    {
        HAL_Delay(10);
        key_up = 0;
        if(KEY1_READ() == KEY_PRESSED)     return 1;
        else if(KEY2_READ() == KEY_PRESSED) return 2;
        else if(KEY3_READ() == KEY_PRESSED) return 3;
        else if(KEY4_READ() == KEY_PRESSED) return 4;
    }
    else if(KEY1_READ() != KEY_PRESSED && KEY2_READ() != KEY_PRESSED && KEY3_READ() != KEY_PRESSED && KEY4_READ() != KEY_PRESSED)
    {
        key_up = 1;
    }
    return 0;
}


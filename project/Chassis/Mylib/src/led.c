#include "led.h"

/**
 * @brief LED灯初始化(双色灯)
 * @param[in] void
 */
void LED_Configration()
{
    GPIO_InitTypeDef gpio;
    LED_R_RCC_AHBxPeriphClockCmd(LED_R_RCC_AHBxPeriph_GPIOx, ENABLE);
    gpio.GPIO_Pin = LED_R_GPIO_Pin_x;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(LED_R_GPIOx, &gpio);

    LED_B_RCC_AHBxPeriphClockCmd(LED_B_RCC_AHBxPeriph_GPIOx, ENABLE);
    gpio.GPIO_Pin = LED_B_GPIO_Pin_x;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(LED_B_GPIOx, &gpio);
}

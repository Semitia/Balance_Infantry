#ifndef _LED_H
#define _LED_H

#include <stm32f4xx.h>
#include <stm32f4xx_conf.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

void LED_Configration(void);

#define LED_R_RCC_AHBxPeriphClockCmd RCC_AHB1PeriphClockCmd
#define LED_R_RCC_AHBxPeriph_GPIOx RCC_AHB1Periph_GPIOC
#define LED_R_GPIOx GPIOC
#define LED_R_GPIO_Pin_x GPIO_Pin_9

#define LED_B_RCC_AHBxPeriphClockCmd RCC_AHB1PeriphClockCmd
#define LED_B_RCC_AHBxPeriph_GPIOx RCC_AHB1Periph_GPIOC
#define LED_B_GPIOx GPIOC
#define LED_B_GPIO_Pin_x GPIO_Pin_8

#define LED_R_ON GPIO_ResetBits(LED_R_GPIOx, LED_R_GPIO_Pin_x)
#define LED_R_OFF GPIO_SetBits(LED_R_GPIOx, LED_R_GPIO_Pin_x)
#define LED_B_ON GPIO_ResetBits(LED_B_GPIOx, LED_B_GPIO_Pin_x)
#define LED_B_OFF GPIO_SetBits(LED_B_GPIOx, LED_B_GPIO_Pin_x)

#define ONLY_LED_B_ON \
    LED_B_ON;         \
    LED_R_OFF

#define ONLY_LED_R_ON \
    LED_R_ON;         \
    LED_B_OFF

#define CLOLOR_LED_ON \
    LED_R_ON;         \
    LED_B_ON

#define ALL_LED_OFF \
    LED_R_OFF;      \
    LED_B_OFF

#endif // !_LED_H

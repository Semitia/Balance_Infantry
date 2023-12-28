#ifndef _BSP_SERVOS_H
#define _BSP_SERVOS_H

#include <stm32f4xx.h>
#include <stm32f4xx_conf.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define SERVOS_TIM_APBxClockCmd RCC_APB1PeriphClockCmd
#define SERVOS_RCC_APBxPeriph_TIMx RCC_APB1Periph_TIM3
#define SERVOS_GPIO_AHBxPeriphClockCmd RCC_AHB1PeriphClockCmd
#define SERVOS_RCC_AHBxPeriph_GPIOx RCC_AHB1Periph_GPIOB

#define SERVOS_GPIOx GPIOB
#define SERVOS_GPIO_PinSourcex GPIO_PinSource1
#define SERVOS_GPIO_AF_TIMx GPIO_AF_TIM3
#define SERVOS_GPIO_Pin_x GPIO_Pin_1

#define SERVOS_TIMx TIM3

#define SERVOS_INIT_POS 3050 //舵机初始角度

// PWM以及定时器的设置到特定位置去调整

// x 为第几个channel
#define SERVOS_TIM_OCxInit TIM_OC4Init
#define SERVOS_TIM_OCxPreloadConfig TIM_OC4PreloadConfig
#define SERVOS_TIM_SetComparex TIM_SetCompare4

void Servos_Configuration(void);

#endif

#ifndef __LASER_H__
#define __LASER_H__

#include <stm32f4xx.h>
#include <stm32f4xx_conf.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

void LaserConfigration(void);

#define LASER_RCC_AHBxPeriphClockCmd RCC_AHB1PeriphClockCmd
#define LASER_RCC_AHBxPeriph_GPIOx RCC_AHB1Periph_GPIOC
#define LASER_GPIOx GPIOC
#define LASER_GPIO_Pin_x GPIO_Pin_15

#define LASER_ON GPIO_ResetBits(LASER_GPIOx, LASER_GPIO_Pin_x)
#define LASER_OFF GPIO_SetBits(LASER_GPIOx, LASER_GPIO_Pin_x)

#endif

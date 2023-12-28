/**
 ******************************************************************************
 * @file    laser.c
 * @brief   激光端口配置
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "laser.h"

/**
 * @brief 激光初始化
 * @param[in] void
 */
void LaserConfigration()
{
	GPIO_InitTypeDef gpio;
	LASER_RCC_AHBxPeriphClockCmd(LASER_RCC_AHBxPeriph_GPIOx, ENABLE);
	gpio.GPIO_Pin = LASER_GPIO_Pin_x;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(LASER_GPIOx, &gpio);
}

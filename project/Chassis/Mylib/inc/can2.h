#ifndef __CAN2_H__
#define __CAN2_H__

#include <stm32f4xx.h>
#include <stm32f4xx_conf.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "can2_receive.h"
#include "counter.h"

#include "debug.h"

/*  params */
#define CAN2_RCC_AHBx_GPIOx RCC_AHB1Periph_GPIOB
#define CAN2_GPIOx GPIOB
#define CAN2_GPIO_PinSource_x1 GPIO_PinSource13
#define CAN2_GPIO_PinSource_x2 GPIO_PinSource12
#define CAN2_GPIO_Pin_x1 GPIO_Pin_13
#define CAN2_GPIO_Pin_x2 GPIO_Pin_12
/* GPIO特性设置到特定的位置修改 */
/* NVIC优先级到特定位置修改 */
/* CAN2特性配置到特定位置修改 */
/* 过滤器的其它设置到特定位置改 */

void CAN2_Configuration(void);

#endif

#ifndef _DJI_REMOTE_H
#define _DJI_REMOTE_H

#include <stm32f4xx.h>
#include <stm32f4xx_conf.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "remote_control.h"

#define DJI_DBUS_RECEIVE_NUM 20
#define DJI_DBUS_DATA_NUM 18

#define DJI_REMOTE_GPIOx_CLOCK_CMD RCC_AHB1PeriphClockCmd
#define DJI_REMOTE_RCC_AxBxPeriph_GPIOx RCC_AHB1Periph_GPIOA
#define DJI_REMOTE_UARTx_CLOCK_CMD RCC_APB2PeriphClockCmd
#define DJI_REMOTE_RCC_AxBxPeriph_UARTx RCC_APB2Periph_USART1
#define DJI_REMOTE_GPIOx GPIOA
#define DJI_REMOTE_UARTx USART1
#define DJI_REMOTE_GPIO_AF_USARTx GPIO_AF_USART1
#define DJI_REMOTE_GPIO_PinSource GPIO_PinSource10
#define DJI_REMOTE_GPIO_Pin_x GPIO_Pin_10
#define DJI_REMOTE_BaudRate 100000
#define DJI_REMOTE_IRQn USART1_IRQn
#define DJI_REMOTE_IRQHandler USART1_IRQHandler

#define DJI_REMOTE_RECV_DMAx_Streamx DMA2_Stream5
#define DJI_REMOTE_RECV_RCC_AxBxPeriphClockCmd RCC_AHB1PeriphClockCmd
#define DJI_REMOTE_RECV_RCC_AxBxPeriph_DMAx RCC_AHB1Periph_DMA2
#define DJI_REMOTE_RECV_DMA_Channel_x DMA_Channel_4

/* 其它GPIO配置到特定位置修改 */
/* 其它UART配置到特定位置修改 */
/* 其它NVIC配置到特定位置修改 */
/* 其它DMA配置到特定位置修改 */

void DJI_REMOTE_Configuration(void);

#endif // !_DJI_REMOTE_H

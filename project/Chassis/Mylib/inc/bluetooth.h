#ifndef _BLUE_TOOTH_H
#define _BLUE_TOOTH_H

#include <stm32f4xx.h>
#include <stm32f4xx_conf.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "remote_control.h"

/* params */
#define BLUE_TOOTH_GPIOx_CLOCK_CMD RCC_AHB1PeriphClockCmd
#define BLUE_TOOTH_RCC_AxBxPeriph_GPIOx RCC_AHB1Periph_GPIOA
#define BLUE_TOOTH_UARTx_CLOCK_CMD RCC_APB1PeriphClockCmd
#define BLUE_TOOTH_RCC_AxBxPeriph_UARTx RCC_APB1Periph_USART2
#define BLUE_TOOTH_GPIOx GPIOA
#define BLUE_TOOTH_UARTx USART2
#define BLUE_TOOTH_GPIO_AF_USARTx GPIO_AF_USART2
#define BLUE_TOOTH_GPIO_PinSourcex1 GPIO_PinSource2
#define BLUE_TOOTH_GPIO_PinSourcex2 GPIO_PinSource3
#define BLUE_TOOTH_GPIO_Pin_x1 GPIO_Pin_2
#define BLUE_TOOTH_GPIO_Pin_x2 GPIO_Pin_3
#define BLUE_TOOTH_BaudRate 115200
#define BLUE_TOOTH_IRQn USART2_IRQn
#define BLUE_TOOTH_IRQHandler USART2_IRQHandler

#define BLUE_TOOTH_SEND_DMAx_Streamx_IRQn DMA1_Stream6_IRQn
#define BLUE_TOOTH_SEND_DMAx_Streamx DMA1_Stream6
#define BLUE_TOOTH_SEND_RCC_AxBxPeriphClockCmd RCC_AHB1PeriphClockCmd
#define BLUE_TOOTH_SEND_RCC_AxBxPeriph_DMAx RCC_AHB1Periph_DMA1
#define BLUE_TOOTH_SEND_DMA_Channel_x DMA_Channel_4
#define BLUE_TOOTH_SEND_DMAx_Streamx_IRQHandler DMA1_Stream6_IRQHandler
#define BLUE_TOOTH_SEND_DMA_FLAG_TCIFx DMA_FLAG_TCIF6
#define BLUE_TOOTH_SEND_DMA_IT_TCIFx DMA_IT_TCIF6

/* 其它GPIO配置到特定位置修改 */
/* 其它UART配置到特定位置修改 */
/* 其它NVIC配置到特定位置修改 */

#pragma pack(push, 1)
#define CH_COUNT_B 6
typedef struct BlueToothSendData
{
    float fdata[CH_COUNT_B];
    unsigned char tail[4];
} BlueToothSendData;
#pragma pack(pop)

#define BlueTooth_SENDBUF_SIZE sizeof(BlueToothSendData)

void BLUE_TOOTH_Configuration(void);
void BLUE_TOOTHSendData(BlueToothSendData *data);

#endif // !_BLUE_TOOTH_H

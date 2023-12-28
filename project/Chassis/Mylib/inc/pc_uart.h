#ifndef _PC_UART_H
#define _PC_UART_H

#include <stm32f4xx.h>
#include <stm32f4xx_conf.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// /* params */
// #define PC_UART_GPIOx_CLOCK_CMD RCC_AHB1PeriphClockCmd
// #define PC_UART_RCC_AxBxPeriph_GPIOx RCC_AHB1Periph_GPIOA
// #define PC_UART_UARTx_CLOCK_CMD RCC_APB1PeriphClockCmd
// #define PC_UART_RCC_AxBxPeriph_UARTx RCC_APB1Periph_USART2
// #define PC_UART_GPIOx GPIOA
// #define PC_UART_UARTx USART2
// #define PC_UART_GPIO_AF_USARTx GPIO_AF_USART2
// #define PC_UART_GPIO_PinSourcex1 GPIO_PinSource2
// #define PC_UART_GPIO_PinSourcex2 GPIO_PinSource3
// #define PC_UART_GPIO_Pin_x1 GPIO_Pin_2
// #define PC_UART_GPIO_Pin_x2 GPIO_Pin_3
// #define PC_UART_BaudRate 115200

// #define PC_UART_RECV_DMAx_Streamx_IRQn DMA1_Stream1_IRQn
// #define PC_UART_RECV_DMAx_Streamx DMA1_Stream1
// #define PC_UART_RECV_RCC_AxBxPeriphClockCmd RCC_AHB1PeriphClockCmd
// #define PC_UART_RECV_RCC_AxBxPeriph_DMAx RCC_AHB1Periph_DMA1
// #define PC_UART_RECV_DMA_Channel_x DMA_Channel_4
// #define PC_UART_RECV_DMAx_Streamx_IRQHandler DMA1_Stream1_IRQHandler
// #define PC_UART_RECV_DMA_FLAG_TCIFx DMA_FLAG_TCIF5
// #define PC_UART_RECV_DMA_IT_TCIFx DMA_IT_TCIF5

// #define PC_UART_SEND_DMAx_Streamx_IRQn DMA1_Stream4_IRQn
// #define PC_UART_SEND_DMAx_Streamx DMA1_Stream4
// #define PC_UART_SEND_RCC_AxBxPeriphClockCmd RCC_AHB1PeriphClockCmd
// #define PC_UART_SEND_RCC_AxBxPeriph_DMAx RCC_AHB1Periph_DMA1
// #define PC_UART_SEND_DMA_Channel_x DMA_Channel_4
// #define PC_UART_SEND_DMAx_Streamx_IRQHandler DMA1_Stream4_IRQHandler
// #define PC_UART_SEND_DMA_FLAG_TCIFx DMA_FLAG_TCIF6
// #define PC_UART_SEND_DMA_IT_TCIFx DMA_IT_TCIF6

// /* 其它GPIO配置到特定位置修改 */
// /* 其它UART配置到特定位置修改 */
// /* 其它NVIC配置到特定位置修改 */
// /* 其它DMA配置到特定位置修改 */

// /*   发送数据定义 */

#pragma pack(push, 1)     //不进行字节对齐
typedef struct PCSendData //数据顺序不能变,注意32字节对齐
{
    uint8_t start_flag;
    int32_t yaw;
    int16_t pitch;
    int16_t roll;
    uint8_t crc_8;
} PCSendData;

typedef struct PCRecvData
{
    uint8_t start_flag;
    uint8_t crc_8;
} PCRecvData;

#pragma pack(pop) //不进行字节对齐

#define PC_SENDBUF_SIZE sizeof(PCSendData)
#define PC_RECVBUF_SIZE sizeof(PCRecvData)

// void PC_UART_Configuration(void);

#endif // !_PC_UART_H

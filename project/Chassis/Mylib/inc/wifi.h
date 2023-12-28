#ifndef _WIFI_H
#define _WIFI_H

// #include <stm32f4xx.h>
// #include <stm32f4xx_conf.h>
// #include <string.h>
// #include <stdint.h>
// #include <math.h>
// #include <stdio.h>
// #include <stdlib.h>

// /* params */
// #define WIFI_GPIOx_CLOCK_CMD RCC_AHB1PeriphClockCmd
// #define WIFI_RCC_AxBxPeriph_GPIOx RCC_AHB1Periph_GPIOA
// #define WIFI_UARTx_CLOCK_CMD RCC_APB1PeriphClockCmd
// #define WIFI_RCC_AxBxPeriph_UARTx RCC_APB1Periph_USART2
// #define WIFI_GPIOx GPIOA
// #define WIFI_UARTx USART2
// #define WIFI_GPIO_AF_USARTx GPIO_AF_USART2
// #define WIFI_GPIO_PinSourcex1 GPIO_PinSource2
// #define WIFI_GPIO_PinSourcex2 GPIO_PinSource3
// #define WIFI_GPIO_Pin_x1 GPIO_Pin_2
// #define WIFI_GPIO_Pin_x2 GPIO_Pin_3
// #define WIFI_BaudRate 9600

// #define WIFI_SEND_DMAx_Streamx_IRQn DMA1_Stream3_IRQn
// #define WIFI_SEND_DMAx_Streamx DMA1_Stream3
// #define WIFI_SEND_RCC_AxBxPeriphClockCmd RCC_AHB1PeriphClockCmd
// #define WIFI_SEND_RCC_AxBxPeriph_DMAx RCC_AHB1Periph_DMA1
// #define WIFI_SEND_DMA_Channel_x DMA_Channel_4
// #define WIFI_SEND_DMAx_Streamx_IRQHandler DMA1_Stream3_IRQHandler
// #define WIFI_SEND_DMA_FLAG_TCIFx DMA_FLAG_TCIF6
// #define WIFI_SEND_DMA_IT_TCIFx DMA_IT_TCIF6

// #define WIFI_RECV_DMAx_Streamx_IRQn DMA1_Stream2_IRQn
// #define WIFI_RECV_DMAx_Streamx DMA1_Stream2
// #define WIFI_RECV_RCC_AxBxPeriphClockCmd RCC_AHB1PeriphClockCmd
// #define WIFI_RECV_RCC_AxBxPeriph_DMAx RCC_AHB1Periph_DMA1
// #define WIFI_RECV_DMA_Channel_x DMA_Channel_4
// #define WIFI_RECV_DMAx_Streamx_IRQHandler DMA1_Stream2_IRQHandler
// #define WIFI_RECV_DMA_FLAG_TCIFx DMA_FLAG_TCIF5
// #define WIFI_RECV_DMA_IT_TCIFx DMA_IT_TCIF5

// /* 其它GPIO配置到特定位置修改 */
// /* 其它UART配置到特定位置修改 */
// /* 其它NVIC配置到特定位置修改 */
// /* 其它DMA配置到特定位置修改 */

// /*   发送数据定义 */

// //仅发送数据，不接收数据
// #define CH_COUNT 4
// #pragma pack(push, 1)       //不进行字节对齐
// typedef struct WifiSendData //数据顺序不能变,注意32字节对齐
// {
//     float fdata[CH_COUNT];
//     unsigned char tail[4];
// } WifiSendData;

// typedef struct WifiRecvData
// {
//     uint8_t start_flag;
//     uint8_t crc_8;
//     uint8_t end_flag;
// } WifiRecvData;

// #pragma pack(pop) //不进行字节对齐

// #define WIFI_SENDBUF_SIZE sizeof(WifiSendData)
// #define WIFI_RECVBUF_SIZE sizeof(WifiRecvData)

// void WIFI_Configuration(void);
// void wifiSendData(WifiSendData *data);

#endif // !_WIFI_H

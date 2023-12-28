#include "wifi.h"

// unsigned char SendToWifi_Buff[WIFI_SENDBUF_SIZE];
// unsigned char WIFIbuffer[WIFI_RECVBUF_SIZE];

// /**
//  * @brief 与PC通信的UART
//  * @param[in] void
//  */
// void WIFI_Configuration(void)
// {
//     USART_InitTypeDef usart;
//     GPIO_InitTypeDef gpio;
//     NVIC_InitTypeDef nvic;

//     WIFI_GPIOx_CLOCK_CMD(WIFI_RCC_AxBxPeriph_GPIOx, ENABLE);
//     WIFI_UARTx_CLOCK_CMD(WIFI_RCC_AxBxPeriph_UARTx, ENABLE);

//     GPIO_PinAFConfig(WIFI_GPIOx, WIFI_GPIO_PinSourcex1, WIFI_GPIO_AF_USARTx);
//     GPIO_PinAFConfig(WIFI_GPIOx, WIFI_GPIO_PinSourcex2, WIFI_GPIO_AF_USARTx);

//     gpio.GPIO_Pin = WIFI_GPIO_Pin_x1 | WIFI_GPIO_Pin_x2;
//     gpio.GPIO_Mode = GPIO_Mode_AF;
//     gpio.GPIO_OType = GPIO_OType_PP;
//     gpio.GPIO_Speed = GPIO_Speed_100MHz;
//     gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
//     GPIO_Init(WIFI_GPIOx, &gpio);

//     usart.USART_BaudRate = WIFI_BaudRate;
//     usart.USART_WordLength = USART_WordLength_8b;
//     usart.USART_StopBits = USART_StopBits_1;
//     usart.USART_Parity = USART_Parity_No;
//     usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
//     usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//     USART_Init(WIFI_UARTx, &usart);

//     USART_ITConfig(WIFI_UARTx, USART_IT_IDLE, ENABLE);
//     USART_Cmd(WIFI_UARTx, ENABLE);

//     USART_Cmd(WIFI_UARTx, ENABLE);
//     USART_DMACmd(WIFI_UARTx, USART_DMAReq_Rx, ENABLE);
//     USART_DMACmd(WIFI_UARTx, USART_DMAReq_Tx, ENABLE);

//     nvic.NVIC_IRQChannel = WIFI_RECV_DMAx_Streamx_IRQn;
//     nvic.NVIC_IRQChannelPreemptionPriority = 0;
//     nvic.NVIC_IRQChannelSubPriority = 0;
//     nvic.NVIC_IRQChannelCmd = ENABLE;
//     NVIC_Init(&nvic);

//     nvic.NVIC_IRQChannel = WIFI_SEND_DMAx_Streamx_IRQn;
//     nvic.NVIC_IRQChannelPreemptionPriority = 0;
//     nvic.NVIC_IRQChannelSubPriority = 1;
//     nvic.NVIC_IRQChannelCmd = ENABLE;
//     NVIC_Init(&nvic);

//     { // RX
//         DMA_InitTypeDef dma;
//         WIFI_RECV_RCC_AxBxPeriphClockCmd(WIFI_RECV_RCC_AxBxPeriph_DMAx, ENABLE);
//         DMA_DeInit(WIFI_RECV_DMAx_Streamx);
//         dma.DMA_Channel = WIFI_RECV_DMA_Channel_x;
//         dma.DMA_PeripheralBaseAddr = (uint32_t) & (WIFI_UARTx->DR);
//         dma.DMA_Memory0BaseAddr = (uint32_t)WIFIbuffer;
//         dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
//         dma.DMA_BufferSize = WIFI_RECVBUF_SIZE;
//         dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//         dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
//         dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//         dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//         dma.DMA_Mode = DMA_Mode_Circular;
//         dma.DMA_Priority = DMA_Priority_VeryHigh;
//         dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
//         dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//         dma.DMA_MemoryBurst = DMA_Mode_Normal;
//         dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//         DMA_Init(WIFI_RECV_DMAx_Streamx, &dma);
//         DMA_ITConfig(WIFI_RECV_DMAx_Streamx, DMA_IT_TC, ENABLE);
//         DMA_Cmd(WIFI_RECV_DMAx_Streamx, ENABLE);
//     }

//     { //  TX
//         DMA_InitTypeDef dma;
//         WIFI_SEND_RCC_AxBxPeriphClockCmd(WIFI_SEND_RCC_AxBxPeriph_DMAx, ENABLE);
//         DMA_DeInit(WIFI_SEND_DMAx_Streamx);
//         dma.DMA_Channel = WIFI_SEND_DMA_Channel_x;
//         dma.DMA_PeripheralBaseAddr = (uint32_t) & (WIFI_UARTx->DR);
//         dma.DMA_Memory0BaseAddr = (uint32_t)SendToWifi_Buff;
//         dma.DMA_DIR = DMA_DIR_MemoryToPeripheral;
//         dma.DMA_BufferSize = WIFI_SENDBUF_SIZE;
//         dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//         dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
//         dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//         dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//         dma.DMA_Mode = DMA_Mode_Circular;
//         dma.DMA_Priority = DMA_Priority_VeryHigh;
//         dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
//         dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//         dma.DMA_MemoryBurst = DMA_Mode_Normal;
//         dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//         DMA_Init(WIFI_SEND_DMAx_Streamx, &dma);
//         DMA_Cmd(WIFI_SEND_DMAx_Streamx, DISABLE);
//         DMA_ITConfig(WIFI_SEND_DMAx_Streamx, DMA_IT_TC, ENABLE);
//     }
// }

// /**
//  * @brief 与PC通信的发送中断
//  * @param[in] void
//  */
// void WIFI_SEND_DMAx_Streamx_IRQHandler(void)
// {
//     if (DMA_GetITStatus(WIFI_SEND_DMAx_Streamx, WIFI_SEND_DMA_IT_TCIFx))
//     {
//         DMA_ClearFlag(WIFI_SEND_DMAx_Streamx, WIFI_SEND_DMA_FLAG_TCIFx);
//         DMA_ClearITPendingBit(WIFI_SEND_DMAx_Streamx, WIFI_SEND_DMA_IT_TCIFx);
//     }
// }

// /**
//  * @brief 与PC通信的接收中断
//  * @param[in] void
//  */
// void WIFI_RECV_DMAx_Streamx_IRQHandler(void)
// {
//     if (DMA_GetITStatus(WIFI_RECV_DMAx_Streamx, WIFI_RECV_DMA_IT_TCIFx))
//     {
//         DMA_ClearFlag(WIFI_RECV_DMAx_Streamx, WIFI_RECV_DMA_FLAG_TCIFx);
//         DMA_ClearITPendingBit(WIFI_RECV_DMAx_Streamx, WIFI_RECV_DMA_IT_TCIFx);
//     }
// }

// void wifiSendData(WifiSendData *data)
// {
//     memcpy(SendToWifi_Buff, data, sizeof(WifiSendData));
//     SendToWifi_Buff[sizeof(WifiSendData) - 1] = 0x7f;
//     SendToWifi_Buff[sizeof(WifiSendData) - 2] = 0x80;
//     SendToWifi_Buff[sizeof(WifiSendData) - 3] = 0x00;
//     SendToWifi_Buff[sizeof(WifiSendData) - 4] = 0x00;

//     DMA_Cmd(WIFI_SEND_DMAx_Streamx, ENABLE);
// }

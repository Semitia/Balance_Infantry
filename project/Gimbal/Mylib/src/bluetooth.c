// #include "bluetooth.h"

// unsigned char BlueToothSend_Buff[BlueTooth_SENDBUF_SIZE];

// void BLUE_TOOTH_Configuration(void)
// {
//     USART_InitTypeDef usart;
//     GPIO_InitTypeDef gpio;
//     NVIC_InitTypeDef nvic;

//     BLUE_TOOTH_GPIOx_CLOCK_CMD(BLUE_TOOTH_RCC_AxBxPeriph_GPIOx, ENABLE);
//     BLUE_TOOTH_UARTx_CLOCK_CMD(BLUE_TOOTH_RCC_AxBxPeriph_UARTx, ENABLE);

//     GPIO_PinAFConfig(BLUE_TOOTH_GPIOx, BLUE_TOOTH_GPIO_PinSourcex1, BLUE_TOOTH_GPIO_AF_USARTx);
//     GPIO_PinAFConfig(BLUE_TOOTH_GPIOx, BLUE_TOOTH_GPIO_PinSourcex2, BLUE_TOOTH_GPIO_AF_USARTx);

//     gpio.GPIO_Pin = BLUE_TOOTH_GPIO_Pin_x1 | BLUE_TOOTH_GPIO_Pin_x2;
//     gpio.GPIO_Mode = GPIO_Mode_AF;
//     gpio.GPIO_OType = GPIO_OType_PP;
//     gpio.GPIO_Speed = GPIO_Speed_100MHz;
//     gpio.GPIO_PuPd = GPIO_PuPd_UP;
//     GPIO_Init(BLUE_TOOTH_GPIOx, &gpio);

//     usart.USART_BaudRate = BLUE_TOOTH_BaudRate;
//     usart.USART_WordLength = USART_WordLength_8b;
//     usart.USART_StopBits = USART_StopBits_1;
//     usart.USART_Parity = USART_Parity_No;
//     usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
//     usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//     USART_Init(BLUE_TOOTH_UARTx, &usart);

//     USART_ITConfig(BLUE_TOOTH_UARTx, USART_IT_RXNE, ENABLE); //开启串口接受中断
//     USART_Cmd(BLUE_TOOTH_UARTx, ENABLE);
//     USART_DMACmd(BLUE_TOOTH_UARTx, USART_DMAReq_Tx, ENABLE);

//     nvic.NVIC_IRQChannel = BLUE_TOOTH_IRQn;
//     nvic.NVIC_IRQChannelPreemptionPriority = 0;
//     nvic.NVIC_IRQChannelSubPriority = 0;
//     nvic.NVIC_IRQChannelCmd = ENABLE;
//     NVIC_Init(&nvic);

//     nvic.NVIC_IRQChannel = BLUE_TOOTH_SEND_DMAx_Streamx_IRQn;
//     nvic.NVIC_IRQChannelPreemptionPriority = 0;
//     nvic.NVIC_IRQChannelSubPriority = 1;
//     nvic.NVIC_IRQChannelCmd = ENABLE;
//     NVIC_Init(&nvic);

//     { //  TX
//         DMA_InitTypeDef dma;
//         BLUE_TOOTH_SEND_RCC_AxBxPeriphClockCmd(BLUE_TOOTH_SEND_RCC_AxBxPeriph_DMAx, ENABLE);
//         DMA_DeInit(BLUE_TOOTH_SEND_DMAx_Streamx);
//         dma.DMA_Channel = BLUE_TOOTH_SEND_DMA_Channel_x;
//         dma.DMA_PeripheralBaseAddr = (uint32_t) & (BLUE_TOOTH_UARTx->DR);
//         dma.DMA_Memory0BaseAddr = (uint32_t)BlueToothSend_Buff;
//         dma.DMA_DIR = DMA_DIR_MemoryToPeripheral;
//         dma.DMA_BufferSize = BlueTooth_SENDBUF_SIZE;
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
//         DMA_Init(BLUE_TOOTH_SEND_DMAx_Streamx, &dma);
//         DMA_Cmd(BLUE_TOOTH_SEND_DMAx_Streamx, DISABLE);
//         DMA_ITConfig(BLUE_TOOTH_SEND_DMAx_Streamx, DMA_IT_TC, ENABLE);
//     }
// }

// void BLUE_TOOTH_IRQHandler(void) //串口1中断服务程序
// {
//     uint16_t blue_tooth_recv_data;
//     if (USART_GetITStatus(BLUE_TOOTH_UARTx, USART_IT_RXNE) != RESET)
//     {
//         USART_ClearITPendingBit(BLUE_TOOTH_UARTx, USART_IT_RXNE);
//         blue_tooth_recv_data = USART_ReceiveData(BLUE_TOOTH_UARTx);
//         Blue_Tooth_Deal(&blue_tooth_recv_data);
//     }
// }

// //给蓝牙模块发数据
// void BLUE_TOOTH_SendString(const uint8_t *str)
// {
//     u8 i = 0;
//     while (*(str + i) != '\n')
//     {
//         USART_SendData(BLUE_TOOTH_UARTx, *(str + i));
//         while (USART_GetFlagStatus(BLUE_TOOTH_UARTx, USART_FLAG_TXE) == RESET)
//             ;
//         i++;
//     }
// }

// /**
//  * @brief 与PC通信的发送中断
//  * @param[in] void
//  */
// void BLUE_TOOTH_SEND_DMAx_Streamx_IRQHandler(void)
// {
//     if (DMA_GetITStatus(BLUE_TOOTH_SEND_DMAx_Streamx, BLUE_TOOTH_SEND_DMA_IT_TCIFx))
//     {
//         DMA_ClearFlag(BLUE_TOOTH_SEND_DMAx_Streamx, BLUE_TOOTH_SEND_DMA_FLAG_TCIFx);
//         DMA_ClearITPendingBit(BLUE_TOOTH_SEND_DMAx_Streamx, BLUE_TOOTH_SEND_DMA_IT_TCIFx);
//     }
// }

// void BLUE_TOOTHSendData(BlueToothSendData *data)
// {
//     memcpy(BlueToothSend_Buff, data, BlueTooth_SENDBUF_SIZE);
//     BlueToothSend_Buff[BlueTooth_SENDBUF_SIZE - 1] = 0x7f;
//     BlueToothSend_Buff[BlueTooth_SENDBUF_SIZE - 2] = 0x80;
//     BlueToothSend_Buff[BlueTooth_SENDBUF_SIZE - 3] = 0x00;
//     BlueToothSend_Buff[BlueTooth_SENDBUF_SIZE - 4] = 0x00;

//     DMA_Cmd(BLUE_TOOTH_SEND_DMAx_Streamx, ENABLE);
// }

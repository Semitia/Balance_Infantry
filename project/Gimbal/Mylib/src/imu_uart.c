// #include "imu_uart.h"

// #include "GimbalEstimateTask.h"

// unsigned char imu_rx_buffer[IMU_RECEIVE_NUM];

// void IMU_Receive(uint8_t *data)
// {
//     if (data[0] == '!' && Verify_CRC16_Check_Sum(data, IMU_RECEIVE_NUM))
//     {
//         memcpy(IMUReceive.Gyro, &data[1], 6);
//         memcpy(IMUReceive.Acc, &data[7], 6);

//         LossUpdate(&global_debugger.imu_debugger[0], 0.0015);
//     }
// }
// /**
//  * @brief DJI遥控器串口接收配置
//  * @param[in] void
//  */
// void IMU_UART_Configuration(void)
// {
//     USART_InitTypeDef usart;
//     GPIO_InitTypeDef gpio;
//     NVIC_InitTypeDef nvic;

//     IMU_UART_GPIOx_CLOCK_CMD(IMU_UART_RCC_AxBxPeriph_GPIOx, ENABLE);
//     IMU_UART_UARTx_CLOCK_CMD(IMU_UART_RCC_AxBxPeriph_UARTx, ENABLE);
//     GPIO_PinAFConfig(IMU_UART_GPIOx, IMU_UART_GPIO_PinSource, IMU_UART_GPIO_AF_USARTx);

//     gpio.GPIO_Pin = IMU_UART_GPIO_Pin_x;
//     gpio.GPIO_Mode = GPIO_Mode_AF;
//     gpio.GPIO_OType = GPIO_OType_PP;
//     gpio.GPIO_Speed = GPIO_Speed_100MHz;
//     gpio.GPIO_PuPd = GPIO_PuPd_UP;
//     GPIO_Init(IMU_UART_GPIOx, &gpio);

//     USART_DeInit(IMU_UART_UARTx);
//     usart.USART_BaudRate = IMU_UART_BaudRate;
//     usart.USART_WordLength = USART_WordLength_8b;
//     usart.USART_StopBits = USART_StopBits_1;
//     usart.USART_Parity = USART_Parity_No;
//     usart.USART_Mode = USART_Mode_Rx;
//     usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//     USART_Init(IMU_UART_UARTx, &usart);

//     USART_ITConfig(IMU_UART_UARTx, USART_IT_IDLE, ENABLE);
//     USART_Cmd(IMU_UART_UARTx, ENABLE);
//     USART_DMACmd(IMU_UART_UARTx, USART_DMAReq_Rx, ENABLE);
//     USART_ClearFlag(IMU_UART_UARTx, USART_FLAG_TC);

//     nvic.NVIC_IRQChannel = IMU_UART_IRQn;
//     nvic.NVIC_IRQChannelPreemptionPriority = 0;
//     nvic.NVIC_IRQChannelSubPriority = 0;
//     nvic.NVIC_IRQChannelCmd = ENABLE;
//     NVIC_Init(&nvic);

//     {
//         DMA_InitTypeDef dma;
//         IMU_UART_RECV_RCC_AxBxPeriphClockCmd(IMU_UART_RECV_RCC_AxBxPeriph_DMAx, ENABLE);
//         DMA_DeInit(IMU_UART_RECV_DMAx_Streamx);
//         while (DMA_GetCmdStatus(IMU_UART_RECV_DMAx_Streamx) != DISABLE)
//         {
//         };
//         dma.DMA_Channel = IMU_UART_RECV_DMA_Channel_x;
//         dma.DMA_PeripheralBaseAddr = (uint32_t) & (IMU_UART_UARTx->DR);
//         dma.DMA_Memory0BaseAddr = (uint32_t)imu_rx_buffer;
//         dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
//         dma.DMA_BufferSize = IMU_RECEIVE_NUM;
//         dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//         dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
//         dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//         dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//         dma.DMA_Mode = DMA_Mode_Circular;
//         dma.DMA_Priority = DMA_Priority_VeryHigh;
//         dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
//         dma.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
//         dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//         dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//         DMA_Init(IMU_UART_RECV_DMAx_Streamx, &dma);
//         DMA_ITConfig(IMU_UART_RECV_DMAx_Streamx, DMA_IT_TC, ENABLE);
//         DMA_Cmd(IMU_UART_RECV_DMAx_Streamx, ENABLE);
//     }
// }

// /**
//  * @brief DJI遥控器串口接收中断配置
//  * @param[in] void
//  */
// void IMU_UART_IRQHandler(void)
// {
// //    short DATA_LENGTH = 0;
//     if (USART_GetITStatus(IMU_UART_UARTx, USART_IT_IDLE) != RESET)
//     {
//         (void)IMU_UART_UARTx->SR;
//         (void)IMU_UART_UARTx->DR;
//         DMA_Cmd(IMU_UART_RECV_DMAx_Streamx, DISABLE);
//         DMA_ClearFlag(IMU_UART_RECV_DMAx_Streamx, IMU_UART_RECV_DMA_FLAG_TCIFx);
//         DMA_ClearITPendingBit(IMU_UART_RECV_DMAx_Streamx, IMU_UART_RECV_DMA_IT_TCIFx);
// //        DATA_LENGTH = IMU_RECEIVE_NUM - IMU_UART_RECV_DMAx_Streamx->NDTR;
// //        if (DATA_LENGTH == IMU_RECEIVE_NUM)
// //        {
//             IMU_Receive((uint8_t *)imu_rx_buffer);
// //        }
//         DMA_SetCurrDataCounter(IMU_UART_RECV_DMAx_Streamx, IMU_RECEIVE_NUM);
//         DMA_Cmd(IMU_UART_RECV_DMAx_Streamx, ENABLE);
//     }
// }

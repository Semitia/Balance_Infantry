/**
 ******************************************************************************
 * @file    motor1_rs485.c
 * @brief   485通信
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "motor_rs485.h"

/*  数据定义  */
uint8_t A1Motorbuffer[2][A1MOTOR_RECVBUF_SIZE] = {0};
uint8_t SendToA1Motor_Buff[2][SEND_TO_A1MOTOR_BUFF_SIZE] = {0};

/**
 * @brief 与电机通信的RS485配置
 * @param[in] void
 */
void MOTOR1_RS485_Configuration(void)
{
    USART_InitTypeDef usart;
    GPIO_InitTypeDef gpio;
    NVIC_InitTypeDef nvic;

    /* 如果是通信频率不够，需要加入这句  */
    // USART_OverSampling8Cmd(MOTOR1_RS485_RCC_AxBxPeriph_UARTx, ENABLE);

    MOTOR1_RS485_GPIOx_CLOCK_CMD(MOTOR1_RS485_RCC_AxBxPeriph_GPIOx, ENABLE);
    MOTOR1_RS485_UARTx_CLOCK_CMD(MOTOR1_RS485_RCC_AxBxPeriph_UARTx, ENABLE);
    MOTOR1_CONTROL_GPIOx_CLOCK_CMD(MOTOR1_CONTROL_RCC_AxBxPeriph_GPIOx, ENABLE);

    GPIO_PinAFConfig(MOTOR1_RS485_GPIOx, MOTOR1_RS485_GPIO_PinSourcex1, MOTOR1_RS485_GPIO_AF_USARTx);
    GPIO_PinAFConfig(MOTOR1_RS485_GPIOx, MOTOR1_RS485_GPIO_PinSourcex2, MOTOR1_RS485_GPIO_AF_USARTx);

    gpio.GPIO_Pin = MOTOR1_RS485_GPIO_Pin_x1 | MOTOR1_RS485_GPIO_Pin_x2;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(MOTOR1_RS485_GPIOx, &gpio);

    /* 485收发控制管脚 */
    gpio.GPIO_Pin = MOTOR1_RS485_RE_PIN;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(MOTOR1_RS485_RE_GPIO_PORT, &gpio);

    USART_DeInit(MOTOR1_RS485_UARTx);
    usart.USART_BaudRate = MOTOR1_RS485_BaudRate;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(MOTOR1_RS485_UARTx, &usart);

    USART_ITConfig(MOTOR1_RS485_UARTx, USART_IT_IDLE, ENABLE);

    USART_Cmd(MOTOR1_RS485_UARTx, ENABLE);

    USART_DMACmd(MOTOR1_RS485_UARTx, USART_DMAReq_Rx, ENABLE);
    USART_DMACmd(MOTOR1_RS485_UARTx, USART_DMAReq_Tx, ENABLE);
    USART_ClearFlag(MOTOR1_RS485_UARTx, USART_FLAG_TC);

    // nvic.NVIC_IRQChannel = MOTOR1_RS485_RECV_DMAx_Streamx_IRQn;
    // nvic.NVIC_IRQChannelPreemptionPriority = 0;
    // nvic.NVIC_IRQChannelSubPriority = 0;
    // nvic.NVIC_IRQChannelCmd = ENABLE;
    // NVIC_Init(&nvic);
    nvic.NVIC_IRQChannel = MOTOR1_RS485_RECV_USARTx_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel = MOTOR1_RS485_SEND_DMAx_Streamx_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    { // RX
        DMA_InitTypeDef dma;
        MOTOR1_RS485_RECV_RCC_AxBxPeriphClockCmd(MOTOR1_RS485_RECV_RCC_AxBxPeriph_DMAx, ENABLE);
        DMA_DeInit(MOTOR1_RS485_RECV_DMAx_Streamx);
        while (DMA_GetCmdStatus(MOTOR1_RS485_RECV_DMAx_Streamx) != DISABLE)
        {
        };
        dma.DMA_Channel = MOTOR1_RS485_RECV_DMA_Channel_x;
        dma.DMA_PeripheralBaseAddr = (uint32_t) & (MOTOR1_RS485_UARTx->DR);
        dma.DMA_Memory0BaseAddr = (uint32_t)A1Motorbuffer[0];
        dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
        dma.DMA_BufferSize = A1MOTOR_RECVBUF_SIZE;
        dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
        dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        dma.DMA_Mode = DMA_Mode_Circular;
        dma.DMA_Priority = DMA_Priority_Medium;
        dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
        dma.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
        dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(MOTOR1_RS485_RECV_DMAx_Streamx, &dma);
        DMA_ITConfig(MOTOR1_RS485_RECV_DMAx_Streamx, DMA_IT_TC, ENABLE);
        DMA_Cmd(MOTOR1_RS485_RECV_DMAx_Streamx, ENABLE);
    }

    { //  TX
        DMA_InitTypeDef dma;
        MOTOR1_RS485_SEND_RCC_AxBxPeriphClockCmd(MOTOR1_RS485_SEND_RCC_AxBxPeriph_DMAx, ENABLE);
        DMA_DeInit(MOTOR1_RS485_SEND_DMAx_Streamx);
        while (DMA_GetCmdStatus(MOTOR1_RS485_SEND_DMAx_Streamx) != DISABLE)
        {
        };
        dma.DMA_Channel = MOTOR1_RS485_SEND_DMA_Channel_x;
        dma.DMA_PeripheralBaseAddr = (uint32_t) & (MOTOR1_RS485_UARTx->DR);
        dma.DMA_Memory0BaseAddr = (uint32_t)SendToA1Motor_Buff[0];
        dma.DMA_DIR = DMA_DIR_MemoryToPeripheral;
        dma.DMA_BufferSize = SEND_TO_A1MOTOR_BUFF_SIZE;
        dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
        dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        dma.DMA_Mode = DMA_Mode_Normal;
        dma.DMA_Priority = DMA_Priority_VeryHigh;
        dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
        dma.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
        dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(MOTOR1_RS485_SEND_DMAx_Streamx, &dma);
        DMA_Cmd(MOTOR1_RS485_SEND_DMAx_Streamx, DISABLE);
        DMA_ITConfig(MOTOR1_RS485_SEND_DMAx_Streamx, DMA_IT_TC, ENABLE);
    }
    MOTOR1_START_READ();
}

/**
 * @brief RS485的接收中断
 * @param[in] void
 */
void MOTOR1_RS485_RECV_USARTx_IRQHandler(void)
{
    if (USART_GetITStatus(MOTOR1_RS485_UARTx, USART_IT_IDLE) != RESET)
    {
        //        uint8_t DATA_LENGTH;

        (void)MOTOR1_RS485_UARTx->SR;
        (void)MOTOR1_RS485_UARTx->DR;
        DMA_Cmd(MOTOR1_RS485_RECV_DMAx_Streamx, DISABLE);
        DMA_ClearFlag(MOTOR1_RS485_RECV_DMAx_Streamx, MOTOR1_RS485_RECV_DMA_FLAG_TCIFx);
        DMA_ClearITPendingBit(MOTOR1_RS485_RECV_DMAx_Streamx, MOTOR1_RS485_RECV_DMA_IT_TCIFx);

        RS485Receive(1);

        DMA_SetCurrDataCounter(MOTOR1_RS485_RECV_DMAx_Streamx, A1MOTOR_RECVBUF_SIZE);
        DMA_Cmd(MOTOR1_RS485_RECV_DMAx_Streamx, ENABLE);
    }
}
// void MOTOR1_RS485_RECV_DMAx_Streamx_IRQHandler(void)
// {
//     if (DMA_GetITStatus(MOTOR1_RS485_RECV_DMAx_Streamx, MOTOR1_RS485_RECV_DMA_IT_TCIFx))
//     {
//         DMA_ClearFlag(MOTOR1_RS485_RECV_DMAx_Streamx, MOTOR1_RS485_RECV_DMA_FLAG_TCIFx);
//         DMA_ClearITPendingBit(MOTOR1_RS485_RECV_DMAx_Streamx, MOTOR1_RS485_RECV_DMA_IT_TCIFx);
//         RS485Receive(1);
//     }
// }

/**
 * @brief RS485的发送中断
 * @param[in] void
 */
void MOTOR1_RS485_SEND_DMAx_Streamx_IRQHandler(void)
{
    if (DMA_GetITStatus(MOTOR1_RS485_SEND_DMAx_Streamx, MOTOR1_RS485_SEND_DMA_IT_TCIFx))
    {
        DMA_ClearFlag(MOTOR1_RS485_SEND_DMAx_Streamx, MOTOR1_RS485_SEND_DMA_FLAG_TCIFx);
        DMA_ClearITPendingBit(MOTOR1_RS485_SEND_DMAx_Streamx, MOTOR1_RS485_SEND_DMA_IT_TCIFx);
        while (USART_GetFlagStatus(MOTOR1_RS485_UARTx, USART_FLAG_TC) == RESET)
            ;
        USART_ClearFlag(MOTOR1_RS485_UARTx, USART_FLAG_TC);
        global_debugger.a1_motor_debugger[0].send_msgs_num++;
        global_debugger.a1_motor_debugger[1].send_msgs_num++;
        MOTOR1_START_READ();
    }
}

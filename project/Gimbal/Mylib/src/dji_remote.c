#include "dji_remote.h"

volatile unsigned char sbus_rx_buffer[DJI_DBUS_RECEIVE_NUM];
/**
 * @brief DJI遥控器串口接收配置
 * @param[in] void
 */
void DJI_REMOTE_Configuration(void)
{
    USART_InitTypeDef usart;
    GPIO_InitTypeDef gpio;
    NVIC_InitTypeDef nvic;

    DJI_REMOTE_GPIOx_CLOCK_CMD(DJI_REMOTE_RCC_AxBxPeriph_GPIOx, ENABLE);
    DJI_REMOTE_UARTx_CLOCK_CMD(DJI_REMOTE_RCC_AxBxPeriph_UARTx, ENABLE);
    GPIO_PinAFConfig(DJI_REMOTE_GPIOx, DJI_REMOTE_GPIO_PinSource, DJI_REMOTE_GPIO_AF_USARTx);

    gpio.GPIO_Pin = DJI_REMOTE_GPIO_Pin_x;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(DJI_REMOTE_GPIOx, &gpio);

    usart.USART_BaudRate = DJI_REMOTE_BaudRate;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_Even;
    usart.USART_Mode = USART_Mode_Rx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(DJI_REMOTE_UARTx, &usart);

    USART_ITConfig(DJI_REMOTE_UARTx, USART_IT_IDLE, ENABLE);
    USART_Cmd(DJI_REMOTE_UARTx, ENABLE);
    USART_DMACmd(DJI_REMOTE_UARTx, USART_DMAReq_Rx, ENABLE);

    nvic.NVIC_IRQChannel = DJI_REMOTE_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 2;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    {
        DMA_InitTypeDef dma;
        DJI_REMOTE_RECV_RCC_AxBxPeriphClockCmd(DJI_REMOTE_RECV_RCC_AxBxPeriph_DMAx, ENABLE);
        DMA_DeInit(DJI_REMOTE_RECV_DMAx_Streamx);
        dma.DMA_Channel = DJI_REMOTE_RECV_DMA_Channel_x;
        dma.DMA_PeripheralBaseAddr = (uint32_t) & (DJI_REMOTE_UARTx->DR);
        dma.DMA_Memory0BaseAddr = (uint32_t)sbus_rx_buffer;
        dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
        dma.DMA_BufferSize = DJI_DBUS_RECEIVE_NUM;
        dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
        dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        dma.DMA_Mode = DMA_Mode_Circular;
        dma.DMA_Priority = DMA_Priority_VeryHigh;
        dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
        dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DJI_REMOTE_RECV_DMAx_Streamx, &dma);
        DMA_ITConfig(DJI_REMOTE_RECV_DMAx_Streamx, DMA_IT_TC, ENABLE);
        DMA_Cmd(DJI_REMOTE_RECV_DMAx_Streamx, ENABLE);
    }
}

/**
 * @brief DJI遥控器串口接收中断配置
 * @param[in] void
 */
void DJI_REMOTE_IRQHandler(void)
{
    short DATA_LENGTH = 0;
    if (USART_GetITStatus(DJI_REMOTE_UARTx, USART_IT_IDLE) != RESET)
    {
        (void)DJI_REMOTE_UARTx->SR;
        (void)DJI_REMOTE_UARTx->DR;
        DMA_Cmd(DJI_REMOTE_RECV_DMAx_Streamx, DISABLE);
        DATA_LENGTH = DJI_DBUS_RECEIVE_NUM - DJI_REMOTE_RECV_DMAx_Streamx->NDTR;
        if (DATA_LENGTH == DJI_DBUS_DATA_NUM)
        {
            RemoteReceive(sbus_rx_buffer);
        }
        DJI_REMOTE_RECV_DMAx_Streamx->NDTR = DJI_DBUS_RECEIVE_NUM;
        DMA_Cmd(DJI_REMOTE_RECV_DMAx_Streamx, ENABLE);
    }
}

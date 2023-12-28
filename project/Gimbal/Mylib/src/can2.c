/**
 ******************************************************************************
 * @file    can2.c
 * @brief   CAN2 配置以及中断处理
 ******************************************************************************
 * @attention
 ******************************************************************************
 */
#include "can2.h"

/**
 * @brief CAN2配置
 * @param[in] void
 */
void CAN2_Configuration(void)
{
    CAN_InitTypeDef can;
    CAN_FilterInitTypeDef can_filter;
    GPIO_InitTypeDef gpio;
    NVIC_InitTypeDef nvic;

    /*  RCC config */
    RCC_AHB1PeriphClockCmd(CAN2_RCC_AHBx_GPIOx, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

    /* GPIO config */
    GPIO_PinAFConfig(CAN2_GPIOx, CAN2_GPIO_PinSource_x1, GPIO_AF_CAN2);
    GPIO_PinAFConfig(CAN2_GPIOx, CAN2_GPIO_PinSource_x2, GPIO_AF_CAN2);
    gpio.GPIO_Pin = CAN2_GPIO_Pin_x1 | CAN2_GPIO_Pin_x2;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(CAN2_GPIOx, &gpio);

    // NVIC Config
    // RX0
    nvic.NVIC_IRQChannel = CAN2_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    // RX1
    nvic.NVIC_IRQChannel = CAN2_RX1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 2;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    // TX
    nvic.NVIC_IRQChannel = CAN2_TX_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    // CAN config
    CAN_DeInit(CAN2);
    CAN_StructInit(&can);
    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = ENABLE;
    can.CAN_AWUM = ENABLE;
    can.CAN_NART = DISABLE; //关闭失败自动重传
    can.CAN_RFLM = DISABLE;
    can.CAN_TXFP = ENABLE;
    can.CAN_Mode = CAN_Mode_Normal;
    can.CAN_SJW = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_11tq;
    can.CAN_BS2 = CAN_BS2_2tq;
    can.CAN_Prescaler = 3; // CAN BaudRate 42/(1+9+4)/3=1Mbps
    CAN_Init(CAN2, &can);

    // FIFO0
    can_filter.CAN_FilterNumber = 15; //选择过滤器15
    can_filter.CAN_FilterMode = CAN_FilterMode_IdList;
    can_filter.CAN_FilterScale = CAN_FilterScale_16bit;
    can_filter.CAN_FilterIdHigh = CAN2_FIFO0_ID0 << 5;
    can_filter.CAN_FilterIdLow = CAN2_FIFO0_ID1 << 5;
    can_filter.CAN_FilterMaskIdHigh = CAN2_FIFO0_ID2 << 5;
    can_filter.CAN_FilterMaskIdLow = CAN2_FIFO0_ID3 << 5;
    can_filter.CAN_FilterFIFOAssignment = 0; // fifo0
    can_filter.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&can_filter);

    // FIFO1
    can_filter.CAN_FilterNumber = 16;                  //选择过滤器16
    can_filter.CAN_FilterMode = CAN_FilterMode_IdList; //列表模式
    can_filter.CAN_FilterScale = CAN_FilterScale_16bit;
    can_filter.CAN_FilterIdHigh = CAN2_FIFO1_ID0 << 5;
    can_filter.CAN_FilterIdLow = CAN2_FIFO1_ID1 << 5;
    can_filter.CAN_FilterMaskIdHigh = CAN2_FIFO1_ID2 << 5;
    can_filter.CAN_FilterMaskIdLow = CAN2_FIFO1_ID3 << 5;
    can_filter.CAN_FilterFIFOAssignment = 1; // fifo1
    can_filter.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&can_filter);

    // CAN中断配置
    CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);
    CAN_ITConfig(CAN2, CAN_IT_FMP1, ENABLE);
    CAN_ITConfig(CAN2, CAN_IT_TME, ENABLE);
}

/**
 * @brief CAN2发送中断
 * @param[in] void
 */
void CAN2_TX_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN2, CAN_IT_TME) != RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_TME);
    }
}

/**
 * @brief CAN2接收中断0
 * @param[in] void
 */
void CAN2_RX0_IRQHandler(void)
{
    CanRxMsg rx_message0;
    if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_Receive(CAN2, CAN_FIFO0, &rx_message0);
        Can2Receive0(rx_message0);
    }
}

/**
 * @brief CAN2接收中断1
 * @param[in] void
 */
void CAN2_RX1_IRQHandler(void)
{
    CanRxMsg rx_message1;
    if (CAN_GetITStatus(CAN2, CAN_IT_FMP1) != RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP1);
        CAN_Receive(CAN2, CAN_FIFO1, &rx_message1);
        Can2Receive1(rx_message1);
    }
}

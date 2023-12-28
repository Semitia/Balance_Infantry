/**
 ******************************************************************************
 * @file    bsp_servos.c
 * @brief   �����ʼ��
 ******************************************************************************
 * @attention
 ******************************************************************************
 */
#include "bsp_servos.h"

/**
 * @brief ���ոǶ����ʼ��
 * @param[in] void
 */
void Servos_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct; //��ʼ���ṹ��

	SERVOS_TIM_APBxClockCmd(SERVOS_RCC_APBxPeriph_TIMx, ENABLE);
	SERVOS_GPIO_AHBxPeriphClockCmd(SERVOS_RCC_AHBxPeriph_GPIOx, ENABLE);		 //ʱ������
	GPIO_PinAFConfig(SERVOS_GPIOx, SERVOS_GPIO_PinSourcex, SERVOS_GPIO_AF_TIMx); //���Ÿ���

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; // GPIO����
	GPIO_InitStruct.GPIO_Pin = SERVOS_GPIO_Pin_x;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(SERVOS_GPIOx, &GPIO_InitStruct);

	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1; // Timer����
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 20000 - 1;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 42 - 1;
	TIM_TimeBaseInit(SERVOS_TIMx, &TIM_TimeBaseInitStruct);

	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse = SERVOS_INIT_POS;
	SERVOS_TIM_OCxInit(SERVOS_TIMx, &TIM_OCInitStruct); // Channel4����

	SERVOS_TIM_OCxPreloadConfig(SERVOS_TIMx, ENABLE);

	TIM_ARRPreloadConfig(SERVOS_TIMx, ENABLE);

	TIM_CtrlPWMOutputs(SERVOS_TIMx, ENABLE);

	TIM_Cmd(SERVOS_TIMx, ENABLE);

	SERVOS_TIM_SetComparex(SERVOS_TIMx, SERVOS_INIT_POS);
}

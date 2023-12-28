/**
 ******************************************************************************
 * @file    main.c
 * @brief   主函数，程序进口
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "main.h"

/* clock frequency */
RCC_ClocksTypeDef get_rcc_clock;

int main(void)
{
    /* get clock frequency */
    RCC_GetClocksFreq(&get_rcc_clock);

#ifdef DEBUG_MODE
    SEGGER_RTT_Init();
    LOG_CLEAR();

#ifdef JSCOPE_RTT_MODE
    JscopeRTTInit();
#endif // JSCOPE_RTT_MODE
#endif // DEBUG_MODE

    /* init */
    BSP_Init();
#if !TEST_TASK_ON
    Robot_Init();
#endif // !TEST_TASK_ON

    /* create task */
    startTask();

    /* 调度 */
    vTaskStartScheduler();

    while (1)
        ;
}

/**
 * @brief STM32初始化
 * @param[in] void
 */
void BSP_Init(void)
{
#ifdef DEBUG_MODE_FREERTOS
    /*  TIM7初始化，用于FreeRTOS 计算CPU占用情况 */
    OS_TICK_Configuration();
    delay_ms(100);
#endif // DEBUG_MODE_FREERTOS

    /*  计数器初始化，微妙级计时 */
    COUNTER_Configuration();
    delay_ms(100);

    /*  CAN1初始化 */
    CAN1_Configuration();
    delay_ms(100);

    /*  CAN2初始化,初始化前CAN1需初始化 */
    CAN2_Configuration();
    delay_ms(100);

    DJI_REMOTE_Configuration();
    delay_ms(100);

    /*  与PC间的串口通信 */
    PC_UART_Configuration();
    delay_ms(100);

    // IMU_UART_Configuration();
    // delay_ms(100);

    // WIFI_Configuration();
    // delay_ms(100);

    //    BLUE_TOOTH_Configuration();
    //    delay_ms(100);

    Servos_Configuration();
    delay_ms(100);
}

/**
 * @brief 机器人初始化
 * @param[in] void
 */
void Robot_Init(void)
{
    delay_ms(1000);

    /* init */
    while (!checkIMUOn()) //检查IMU是否开启
        ;

    /* IMU初始化 */
    INS_Init();
    ICM20602_init(&IMUReceive, &IMUData);

    initRemoteControl(DJI_REMOTE_CONTROL);
}

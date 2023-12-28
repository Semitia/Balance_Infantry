/**
 ******************************************************************************
 * @file    PC_Task.c
 * @brief   与PC通信任务
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "PC_Task.h"

/* 接收数据变量  */
extern unsigned char PCbuffer[PC_RECVBUF_SIZE];

/**
 * @brief 与PC通信串口通信任务
 * @param[in] void
 */
void PCReceive_task(void)
{
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); //若无通知更新，则不唤醒,通知实现二值信号量

        PCReceive(PCbuffer);
    }
}

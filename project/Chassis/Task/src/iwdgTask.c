#include "iwdg.h"

EventBits_t uxBits;

void Iwdg_task(void *pvParameters)
{
    vTaskDelay(5000);
    const TickType_t xTicksToWait = 1200; /* 最大延迟500ms */

    IWDG_Config(IWDG_Prescaler_64, 625 * 2); // 2s需要喂一次狗
    while (1)
    {
        /* 等待所有任务发来事件标志 */
        uxBits = xEventGroupWaitBits(xCreatedEventGroup,           /* 事件标志组句柄 */
                                     TASK_BIT_ALL,                 /* 等待TASK_BIT_ALL被设置 */
                                     pdTRUE,                       /* 退出前TASK_BIT_ALL被清除，这里是TASK_BIT_ALL都被设置才表示“退出”*/
                                     pdTRUE,                       /* 设置为pdTRUE表示等待TASK_BIT_ALL都被设置*/
                                     pdMS_TO_TICKS(xTicksToWait)); /* 等待延迟时间 */
        if ((uxBits & TASK_BIT_ALL) == TASK_BIT_ALL)               // 判断各个任务是否执行
        {
            IWDG_Feed();
        }
    }
}

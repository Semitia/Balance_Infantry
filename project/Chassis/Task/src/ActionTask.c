#include "ActionTask.h"

void Action_task(void *pvParameters)
{
    portTickType xLastWakeTime;

    vTaskDelay(2000);

    while (1)
    {
        xLastWakeTime = xTaskGetTickCount();

        Gimbal_msgs_Decode1();
        Gimbal_msgs_Decode2();

        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}

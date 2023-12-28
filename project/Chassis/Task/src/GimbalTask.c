#include "GimbalTask.h"

int8_t send_to_gimbal_data[8];

void SendToGimbalPack()
{
    GimbalSendPack();
    memcpy(send_to_gimbal_data, &gimbal_pack_send_1, 8);
}

/**
 * @brief 云台控制任务
 * @param[in] void
 */
void GimbalTask(void *pvParameters)
{
    portTickType xLastWakeTime;

    vTaskDelay(pdMS_TO_TICKS(2000));

    while (1)
    {
        xLastWakeTime = xTaskGetTickCount();

        HeatUpdate();
        SendToGimbalPack();

        CanSend(GIMBAL_CAN_COMM_CANx, send_to_gimbal_data, SEND_TO_GIMBAL_CAN_ID_1, 8);

        xEventGroupSetBits(xCreatedEventGroup, GIMBAL_TASK_BIT); // 标志位置一

        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
    }
}

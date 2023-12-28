#include "ChassisTask.h"

int8_t send_to_chassis_data_1[8]; // 模式
int8_t send_to_chassis_data_2[8]; // pitch,yaw

/**
 * @brief 处理速度数据，将底盘期望速度发送给底盘stm32
 * @param[in] void
 */
void ChassisTask(void *pvParameters)
{
    portTickType xLastWakeTime;
    const portTickType xFrequency = 1; // 1000HZ

    static int i = 0;

    vTaskDelay(6000);

    while (1)
    {
        xLastWakeTime = xTaskGetTickCount();

        if (i % 8 == 0) // 125HZ
        {
            Pack_InfantryMode();

            memcpy(send_to_chassis_data_1, &chassis_send_pack1, 8);

            CanSend(CHASSIS_CAN_COMM_CANx, send_to_chassis_data_1, SEND_TO_CHASSIS_CAN_ID_1, 8);
        }

        // 1 kHZ
        Pack_Yaw();

        memcpy(send_to_chassis_data_2, &chassis_send_pack2, 8);

        CanSend(CHASSIS_CAN_COMM_CANx, send_to_chassis_data_2, SEND_TO_CHASSIS_CAN_ID_2, 8);

        i++;

        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

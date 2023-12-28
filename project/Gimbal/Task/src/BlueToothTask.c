#include "BlueToothTask.h"

/**
 * @brief 蓝牙发送debug任务
 * @param[in] void
 */
void BlueToothTask(void *pvParameters)
{
    portTickType xLastWakeTime;
    const portTickType xFrequency = 5; // 1kHZ

    vTaskDelay(2000);

//    BlueToothSendData data;

    while (1)
    {
        xLastWakeTime = xTaskGetTickCount();

//        data.fdata[0] = balance_infantry.theta_left;
//        data.fdata[1] = balance_infantry.theta_right;
//        data.fdata[2] = balance_infantry.L_left[0];
//        data.fdata[3] = balance_infantry.L_right[0];

//        BLUE_TOOTHSendData(&data);

        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

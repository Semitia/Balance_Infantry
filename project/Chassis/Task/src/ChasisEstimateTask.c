/**
 ******************************************************************************
 * @file    ChasisEstimateTask.c
 * @brief   底盘位姿估计任务
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "ChasisEstimateTask.h"

/* IMU */
IMU IMUReceive;
IMU_Data_t IMUData;

/**
 * @brief 底盘位姿估计任务
 * @param[in] void
 */
void ChasisEstimate_task(void *pvParameters)
{
    portTickType xLastWakeTime;

    // 适当延时
    //  vTaskDelay(100);

    while (1)
    {
        xLastWakeTime = xTaskGetTickCount();

        INS_Task(&IMUReceive, &IMUData);
        // SendtoPC();
        // SendIMUByCan();

        /*  喂狗 */
        xEventGroupSetBits(xCreatedEventGroup, CHASIS_ESTIMATE_BIT); // 标志位置一

        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}

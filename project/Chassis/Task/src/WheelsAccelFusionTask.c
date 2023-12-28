/**
 ******************************************************************************
 * @file    WheelsAccelFusionTask.c
 * @brief   轮式里程计与加速度计数据融合任务
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "WheelsAccelFusionTask.h"

/**
 * @brief 轮式里程计与加速度计数据融合任务
 * @param[in] void
 */
void WheelsAccelFusion_task(void *pvParameters)
{
    portTickType xLastWakeTime;
    float speed;

    vTaskDelay(pdMS_TO_TICKS(2000));

    KF_Wheel_Accel_Init(10, 10, 1000);

    while (1)
    {
        xLastWakeTime = xTaskGetTickCount();

        GetDeltaT(&global_debugger.robot_debugger.last_cnt);
        KF_Wheel_Accel_Update(speed,
                              balance_infantry.INS->MotionAccel_n[1], 0.001);
        global_debugger.robot_debugger.dt = GetDeltaT(&global_debugger.robot_debugger.last_cnt);

        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}

/**
 ******************************************************************************
 * @file    ChasisControlTask.c
 * @brief   底盘控制任务
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "ChasisControlTask.h"

/**
 * @brief 底盘控制任务
 * @param[in] void
 */
void ChasisControl_task(void *pvParameters)
{
    portTickType xLastWakeTime;

    vTaskDelay(1000);

    /* 平衡步兵初始化 */
    balance_infantry.delta_t = GetDeltaT(&balance_infantry.last_cnt);
    BalanceInfantryInit(&balance_infantry);

    // 等待状态估计任务开始并且估计出初始yaw
    YawInit(&balance_infantry);

    KF_Wheel_Accel_Init(10, 10, 1000);

    SuperPowerInit();

    MF9025DetectInit();

    // 初始化时间差
    balance_infantry.delta_t = GetDeltaT(&balance_infantry.last_cnt);
    vTaskDelay(1);

    while (1)
    {
        xLastWakeTime = xTaskGetTickCount();

        // 获取两帧之间时间差
        balance_infantry.delta_t = GetDeltaT(&balance_infantry.last_cnt);

        /* 获得遥控器或者蓝牙控制信息  */
        // get_control_info(&balance_infantry);

        /* 获得传感器信息  */
        get_sensor_info(&balance_infantry);

        /* 根据加速度计与里程计信息进行传感器数据融合 */
        accel_odom_fusion(&balance_infantry);

        /* 滑移角计算 */
        // slid_angle_solve(&balance_infantry);

        /*  异常检测 */
        AbnormalDetect(&balance_infantry);

        /*  收腿结束判断 */
        shrink_leg_finish_judge(&balance_infantry);

        // 异常状态检测
        // MF9025StateDetect(balance_infantry.sensors_info.wheel_speed[LEFT_WHEEL_ID], balance_infantry.sensors_info.wheel_speed[RIGHT_WHEEL_ID],
        //                   balance_infantry.sensors_info.wheel_torque[LEFT_WHEEL_ID], balance_infantry.sensors_info.wheel_torque[RIGHT_WHEEL_ID], balance_infantry.delta_t);

        /* 控制 */
        // if (remote_controller.control_mode_action == SHRINK_LEG_MODE)
        // {
        //     //收腿PID
        //     shrink_leg(&balance_infantry);
        // }
        // else
        // {

        main_control(&balance_infantry);
        // }

        /* 执行控制 */
        execute_control(&balance_infantry.excute_info);

        // debug
        // 状态空间方程调试
        // Ozone[0] = balance_infantry.INS->MotionAccel_b[1] * 100.0f;
        // Ozone[1] = Wheel_Accel_Fusion.x_v * 100.0f;
        // Ozone[2] = balance_infantry.state_vector[2] * 100.0f;
        // Ozone[3] = balance_infantry.state_vector[3] * 100.0f;

        /*  喂狗 */
        xEventGroupSetBits(xCreatedEventGroup, CHASIS_CONTROL_BIT); // 标志位置一

        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}

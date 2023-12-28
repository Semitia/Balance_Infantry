#include "Offline_Task.h"

OfflineDetector offline_detector;

void Offline_task(void *pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(5000)); // 待机器人初始化后开始检测

    while (1)
    {
        int sensor_on_num = 0; // 传感器开启数量
        int knee_motor_error_num = 0;
        // IMU
        for (int i = 0; i < 2; i++)
        {
            if (global_debugger.imu_debugger[i].recv_msgs_num != offline_detector.imu_receive_num[i])
            {
                offline_detector.imu_state[i] = IMU_ON;
                offline_detector.imu_receive_num[i] = global_debugger.imu_debugger[i].recv_msgs_num;

                sensor_on_num++;
            }
            else
            {
                offline_detector.imu_state[i] = IMU_OFF;
            }
        }

        // 轮毂电机
        for (int i = 0; i < 2; i++)
        {
            if (global_debugger.mf9025_debugger[i].recv_msgs_num != offline_detector.MF9025_receive_num[i])
            {
                offline_detector.MF9025_state[i] = MF9025_ON;
                offline_detector.MF9025_receive_num[i] = global_debugger.mf9025_debugger[i].recv_msgs_num;

                sensor_on_num++;
            }
            else
            {
                offline_detector.MF9025_state[i] = MF9025_OFF;
            }
        }

        // 关节电机
        for (int i = 0; i < 4; i++)
        {
            if (global_debugger.a1_motor_debugger[i].recv_msgs_num != offline_detector.unitree_a1_receive_num[i])
            {
                offline_detector.unitree_motor_state[i] = UNITREE_A1_MOTOR_ON;
                offline_detector.unitree_a1_receive_num[i] = global_debugger.a1_motor_debugger[i].recv_msgs_num;

                sensor_on_num++;
            }
            else
            {
                offline_detector.unitree_motor_state[i] = UNITREE_A1_MOTOR_OFF;
            }

            if (unitree_a1_motors.a1_motor_recv[i].MError != 0x00) // 电机报错检测
            {
                offline_detector.unitree_motor_status[i] = UNITREE_A1_MOTOR_ERROR;
                knee_motor_error_num++;
            }
            else
            {
                offline_detector.unitree_motor_status[i] = UNITREE_A1_MOTOR_OK;
            }
        }

        if (knee_motor_error_num != 0) // 检测电机报错信息
        {
            offline_detector.is_motor_error = TRUE;
        }
        else
        {
            offline_detector.is_motor_error = FALSE;
        }

        // 遥控器
        if (global_debugger.remote_debugger.recv_msgs_num != offline_detector.remote_receive_num)
        {
            offline_detector.remote_state = REMOTE_ON;
            offline_detector.remote_receive_num = global_debugger.remote_debugger.recv_msgs_num;
        }
        else
        {
            offline_detector.remote_state = REMOTE_OFF;
        }

        if (sensor_on_num >= 8) // 需要大于传感器数量
        {
            offline_detector.is_sensor_off = FALSE;
        }
        else
        {
            offline_detector.is_sensor_off = TRUE;
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // 所有数据都应该超过5HZ
    }
}

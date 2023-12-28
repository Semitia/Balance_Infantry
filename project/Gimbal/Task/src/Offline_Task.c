#include "Offline_Task.h"

OfflineDetector offline_detector;

void Offline_task(void *pvParameters)
{
    vTaskDelay(5000); //待机器人初始化后开始检测

    while (1)
    {
        // IMU
        for (int i = 0; i < 2; i++)
        {
            if (global_debugger.imu_debugger[i].recv_msgs_num != offline_detector.imu_receive_num[i])
            {
                offline_detector.imu_state[i] = IMU_ON;
                offline_detector.imu_receive_num[i] = global_debugger.imu_debugger[i].recv_msgs_num;
            }
            else
            {
                offline_detector.imu_state[i] = IMU_OFF;
            }
        }

        //遥控器
        if (global_debugger.remote_debugger.recv_msgs_num != offline_detector.remote_receive_num)
        {
            offline_detector.remote_state = REMOTE_ON;
            offline_detector.remote_receive_num = global_debugger.remote_debugger.recv_msgs_num;
        }
        else
        {
            offline_detector.remote_state = REMOTE_OFF;
        }

        // pitch电机
        if (global_debugger.gimbal_debugger[0].recv_msgs_num != offline_detector.pitch_motor_receive_num)
        {
            offline_detector.pitch_motor_state = PITCH_MOTOR_ON;
            offline_detector.pitch_motor_receive_num = global_debugger.gimbal_debugger[0].recv_msgs_num;
        }
        else
        {
            offline_detector.pitch_motor_state = PITCH_MOTOR_OFF;
        }

        // YAW电机
        if (global_debugger.gimbal_debugger[1].recv_msgs_num != offline_detector.yaw_motor_receive_num)
        {
            offline_detector.yaw_motor_state = YAW_MOTOR_ON;
            offline_detector.yaw_motor_receive_num = global_debugger.gimbal_debugger[1].recv_msgs_num;
        }
        else
        {
            offline_detector.yaw_motor_state = YAW_MOTOR_OFF;
        }

        //摩擦轮电机
        for (int i = 0; i < 2; i++)
        {
            if (global_debugger.friction_debugger[i].recv_msgs_num != offline_detector.friction_motor_receive_num[i])
            {
                offline_detector.friction_motor_state[i] = FRICTION_WHEEL_MOTOR_ON;
                offline_detector.friction_motor_receive_num[i] = global_debugger.friction_debugger[i].recv_msgs_num;
            }
            else
            {
                offline_detector.friction_motor_state[i] = FRICTION_WHEELS_MOTOR_OFF;
            }
        }

        //拨盘电机
        if (global_debugger.toggle_debugger.recv_msgs_num != offline_detector.toggle_motor_receive_num)
        {
            offline_detector.toggle_motor_state = TOGGLE_MOTOR_ON;
            offline_detector.toggle_motor_receive_num = global_debugger.toggle_debugger.recv_msgs_num;
        }
        else
        {
            offline_detector.toggle_motor_state = TOGGLE_MOTOR_OFF;
        }

        if (global_debugger.pc_receive_debugger.recv_msgs_num != offline_detector.pc_receive_num)
        {
            offline_detector.pc_state = PC_ON;
            offline_detector.pc_receive_num = global_debugger.pc_receive_debugger.recv_msgs_num;
        }
        else
        {
            offline_detector.pc_state = PC_OFF;
        }

        vTaskDelay(1000); //所有数据都应该超过5HZ
    }
}

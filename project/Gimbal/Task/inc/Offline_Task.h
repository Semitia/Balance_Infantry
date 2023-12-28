#ifndef _OFFLINE_TASK_H
#define _OFFLINE_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

enum ROBOT_SENSORS_DETECT
{
    NOT_INIT, //未初始化掉线检测
    IMU_ON,   // IMU
    IMU_OFF,
    REMOTE_ON, //遥控器
    REMOTE_OFF,
    PITCH_MOTOR_ON,
    PITCH_MOTOR_OFF,
    YAW_MOTOR_ON,
    YAW_MOTOR_OFF,
    FRICTION_WHEEL_MOTOR_ON,
    FRICTION_WHEELS_MOTOR_OFF,
    TOGGLE_MOTOR_ON,
    TOGGLE_MOTOR_OFF,
    PC_ON,
    PC_OFF
};
typedef struct
{
    int16_t imu_receive_num[2];
    int16_t remote_receive_num;
    int16_t pitch_motor_receive_num;
    int16_t yaw_motor_receive_num;
    int16_t friction_motor_receive_num[2];
    int16_t toggle_motor_receive_num;
    int16_t pc_receive_num;

    enum ROBOT_SENSORS_DETECT imu_state[2];
    enum ROBOT_SENSORS_DETECT remote_state;
    enum ROBOT_SENSORS_DETECT pitch_motor_state;
    enum ROBOT_SENSORS_DETECT yaw_motor_state;
    enum ROBOT_SENSORS_DETECT friction_motor_state[2];
    enum ROBOT_SENSORS_DETECT toggle_motor_state;
    enum ROBOT_SENSORS_DETECT pc_state;
} OfflineDetector;

void Offline_task(void *pvParameters);

extern OfflineDetector offline_detector;

#endif // !_OFFLINE_TASK_H

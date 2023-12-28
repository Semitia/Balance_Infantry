#ifndef _OFFLINE_TASK_H
#define _OFFLINE_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"
#include "UniTreeA1.h"

enum ROBOT_SENSORS_DETECT
{
    NOT_INIT, // 未初始化掉线检测
    IMU_ON,   // IMU
    IMU_OFF,
    UNITREE_A1_MOTOR_ON, // 关节电机
    UNITREE_A1_MOTOR_OFF,
    MF9025_ON, // 轮毂电机
    MF9025_OFF,
    REMOTE_ON, // 遥控器
    REMOTE_OFF,
    UNITREE_A1_MOTOR_OK, // 电机是否有报错
    UNITREE_A1_MOTOR_ERROR
};
#pragma pack(1)
typedef struct
{
    int16_t imu_receive_num[2];
    int16_t unitree_a1_receive_num[4];
    int16_t MF9025_receive_num[2];
    int16_t remote_receive_num;

    enum ROBOT_SENSORS_DETECT imu_state[2];
    enum ROBOT_SENSORS_DETECT unitree_motor_state[4];
    enum ROBOT_SENSORS_DETECT MF9025_state[2];
    enum ROBOT_SENSORS_DETECT remote_state;
    enum ROBOT_SENSORS_DETECT unitree_motor_status[4]; // 判断电机是否有保护

    int8_t is_sensor_off;  // 是否有关键传感器掉线
    int8_t is_motor_error; // 电机报错检测
} OfflineDetector;
#pragma pack()

void Offline_task(void *pvParameters);

extern OfflineDetector offline_detector;

#endif // !_OFFLINE_TASK_H

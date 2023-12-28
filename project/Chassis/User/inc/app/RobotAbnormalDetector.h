#ifndef _ROBOT_ABNORMAL_DETECTOR_H
#define _ROBOT_ABNORMAL_DETECTOR_H

#include <stm32f4xx.h>
#include <stm32f4xx_conf.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "my_filter.h"
#include "user_lib.h"
#include "TD.h"

#define MF9025_J 0.003784f
#define MF9025_B 0.003185f

typedef enum WHEELS_STATE
{
    MF9025_NORMAL, // 电机正常
    MF9025_LOCKED, // 电机堵转
    MF9025_Idle    // 电机空载
} WHEELS_STATE;

typedef struct RobotAbnormalDetector
{
    /* 平衡步兵轮子堵转检测 */
    float filte_wheel_speed[2]; // 滤波后的轮子速度 弧度/s^2
    float filte_wheel_accel[2]; // 低通滤波后的轮子加速度 弧度/s^2
    float last_filte_wheel_speed[2];
    float filte_wheel_current[2];          // 轮子反馈电流滤波 N.m
    float estimate_wheel_load[2];          // 两个电机的估算负载 N.m * 1000
    Ordinary_Least_Squares_t speed_derive; //速度微分器
    TD_t speed_td_1;
    TD_t speed_td_2;
    enum WHEELS_STATE mf9025_motors_state[2]; // 电机状态

} RobotAbnormalDetector;

void MF9025DetectInit(void);
WHEELS_STATE MF9025StateDetect(float motor_speed_1, float motor_speed_2, float motor_current_1, float motor_current_2, float delta_t);

extern RobotAbnormalDetector robot_abnormal_detector;

#endif // !_ROBOT_ABNORMAL_DETECTOR_H

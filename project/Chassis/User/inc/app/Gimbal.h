#ifndef _GIMBAL_H
#define _GIMBAL_H

#include "pid.h"
#include "GM6020.h"
#include "GimbalReceive.h"
#include "counter.h"
#include "my_filter.h"

#include "TD.h"

//作为云台控制的yaw角度需要以逆时针为正(角度增加)
#define GIMBAL_SIGN -1 //用来标记yaw轴陀螺仪角和电机角方向，方向相反为负

//摩擦力模型调参
#define BORDER_FRICTION_SPEED 4.0f    //临界计算摩擦力速度，大于此速度将是全摩擦力补偿
#define FRICTION_CURRENT_COMP 2800.0f //辨识所得到的摩擦力电流发送值
#define FRICTION_FORWARD_COEF 0.9f    //前馈补偿系数

typedef struct GimbalController
{
    // Pitch轴在云台控制
    //  Pitch 轴
    //  PID_t pitch_current_pid;           //电流环
    //  PID_t pitch_speed_pid;             //速度环
    //  PID_t pitch_angle_pid;             //角度环
    //  Feedforward_t pitch_speed_forward; //速度环前馈
    //  Feedforward_t pitch_angle_forward; //角度环前馈

    // Yaw在底盘控制
    // // Yaw 轴
    PID_t yaw_current_pid;           //电流环
    PID_t yaw_speed_pid;             //速度环
    PID_t yaw_angle_pid;             //角度环
    Feedforward_t yaw_speed_forward; //速度环前馈
    Feedforward_t yaw_angle_forward; //角度环前馈
    GM6020_Recv yaw_recv;
    GM6020_Info yaw_info; //电机信息

    float set_speed;
    float set_current;
    float set_angle;
    float set_vol;

    //陀螺仪信息及其解算
    float gyro_yaw_speed;
    float gyro_yaw_angle;
    float gyro_last_yaw_angle;
    uint32_t last_cnt;
    float delta_t; //两帧计算之间的时间差

    TD_t pos_td; //位置跟踪微分器
} GimbalController;

extern GimbalController gimbal_controller;

void GimbalPidInit(void);
float Gimbal_Calculate(float set_point);
float Gimbal_Speed_Calculate(float set_point);
void GimbalClear(void);
void UpdateGyroYaw(void);

float GimbalFrictionModel(void);

#endif // !_GIMBAL_H

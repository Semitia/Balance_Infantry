#ifndef _GIMBAL_H
#define _GIMBAL_H

#include "pid.h"
#include "GM6020.h"
#include "ins.h"

#include "my_filter.h"
#include "TD.h"

// pitch
#define GIMBAL_PITCH_GYRO_SIGN 1.0f // pitch符号，向上为正
#define GIMBAL_PITCH_BIAS 0.0f      // pitch最低角度(imu测得) - 实际最低角度(机械处测得)

#define GIMBAL_PITCH_MOTOR_SIGN -1.0f // 云台PITCH电机方向，向上为正

#define GIMBAL_ANGLE_MIN -220.0f // 电机角软限位
#define GIMBAL_ANGLE_MAX -160.0f

#define GIMBAL_PITCH_COMP 4000.0f
#define GIMBAL_PITCH_COMP_COEF 1.0f

// yaw
// 作为云台控制的yaw角度需要以逆时针为正(角度增加)
#define GIMBAL_YAW_MOTOR_SIGN 1.0f       // 用来标记电机的方向，逆时针为正
#define GIMBAL_YAW_GYRO_SIGN 1.0f        // 用来标记gyro的方向，逆时针为正
#define GIMBAL_YAW_POS_FORWARD_COEF 0.6f // 角度环前馈系数
#define GIMBAL_YAW_SPEED_FORWARD_COEF 0.f
#define GIMBAL_YAW_J 4.15f
#define GIMBAL_YAW_B 18.54f

// 摩擦力模型调参
#define BORDER_FRICTION_SPEED 6.0f    // 临界计算摩擦力速度，大于此速度将是全摩擦力补偿
#define FRICTION_CURRENT_COMP 1500.0f // 辨识所得到的摩擦力电流发送值
#define FRICTION_FORWARD_COEF 0.0f    // 前馈补偿系数

typedef struct GimbalController
{
    // Pitch 轴
    PID_t pitch_current_pid;           // 电流环
    PID_t pitch_speed_pid;             // 速度环
    PID_t pitch_angle_pid;             // 角度环
    Feedforward_t pitch_speed_forward; // 速度环前馈
    Feedforward_t pitch_angle_forward; // 角度环前馈

    GM6020_Recv pitch_recv;
    GM6020_Info pitch_info;

    float set_pitch_speed;
    float set_pitch_current;
    float set_pitch_angle;
    float set_pitch_vol;
    float comp_pitch_current; // 重力补偿

    // 陀螺仪信息及其解算
    float gyro_pitch_speed;
    float gyro_pitch_angle;
    float gyro_last_pitch_angle;
    uint32_t last_cnt;
    float delta_t; // 两帧计算之间的时间差

    float target_pitch_angle; // 设定的角度值

    // Yaw在底盘控制
    // // Yaw 轴
    PID_t yaw_current_pid;           // 电流环
    PID_t yaw_speed_pid;             // 速度环
    PID_t yaw_angle_pid;             // 角度环
    Feedforward_t yaw_speed_forward; // 速度环前馈
    Feedforward_t yaw_angle_forward; // 角度环前馈
    GM6020_Recv yaw_recv;
    GM6020_Info yaw_info; // 电机信息

    float set_yaw_speed;
    float set_yaw_current;
    float set_yaw_angle;
    float set_yaw_vol;

    // 陀螺仪信息及其解算
    float gyro_yaw_speed;
    float gyro_yaw_angle;
    float gyro_last_yaw_angle;

    float target_yaw_angle;

    TD_t pos_yaw_td; // 位置跟踪微分器
    TD_t speed_yaw_td;

    // pitch 限位计算
    float pitch_max_gyro_angle;
    float pitch_min_gyro_angle;
} GimbalController;

extern GimbalController gimbal_controller;

void GimbalPidInit(void);
void GimbalClear(void);
void updateGyro(void);

// pitch
void limitPitchAngle(void);
float GimbalPitchComp(void);
float Gimbal_Pitch_Calculate(float set_point);

// Yaw
float Gimbal_Yaw_Calculate(float set_point);
float Gimbal_Speed_Calculate(float set_point);
float GimbalFrictionModel(void);

#endif // !_GIMBAL_H

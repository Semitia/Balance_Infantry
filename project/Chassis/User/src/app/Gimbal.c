#include "Gimbal.h"

GimbalController gimbal_controller;

/**
 * @brief 云台PID初始化(仅yaw值)
 * @param[in] void
 */
void GimbalPidInit()
{
    float c_angle[3] = {0, 0, 0};
    float c_speed[3] = {0, 0, 0};
    PID_Init(&gimbal_controller.yaw_angle_pid, 250.0, 0, 0, 17.5f, 0, 0, 0, 0, 0, 0, 1, NONE);
    PID_Init(&gimbal_controller.yaw_speed_pid, GM6020_MAX_CURRENT, 9000, 0.0, 350.0f, 200.0f, 0, 0, 0, 0, 0.003, 1, Integral_Limit | Trapezoid_Intergral | OutputFilter);
    PID_Init(&gimbal_controller.yaw_current_pid, GM6020_MAX_VOLTAGE, 12000, 0, 1.6f, 13.0f, 0, 2000, 2000, 0.006, 0, 1, Trapezoid_Intergral | OutputFilter | Integral_Limit);

    Feedforward_Init(&gimbal_controller.yaw_angle_forward, 0, c_angle, 0, 1, 1);
    Feedforward_Init(&gimbal_controller.yaw_speed_forward, 0, c_speed, 0, 1, 1);

    //跟踪微分器
    TD_Init(&gimbal_controller.pos_td, 9000, 0.01);
}

/**
 * @brief 云台控制
 * @param[in] set_point 角度值设定 度
 */
float Gimbal_Calculate(float set_point)
{
    gimbal_controller.set_angle = TD_Calculate(&gimbal_controller.pos_td, set_point);
    gimbal_controller.set_speed = PID_Calculate(&gimbal_controller.yaw_angle_pid, gimbal_controller.gyro_yaw_angle, set_point);
    gimbal_controller.set_current = GIMBAL_SIGN * (PID_Calculate(&gimbal_controller.yaw_speed_pid, gimbal_controller.gyro_yaw_speed, gimbal_controller.set_speed) + GimbalFrictionModel());
    gimbal_controller.set_vol = PID_Calculate(&gimbal_controller.yaw_current_pid, gimbal_controller.yaw_info.torque_current, gimbal_controller.set_current);
    return gimbal_controller.set_vol;
}

/**
 * @brief 云台控制(速度控制)
 * @param[in] set_point 角度值设定 度/s
 */
float Gimbal_Speed_Calculate(float set_point)
{
    gimbal_controller.set_angle = gimbal_controller.gyro_yaw_angle;
    PID_Clear(&gimbal_controller.yaw_angle_pid);
    gimbal_controller.set_speed = set_point;
    gimbal_controller.set_current = GIMBAL_SIGN * PID_Calculate(&gimbal_controller.yaw_speed_pid, gimbal_controller.gyro_yaw_speed, gimbal_controller.set_speed);
    gimbal_controller.set_vol = PID_Calculate(&gimbal_controller.yaw_current_pid, gimbal_controller.yaw_info.torque_current, gimbal_controller.set_current);
    return gimbal_controller.set_vol;
}

void GimbalClear(void)
{
    PID_Clear(&gimbal_controller.yaw_angle_pid);
    PID_Clear(&gimbal_controller.yaw_speed_pid);
    PID_Clear(&gimbal_controller.yaw_current_pid);

    Feedforward_Clear(&gimbal_controller.yaw_speed_forward);
    Feedforward_Clear(&gimbal_controller.yaw_angle_forward);

    TD_Clear(&gimbal_controller.pos_td, gimbal_controller.gyro_yaw_angle);

    gimbal_controller.set_angle = gimbal_controller.gyro_yaw_angle;
    gimbal_controller.set_speed = 0;
    gimbal_controller.set_current = 0;
    gimbal_controller.set_vol = 0;
}

void UpdateGyroYaw()
{
//    gimbal_controller.delta_t = GetDeltaT(&gimbal_controller.last_cnt);
//    gimbal_controller.gyro_yaw_angle = gimbal_receiver_pack2.yaw;
//    float speed = (gimbal_controller.gyro_yaw_angle - gimbal_controller.gyro_last_yaw_angle) / gimbal_controller.delta_t;

//    iir(&gimbal_controller.gyro_yaw_speed, speed, 0.90);
//    gimbal_controller.gyro_last_yaw_angle = gimbal_controller.gyro_yaw_angle;
}

/**
 * @brief 云台摩擦力模型，只使用库伦摩擦力，因粘性摩擦力在辨识中表现不明显，故忽略
 * @param[in] void
 */
float GimbalFrictionModel()
{
    //根据转速判断符号
    if (fabsf(gimbal_controller.gyro_yaw_speed) < BORDER_FRICTION_SPEED)
    {
        //部分补偿
        return gimbal_controller.gyro_yaw_speed / BORDER_FRICTION_SPEED * FRICTION_CURRENT_COMP * FRICTION_FORWARD_COEF;
    }
    //全补偿
    return FRICTION_CURRENT_COMP * FRICTION_FORWARD_COEF * sign(gimbal_controller.gyro_yaw_speed);
}

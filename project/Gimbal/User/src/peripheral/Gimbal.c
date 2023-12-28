#include "Gimbal.h"

GimbalController gimbal_controller;

/**
 * @brief 云台PID初始化(仅Pitch值)
 * @param[in] void
 */
void GimbalPidInit()
{
    // pitch
    // float c_pitch_angle[3] = {0, 0, 0};
    // float c_pitch_speed[3] = {0, 0, 0};
    PID_Init(&gimbal_controller.pitch_angle_pid, 280.0f, 0, .0f, 24.0f, 0, 0, 0, 0, 0, 0, 1, NONE);
    PID_Init(&gimbal_controller.pitch_speed_pid, GM6020_MAX_CURRENT, 11000, .0f, 360.0f, 120.0f, 0, 0, 0, 0.0, 0, 1, Integral_Limit | Trapezoid_Intergral | OutputFilter);
    PID_Init(&gimbal_controller.pitch_current_pid, GM6020_MAX_VOLTAGE, 18000, 0, 1.6f, 15.5f, 0, 0, 0, 0.006, 0, 1, Trapezoid_Intergral | OutputFilter | Integral_Limit);

    // Feedforward_Init(&gimbal_controller.pitch_angle_forward, 0, c_pitch_angle, 0, 1, 1);
    // Feedforward_Init(&gimbal_controller.pitch_speed_forward, 0, c_pitch_speed, 0, 1, 1);

    // yaw
    // float c_yaw_angle[3] = {0, 0, 0};
    // float c_yaw_speed[3] = {0, 0, 0};
    PID_Init(&gimbal_controller.yaw_angle_pid, 280.0, 0, 0, 20.0f, 0, 0, 0, 0, 0.0, 0, 1, NONE);
    PID_Init(&gimbal_controller.yaw_speed_pid, GM6020_MAX_CURRENT, 9000, 0.0, 360.0f, 15.0f, 0, 0, 0, 0.0018, 0, 1, Integral_Limit | Trapezoid_Intergral | OutputFilter);
    PID_Init(&gimbal_controller.yaw_current_pid, GM6020_MAX_VOLTAGE, 12000, 0, 2.0f, 15.5f, 0, 0, 0, 0.0030, 0, 1, Trapezoid_Intergral | OutputFilter | Integral_Limit);

    // Feedforward_Init(&gimbal_controller.yaw_angle_forward, 0, c_yaw_angle, 0, 1, 1);
    // Feedforward_Init(&gimbal_controller.yaw_speed_forward, 0, c_yaw_speed, 0, 1, 1);

    // 双环
    //  PID_Init(&gimbal_controller.pitch_angle_pid, 180.0f, 0, 0.f, 20.0f, 0, 0, 0, 0, 0, 0, 1, NONE);
    //  PID_Init(&gimbal_controller.pitch_speed_pid, GM6020_MAX_VOLTAGE, 11000, .0f, 350.0f, 1000.0f, 0, 0, 0, 0.002, 0, 1, Integral_Limit | Trapezoid_Intergral | OutputFilter);

    // PID_Init(&gimbal_controller.yaw_angle_pid, 200.0f, 0, .0f, 35.0f, 0, 0.0f, 0, 0, 0.00, 0.00, 1, NONE);
    // PID_Init(&gimbal_controller.yaw_speed_pid, GM6020_MAX_VOLTAGE, 11000, 0.f, 400.0f, 800.0f, 0, 0, 0, 0.005, 0, 1, Integral_Limit | Trapezoid_Intergral | OutputFilter);

    // 跟踪微分器
    TD_Init(&gimbal_controller.pos_yaw_td, 9000, 0.01);
    TD_Init(&gimbal_controller.speed_yaw_td, 90000, 0.01);
}

/**
 * @brief 云台控制
 * @param[in] set_point 角度值设定 度
 */
float Gimbal_Pitch_Calculate(float set_point)
{
    // pitch 三环
    gimbal_controller.set_pitch_angle = set_point;
    gimbal_controller.set_pitch_speed = PID_Calculate(&gimbal_controller.pitch_angle_pid, gimbal_controller.gyro_pitch_angle, set_point);
    gimbal_controller.set_pitch_current = GIMBAL_PITCH_MOTOR_SIGN * PID_Calculate(&gimbal_controller.pitch_speed_pid, gimbal_controller.gyro_pitch_speed, gimbal_controller.set_pitch_speed);
    gimbal_controller.set_pitch_vol = PID_Calculate(&gimbal_controller.pitch_current_pid, gimbal_controller.pitch_info.torque_current, gimbal_controller.set_pitch_current);
    return gimbal_controller.set_pitch_vol;

    // gimbal_controller.set_pitch_angle = set_point;
    // gimbal_controller.set_pitch_speed = PID_Calculate(&gimbal_controller.pitch_angle_pid, gimbal_controller.gyro_pitch_angle, set_point);
    // gimbal_controller.set_pitch_vol = GIMBAL_PITCH_MOTOR_SIGN * PID_Calculate(&gimbal_controller.pitch_speed_pid, gimbal_controller.gyro_pitch_speed, gimbal_controller.set_pitch_speed);
    // return gimbal_controller.set_pitch_vol;
}

float Gimbal_Yaw_Calculate(float set_point)
{
    gimbal_controller.set_yaw_angle = TD_Calculate(&gimbal_controller.pos_yaw_td, set_point);
    gimbal_controller.set_yaw_speed = PID_Calculate(&gimbal_controller.yaw_angle_pid, gimbal_controller.gyro_yaw_angle, set_point) + GIMBAL_YAW_POS_FORWARD_COEF * gimbal_controller.pos_yaw_td.dx;
    TD_Calculate(&gimbal_controller.speed_yaw_td, gimbal_controller.set_yaw_speed);
    gimbal_controller.set_yaw_current = GIMBAL_YAW_MOTOR_SIGN * (PID_Calculate(&gimbal_controller.yaw_speed_pid, gimbal_controller.gyro_yaw_speed, gimbal_controller.set_yaw_speed) + GimbalFrictionModel() + GIMBAL_YAW_SPEED_FORWARD_COEF * (GIMBAL_YAW_J * gimbal_controller.speed_yaw_td.dx + GIMBAL_YAW_B * gimbal_controller.speed_yaw_td.x));
    gimbal_controller.set_yaw_vol = PID_Calculate(&gimbal_controller.yaw_current_pid, gimbal_controller.yaw_info.torque_current, gimbal_controller.set_yaw_current);
    return gimbal_controller.set_yaw_vol;

    //     gimbal_controller.set_yaw_angle = set_point;
    //     gimbal_controller.set_yaw_speed = PID_Calculate(&gimbal_controller.yaw_angle_pid, gimbal_controller.gyro_yaw_angle, set_point);
    //     gimbal_controller.set_yaw_vol = GIMBAL_YAW_MOTOR_SIGN * (PID_Calculate(&gimbal_controller.yaw_speed_pid, gimbal_controller.gyro_yaw_speed, gimbal_controller.set_yaw_speed));
    //     return gimbal_controller.set_yaw_vol;
}

void GimbalClear(void)
{
    PID_Clear(&gimbal_controller.pitch_angle_pid);
    PID_Clear(&gimbal_controller.pitch_speed_pid);
    PID_Clear(&gimbal_controller.pitch_current_pid);

    Feedforward_Clear(&gimbal_controller.pitch_speed_forward);
    Feedforward_Clear(&gimbal_controller.pitch_angle_forward);

    gimbal_controller.target_pitch_angle = gimbal_controller.gyro_pitch_angle;
    gimbal_controller.set_pitch_angle = gimbal_controller.gyro_pitch_angle;
    gimbal_controller.set_pitch_speed = 0;
    gimbal_controller.set_pitch_current = 0;
    gimbal_controller.set_pitch_vol = 0;
    gimbal_controller.comp_pitch_current = 0;

    // yaw
    PID_Clear(&gimbal_controller.yaw_angle_pid);
    PID_Clear(&gimbal_controller.yaw_speed_pid);
    PID_Clear(&gimbal_controller.yaw_current_pid);

    Feedforward_Clear(&gimbal_controller.yaw_speed_forward);
    Feedforward_Clear(&gimbal_controller.yaw_angle_forward);

    TD_Clear(&gimbal_controller.pos_yaw_td, gimbal_controller.gyro_yaw_angle);

    gimbal_controller.target_yaw_angle = gimbal_controller.gyro_yaw_angle;
    gimbal_controller.set_yaw_angle = gimbal_controller.gyro_yaw_angle;
    gimbal_controller.set_yaw_speed = 0;
    gimbal_controller.set_yaw_current = 0;
    gimbal_controller.set_yaw_vol = 0;
}

/**
 * @brief 云台控制(速度控制)
 * @param[in] set_point 角度值设定 度/s
 */
float Gimbal_Speed_Calculate(float set_point)
{
    gimbal_controller.set_yaw_angle = gimbal_controller.gyro_yaw_angle;
    PID_Clear(&gimbal_controller.yaw_angle_pid);
    gimbal_controller.set_yaw_speed = set_point;
    TD_Calculate(&gimbal_controller.speed_yaw_td, gimbal_controller.set_yaw_speed);
    gimbal_controller.set_yaw_current = GIMBAL_YAW_MOTOR_SIGN * (PID_Calculate(&gimbal_controller.yaw_speed_pid, gimbal_controller.gyro_yaw_speed, gimbal_controller.set_yaw_speed) + GimbalFrictionModel());
    gimbal_controller.set_yaw_vol = PID_Calculate(&gimbal_controller.yaw_current_pid, gimbal_controller.yaw_info.torque_current, gimbal_controller.set_yaw_current);
    return gimbal_controller.set_yaw_vol;
}

/**
 * @brief 限制设置的pitch角度大小
 */
void limitPitchAngle()
{
    float cur_motor_angle = GIMBAL_PITCH_MOTOR_SIGN * gimbal_controller.pitch_info.angle;
    float cur_gyro_angle = gimbal_controller.gyro_pitch_angle;
    gimbal_controller.pitch_max_gyro_angle = cur_gyro_angle + GIMBAL_ANGLE_MAX - cur_motor_angle;
    gimbal_controller.pitch_min_gyro_angle = cur_gyro_angle + GIMBAL_ANGLE_MIN - cur_motor_angle;
    gimbal_controller.target_pitch_angle = LIMIT_MAX_MIN(gimbal_controller.target_pitch_angle, gimbal_controller.pitch_max_gyro_angle, gimbal_controller.pitch_min_gyro_angle);
}

/**
 * @brief 更新pitch角速度，以及角度(注意需要标定零点)
 */
void updateGyro()
{
    // 注意陀螺仪安装的Pitch和roll轴方向
    gimbal_controller.delta_t = GetDeltaT(&gimbal_controller.last_cnt);
    gimbal_controller.gyro_pitch_angle = GIMBAL_PITCH_GYRO_SIGN * (INS.Roll - GIMBAL_PITCH_BIAS);
    float speed = (gimbal_controller.gyro_pitch_angle - gimbal_controller.gyro_last_pitch_angle) / gimbal_controller.delta_t;

    iir(&gimbal_controller.gyro_pitch_speed, speed, 0.5);
    gimbal_controller.gyro_last_pitch_angle = gimbal_controller.gyro_pitch_angle;

    // yaw
    gimbal_controller.gyro_yaw_angle = GIMBAL_YAW_GYRO_SIGN * INS.YawTotalAngle;
    speed = (gimbal_controller.gyro_yaw_angle - gimbal_controller.gyro_last_yaw_angle) / gimbal_controller.delta_t;

    iir(&gimbal_controller.gyro_yaw_speed, speed, 0.4);
    gimbal_controller.gyro_last_yaw_angle = gimbal_controller.gyro_yaw_angle;
}

/**
 * @brief 由于重力补偿的作用，云台需要施加一个非线性力抵消重力影响，该力需要根据实际来进行测定
 */
float GimbalPitchComp()
{
    // //记得每调一台车都需要重新更新参数
    // const static float pitch_comp[5] = {0.1399, -0.9144, -10.2, 9.038, -3337};
    // float x[4];

    // //解析静止时的非线性函数，只能大致补偿，然后靠PID的I使最终无静差
    // //低于一定角度或高于一定角度，根据测量结果，输出应大致不变
    // x[3] = LIMIT_MAX_MIN(gimbal_controller.gyro_pitch_angle, 8, -12);
    // x[2] = x[3] * x[3];
    // x[1] = x[2] * x[3];
    // x[0] = x[1] * x[3];

    // float sum = pitch_comp[4];
    // for (int i = 0; i < 4; i++)
    // {
    //     sum += x[i] * pitch_comp[i];
    // }
    // iir(&gimbal_controller.comp_pitch_current, sum * GIMBAL_PITCH_COMP_COEF, 0.7);
    // return gimbal_controller.comp_pitch_current;
    iir(&gimbal_controller.comp_pitch_current, GIMBAL_PITCH_COMP * arm_cos_f32(gimbal_controller.gyro_pitch_angle * ANGLE_TO_RAD_COEF) * GIMBAL_PITCH_COMP_COEF, 0.7);
    return gimbal_controller.comp_pitch_current;
}

/**
 * @brief 云台摩擦力模型，只使用库伦摩擦力，因粘性摩擦力在辨识中表现不明显，故忽略
 * @param[in] void
 */
float GimbalFrictionModel()
{
    // 根据转速判断符号
    if (fabsf(gimbal_controller.gyro_yaw_speed) < BORDER_FRICTION_SPEED)
    {
        // 部分补偿
        return gimbal_controller.gyro_yaw_speed / BORDER_FRICTION_SPEED * FRICTION_CURRENT_COMP * FRICTION_FORWARD_COEF;
    }
    // 全补偿
    return FRICTION_CURRENT_COMP * FRICTION_FORWARD_COEF * sign(gimbal_controller.gyro_yaw_speed);
}

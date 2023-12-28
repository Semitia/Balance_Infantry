#include "ChasisController.h"

BalanceInfantry balance_infantry;

void YawInit(BalanceInfantry *balance_infantry)
{
    balance_infantry->target_yaw = balance_infantry->INS->YawTotalAngle * PI / 180.0f;
}

/**
 * @brief 更新一帧传感器信息
 * @param[in] sensors_info
 */
void get_sensor_info(BalanceInfantry *balance_infantry)
{
    MF9025_Decode(&Wheels_Motor.decoded_msgs[LEFT_WHEEL_ID], &Wheels_Motor.recv_msgs[LEFT_WHEEL_ID]);
    MF9025_Decode(&Wheels_Motor.decoded_msgs[RIGHT_WHEEL_ID], &Wheels_Motor.recv_msgs[RIGHT_WHEEL_ID]);

    // IMU
    balance_infantry->sensors_info.pitch = SIGN_PITCH * (balance_infantry->INS->Pitch - MECHANICAL_PITCH) * ANGLE_TO_RAD_COEF;
    balance_infantry->sensors_info.yaw = SIGN_YAW * (balance_infantry->INS->YawTotalAngle) * ANGLE_TO_RAD_COEF;
    balance_infantry->sensors_info.roll = SIGN_ROLL * (balance_infantry->INS->Roll - MECHANICAL_ROLL) * ANGLE_TO_RAD_COEF;

    // 关节电机，角度转化(即处理减速比)
    balance_infantry->sensors_info.knee_angle[LEFT_KNEE_LEFT] =
        (balance_infantry->unitree_a1_motors->a1_motor_recv[LEFT_KNEE_LEFT].Pos - balance_infantry->unitree_a1_motors->init_pos[LEFT_KNEE_LEFT]) / REDUCTION_RATIO - FORBIDDEN_ANGLE;
    balance_infantry->sensors_info.knee_angle[LEFT_KNEE_RIGHT] =
        (balance_infantry->unitree_a1_motors->a1_motor_recv[LEFT_KNEE_RIGHT].Pos - balance_infantry->unitree_a1_motors->init_pos[LEFT_KNEE_RIGHT]) / REDUCTION_RATIO + FORBIDDEN_ANGLE + PI;
    balance_infantry->sensors_info.knee_angle[RIGHT_KNEE_LEFT] =
        -(balance_infantry->unitree_a1_motors->a1_motor_recv[RIGHT_KNEE_LEFT].Pos - balance_infantry->unitree_a1_motors->init_pos[RIGHT_KNEE_LEFT]) / REDUCTION_RATIO + FORBIDDEN_ANGLE + PI;
    balance_infantry->sensors_info.knee_angle[RIGHT_KNEE_RIGHT] =
        -(balance_infantry->unitree_a1_motors->a1_motor_recv[RIGHT_KNEE_RIGHT].Pos - balance_infantry->unitree_a1_motors->init_pos[RIGHT_KNEE_RIGHT]) / REDUCTION_RATIO - FORBIDDEN_ANGLE;

    // 轮毂电机(角度连续化)
    balance_infantry->sensors_info.wheel_pos[LEFT_WHEEL_ID] =
        balance_infantry->Wheels_Motor->decoded_msgs[LEFT_WHEEL_ID].pos * ANGLE_TO_RAD_COEF;
    balance_infantry->sensors_info.wheel_pos[RIGHT_WHEEL_ID] =
        balance_infantry->Wheels_Motor->decoded_msgs[RIGHT_WHEEL_ID].pos * ANGLE_TO_RAD_COEF;

    balance_infantry->sensors_info.wheel_speed[LEFT_WHEEL_ID] =
        balance_infantry->Wheels_Motor->decoded_msgs[LEFT_WHEEL_ID].rotate_speed * ANGLE_TO_RAD_COEF;
    balance_infantry->sensors_info.wheel_speed[RIGHT_WHEEL_ID] =
        balance_infantry->Wheels_Motor->decoded_msgs[RIGHT_WHEEL_ID].rotate_speed * ANGLE_TO_RAD_COEF;

    for (int i = 0; i < 4; i++)
    {
        balance_infantry->sensors_info.torque_feed_back[i] = balance_infantry->unitree_a1_motors->a1_motor_recv[i].T * REDUCTION_RATIO;
    }

    // 轮毂电机电流
    balance_infantry->sensors_info.wheel_torque[LEFT_WHEEL_ID] = balance_infantry->Wheels_Motor->decoded_msgs[LEFT_WHEEL_ID].current * MF9025_TORQUE_CURRENT_RATIO;
    balance_infantry->sensors_info.wheel_torque[RIGHT_WHEEL_ID] = balance_infantry->Wheels_Motor->decoded_msgs[RIGHT_WHEEL_ID].current * MF9025_TORQUE_CURRENT_RATIO;
}

void blueToothStateUpdate(BalanceInfantry *balance_infantry)
{
    // switch (remote_controller.blue_tooth_key)
    // {
    // case BLUE_TOOTH_UP:
    //     balance_infantry->target_v = 0.5f;
    //     break;
    // case BLUE_TOOTH_DOWN:
    //     balance_infantry->target_v = -0.5f;
    //     break;
    // case BLUE_TOOTH_STABLE:
    //     balance_infantry->target_v = 0;
    //     balance_infantry->target_yaw_v = 0;
    //     break;
    // case BLUE_TOOTH_LEFT:
    //     balance_infantry->target_yaw_v += 0.5f;
    //     break;
    // case BLUE_TOOTH_RIGHT:
    //     balance_infantry->target_yaw_v -= 0.5f;
    //     break;
    // default:
    //     break;
    // }
    // remote_controller.blue_tooth_key = BLUE_TOOTH_NO_ACTION;
}

// void DJIRemoteUpdate(BalanceInfantry *balance_infantry)
// {
//     //判断状态
//     if (offline_detector.remote_state == REMOTE_OFF || remote_controller.dji_remote.rc.s[RIGHT_SW] == Down)
//     {
//         setRobotState(OFFLINE_MODE);
//         balance_infantry->target_v = 0;
//         balance_infantry->target_yaw_v = 0;
//     }
//     //右拨杆
//     else if (remote_controller.dji_remote.rc.s[RIGHT_SW] == Up)
//     {
//         //小陀螺
//         setRobotState(CONTROL_MODE);
//         setControlModeAction(SELF_ROTATE);
//         balance_infantry->target_v = 0;
//         balance_infantry->target_yaw_v = SPEED_W_MAX;
//     }
//     else if (remote_controller.dji_remote.rc.s[RIGHT_SW] == Mid && remote_controller.dji_remote.rc.s[LEFT_SW] == Mid)
//     {
//         setRobotState(CONTROL_MODE);
//         setControlModeAction(BALANCE);
//         balance_infantry->target_v = (remote_controller.dji_remote.rc.ch[RIGHT_CH_UD] - CH_MIDDLE) * SPEED_MAX / CH_RANGE;
//         // balance_infantry->target_yaw_v = -(remote_controller.dji_remote.rc.ch[RIGHT_CH_LR] - CH_MIDDLE) * SPEED_W_MAX / CH_RANGE;
//         balance_infantry->leg_speed = (remote_controller.dji_remote.rc.ch[LEFT_CH_UD] - CH_MIDDLE) * MAX_LEG_SPEED / CH_RANGE;
//     }
//     else if (remote_controller.dji_remote.rc.s[RIGHT_SW] == Mid && remote_controller.dji_remote.rc.s[LEFT_SW] == Down)
//     {
//         setRobotState(CONTROL_MODE);
//         setControlModeAction(RECOVER);
//         balance_infantry->target_v = (remote_controller.dji_remote.rc.ch[RIGHT_CH_UD] - CH_MIDDLE) * SPEED_MAX / CH_RANGE;
//         // balance_infantry->target_yaw_v = -(remote_controller.dji_remote.rc.ch[RIGHT_CH_LR] - CH_MIDDLE) * SPEED_W_MAX / CH_RANGE;
//         balance_infantry->leg_speed = 0;
//     }
// }

// /**
//  * @brief 根据遥控器或者蓝牙更新控制状态
//  * @param[in] balance_infantry
//  */
// void get_control_info(BalanceInfantry *balance_infantry)
// {
//     switch (remote_controller.control_type)
//     {
//     case BLUE_TOOTH:
//         blueToothStateUpdate(balance_infantry);
//         break;
//     case DJI_REMOTE_CONTROL:
//         DJIRemoteUpdate(balance_infantry);
//         break;
//     default:
//         break;
//     }
// }

/**
 * @brief  转角限制到±180度
 * @param  输入转角
 * @retval 输出转角
 */
float limit_pi(float in)
{
    while (in < -180.0f || in > 180.0f)
    {
        if (in < -180.0f)
            in = in + 360.0f;
        if (in > 180.0f)
            in = in - 360.0f;
    }
    return in;
}

/**
 * @brief 侧滑角计算
 * @param[in] balance_infantry
 */
void slid_angle_solve(BalanceInfantry *balance_infantry)
{
    float yaw_line_v = ((balance_infantry->sensors_info.wheel_speed[LEFT_WHEEL_ID] +
                         balance_infantry->sensors_info.wheel_speed[RIGHT_WHEEL_ID]) *
                        WHEEL_RADIUS / 2) *
                           0.1f +
                       balance_infantry->yaw_v * 0.21f / 2 * 0.9f;
    float temp_v = Sqrt(yaw_line_v * yaw_line_v + balance_infantry->state_vector[STATE_X_V] * balance_infantry->state_vector[STATE_X_V]);
    float temp_target_v = Sqrt(balance_infantry->target_v * balance_infantry->target_v + balance_infantry->set_yaw_v * balance_infantry->set_yaw_v * 0.011f);

    if ((yaw_line_v < 0.3f && balance_infantry->state_vector[STATE_X_V] < 0.3f) || fabsf(temp_v * temp_target_v) < 1e-4f) // 值很小，可以认为运动正常
    {
        balance_infantry->slid_angle = 0.0f;
    }
    else
    {
        float v = yaw_line_v * balance_infantry->set_yaw_v + balance_infantry->target_v * balance_infantry->state_vector[STATE_X_V];
        balance_infantry->slid_angle = acosf(v / (temp_v * temp_target_v));
    }

    // 测试
    //  float yaw_line_v = balance_infantry->yaw_v * 0.21f / 2; //车子半径为10.5cm
    //  float yaw_line_v = ((balance_infantry->sensors_info.wheel_speed[LEFT_WHEEL_ID] +
    //                       balance_infantry->sensors_info.wheel_speed[RIGHT_WHEEL_ID]) *
    //                      WHEEL_RADIUS / 2) *
    //                         1 +
    //                     balance_infantry->yaw_v * 0.21f / 2 * 0;
    //  float temp_v = Sqrt(yaw_line_v * yaw_line_v + balance_infantry->state_vector[STATE_X_V] * balance_infantry->state_vector[STATE_X_V]);
    //  float temp_target_v = Sqrt(1 * 1 + balance_infantry->set_yaw_v * balance_infantry->set_yaw_v * 0.011f);

    // if ((yaw_line_v < 0.3f && balance_infantry->state_vector[STATE_X_V] < 0.3f) || fabsf(temp_v * temp_target_v) < 1e-4f) //值很小，可以认为运动正常
    // {
    //     balance_infantry->slid_angle = 0.0f;
    // }
    // else
    // {

    //     float v = yaw_line_v * balance_infantry->set_yaw_v + 1 * balance_infantry->state_vector[STATE_X_V];
    //     balance_infantry->slid_angle = acosf(v / (temp_v * temp_target_v));
    // }
}

/**
 * @brief 目标向量设置
 * @param[in] balance_infantry
 */
static void init_target_vector(BalanceInfantry *balance_infantry)
{
    balance_infantry->target_vector[STATE_X] = (balance_infantry->sensors_info.wheel_pos[LEFT_WHEEL_ID] - balance_infantry->sensors_info.wheel_pos[RIGHT_WHEEL_ID]) * WHEEL_RADIUS / 2;
}

void getMaxXLimit(BalanceInfantry *balance_infantry)
{
    const static float x_limit = 1.3f;
    float temp_x_v = LIMIT_MAX_MIN(balance_infantry->state_vector[STATE_X_V], x_limit, -x_limit);
    balance_infantry->limit_err_x_max = temp_x_v + x_limit;
    balance_infantry->limit_err_x_min = temp_x_v - x_limit;
}
static void set_target_vector(BalanceInfantry *balance_infantry)
{
    float AngErr_front = limit_pi(GIMBAL_FOLLOW_ZERO / 22.755555556f - gimbal_receiver_pack2.yaw_motor_angle / 22.755555556f) * ANGLE_TO_RAD_COEF;
    balance_infantry->sin_dir = arm_sin_f32(AngErr_front);
    balance_infantry->cos_dir = arm_cos_f32(AngErr_front);

    if (remote_controller.robot_state == OFFLINE_MODE || remote_controller.control_mode_action == FLY_MODE || remote_controller.control_mode_action == LAND_MODE || remote_controller.control_mode_action == JUMP_DOWN_MODE)
    {
        // 运动模式下也直接清零
        // 状态设置为现状
        init_target_vector(balance_infantry);
        YawInit(balance_infantry);
        balance_infantry->target_vector[STATE_X_V] = 0.0f;
        balance_infantry->target_vector[STATE_THETA] = 0.0f;
        balance_infantry->target_yaw_v = 0.0f;
        balance_infantry->target_v = 0.0f;
    }
    else if (remote_controller.control_mode_action == FELL_RUN_MODE)
    {
        // 运动模式下也直接清零
        // 状态设置为现状
        init_target_vector(balance_infantry);
        YawInit(balance_infantry);
        balance_infantry->target_vector[STATE_X_V] = balance_infantry->cos_dir * balance_infantry->target_x_v + balance_infantry->sin_dir * balance_infantry->target_y_v;
        balance_infantry->target_vector[STATE_THETA] = 0.0f;
        balance_infantry->target_yaw_v = 0.0f;
        balance_infantry->target_v = balance_infantry->cos_dir * balance_infantry->target_x_v + balance_infantry->sin_dir * balance_infantry->target_y_v;
    }
    else if (remote_controller.control_mode_action == SHRINK_LEG_MODE)
    {
        init_target_vector(balance_infantry);
        YawInit(balance_infantry);
        balance_infantry->target_vector[STATE_X_V] = 0.0f;
        balance_infantry->target_vector[STATE_THETA] = -balance_infantry->sensors_info.pitch;
        balance_infantry->target_yaw_v = 0.0f;
        balance_infantry->target_v = 0.0f;
    }
    // else if (remote_controller.control_mode_action == SLIDDED_MODE)
    // {
    //     init_target_vector(balance_infantry);
    //     YawInit(balance_infantry);
    //     balance_infantry->target_vector[STATE_X_V] = 0.0f;
    //     balance_infantry->target_vector[STATE_THETA] = 0.0f;
    //     balance_infantry->target_yaw_v = 0.0f;
    //     balance_infantry->target_v = 0.0f;
    // }
    else
    {
        // balance_infantry->target_vector[STATE_X] += balance_infantry->target_v * balance_infantry->delta_t;
        // 限制输出幅值过大导致发散，在位移较大下可以进行一定的保护
        // 将速度分解到云台所在方向
        float target_v = balance_infantry->cos_dir * balance_infantry->target_x_v + balance_infantry->sin_dir * balance_infantry->target_y_v;
        if (buffer_energy.max_Power >= 99.0f)
        {
            target_v *= 1.25f;
        }
        else if (buffer_energy.max_Power >= 79.0f)
        {
            target_v *= 1.125f; // 不同功率速度变化
        }
        if (fabs(target_v) < 1e-1) // 当速度为0时，停止加位移
        {
            balance_infantry->target_v = 0;
        }
        else // 约0.5s后加速到最大速度
        {
            iir(&balance_infantry->target_v, target_v, 0.998);
        }

        balance_infantry->target_vector[STATE_X] += balance_infantry->target_v * balance_infantry->delta_t;
        getMaxXLimit(balance_infantry);
        balance_infantry->target_vector[STATE_X] = balance_infantry->state_vector[STATE_X] +
                                                   LIMIT_MAX_MIN(balance_infantry->target_vector[STATE_X] - balance_infantry->state_vector[STATE_X],
                                                                 balance_infantry->limit_err_x_max, balance_infantry->limit_err_x_min);
        balance_infantry->target_vector[STATE_THETA] = 0.0f;
        balance_infantry->target_vector[STATE_X_V] = 0.0f;
    }

    // 腿
    if (remote_controller.control_mode_action == FLY_MODE)
    {
        balance_infantry->target_L = 0.21f;
    }
    else if (remote_controller.leg_action == HIGH_LENGTH)
    {
        balance_infantry->target_L = 0.26f;
    }
    else if (remote_controller.leg_action == NOT_STRETCH)
    {
        balance_infantry->target_L = 0.12f;
    }
    // else if (remote_controller.control_mode_action == JUMP_PRE_MODE)
    // {
    //     balance_infantry->target_L = 0.11f;
    // }
    // else if (remote_controller.control_mode_action == JUMP_UP_MODE)
    // {
    //     balance_infantry->target_L = 0.28f;
    // }
    // else if (remote_controller.control_mode_action == JUMP_DOWN_MODE)
    // {
    //     balance_infantry->target_L = 0.14f;
    // }
    else
    {
        balance_infantry->target_L = 0.17f;
    }
    // balance_infantry->target_L += balance_infantry->target_v * 0.08f * balance_infantry->delta_t;
    // balance_infantry->target_L = LIMIT_MAX_MIN(balance_infantry->target_L, MAX_L, MIN_L);
}

/**
 * @brief 从传感器获取必要信息
 * @param[in] balance_infantry
 */

static void get_info_from_sensors(BalanceInfantry *balance_infantry)
{
    // STATE 2,3状态更新
    // balance_infantry->state_vector[STATE_X] = (balance_infantry->sensors_info.wheel_pos[LEFT_WHEEL_ID] -
    //                                            balance_infantry->sensors_info.wheel_pos[RIGHT_WHEEL_ID]) *
    //                                           WHEEL_RADIUS / 2;

    // // 得到速度的方法1： 一阶低通滤波
    // float speed = (balance_infantry->sensors_info.wheel_speed[LEFT_WHEEL_ID] -
    //                balance_infantry->sensors_info.wheel_speed[RIGHT_WHEEL_ID]) *
    //               WHEEL_RADIUS / 2;
    // rc_filter(&balance_infantry->state_vector[STATE_X_V], speed, balance_infantry->delta_t, LP_X_V_RC);
    balance_infantry->state_vector[STATE_X] = Wheel_Accel_Fusion.x;
    balance_infantry->state_vector[STATE_X_V] = Wheel_Accel_Fusion.x_v;

    // STATE 4,5状态更新
    balance_infantry->state_vector[STATE_PITCH] = balance_infantry->sensors_info.pitch;

    // 得到速度的方法1： 一阶低通滤波
    rc_filter(&balance_infantry->state_vector[STATE_PITCH_V], (balance_infantry->state_vector[STATE_PITCH] - balance_infantry->pitch_last) / balance_infantry->delta_t, balance_infantry->delta_t, LP_PITCH_V_RC);
    balance_infantry->pitch_last = balance_infantry->sensors_info.pitch;

    // 获得yaw轴速度
    rc_filter(&balance_infantry->yaw_v, (balance_infantry->sensors_info.yaw - balance_infantry->last_yaw_angle) / balance_infantry->delta_t, balance_infantry->delta_t, LP_YAW_V_RC);
    balance_infantry->last_yaw_angle = balance_infantry->sensors_info.yaw;
    // TD_Calculate(&balance_infantry->pitch_td, balance_infantry->sensors_info.pitch);

    // Deadzone(&balance_infantry->state_vector[STATE_PITCH_V], 0.02);

    // //得到速度的方法2： 最小二乘法提取信号微分
    // pitch_diff_test = OLS_Derivative(&balance_infantry->pitch_diff, balance_infantry->delta_t, balance_infantry->state_vector[STATE_PITCH]);

    //
    balance_infantry->phi_left[1] = balance_infantry->sensors_info.knee_angle[LEFT_KNEE_RIGHT];
    balance_infantry->phi_left[4] = balance_infantry->sensors_info.knee_angle[LEFT_KNEE_LEFT];

    balance_infantry->phi_right[1] = balance_infantry->sensors_info.knee_angle[RIGHT_KNEE_LEFT];
    balance_infantry->phi_right[4] = balance_infantry->sensors_info.knee_angle[RIGHT_KNEE_RIGHT];
}

/**
 * @brief 正运动学解算
 * @param[in] L 腿部长度
 * @param[out] phi 腿部姿态角
 */
static void MotionSolve(float *L, float *phi, MotionSolver *solver) // 运动学解算
{
    float x_B, y_B, x_D, y_D, B_D_distance_2, A_0, B_0, C_0, temp, x_C, y_C;

    x_B = L[1] * arm_cos_f32(phi[1]);
    y_B = L[1] * arm_sin_f32(phi[1]);
    x_D = L[4] * arm_cos_f32(phi[4]) + L[5];
    y_D = L[4] * arm_sin_f32(phi[4]);
    B_D_distance_2 = (x_D - x_B) * (x_D - x_B) + (y_D - y_B) * (y_D - y_B);
    A_0 = 2 * L[2] * (x_D - x_B);
    B_0 = 2 * L[2] * (y_D - y_B);
    C_0 = L[2] * L[2] + B_D_distance_2 - L[3] * L[3];

    temp = (B_0 + Sqrt(A_0 * A_0 + B_0 * B_0 - C_0 * C_0));
    phi[2] = 2 * arm_atan2_f32(temp, A_0 + C_0);

    x_C = x_B + L[2] * arm_cos_f32(phi[2]);
    y_C = y_B + L[2] * arm_sin_f32(phi[2]);

    phi[3] = arm_atan2_f32(y_C - y_D, x_C - x_D);

    x_C -= (L[5] / 2);

    L[0] = Sqrt(x_C * x_C + y_C * y_C);

    phi[0] = arm_atan2_f32(y_C, x_C);

    solver->x_B = x_B;
    solver->x_C = x_C + (L[5] / 2);
    solver->x_D = x_D;
    solver->y_B = y_B;
    solver->y_C = y_C;
    solver->y_D = y_D;
}

/**
 * @brief 从运动学解算中计算腿部姿态角
 * @param[in] balance_infantry
 */
static void getTheta(BalanceInfantry *balance_infantry)
{
    // 计算左腿姿态角
    float temp = -(PI / 2 - balance_infantry->phi_left[0] + balance_infantry->sensors_info.pitch);

    // 得到速度的方法1： 一阶低通滤波
    rc_filter(&balance_infantry->theta_left_w, (temp - balance_infantry->theta_left) / balance_infantry->delta_t, balance_infantry->delta_t, LP_THETA_V_RC);
    balance_infantry->theta_left = temp;

    // 计算右腿姿态角
    temp = -(PI / 2 - balance_infantry->phi_right[0] + balance_infantry->sensors_info.pitch);

    // 得到速度的方法1： 一阶低通滤波
    rc_filter(&balance_infantry->theta_right_w, (temp - balance_infantry->theta_right) / balance_infantry->delta_t, balance_infantry->delta_t, LP_THETA_V_RC);
    balance_infantry->theta_right = temp;
}

/*  自救模式下完成的判断  */
int8_t recover_finish_judge()
{
    return fabs(balance_infantry.sensors_info.pitch) < 0.06;
}

/**
 * @brief LQR计算
 * @param[in] balance_infantry
 * @param[out] virtual_torque 虚拟力矩输出
 */
static void LQR_Solve(BalanceInfantry *balance_infantry, float *virtual_torque)
{
    if (remote_controller.control_mode_action != SHRINK_LEG_MODE && remote_controller.control_mode_action != FELL_RUN_MODE)
    {
        for (int i = 0; i < 2; i++)
        {
            virtual_torque[i] = 0;
            for (int j = 0; j < 6; j++)
            {
                virtual_torque[i] += balance_infantry->K[i][j] * (balance_infantry->target_vector[j] - balance_infantry->state_vector[j]);
            }
            // virtual_torque[i] /= 2; //两端承担故除以2
        }
    }
    else if (remote_controller.control_mode_action == FELL_RUN_MODE)
    {
        // 該模式下PID計算速度
        virtual_torque[0] = 1.0f * (balance_infantry->target_vector[STATE_X_V] - balance_infantry->state_vector[STATE_X_V]);
        virtual_torque[1] = 0.0f;
    }
    else // 收腿模式下仅仅计算前两项
    {
        for (int i = 1; i < 2; i++)
        {
            virtual_torque[i] = 0;
            for (int j = 0; j < 2; j++)
            {
                virtual_torque[i] += balance_infantry->K[i][j] * (balance_infantry->target_vector[j] - balance_infantry->state_vector[j]);
            }
            // virtual_torque[i] += balance_infantry->K[i][STATE_X_V] * (balance_infantry->target_vector[STATE_X_V] - balance_infantry->state_vector[STATE_X_V]);
            // virtual_torque[i] /= 2; //两端承担故除以2
        }
        // 速度环衰减
        virtual_torque[0] = -balance_infantry->K[0][STATE_X_V] / 15.0f * (balance_infantry->target_vector[STATE_X_V] - balance_infantry->state_vector[STATE_X_V]);
        if (balance_infantry->is_shrink_finish_1 || balance_infantry->is_shrink_finish_2)
        {
            virtual_torque[1] = 0;
            virtual_torque[0] = 0;
        }
    }
}

/**
 * @brief VMC计算
 * @param[in] l 杆部长度
 * @param[in] phi 腿部角度
 * @param[in] virtual_torque 虚拟力矩输入
 * @param[out] J 雅可比矩阵
 * @param[in] extra_force 额外力
 * @param[in] extra_torque 额外力矩
 * @param[out] output_torque 输出关节力矩
 */
static void VMC_Solve(float *l, float *phi, float *virtual_torque, float J[][2], float *extra_force, float *extra_torque, float *output_torque)
{
    float sigma_1, sigma_2, sigma_3;

    sigma_1 = arm_sin_f32(phi[2] - phi[3]);
    sigma_2 = arm_sin_f32(phi[3] - phi[4]);
    sigma_3 = arm_sin_f32(phi[1] - phi[2]);

    J[0][0] = -l[1] * arm_cos_f32(phi[0] - phi[3]) * sigma_3 / l[0] / sigma_1;
    J[0][1] = -l[1] * arm_sin_f32(phi[0] - phi[3]) * sigma_3 / sigma_1;
    J[1][0] = -l[4] * arm_cos_f32(phi[0] - phi[2]) * sigma_2 / l[0] / sigma_1;
    J[1][1] = -l[4] * arm_sin_f32(phi[0] - phi[2]) * sigma_2 / sigma_1;

    output_torque[0] = -virtual_torque[0];
    virtual_torque[1] = LIMIT_MAX_MIN(-(virtual_torque[1] + *extra_torque), MAX_VMC_TORQUE, MIN_VMC_TORQUE);

    // 虚拟力作限幅以及低通滤波处理
    virtual_torque[2] = LIMIT_MAX_MIN(*extra_force, MAX_VMC_FORCE, -MAX_VMC_FORCE);

    for (int i = 1; i < 3; i++)
    {
        output_torque[i] = 0;
    }
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            output_torque[i + 1] += J[i][j] * virtual_torque[j + 1];
        }
    }
}

/**
 * @brief 飞坡状态下的增益矩阵，将其它无关的系数去掉
 * @param[in] balance_infantry
 */
static void jumpUpdateK(BalanceInfantry *balance_infantry, float L0)
{
    float pow_2 = L0 * L0;

    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            balance_infantry->K[i][j] = 0;
        }
    }
    balance_infantry->K[1][0] = balance_infantry->K_coef[1 * 6 + 0][0] * pow_2 + L0 * balance_infantry->K_coef[1 * 6 + 0][1] + balance_infantry->K_coef[1 * 6 + 0][2];
    balance_infantry->K[1][1] = balance_infantry->K_coef[1 * 6 + 1][0] * pow_2 + L0 * balance_infantry->K_coef[1 * 6 + 1][1] + balance_infantry->K_coef[1 * 6 + 1][2];
}

/**
 * @brief 根据腿长更新LQR增益矩阵
 * @param[in] balance_infantry
 * @param[in] L0 腿长
 */
static void updateK(BalanceInfantry *balance_infantry, float L0)
{
    if (remote_controller.control_mode_action == FLY_MODE || remote_controller.control_mode_action == JUMP_DOWN_MODE)
    {
        jumpUpdateK(balance_infantry, L0);
    }
    else
    {
        float pow_2 = L0 * L0;
        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                balance_infantry->K[i][j] = balance_infantry->K_coef[i * 6 + j][0] * pow_2 + L0 * balance_infantry->K_coef[i * 6 + j][1] + balance_infantry->K_coef[i * 6 + j][2];
            }
        }
    }
}

/**
 * @brief 更新状态向量
 */
static void get_info_from_solve(BalanceInfantry *balance_infantry, float theta, float theta_w)
{
    balance_infantry->state_vector[STATE_THETA] = theta;
    balance_infantry->state_vector[STATE_THETA_V] = theta_w;
}
static void TidyOutput(BalanceInfantry *balance_infantry)
{
    balance_infantry->excute_info.wheels_torque[LEFT_WHEEL_ID] = -balance_infantry->output_torque_l[0];
    balance_infantry->excute_info.wheels_torque[RIGHT_WHEEL_ID] = balance_infantry->output_torque_r[0];

    balance_infantry->excute_info.knee_torque[LEFT_KNEE_LEFT] = -balance_infantry->output_torque_l[2];
    balance_infantry->excute_info.knee_torque[LEFT_KNEE_RIGHT] = -balance_infantry->output_torque_l[1];

    balance_infantry->excute_info.knee_torque[RIGHT_KNEE_LEFT] = balance_infantry->output_torque_r[1];
    balance_infantry->excute_info.knee_torque[RIGHT_KNEE_RIGHT] = balance_infantry->output_torque_r[2];
}
/**
 * @brief 转向控制
 * @param[in] balance_infantry
 */
static void turnSolve(BalanceInfantry *balance_infantry)
{
    float pid_out;
    // 获取角度差

    switch (remote_controller.control_mode_action)
    {
    case BALANCE:
    case RECOVER:
    case SLIDDED_MODE:
    case JUMP_PRE_MODE:
    case JUMP_UP_MODE:
        balance_infantry->set_yaw_v = 0.0f;
        pid_out = PID_Calculate(&balance_infantry->turn_speed_pid, balance_infantry->yaw_v, balance_infantry->set_yaw_v);

        balance_infantry->output_torque_l[0] += pid_out;
        balance_infantry->output_torque_r[0] -= pid_out;

        PID_Clear_withMeasure(&balance_infantry->turn_pid, balance_infantry->err_angle, 0);
        break;
    case SELF_ROTATE:
        balance_infantry->set_yaw_v = ramp_control(balance_infantry->yaw_v, balance_infantry->target_yaw_v, 0.18);
        pid_out = PID_Calculate(&balance_infantry->turn_speed_pid, balance_infantry->yaw_v, balance_infantry->set_yaw_v);

        balance_infantry->output_torque_l[0] += pid_out;
        balance_infantry->output_torque_r[0] -= pid_out;

        PID_Clear_withMeasure(&balance_infantry->turn_pid, balance_infantry->err_angle, 0);
        break;
    case FOLLOW_GIMBAL:
    case LAND_MODE:
        balance_infantry->err_angle = angle_z_err_get(gimbal_receiver_pack2.yaw_motor_angle / 22.755555556f, GIMBAL_FOLLOW_ZERO) * ANGLE_TO_RAD_COEF;
        balance_infantry->target_pid_yaw_v = PID_Calculate(&balance_infantry->turn_pid, balance_infantry->err_angle, 0);
        balance_infantry->set_yaw_v = ramp_control(balance_infantry->yaw_v, balance_infantry->target_pid_yaw_v, 0.18);
        pid_out = PID_Calculate(&balance_infantry->turn_speed_pid, balance_infantry->yaw_v, balance_infantry->set_yaw_v);
        balance_infantry->output_torque_l[0] += pid_out;
        balance_infantry->output_torque_r[0] -= pid_out;
        break;
    case FOLLOW_BESIDES:
        balance_infantry->err_angle = angle_z_err_get(gimbal_receiver_pack2.yaw_motor_angle / 22.755555556f, GIMBAL_FOLLOW_BESIDES_ZEROS) * ANGLE_TO_RAD_COEF;
        balance_infantry->target_pid_yaw_v = PID_Calculate(&balance_infantry->turn_pid, balance_infantry->err_angle, 0);
        balance_infantry->set_yaw_v = ramp_control(balance_infantry->yaw_v, balance_infantry->target_pid_yaw_v, 0.18);
        pid_out = PID_Calculate(&balance_infantry->turn_speed_pid, balance_infantry->yaw_v, balance_infantry->set_yaw_v);

        balance_infantry->output_torque_l[0] += pid_out;
        balance_infantry->output_torque_r[0] -= pid_out;
        break;
    default:
        PID_Clear(&balance_infantry->turn_speed_pid);
        PID_Clear_withMeasure(&balance_infantry->turn_pid, balance_infantry->err_angle, 0);
        break;
    }
}
/**
 * @brief 腿长控制
 * @param[in] L0 腿长
 * @param[in] L_target 目标腿长
 * @param[out] output PID输出
 */
static void leg_control(PID_t *leg_control_pid, float L0, float L_target, float *output, float leg_angle) // 计算额外力
{
    switch (remote_controller.control_mode_action)
    {
    case BALANCE:
    case SELF_ROTATE:
    case FOLLOW_GIMBAL:
    case FOLLOW_BESIDES:
    case SLIDDED_MODE:
    case FLY_MODE:
    case LAND_MODE:
    case JUMP_PRE_MODE:
    case JUMP_DOWN_MODE:
    case JUMP_UP_MODE:
        *output = -PID_Calculate(leg_control_pid, L0, L_target) - GRAVITY_FEED_FORWARD;
        break;

    default:
        PID_Clear_withMeasure(leg_control_pid, L0, L_target);
        *output = 0;
        break;
    }
}

/**
 * @brief roll轴控制
 * @param[in] balance_infantry
 * @param[out] output PID输出
 */
static void roll_control(BalanceInfantry *balance_infantry, float *output_l, float *output_r) // 计算额外力
{
    float output;

    switch (remote_controller.control_mode_action)
    {
    case BALANCE:
    case SELF_ROTATE:
    case FOLLOW_GIMBAL:
    case FOLLOW_BESIDES:
    case SLIDDED_MODE:
    case LAND_MODE:
    case JUMP_PRE_MODE:
    case JUMP_UP_MODE:
        output = PID_Calculate(&balance_infantry->roll_control_pid, balance_infantry->sensors_info.roll, 0);
        *output_l -= output;
        *output_r += output;
        break;

    default:
        PID_Clear_withMeasure(&balance_infantry->roll_control_pid, balance_infantry->sensors_info.roll, 0);
        break;
    }
}

static void leg_cooperate_control(BalanceInfantry *balance_infantry, float *output_l, float *output_r)
{
    float output;
    output = PID_Calculate(&balance_infantry->legs_cooperate_pid,
                           balance_infantry->theta_left - balance_infantry->theta_right, 0);
    if (remote_controller.control_mode_action == SHRINK_LEG_MODE)
    {
        *output_l = 0;
        *output_r = 0;
    }
    else
    {
        *output_l = output;
        *output_r = -output;
    }
}

/**
 * @brief 腿长限制
 * @param[in] balance_infantry
 */
// static void limitLegLength(BalanceInfantry *balance_infantry)
//{
//     balance_infantry->target_L = LIMIT_MAX_MIN(balance_infantry->target_L, MAX_L, MIN_L);
// }

void accel_odom_fusion(BalanceInfantry *balance_infantry)
{
    //    float pos;
    float speed;
    //    pos = (balance_infantry->sensors_info.wheel_pos[LEFT_WHEEL_ID] -
    //           balance_infantry->sensors_info.wheel_pos[RIGHT_WHEEL_ID]) *
    //          WHEEL_RADIUS / 2;

    speed = (balance_infantry->sensors_info.wheel_speed[LEFT_WHEEL_ID] -
             balance_infantry->sensors_info.wheel_speed[RIGHT_WHEEL_ID]) *
            WHEEL_RADIUS / 2;

    // balance_infantry->Wheel_Accel_Fusion->x = pos;
    // balance_infantry->Wheel_Accel_Fusion->x_v = speed * 0.05f + balance_infantry->Wheel_Accel_Fusion->x_v * 0.95f;

    balance_infantry->Wheel_Accel_Fusion->Q1 = 0.0005 * 10;
    balance_infantry->Wheel_Accel_Fusion->Q2 = 10;
    balance_infantry->Wheel_Accel_Fusion->R1 = 200;

    KF_Wheel_Accel_Update(speed,
                          -balance_infantry->INS->MotionAccel_b[1], balance_infantry->delta_t);
}

static void get_feedback_f_torque(float torque_1, float torque_4, float J[][2], float *feedback) // 通过转矩反馈反解算力
{
    float J_inv[2][2];
    float coef = J[0][0] * J[1][1] - J[0][1] * J[1][0];
    J_inv[0][0] = J[1][1] / coef;
    J_inv[0][1] = -J[0][1] / coef;
    J_inv[1][0] = -J[1][0] / coef;
    J_inv[1][1] = J[0][0] / coef;

    feedback[0] = J_inv[0][0] * torque_1 + J_inv[0][1] * torque_4;
    feedback[1] = J_inv[1][0] * torque_1 + J_inv[1][1] * torque_4;
}

static void off_ground_detect(BalanceInfantry *balance_infantry)
{
    float f_l, f_r;

    get_feedback_f_torque(balance_infantry->sensors_info.torque_feed_back[LEFT_KNEE_RIGHT],
                          balance_infantry->sensors_info.torque_feed_back[LEFT_KNEE_LEFT],
                          balance_infantry->J_l, balance_infantry->wheels_f_torque_feedback_l);
    get_feedback_f_torque(balance_infantry->sensors_info.torque_feed_back[RIGHT_KNEE_LEFT],
                          balance_infantry->sensors_info.torque_feed_back[RIGHT_KNEE_RIGHT],
                          balance_infantry->J_r, balance_infantry->wheels_f_torque_feedback_r);

    float L_l = balance_infantry->L_left[0] * arm_cos_f32(balance_infantry->theta_left);
    float L_r = balance_infantry->L_right[0] * arm_cos_f32(balance_infantry->theta_right);

    // 速度估计
    rc_filter(&balance_infantry->L_theta_v_l, (L_l - balance_infantry->last_l_theta_l) / balance_infantry->delta_t, balance_infantry->delta_t, 0.0018);
    rc_filter(&balance_infantry->L_theta_v_r, (L_r - balance_infantry->last_l_theta_r) / balance_infantry->delta_t, balance_infantry->delta_t, 0.0018);
    balance_infantry->last_l_theta_l = L_l;
    balance_infantry->last_l_theta_r = L_r;

    // 加速度估计
    rc_filter(&balance_infantry->L_theta_a_l, (balance_infantry->L_theta_v_l - balance_infantry->last_l_theta_v_l) / balance_infantry->delta_t, balance_infantry->delta_t, 0.0018);
    rc_filter(&balance_infantry->L_theta_a_r, (balance_infantry->L_theta_v_r - balance_infantry->last_l_theta_v_r) / balance_infantry->delta_t, balance_infantry->delta_t, 0.0018);
    balance_infantry->last_l_theta_v_l = balance_infantry->L_theta_v_l;
    balance_infantry->last_l_theta_v_r = balance_infantry->L_theta_v_r;

    balance_infantry->wheels_f_torque_feedback_r[1] = -balance_infantry->wheels_f_torque_feedback_r[1];
    balance_infantry->wheels_f_torque_feedback_r[0] = -balance_infantry->wheels_f_torque_feedback_r[0];

    f_l = balance_infantry->wheels_f_torque_feedback_l[1] * arm_cos_f32(balance_infantry->theta_left) +
          balance_infantry->wheels_f_torque_feedback_l[0] * arm_sin_f32(balance_infantry->theta_left) / balance_infantry->L_left[0] +
          WHEEL_WEIGHT * (GRAVITY + balance_infantry->INS->MotionAccel_b[2] - balance_infantry->L_theta_a_l);
    f_r = balance_infantry->wheels_f_torque_feedback_r[1] * arm_cos_f32(balance_infantry->theta_right) +
          balance_infantry->wheels_f_torque_feedback_r[0] * arm_sin_f32(balance_infantry->theta_right) / balance_infantry->L_right[0] +
          WHEEL_WEIGHT * (GRAVITY + balance_infantry->INS->MotionAccel_b[2] - balance_infantry->L_theta_a_r);

    // 进行一波低通滤波
    balance_infantry->FN_l = balance_infantry->FN_l * 0.95f + f_l * 0.05f;
    balance_infantry->FN_r = balance_infantry->FN_r * 0.95f + f_r * 0.05f;

    if (balance_infantry->FN_l < 20 && balance_infantry->FN_r < 20)
    {
        balance_infantry->is_get_enough_force = FALSE;
    }
    else if (balance_infantry->FN_l > 65 && balance_infantry->FN_r > 65)
    {
        balance_infantry->is_get_enough_force = TRUE;
    }
}

///**
// * @brief 飞坡状态机
// * @param[in] balance_infantry
//// */
// static void state_trans(BalanceInfantry *balance_infantry)
//{
//     //恢复状态下不检测飞坡
//     // if (remote_controller.control_mode_action == RECOVER || remote_controller.robot_state == OFFLINE_MODE)
//     // {
//     //     GetDeltaT(&balance_infantry->balance_start_time);
//     //     balance_infantry->fly_detect_symbol = 0;
//     // }
//     // //未开始检测，过一段时间后开始检测
//     // if (!balance_infantry->fly_detect_symbol && remote_controller.control_mode_action == BALANCE)
//     // {
//     //     if (GetDeltaTtoNow(&balance_infantry->balance_start_time) > 1.0f)
//     //     {
//     //         balance_infantry->fly_detect_symbol = 1;
//     //     }
//     // }
//     // // 平衡状态下，转移至跳跃态
//     // if (balance_infantry->fly_detect_symbol && remote_controller.control_mode_action == BALANCE && balance_infantry->FN_l < MIN_FORCE && balance_infantry->FN_r < MIN_FORCE)
//     // {
//     //     setControlModeAction(FLY_MODE);
//     //     balance_infantry->target_L = 0.26; //目标腿长

//    //     //重新初始化PID，高刚度，低阻尼
//    //     PID_Init(&balance_infantry->leg_control_pid_l, 60, 0, 0, 2800, 0, 250, 0.005, 0.01, 0.01, 0.02, 1, ChangingIntegrationRate | DerivativeFilter | OutputFilter);
//    //     PID_Init(&balance_infantry->leg_control_pid_r, 60, 0, 0, 2800, 0, 250, 0.005, 0.01, 0.01, 0.02, 1, ChangingIntegrationRate | DerivativeFilter | OutputFilter);
//    // }
//    // // //跳跃态下，转移至落地态
//    // else if (remote_controller.control_mode_action == FLY_MODE && (balance_infantry->FN_l > MAX_FORCE && balance_infantry->FN_r > MAX_FORCE))
//    // {
//    //     setControlModeAction(LAND_MODE);
//    //     balance_infantry->target_L = 0.16;

//    //     //保存落地时的时间
//    //     GetDeltaT(&balance_infantry->land_start_time);

//    //     //重新初始化PID，低刚度，高阻尼
//    //     PID_Init(&balance_infantry->leg_control_pid_l, 60, 0, 0, 1200, 0, 900, 0.005, 0.01, 0.01, 0.02, 1, ChangingIntegrationRate | DerivativeFilter | OutputFilter);
//    //     PID_Init(&balance_infantry->leg_control_pid_r, 60, 0, 0, 1200, 0, 900, 0.005, 0.01, 0.01, 0.02, 1, ChangingIntegrationRate | DerivativeFilter | OutputFilter);
//    // }
//    // else if (remote_controller.control_mode_action == LAND_MODE)
//    // {
//    //     balance_infantry->land_to_now_time = GetDeltaTtoNow(&balance_infantry->land_start_time);

//    //     if (balance_infantry->land_to_now_time > LAND_TO_BALANCE_TIME)
//    //     {
//    //         setControlModeAction(BALANCE);
//    //         balance_infantry->target_L = 0.16;

//    //         //重新初始化PID，高刚度，低阻尼
//    //         PID_Init(&balance_infantry->leg_control_pid_l, 60, 0, 0, 2800, 0, 250, 0.005, 0.01, 0.01, 0.02, 1, ChangingIntegrationRate | DerivativeFilter | OutputFilter);
//    //         PID_Init(&balance_infantry->leg_control_pid_r, 60, 0, 0, 2800, 0, 250, 0.005, 0.01, 0.01, 0.02, 1, ChangingIntegrationRate | DerivativeFilter | OutputFilter);
//    //     }
//    // }
//}

/**
 * @brief  底盘方向偏差获取
 * @param  目标方向(单位为角度)
 * @retval 方向偏差
 */
float angle_z_err_get(float target_ang, float zeros_angle)
{
    float AngErr_front, AngErr_back;
    AngErr_front = limit_pi(zeros_angle / 22.755555556f - target_ang);
    AngErr_back = limit_pi((zeros_angle + 4096) / 22.755555556f - target_ang);
    if (fabs(AngErr_front) > fabs(AngErr_back))
    {
        balance_infantry.chassis_direction = CHASSIS_BACK;
        return AngErr_back;
    }
    else
    {
        balance_infantry.chassis_direction = CHASSIS_FRONT;
        return AngErr_front;
    }
}

/**
 * @brief  梯形控制
 * @param  反馈；设定；加速度
 * @retval 输出值
 */
float ramp_control(float ref, float set, float accel)
{
    float ramp = LIMIT_MAX_MIN(accel, 1, 0) * (set - ref);
    return ref + ramp;
}

static void setTargetL(BalanceInfantry *balance_infantry)
{
    if (remote_controller.control_mode_action != FLY_MODE && remote_controller.control_mode_action != JUMP_DOWN_MODE && remote_controller.control_mode_action != JUMP_PRE_MODE &&
        remote_controller.control_mode_action != JUMP_UP_MODE && remote_controller.control_mode_action != LAND_MODE)
    {
        float target_L_left, target_L_right;
        // 估算斜坡高，以左为支点
        float slope_h = balance_infantry->L_left[0] * arm_cos_f32(balance_infantry->theta_left) - ROBOT_WIDTH * tanf(balance_infantry->sensors_info.roll) - balance_infantry->L_right[0] * arm_cos_f32(balance_infantry->theta_right);

        // 计算斜坡角度
        float slope_angle = arm_atan2_f32(slope_h, ROBOT_WIDTH);
        balance_infantry->slope_angle = LIMIT_MAX_MIN(slope_angle, 0.30f, -0.30f);

        // 计算期望的腿长差
        float diff_len = ROBOT_WIDTH * tanf(balance_infantry->slope_angle) / 2;

        // 计算期望腿长度
        target_L_left = balance_infantry->target_L + diff_len;
        target_L_right = balance_infantry->target_L - diff_len;

        iir(&balance_infantry->target_L_left, target_L_left, 0.50);
        iir(&balance_infantry->target_L_right, target_L_right, 0.50);

        // 限制幅值
        balance_infantry->target_L_left = LIMIT_MAX_MIN(balance_infantry->target_L_left, MAX_L, MIN_L);
        balance_infantry->target_L_right = LIMIT_MAX_MIN(balance_infantry->target_L_right, MAX_L, MIN_L);
    }
    else // 飞坡模式下不满足双轮着地假设
    {
        balance_infantry->target_L_left = balance_infantry->target_L;
        balance_infantry->target_L_right = balance_infantry->target_L;
    }
}

/**
 * @brief 控制计算
 * @param[in] balance_infantry
 */
void main_control(BalanceInfantry *balance_infantry)
{
    get_info_from_sensors(balance_infantry);

    set_target_vector(balance_infantry);

    MotionSolve(balance_infantry->L_left, balance_infantry->phi_left, &balance_infantry->motion_solver_l);
    MotionSolve(balance_infantry->L_right, balance_infantry->phi_right, &balance_infantry->motion_solver_r);
    getTheta(balance_infantry);

    // robotObserverUpdate((balance_infantry->theta_left + balance_infantry->theta_right) / 2,
    //                     balance_infantry->state_vector[STATE_X], balance_infantry->state_vector[STATE_X_V], balance_infantry->state_vector[STATE_PITCH]);
    // memcpy(balance_infantry->state_observe, balance_infantry->robot_observer->x_data, 7 * 4);

    // 左腿力矩计算
    updateK(balance_infantry, balance_infantry->L_left[0]);
    get_info_from_solve(balance_infantry, balance_infantry->theta_left, balance_infantry->theta_left_w);
    LQR_Solve(balance_infantry, balance_infantry->virtual_torque_l);

    // for (int i = 0; i < 3; i++)
    // {
    //     balance_infantry->virtual_torque_r[i] = balance_infantry->virtual_torque_l[i];
    // }

    // //右腿力矩计算
    updateK(balance_infantry, balance_infantry->L_right[0]);
    get_info_from_solve(balance_infantry, balance_infantry->theta_right, balance_infantry->theta_right_w);
    LQR_Solve(balance_infantry, balance_infantry->virtual_torque_r);

    // 腿长限制
    // limitLegLength(balance_infantry);

    setTargetL(balance_infantry);

    // 腿长控制
    leg_control(&balance_infantry->leg_control_pid_l, balance_infantry->L_left[0], balance_infantry->target_L_left, &balance_infantry->extra_leg_force_l, balance_infantry->theta_left);
    leg_control(&balance_infantry->leg_control_pid_r, balance_infantry->L_right[0], balance_infantry->target_L_right, &balance_infantry->extra_leg_force_r, balance_infantry->theta_right);

    // 滚转角控制
    roll_control(balance_infantry, &balance_infantry->extra_leg_force_l, &balance_infantry->extra_leg_force_r);

    // 双腿协同
    leg_cooperate_control(balance_infantry, &balance_infantry->extra_torque_l, &balance_infantry->extra_torque_r);

    // VMC解算
    VMC_Solve(balance_infantry->L_left, balance_infantry->phi_left, balance_infantry->virtual_torque_l,
              balance_infantry->J_l, &balance_infantry->extra_leg_force_l, &balance_infantry->extra_torque_l,
              balance_infantry->output_torque_l);
    VMC_Solve(balance_infantry->L_right, balance_infantry->phi_right, balance_infantry->virtual_torque_r,
              balance_infantry->J_r, &balance_infantry->extra_leg_force_r, &balance_infantry->extra_torque_r,
              balance_infantry->output_torque_r);

    // 转向控制
    turnSolve(balance_infantry);

    off_ground_detect(balance_infantry);

    // 根据支持力解算结果设计状态机
    // state_trans(balance_infantry);

    // 输出整理
    TidyOutput(balance_infantry);

    // 限制幅值
    limitWheelTorque(balance_infantry->excute_info.wheels_torque);
    limitKneeTorque(balance_infantry->excute_info.knee_torque);

#ifdef JSCOPE_RTT_MODE
    JscopeWrite(balance_infantry->target_vector[0], balance_infantry->state_vector[0]);
#endif // JSCOPE_RTT_MODE
}

/**
 * @brief 发送给执行机构
 * @param[in] balance_infantry
 */
void execute_control(ExcuteTorque *excute_info)
{
#ifdef ALL_SEND_ZERO_TORQUE
    // 发送零力矩测试

    for (uint8_t i = 0; i < 4; i++)
    {
        motorOffPack(&excute_info->knee_send[i], A1MotorIDs[i]);
    }
    WheelOffPack(&Wheels_Motor.frame);

#else
    if (remote_controller.robot_state == OFFLINE_MODE || remote_controller.control_mode_action == FELL_RUN_MODE)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            motorOffPack(&excute_info->knee_send[i], A1MotorIDs[i]);
        }
    }
    else if (remote_controller.robot_state == CONTROL_MODE)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            // motorOffPack(&excute_info->knee_send[i], A1MotorIDs[i]);
            motorDataPack(&excute_info->knee_send[i], A1MotorIDs[i], excute_info->knee_torque[i] / REDUCTION_RATIO, 0, 0, 0, 0);
        }
    }

#endif // DEBUG

    memcpy(&SendToA1Motor_Buff[LEFT_KNEE_RS485 - 1], &excute_info->knee_send[LEFT_KNEE_LEFT].motor_send_data, sizeof(MasterComdDataV3));
    LEFT_KNEE_RS485_SEND();

    memcpy(&SendToA1Motor_Buff[RIGHT_KNEE_RS485 - 1], &excute_info->knee_send[RIGHT_KNEE_LEFT].motor_send_data, sizeof(MasterComdDataV3));
    RIGHT_KNEE_RS485_SEND();

    volatile uint32_t tickstart = COUNTER_TIMx->CNT;
    if (remote_controller.robot_state == CONTROL_MODE)
    {
        PowerControl(&Wheels_Motor, excute_info->wheels_torque);
        WheelCurrentPack(&Wheels_Motor.frame, excute_info->wheels_torque);
    }
    else
    {
        excute_info->wheels_torque[0] = excute_info->wheels_torque[1] = 0;
        PowerControl(&Wheels_Motor, excute_info->wheels_torque);
        WheelOffPack(&Wheels_Motor.frame);
    }
    DelayTo(&tickstart, 300);
    // Delay(300);

    memcpy(&SendToA1Motor_Buff[LEFT_KNEE_RS485 - 1], &excute_info->knee_send[LEFT_KNEE_RIGHT].motor_send_data, sizeof(MasterComdDataV3));
    LEFT_KNEE_RS485_SEND();

    memcpy(&SendToA1Motor_Buff[RIGHT_KNEE_RS485 - 1], &excute_info->knee_send[RIGHT_KNEE_RIGHT].motor_send_data, sizeof(MasterComdDataV3));
    RIGHT_KNEE_RS485_SEND();

    WheelCanSend(&Wheels_Motor.frame);
}

void InitJumpPrePid(BalanceInfantry *balance_infantry)
{
    balance_infantry->leg_control_pid_l.MaxOut = 150;
    balance_infantry->leg_control_pid_r.MaxOut = 150;

    balance_infantry->leg_control_pid_l.Kp = 1100;
    balance_infantry->leg_control_pid_r.Kp = 1100;
    balance_infantry->leg_control_pid_l.Kd = 65;
    balance_infantry->leg_control_pid_r.Kd = 65;
}

void InitJumpUpPid(BalanceInfantry *balance_infantry)
{
    balance_infantry->leg_control_pid_l.MaxOut = MAX_VMC_FORCE;
    balance_infantry->leg_control_pid_r.MaxOut = MAX_VMC_FORCE;

    balance_infantry->leg_control_pid_l.Kp = 2000;
    balance_infantry->leg_control_pid_r.Kp = 2000;
    balance_infantry->leg_control_pid_l.Kd = 40;
    balance_infantry->leg_control_pid_r.Kd = 40;
}

void InitJumpDownPid(BalanceInfantry *balance_infantry)
{
    balance_infantry->leg_control_pid_l.MaxOut = 150;
    balance_infantry->leg_control_pid_r.MaxOut = 150;

    balance_infantry->leg_control_pid_l.Kp = 1850;
    balance_infantry->leg_control_pid_r.Kp = 1850;
    balance_infantry->leg_control_pid_l.Kd = 65;
    balance_infantry->leg_control_pid_r.Kd = 65;
}

void InitFlyPid(BalanceInfantry *balance_infantry)
{
    balance_infantry->leg_control_pid_l.MaxOut = 150;
    balance_infantry->leg_control_pid_r.MaxOut = 150;

    balance_infantry->leg_control_pid_l.Kp = 1400;
    balance_infantry->leg_control_pid_r.Kp = 1400;
    balance_infantry->leg_control_pid_l.Kd = 20;
    balance_infantry->leg_control_pid_r.Kd = 20;
}

void InitLandPid(BalanceInfantry *balance_infantry)
{
    balance_infantry->leg_control_pid_l.MaxOut = 150;
    balance_infantry->leg_control_pid_r.MaxOut = 150;

    balance_infantry->leg_control_pid_l.Kp = 1100;
    balance_infantry->leg_control_pid_r.Kp = 1100;
    balance_infantry->leg_control_pid_l.Kd = 350;
    balance_infantry->leg_control_pid_r.Kd = 350;
}

void InitRawLegPid(BalanceInfantry *balance_infantry)
{
    balance_infantry->leg_control_pid_l.MaxOut = 150;
    balance_infantry->leg_control_pid_r.MaxOut = 150;

    balance_infantry->leg_control_pid_l.Kp = 1000;
    balance_infantry->leg_control_pid_r.Kp = 1000;
    balance_infantry->leg_control_pid_l.Kd = 20;
    balance_infantry->leg_control_pid_r.Kd = 20;
}

/**
 * @brief 机器人初始化
 * @param[in] balance_infantry
 */
void BalanceInfantryInit(BalanceInfantry *balance_infantry)
{
    // 腿长前馈初始化参数
    //  float c_left[3] = {300, 0, 0};
    //  float c_right[3] = {300, 0, 0};

    // 五连杆机构参数,L(0)需要计算得到
    balance_infantry->L_left[1] = 0.13;
    balance_infantry->L_left[2] = 0.24;
    balance_infantry->L_left[3] = 0.24;
    balance_infantry->L_left[4] = 0.13;
    balance_infantry->L_left[5] = 0.15;

    balance_infantry->L_right[1] = 0.13;
    balance_infantry->L_right[2] = 0.24;
    balance_infantry->L_right[3] = 0.24;
    balance_infantry->L_right[4] = 0.13;
    balance_infantry->L_right[5] = 0.15;

    // 腿部姿态角初始化
    balance_infantry->phi_left[0] = PI / 2;
    balance_infantry->phi_right[0] = PI / 2;

    // 腿长初始化
    balance_infantry->target_L = 0.18f;
    balance_infantry->target_L_left = 0.18f;
    balance_infantry->target_L_right = 0.18f;

    // 非变腿长LQR参数矩阵初始化
    //  balance_infantry->K[0][0] = -34.772468;
    //  balance_infantry->K[0][1] = -4.708436;
    //  balance_infantry->K[0][2] = -21.939749;
    //  balance_infantry->K[0][3] = -16.985400;
    //  balance_infantry->K[0][4] = 24.981338;
    //  balance_infantry->K[0][5] = 1.757150;
    //  balance_infantry->K[1][0] = 19.353490;
    //  balance_infantry->K[1][1] = 3.013197;
    //  balance_infantry->K[1][2] = 16.570745;
    //  balance_infantry->K[1][3] = 12.185510;
    //  balance_infantry->K[1][4] = 132.301667;
    //  balance_infantry->K[1][5] = 4.169808;

    float K_params[12][3] = {
        {185.393899, -268.157722, -8.942938},
        {-26.257716, -41.261807, -0.406693},
        {47.604259, -30.286812, -16.766969},
        {33.528508, -34.957600, -14.034564},
        {257.308346, -205.849273, 64.741918},
        {22.287324, -17.389337, 7.270292},
        {-185.402094, 87.476974, 22.388070},
        {-33.254883, 23.107830, 2.634811},
        {136.639503, -110.591948, 32.441433},
        {95.839313, -79.301563, 25.847811},
        {-403.523951, 259.631407, 98.159981},
        {-44.339397, 30.237082, 2.663729},
    };

    for (int j = 0; j < 12; j++)
    {
        for (int i = 0; i < 3; i++)
        {
            balance_infantry->K_coef[j][i] = K_params[j][i];
        }
    }

    balance_infantry->abnormal_count = 0;
    balance_infantry->is_abnormal = FALSE;

    // 跟踪微分器初始化
    //  TD_Init(&balance_infantry->pitch_td, 6000, 0.006);
    //  TD_Init(&balance_infantry->theta_left_td, 100, 1);
    //  TD_Init(&balance_infantry->theta_right_td, 100, 1);

    // 转向PID初始化,为方便，输出单位为力矩，输入为弧度
    PID_Init(&balance_infantry->turn_pid, SPEED_W_MAX, 0, 0, 9, 0, 0, 0, 0, 0.001, 0.009, 1, DerivativeFilter | OutputFilter);
    PID_Init(&balance_infantry->turn_speed_pid, 6, 0, 0, 9.5, 0, 0.05, 0, 0, 0.00, 0.004, 1, DerivativeFilter);

    // 腿长控制PID
    PID_Init(&balance_infantry->leg_control_pid_l, 150, 50, 0, 1000, 1080, 20, 0, 0, 0.0, 0.003, 1, Trapezoid_Intergral | DerivativeFilter | Integral_Limit);
    PID_Init(&balance_infantry->leg_control_pid_r, 150, 50, 0, 1000, 1080, 20, 0, 0, 0.0, 0.003, 1, Trapezoid_Intergral | DerivativeFilter | Integral_Limit);

    // roll控制PID
    PID_Init(&balance_infantry->roll_control_pid, 30, 0, 0, 250, 0, 0, 0, 0, 0.009, 0.009, 1, OutputFilter);

    // 双腿协同PID
    PID_Init(&balance_infantry->legs_cooperate_pid, 4.0, 0, 0, 60, 0, 3.0, 0, 0, 0, 0.0018, 1, DerivativeFilter);

    // 收腿PID
    //  PID_Init(&balance_infantry->shrink_leg_pid_l, 10, 0, 0, 1, 0, 0, 0.005, 0.01, 0, 0.02, 1, ChangingIntegrationRate | DerivativeFilter);
    //  PID_Init(&balance_infantry->shrink_leg_pid_r, 10, 0, 0, 1, 0, 0, 0.005, 0.01, 0, 0.02, 1, ChangingIntegrationRate | DerivativeFilter);

    // 解算一帧，各种角度作为初始化角度
    get_sensor_info(balance_infantry);
    get_info_from_sensors(balance_infantry);
    init_target_vector(balance_infantry);
    MotionSolve(balance_infantry->L_left, balance_infantry->phi_left, &balance_infantry->motion_solver_l);
    MotionSolve(balance_infantry->L_right, balance_infantry->phi_right, &balance_infantry->motion_solver_r);
    getTheta(balance_infantry);

    // 重置所有速度，避免无初值导致的发散
    balance_infantry->theta_left_w = 0;
    balance_infantry->theta_right_w = 0;
    balance_infantry->state_vector[STATE_PITCH_V] = 0;

    balance_infantry->FN_l = 12.0f;
    balance_infantry->FN_r = 12.0f;

    balance_infantry->limit_err_x_max = 2.0f;
    balance_infantry->limit_err_x_min = -2.0f;
}

void AbnormalDetect(BalanceInfantry *balance_infantry)
{
    // //卡方检验判断是否踩到大弹丸，没用
    // switch (remote_controller.control_mode_action)
    // {
    // case FOLLOW_GIMBAL:
    // case SELF_ROTATE:
    // case FOLLOW_BESIDES:
    // case BALANCE:
    //     if (balance_infantry->Wheel_Accel_Fusion->ChiSquare_Data[0] > 0.03f)
    //     {
    //         balance_infantry->is_slidded = TRUE;
    //     }
    //     break;
    // default:
    //     break;
    // }

    // 倒地状态检测

    if ((fabsf(balance_infantry->sensors_info.pitch) > 0.8f ||
         fabsf(balance_infantry->sensors_info.roll) > 0.5f ||
         fabsf(balance_infantry->theta_left) > 1.0f ||
         fabsf(balance_infantry->theta_right) > 1.0f))
    {
        balance_infantry->abnormal_count++;
        if (balance_infantry->abnormal_count >= 800)
        {
            balance_infantry->is_abnormal = TRUE;
        }
    }
    else
    {
        balance_infantry->abnormal_count = 0;
        balance_infantry->is_abnormal = FALSE;
    }
}

/* 收腿结束判断 */
void shrink_leg_finish_judge(BalanceInfantry *balance_infantry)
{
    if (remote_controller.control_mode_action == SHRINK_LEG_MODE)
    {
        if (!balance_infantry->is_shrink_finish_1 &&
            fabsf(balance_infantry->theta_left + balance_infantry->sensors_info.pitch) < 0.2f &&
            fabsf(balance_infantry->theta_right + balance_infantry->sensors_info.pitch) < 0.2f &&
            fabsf(balance_infantry->sensors_info.pitch) < 0.8f &&
            fabsf(balance_infantry->sensors_info.roll) < 0.5f &&
            fabsf(balance_infantry->state_vector[STATE_X_V]) < 0.2f &&
            balance_infantry->L_left[0] < 0.10f &&
            balance_infantry->L_right[0] < 0.10f) // 收腿之后(关节电机去力)
        {
            balance_infantry->is_shrink_finish_1 = TRUE;
        }
        if (balance_infantry->is_shrink_finish_1)
        {
            balance_infantry->shrink_finish_count_2++;
            if (balance_infantry->shrink_finish_count_2 >= 600)
            {
                balance_infantry->is_shrink_finish_2 = TRUE;
            }
        }
    }
    else
    {
        balance_infantry->is_shrink_finish_1 = FALSE;
        balance_infantry->is_shrink_finish_2 = FALSE;
        balance_infantry->shrink_finish_count_2 = 0;
    }
}

#include "BlueToothTask.h"

BlueToothSendData data;

/**
 * @brief 蓝牙发送debug任务
 * @param[in] void
 */
void BlueToothTask(void *pvParameters)
{
    portTickType xLastWakeTime;

    vTaskDelay(2000);

    while (1)
    {
        xLastWakeTime = xTaskGetTickCount();

        // data.fdata[0] = balance_infantry.state_vector[0] * 100.0f;
        // data.fdata[1] = balance_infantry.state_vector[1] * 100.0f;
        // data.fdata[2] = (balance_infantry.state_vector[2] - balance_infantry.target_vector[2]) * 100.0f;
        // data.fdata[3] = balance_infantry.state_vector[3] * 100.0f;
        // data.fdata[4] = balance_infantry.state_vector[4] * 100.0f;
        // data.fdata[5] = balance_infantry.state_vector[5] * 100.0f;

        // data.fdata[0] = robot_abnormal_detector.filte_wheel_current[0];
        // data.fdata[1] = robot_abnormal_detector.filte_wheel_speed[0];
        // data.fdata[2] = robot_abnormal_detector.filte_wheel_accel[0];
        // data.fdata[0] = robot_abnormal_detector.filte_wheel_current[1];
        // data.fdata[1] = robot_abnormal_detector.filte_wheel_speed[1];
        // data.fdata[2] = robot_abnormal_detector.filte_wheel_accel[1];

        // data.fdata[0] = INA260_1.Power / 1000.0f;
        // data.fdata[1] = referee_data.Power_Heat_Data.chassis_power;
        // data.fdata[2] = balance_infantry.Wheels_Motor->decoded_msgs[0].current;
        // data.fdata[3] = balance_infantry.Wheels_Motor->decoded_msgs[1].current;
        // data.fdata[4] = balance_infantry.Wheels_Motor->decoded_msgs[0].rotate_speed;
        // data.fdata[5] = balance_infantry.Wheels_Motor->decoded_msgs[1].rotate_speed;

        // data.fdata[0] = INA260_1.Power / 1000.0f;
        // data.fdata[1] = referee_data.Power_Heat_Data.chassis_power;
        // data.fdata[2] = power_limiter.predict_power;
        // data.fdata[3] = power_limiter.predict_send_power;
        // data.fdata[4] = balance_infantry.Wheels_Motor->decoded_msgs[0].current;
        // data.fdata[5] = balance_infantry.excute_info.wheels_torque[0] / 0.32f;
        // data.fdata[4] = power_limiter.send_torque_lower_scale;
        // data.fdata[5] = power_limiter.set_power;

        // 轮式里程计与加速度计速度融合测试
        // static float x, x_v;
        // float x_ = (balance_infantry.sensors_info.wheel_pos[LEFT_WHEEL_ID] -
        //             balance_infantry.sensors_info.wheel_pos[RIGHT_WHEEL_ID]) *
        //            WHEEL_RADIUS / 2;
        // float y_ = (balance_infantry.sensors_info.wheel_speed[LEFT_WHEEL_ID] -
        //             balance_infantry.sensors_info.wheel_speed[RIGHT_WHEEL_ID]) *
        //            WHEEL_RADIUS / 2;
        // rc_filter(&x, x_, 0.005, 0.0018);
        // rc_filter(&x_v, y_, 0.005, 0.0018);
        // data.fdata[0] = balance_infantry.sensors_info.wheel_speed[RIGHT_WHEEL_ID];
        // data.fdata[1] = x_v;
        // data.fdata[2] = balance_infantry.Wheel_Accel_Fusion->ChiSquareTestThreshold * 1e10f;
        // data.fdata[3] = balance_infantry.Wheel_Accel_Fusion->x;
        // data.fdata[4] = balance_infantry.Wheel_Accel_Fusion->x_v;
        // data.fdata[5] = balance_infantry.sensors_info.wheel_speed[LEFT_WHEEL_ID];

        // 滑移角测试
        //  data.fdata[0] = balance_infantry.slid_angle;
        //  data.fdata[1] = balance_infantry.sensors_info.wheel_speed[LEFT_WHEEL_ID];
        //  data.fdata[2] = balance_infantry.sensors_info.wheel_speed[RIGHT_WHEEL_ID];
        //  data.fdata[3] = balance_infantry.yaw_v * 0.21f / 2;
        //  data.fdata[4] = ((balance_infantry.sensors_info.wheel_speed[LEFT_WHEEL_ID] -
        //                    balance_infantry.sensors_info.wheel_speed[RIGHT_WHEEL_ID]) *
        //                   WHEEL_RADIUS / 2);

        // data.fdata[0] = Sqrt(balance_infantry.INS->MotionAccel_b[1] * balance_infantry.INS->MotionAccel_b[1] + balance_infantry.INS->MotionAccel_b[0] * balance_infantry.INS->MotionAccel_b[0] + balance_infantry.INS->MotionAccel_b[2] * balance_infantry.INS->MotionAccel_b[2]);
        // data.fdata[1] = balance_infantry.Wheel_Accel_Fusion->ChiSquare_Data[0] * 1e6f;
        // data.fdata[2] = balance_infantry.Wheel_Accel_Fusion->x;
        // data.fdata[3] = balance_infantry.Wheel_Accel_Fusion->x_v;
        // data.fdata[0] = balance_infantry.FN_l;
        // data.fdata[1] = balance_infantry.FN_r;
        // data.fdata[2] = balance_infantry.theta_left;
        // data.fdata[3] = balance_infantry.theta_right;
        // data.fdata[4] = balance_infantry.L_left[0];
        // data.fdata[5] = balance_infantry.L_right[0];

        // data.fdata[2] = balance_infantry.L_left[0] * arm_cos_f32(balance_infantry.theta_left);
        // data.fdata[3] = balance_infantry.L_theta_v_l;
        // data.fdata[4] = balance_infantry.L_theta_v_l;
        // // data.fdata[3] = balance_infantry.L_right[0] * arm_cos_f32(balance_infantry.theta_right);
        // data.fdata[5] = balance_infantry.INS->MotionAccel_b[2];

        // data.fdata[0] = balance_infantry.virtual_torque_l[1] + balance_infantry.extra_torque_l;
        // data.fdata[1] = balance_infantry.virtual_torque_r[1] + balance_infantry.extra_torque_r;
        // data.fdata[0] = balance_infantry.target_vector[STATE_X] - balance_infantry.state_vector[STATE_X];
        // data.fdata[1] = balance_infantry.state_vector[STATE_X_V];
        // data.fdata[2] = power_limiter.send_torque_lower_scale;

        // 飞坡测试
        // data.fdata[0] = balance_infantry.state_vector[STATE_PITCH];
        // data.fdata[1] = balance_infantry.theta_left;
        // data.fdata[2] = balance_infantry.state_vector[STATE_X] - balance_infantry.target_vector[STATE_X];
        // data.fdata[3] = balance_infantry.excute_info.wheels_torque[0];
        // data.fdata[4] = balance_infantry.virtual_torque_l[1] + balance_infantry.extra_torque_l;
        // data.fdata[5] = balance_infantry.state_vector[STATE_X_V];

        // data.fdata[0] = balance_infantry.state_vector[STATE_X] - balance_infantry.target_vector[STATE_X];
        // data.fdata[1] = balance_infantry.state_vector[STATE_PITCH];
        // data.fdata[2] = balance_infantry.theta_left;
        // data.fdata[3] = balance_infantry.theta_right;
        // data.fdata[0] = balance_infantry.target_L * 100.0f;
        // data.fdata[1] = balance_infantry.L_left[0] * 100.0f;
        // data.fdata[2] = balance_infantry.L_right[0] * 100.0f;

        // 滚转角控制调参
        //  data.fdata[0] = balance_infantry.slope_angle * RAD_TO_ANGLE_COEF;
        //  data.fdata[1] = balance_infantry.target_L_left * 100.0f;
        //  data.fdata[2] = balance_infantry.target_L_right * 100.0f;
        //  data.fdata[3] = balance_infantry.sensors_info.roll * RAD_TO_ANGLE_COEF;

        // 腿长控制调参
        // data.fdata[0] = balance_infantry.target_L * 100.0f;
        // data.fdata[1] = balance_infantry.L_left[0] * 100.0f;
        // data.fdata[2] = balance_infantry.L_right[0] * 100.0f;
        // data.fdata[3] = balance_infantry.leg_control_pid_l.Dout;
        // data.fdata[4] = balance_infantry.leg_control_pid_l.Pout;
        // data.fdata[5] = balance_infantry.sensors_info.roll * RAD_TO_ANGLE_COEF;

        // 预定期望设定测试
        // data.fdata[0] = balance_infantry.target_vector[STATE_X] - balance_infantry.state_vector[STATE_X];
        // data.fdata[1] = balance_infantry.state_vector[STATE_X_V];
        // data.fdata[2] = balance_infantry.limit_err_x_max;
        // data.fdata[3] = balance_infantry.limit_err_x_min;
        // data.fdata[4] = balance_infantry.target_vector[STATE_X] - balance_infantry.state_vector[STATE_X] - balance_infantry.state_vector[STATE_X_V];

        // data.fdata[0] = -balance_infantry.INS->MotionAccel_b[1];
        // data.fdata[1] = balance_infantry.Wheel_Accel_Fusion->ChiSquare_Data[0] * 1e6f;
        // data.fdata[2] = balance_infantry.state_vector[STATE_X];
        // data.fdata[3] = balance_infantry.state_vector[STATE_X_V];
        // data.fdata[4] = (balance_infantry.sensors_info.wheel_speed[LEFT_WHEEL_ID] -
        //                  balance_infantry.sensors_info.wheel_speed[RIGHT_WHEEL_ID]) *
        //                 WHEEL_RADIUS / 2;
        // data.fdata[5] = (balance_infantry.sensors_info.wheel_pos[LEFT_WHEEL_ID] -
        //                  balance_infantry.sensors_info.wheel_pos[RIGHT_WHEEL_ID]) *
        //                     WHEEL_RADIUS / 2 -
        //                 balance_infantry.state_vector[STATE_X];

        // 起跳测试
        // data.fdata[0] = balance_infantry.target_L;
        // data.fdata[1] = balance_infantry.L_left[0];
        // data.fdata[2] = balance_infantry.extra_leg_force_l;
        // data.fdata[3] = balance_infantry.FN_l;
        // data.fdata[4] = balance_infantry.output_torque_l[1];
        // data.fdata[5] = balance_infantry.FN_r;

        // 双腿协同测试
        // data.fdata[0] = (balance_infantry.theta_left - balance_infantry.theta_right) * 100.0f;
        // data.fdata[1] = balance_infantry.target_yaw_v;
        // data.fdata[2] = balance_infantry.yaw_v;
        // data.fdata[3] = balance_infantry.set_yaw_v;
        // data.fdata[4] = balance_infantry.target_pid_yaw_v;
        // data.fdata[5] = balance_infantry.theta_right * 100.0f;

        // 电容功能测试
        data.fdata[0] = INA260_1.Power / 1000.0f;
        data.fdata[1] = referee_data.Power_Heat_Data.chassis_power;
        data.fdata[2] = buffer_energy.buffering_energy;
        data.fdata[3] = super_power.actual_vol;
        data.fdata[4] = (INA260_1.Power - INA260_2.Power) / 1000.0f;
        data.fdata[5] = balance_infantry.state_vector[STATE_X_V];
        // // data.fdata[5] = remote_controller.super_power_state;
        // // data.fdata[5] = super_power.actual_vol * super_power.i_set / ((INA260_1.Power - INA260_2.Power) / 1000.0f);

        // yaw_v
        // data.fdata[0] = balance_infantry.target_yaw_v;
        // data.fdata[1] = balance_infantry.yaw_v;
        // data.fdata[2] = balance_infantry.set_yaw_v;
        // data.fdata[3] = buffer_energy.buffering_energy;
        // data.fdata[4] = referee_data.Power_Heat_Data.chassis_power;

        BLUE_TOOTHSendData(&data);

        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
    }
}

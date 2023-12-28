/**
 ******************************************************************************
 * @file    Judge.c
 * @brief   云台数据接收
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "GimbalReceive.h"
#include "ChasisController.h"

GimbalReceivePack1 gimbal_receiver_pack1;
GimbalReceivePack2 gimbal_receiver_pack2;
int8_t gimbal_receive_1_update; // 更新标志，说明收到了一帧消息
int8_t gimbal_receive_2_update;
int8_t gimbal_receive_3_update;

void ChassisControlSelect(enum CONTROL_MODE_ACTION control_mode_action)
{
    static volatile uint32_t mode_change_start_time;
    // 根据当前模式来选择如何进行状态机切换
    switch (remote_controller.control_mode_action)
    {
    case NOT_CONTROL_MODE:
        // 如果进入的是移动模式，那么将
        if (control_mode_action == FELL_RUN_MODE && offline_detector.is_sensor_off != TRUE)
        {
            mode_change_start_time = COUNTER_TIMx->CNT;
            setRobotState(CONTROL_MODE);
            setControlModeAction(FELL_RUN_MODE);
        }
        // 上一个模式不是控制模式，即掉电模式，则先收腿
        else if (control_mode_action != NOT_CONTROL_MODE && offline_detector.is_sensor_off != TRUE && offline_detector.is_motor_error != TRUE)
        {
            mode_change_start_time = COUNTER_TIMx->CNT;
            setRobotState(CONTROL_MODE);
            setControlModeAction(SHRINK_LEG_MODE);
        }
        else if (control_mode_action != NOT_CONTROL_MODE && offline_detector.is_sensor_off != TRUE && offline_detector.is_motor_error == TRUE)
        {
            mode_change_start_time = COUNTER_TIMx->CNT;
            setRobotState(CONTROL_MODE);
            setControlModeAction(RECOVER);
        }
        else
        {
            setRobotState(OFFLINE_MODE);
        }
        // else if (control_mode_action == SHRINK_LEG_MODE)
        // {
        //     mode_change_start_time = COUNTER_TIMx->CNT;
        //     setControlModeAction(SHRINK_LEG_MODE);
        // }
        break;
    case FELL_RUN_MODE:
        if (control_mode_action != NOT_CONTROL_MODE && control_mode_action != FELL_RUN_MODE)
        {
            mode_change_start_time = COUNTER_TIMx->CNT;
            setRobotState(CONTROL_MODE);
            setControlModeAction(SHRINK_LEG_MODE);
        }
        if (control_mode_action == NOT_CONTROL_MODE)
        {
            setRobotState(OFFLINE_MODE);
            setControlModeAction(NOT_CONTROL_MODE);
        }
        break;
    case SHRINK_LEG_MODE:
        if (balance_infantry.is_shrink_finish_2 && buffer_energy.buffering_energy > 55.0f) // 收腿条件判断，不会长期处于该模式
        {
            super_power.power_limit_state = POWER_LIMIT_BAT;
            mode_change_start_time = COUNTER_TIMx->CNT;
            setControlModeAction(BALANCE);
        }
        if (control_mode_action == NOT_CONTROL_MODE)
        {
            setRobotState(OFFLINE_MODE);
            setControlModeAction(NOT_CONTROL_MODE);
        }

        break;
    case RECOVER:
        // if (GetDeltaTtoNow(&mode_change_start_time) > 1.0f) // 自救模式下自救完成的判断
        // {
        //     mode_change_start_time = COUNTER_TIMx->CNT;
        //     setControlModeAction(BALANCE);
        // }
        if (control_mode_action != NOT_CONTROL_MODE && offline_detector.is_motor_error != TRUE)
        {
            mode_change_start_time = COUNTER_TIMx->CNT;
            setRobotState(CONTROL_MODE);
            setControlModeAction(SHRINK_LEG_MODE);
        }
        if (control_mode_action == NOT_CONTROL_MODE)
        {
            setRobotState(OFFLINE_MODE);
            setControlModeAction(NOT_CONTROL_MODE);
        }
        break;
    case BALANCE:
    case FOLLOW_GIMBAL:
    case FOLLOW_BESIDES:
    case SELF_ROTATE:
        if (control_mode_action != SHRINK_LEG_MODE && control_mode_action != RECOVER && control_mode_action != SLIDDED_MODE && control_mode_action != JUMP_PRE_MODE) // 这些状态不能直接切回到收腿态或者自救态，只能相互转换或者转到掉电态
        {
            setControlModeAction(control_mode_action);
        }

        // 起跳 PID初始化
        if (control_mode_action == JUMP_PRE_MODE)
        {
            mode_change_start_time = COUNTER_TIMx->CNT;

            InitJumpPrePid(&balance_infantry);
            setControlModeAction(control_mode_action);
        }
        if (!balance_infantry.is_get_enough_force && GetDeltaTtoNow(&mode_change_start_time) > 1.0f)
        {
            mode_change_start_time = COUNTER_TIMx->CNT;
            InitFlyPid(&balance_infantry);

            setControlModeAction(FLY_MODE);
        }
        if (balance_infantry.is_abnormal || super_power.power_limit_state == POWER_LIMIT_BAT_ERROR || offline_detector.is_sensor_off == TRUE || offline_detector.is_motor_error == TRUE)
        {
            setRobotState(OFFLINE_MODE);
            setControlModeAction(NOT_CONTROL_MODE); // 归到最初状态
            vTaskDelay(200);                        // 适当延时，保证最后力矩为零
        }
        // else if (balance_infantry.is_slidded)
        // {
        //     mode_change_start_time = COUNTER_TIMx->CNT;
        //     setControlModeAction(SLIDDED_MODE);
        // }

        break;
    // case SLIDDED_MODE:
    //     if (GetDeltaTtoNow(&mode_change_start_time) > 2.0f)
    //     {
    //         mode_change_start_time = COUNTER_TIMx->CNT;
    //         balance_infantry.is_slidded = FALSE;
    //         setControlModeAction(control_mode_action);
    //     }
    //     if (control_mode_action == NOT_CONTROL_MODE)
    //     {
    //         setControlModeAction(NOT_CONTROL_MODE);
    //     }
    //     break;
    case FLY_MODE:
        if (balance_infantry.is_get_enough_force)
        {
            mode_change_start_time = COUNTER_TIMx->CNT;
            setControlModeAction(LAND_MODE);

            // 初始化落地PID
            InitLandPid(&balance_infantry);
        }
        if (control_mode_action == NOT_CONTROL_MODE)
        {
            setRobotState(OFFLINE_MODE);
            setControlModeAction(NOT_CONTROL_MODE);
        }
        if (balance_infantry.is_abnormal || super_power.power_limit_state == POWER_LIMIT_BAT_ERROR || offline_detector.is_sensor_off == TRUE || offline_detector.is_motor_error == TRUE)
        {
            setRobotState(OFFLINE_MODE);
            setControlModeAction(NOT_CONTROL_MODE); // 归到最初状态
            vTaskDelay(200);                        // 适当延时，保证最后力矩为零
        }
        break;
    case LAND_MODE:
        if (GetDeltaTtoNow(&mode_change_start_time) > 1.0f)
        {
            mode_change_start_time = COUNTER_TIMx->CNT;
            setControlModeAction(control_mode_action);

            InitRawLegPid(&balance_infantry);
        }
        if (control_mode_action == NOT_CONTROL_MODE)
        {
            setRobotState(OFFLINE_MODE);
            setControlModeAction(NOT_CONTROL_MODE);
        }
        if (balance_infantry.is_abnormal || super_power.power_limit_state == POWER_LIMIT_BAT_ERROR || offline_detector.is_sensor_off == TRUE || offline_detector.is_motor_error == TRUE)
        {
            setRobotState(OFFLINE_MODE);
            setControlModeAction(NOT_CONTROL_MODE); // 归到最初状态
            vTaskDelay(200);                        // 适当延时，保证最后力矩为零，防止大冲击
        }

        break;
    case JUMP_PRE_MODE:
        if (GetDeltaTtoNow(&mode_change_start_time) > 0.20f)
        {
            mode_change_start_time = COUNTER_TIMx->CNT;
            setControlModeAction(JUMP_UP_MODE);

            InitJumpUpPid(&balance_infantry);
            // setControlModeAction(FOLLOW_GIMBAL);
            // InitRawLegPid(&balance_infantry);
        }
        if (control_mode_action == NOT_CONTROL_MODE)
        {
            setRobotState(OFFLINE_MODE);
            setControlModeAction(NOT_CONTROL_MODE);
        }
        if (balance_infantry.is_abnormal || super_power.power_limit_state == POWER_LIMIT_BAT_ERROR || offline_detector.is_sensor_off == TRUE || offline_detector.is_motor_error == TRUE)
        {
            setRobotState(OFFLINE_MODE);
            setControlModeAction(NOT_CONTROL_MODE); // 归到最初状态
            vTaskDelay(200);                        // 适当延时，保证最后力矩为零
        }
        break;
    case JUMP_UP_MODE:
        if (!balance_infantry.is_get_enough_force)
        {
            mode_change_start_time = COUNTER_TIMx->CNT;
            setControlModeAction(JUMP_DOWN_MODE);

            InitJumpDownPid(&balance_infantry);
        }
        if (control_mode_action == NOT_CONTROL_MODE)
        {
            setRobotState(OFFLINE_MODE);
            setControlModeAction(NOT_CONTROL_MODE);
        }
        if (balance_infantry.is_abnormal || super_power.power_limit_state == POWER_LIMIT_BAT_ERROR || offline_detector.is_sensor_off == TRUE || offline_detector.is_motor_error == TRUE)
        {
            setRobotState(OFFLINE_MODE);
            setControlModeAction(NOT_CONTROL_MODE); // 归到最初状态
            vTaskDelay(200);                        // 适当延时，保证最后力矩为零
        }
        break;
    case JUMP_DOWN_MODE:
        if (balance_infantry.is_get_enough_force)
        {
            mode_change_start_time = COUNTER_TIMx->CNT;
            setControlModeAction(LAND_MODE);

            // 初始化落地PID
            InitLandPid(&balance_infantry);
        }
        if (control_mode_action == NOT_CONTROL_MODE)
        {
            setRobotState(OFFLINE_MODE);
            setControlModeAction(NOT_CONTROL_MODE);
        }
        if (balance_infantry.is_abnormal || super_power.power_limit_state == POWER_LIMIT_BAT_ERROR || offline_detector.is_sensor_off == TRUE || offline_detector.is_motor_error == TRUE)
        {
            setRobotState(OFFLINE_MODE);
            setControlModeAction(NOT_CONTROL_MODE); // 归到最初状态
            vTaskDelay(200);                        // 适当延时，保证最后力矩为零
        }
        break;
    default:
        setRobotState(OFFLINE_MODE);
        setControlModeAction(NOT_CONTROL_MODE);
        break;
    }
}

void Gimbal_msgs_Decode1()
{
    //    enum ROBOT_STATE robot_state = (enum ROBOT_STATE)gimbal_receiver_pack1.robot_state;
    enum CONTROL_TYPE contro_type = (enum CONTROL_TYPE)gimbal_receiver_pack1.control_type;
    enum CONTROL_MODE_ACTION control_mode_action = (enum CONTROL_MODE_ACTION)gimbal_receiver_pack1.control_mode_action;
    enum GIMBAL_ACTION gimbal_action = (enum GIMBAL_ACTION)gimbal_receiver_pack1.gimbal_mode;
    enum SHOOT_ACTION shoot_action = (enum SHOOT_ACTION)gimbal_receiver_pack1.shoot_mode;
    enum LEG_ACTION leg_action = (enum LEG_ACTION)gimbal_receiver_pack1.leg_mode;

    // setRobotState(robot_state);
    setControlMode(contro_type);
    // 若底盘正常，则接收来自云台的控制指令，否则由底盘自己进行调整处理

    ChassisControlSelect(control_mode_action);

    setGimbalAction(gimbal_action);
    setShootAction(shoot_action);
    setLegAction(leg_action);

    balance_infantry.target_yaw_v = gimbal_receiver_pack1.robot_speed_w / 10.0f;
    balance_infantry.target_y_v = gimbal_receiver_pack1.robot_speed_y / 10.0f;
    balance_infantry.target_x_v = gimbal_receiver_pack1.robot_speed_x / 10.0f;
}

void Gimbal_msgs_Decode2()
{
    enum PowerControlState power_state = (enum PowerControlState)gimbal_receiver_pack2.super_power;

    setSuperPower(power_state);

    // balance_infantry.target_vector[STATE_X] += (gimbal_receiver_pack2.right_ch_ud_value - CH_MIDDLE) * SPEED_MAX / CH_RANGE * 0.001f;

    // if (gimbal_receive_2_update)
    // {
    //        if (remote_controller.control_mode_action == BALANCE)
    //        {
    // balance_infantry.err_angle = angle_z_err_get(gimbal_receiver_pack2.yaw_motor_angle / 22.755555556f);
    // balance_infantry.target_yaw -= balance_infantry.err_angle * GIMBAL_FOLLOW_K;
    // balance_infantry.target_vector[STATE_X] += (gimbal_receiver_pack2.right_ch_ud_value - CH_MIDDLE) * SPEED_MAX / CH_RANGE * 0.008f * balance_infantry.chassis_direction;
    //        }

    //     gimbal_receive_2_update = FALSE; //清空标记
    // }
}

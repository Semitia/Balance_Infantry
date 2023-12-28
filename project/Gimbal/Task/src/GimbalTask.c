#include "GimbalTask.h"

int8_t dji_motors_send_data[8];
int8_t dji_motors_send_data_2[8];
float testl;
SI_t SI_obeject;
SawToothWave saw_tooth_wave;

int8_t auto_aim_init_flag;

void Gimbal_Powerdown_Cal()
{
    updateGyro();
    GM6020_Decode(&gimbal_controller.pitch_recv, &gimbal_controller.pitch_info);
    GM6020_Decode(&gimbal_controller.yaw_recv, &gimbal_controller.yaw_info);

    limitPitchAngle();
    GimbalClear();

    GM6020_SendPack(dji_motors_send_data, GM6020_STD_ID_1_4, PITCH_MOTOR_CAN_ID - 0x204, 0);
    GM6020_SendPack(dji_motors_send_data_2, GM6020_STD_ID_1_4, YAW_MOTOR_CAN_ID - 0x204, 0);

    // SawToothInit(&saw_tooth_wave, 16.0f, 10, 200, gimbal_controller.gyro_yaw_angle);
}

void Gimbal_Autoaim_Cal()
{
    //更新传感器信息
    updateGyro();
    GM6020_Decode(&gimbal_controller.pitch_recv, &gimbal_controller.pitch_info);
    GM6020_Decode(&gimbal_controller.yaw_recv, &gimbal_controller.yaw_info);

    // && GetDeltaTtoNow(&global_debugger.pc_receive_debugger.last_can_cnt) < 0.3f

    //设置目标角度
    if (pc_recv_data.enemy_id != 0 && fabsf(gimbal_controller.target_pitch_angle - pc_recv_data.pitch / 100.0f) < 60.0f && fabsf(gimbal_controller.target_yaw_angle - pc_recv_data.yaw) < 70.0f)
    {
        gimbal_controller.target_pitch_angle = pc_recv_data.pitch / 100.0f;
        gimbal_controller.target_yaw_angle = pc_recv_data.yaw;
    }
    pc_send_data.mode = AUTO_AIM; //改变发送的模式

    // pitch限制幅值
    limitPitchAngle();
    Gimbal_Pitch_Calculate(gimbal_controller.target_pitch_angle);
    GM6020_SendPack(dji_motors_send_data, GM6020_STD_ID_1_4, PITCH_MOTOR_CAN_ID - 0x204, (int16_t)gimbal_controller.set_pitch_vol);

    // yaw计算
    Gimbal_Yaw_Calculate(gimbal_controller.target_yaw_angle);
    GM6020_SendPack(dji_motors_send_data_2, GM6020_STD_ID_1_4, YAW_MOTOR_CAN_ID - 0x204, (int16_t)gimbal_controller.set_yaw_vol);
}

void Gimbal_Small_Buff_Cal()
{
    //更新传感器信息
    updateGyro();
    GM6020_Decode(&gimbal_controller.pitch_recv, &gimbal_controller.pitch_info);
    GM6020_Decode(&gimbal_controller.yaw_recv, &gimbal_controller.yaw_info);

    //设置目标角度
    if (pc_recv_data.enemy_id != 0 && fabsf(gimbal_controller.target_pitch_angle - pc_recv_data.pitch / 100.0f) < 60.0f && fabsf(gimbal_controller.target_yaw_angle - pc_recv_data.yaw) < 70.0f)
    {
        gimbal_controller.target_pitch_angle = pc_recv_data.pitch / 100.0f;
        gimbal_controller.target_yaw_angle = pc_recv_data.yaw;
    }
    pc_send_data.mode = SMALL_BUFF; //改变发送的模式

    // pitch限制幅值
    limitPitchAngle();
    Gimbal_Pitch_Calculate(gimbal_controller.target_pitch_angle);
    GM6020_SendPack(dji_motors_send_data, GM6020_STD_ID_1_4, PITCH_MOTOR_CAN_ID - 0x204, (int16_t)gimbal_controller.set_pitch_vol);

    // yaw计算
    Gimbal_Yaw_Calculate(gimbal_controller.target_yaw_angle);
    GM6020_SendPack(dji_motors_send_data_2, GM6020_STD_ID_1_4, YAW_MOTOR_CAN_ID - 0x204, (int16_t)gimbal_controller.set_yaw_vol);
}

void Gimbal_Big_Buff_Cal()
{
    //更新传感器信息
    updateGyro();
    GM6020_Decode(&gimbal_controller.pitch_recv, &gimbal_controller.pitch_info);
    GM6020_Decode(&gimbal_controller.yaw_recv, &gimbal_controller.yaw_info);

    //设置目标角度
    if (pc_recv_data.enemy_id != 0 && fabsf(gimbal_controller.target_pitch_angle - pc_recv_data.pitch / 100.0f) < 60.0f && fabsf(gimbal_controller.target_yaw_angle - pc_recv_data.yaw) < 70.0f)
    {
        gimbal_controller.target_pitch_angle = pc_recv_data.pitch / 100.0f;
        gimbal_controller.target_yaw_angle = pc_recv_data.yaw;
    }
    pc_send_data.mode = BIG_BUFF; //改变发送的模式

    // pitch限制幅值
    limitPitchAngle();
    Gimbal_Pitch_Calculate(gimbal_controller.target_pitch_angle);
    GM6020_SendPack(dji_motors_send_data, GM6020_STD_ID_1_4, PITCH_MOTOR_CAN_ID - 0x204, (int16_t)gimbal_controller.set_pitch_vol);

    // yaw计算
    Gimbal_Yaw_Calculate(gimbal_controller.target_yaw_angle);
    GM6020_SendPack(dji_motors_send_data_2, GM6020_STD_ID_1_4, YAW_MOTOR_CAN_ID - 0x204, (int16_t)gimbal_controller.set_yaw_vol);
}

void Gimbal_SI_Cal()
{
    //更新传感器信息
    updateGyro();
    GM6020_Decode(&gimbal_controller.pitch_recv, &gimbal_controller.pitch_info);
    GM6020_Decode(&gimbal_controller.yaw_recv, &gimbal_controller.yaw_info);

    // yaw计算
    Gimbal_Speed_Calculate(SIRun(&SI_obeject, gimbal_controller.delta_t));
    GM6020_SendPack(dji_motors_send_data_2, GM6020_STD_ID_1_4, YAW_MOTOR_CAN_ID - 0x204, (int16_t)gimbal_controller.set_yaw_vol);
}

void Gimbal_Act_Cal()
{
    //更新传感器信息
    updateGyro();
    GM6020_Decode(&gimbal_controller.pitch_recv, &gimbal_controller.pitch_info);
    GM6020_Decode(&gimbal_controller.yaw_recv, &gimbal_controller.yaw_info);

    pc_send_data.mode = 0; //改变发送的模式

    // pitch限制幅值
    limitPitchAngle();
    Gimbal_Pitch_Calculate(gimbal_controller.target_pitch_angle);
    GM6020_SendPack(dji_motors_send_data, GM6020_STD_ID_1_4, PITCH_MOTOR_CAN_ID - 0x204, (int16_t)gimbal_controller.set_pitch_vol);

    // yaw计算
    Gimbal_Yaw_Calculate(gimbal_controller.target_yaw_angle);
    // Gimbal_Speed_Calculate(90.0f);
    GM6020_SendPack(dji_motors_send_data_2, GM6020_STD_ID_1_4, YAW_MOTOR_CAN_ID - 0x204, (int16_t)gimbal_controller.set_yaw_vol);
}

void Gimbal_Test_Cal()
{
    //更新传感器信息
    updateGyro();
    GM6020_Decode(&gimbal_controller.pitch_recv, &gimbal_controller.pitch_info);
    GM6020_Decode(&gimbal_controller.yaw_recv, &gimbal_controller.yaw_info);

    // pitch限制幅值
    // limitPitchAngle();
    // Gimbal_Pitch_Calculate(gimbal_controller.target_pitch_angle);
    // GM6020_SendPack(dji_motors_send_data, GM6020_STD_ID_1_4, PITCH_MOTOR_CAN_ID - 0x204, (int16_t)gimbal_controller.set_pitch_vol);

    // yaw计算
    Gimbal_Yaw_Calculate(SawWaveRun(&saw_tooth_wave, gimbal_controller.delta_t));
    // Gimbal_Speed_Calculate(90.0f);
    GM6020_SendPack(dji_motors_send_data_2, GM6020_STD_ID_1_4, YAW_MOTOR_CAN_ID - 0x204, (int16_t)gimbal_controller.set_yaw_vol);
}

// int test_bombbay_pos = 1000;
void Shoot_Powerdown_Cal()
{
    //弹舱盖
    BOMB_BAY_OFF;
    // BombBay_Set(test_bombbay_pos);

    //摩擦轮
    for (int i = 0; i < 2; i++)
    {
        M3508_Decode(&friction_wheels.friction_motor_recv[i], &friction_wheels.friction_motor_msgs[i], ONLY_SPEED, 0.90);
    }

    if (friction_wheels.friction_motor_msgs[LEFT_FRICTION_WHEEL].speed > 6000 || friction_wheels.friction_motor_msgs[RIGHT_FRICTION_WHEEL].speed > 6000)
    {
        FrictionWheel_Set(0, 0);
        M3508_SendPack(dji_motors_send_data, C620_STD_ID_5_8, FRICTION_WHEEL_CAN_ID_1 - 0x200, friction_wheels.send_to_motor_current[LEFT_FRICTION_WHEEL], SEND_CURRENT);
        M3508_SendPack(dji_motors_send_data, C620_STD_ID_5_8, FRICTION_WHEEL_CAN_ID_2 - 0x200, friction_wheels.send_to_motor_current[RIGHT_FRICTION_WHEEL], SEND_CURRENT);
    }
    else
    {
        PID_Clear(&friction_wheels.PidFrictionSpeed[LEFT_FRICTION_WHEEL]);
        PID_Clear(&friction_wheels.PidFrictionSpeed[RIGHT_FRICTION_WHEEL]);

        friction_wheels.send_to_motor_current[LEFT_FRICTION_WHEEL] = 0;
        friction_wheels.send_to_motor_current[RIGHT_FRICTION_WHEEL] = 0;
        M3508_SendPack(dji_motors_send_data, C620_STD_ID_5_8, FRICTION_WHEEL_CAN_ID_1 - 0x200, friction_wheels.send_to_motor_current[LEFT_FRICTION_WHEEL], SEND_CURRENT);
        M3508_SendPack(dji_motors_send_data, C620_STD_ID_5_8, FRICTION_WHEEL_CAN_ID_2 - 0x200, friction_wheels.send_to_motor_current[RIGHT_FRICTION_WHEEL], SEND_CURRENT);
    }

    //拨盘电机
    M2006_Decode(&toggle_controller.toggle_recv, &toggle_controller.toggle_info, WITH_REDUCTION, 0.80);
    float send_current = Toggle_Calculate(TOGGLE_STOP, 0.0f);
    M2006_SendPack(dji_motors_send_data, C610_STD_ID_5_8, TOGGLE_MOTOR_CAN_ID - 0x200, send_current);
}

void Shoot_Check_Cal()
{
    //弹舱盖

    //拨盘

    //摩擦轮
}

void Shoot_Fire_Cal()
{
    BOMB_BAY_OFF;

    // 摩擦轮
    for (int i = 0; i < 2; i++)
    {
        M3508_Decode(&friction_wheels.friction_motor_recv[i], &friction_wheels.friction_motor_msgs[i], ONLY_SPEED, 0.90);
    }
    setFrictionSpeed(chassis_pack_get_1.bullet_level);
    FrictionWheel_Set(-friction_wheels.set_speed_l, friction_wheels.set_speed_r);
    M3508_SendPack(dji_motors_send_data, C620_STD_ID_5_8, FRICTION_WHEEL_CAN_ID_1 - 0x200, friction_wheels.send_to_motor_current[LEFT_FRICTION_WHEEL], SEND_CURRENT);
    M3508_SendPack(dji_motors_send_data, C620_STD_ID_5_8, FRICTION_WHEEL_CAN_ID_2 - 0x200, friction_wheels.send_to_motor_current[RIGHT_FRICTION_WHEEL], SEND_CURRENT);

    // 拨盘
    M2006_Decode(&toggle_controller.toggle_recv, &toggle_controller.toggle_info, WITH_REDUCTION, 0.95);

    //设置拨盘转动速度
    selectShootFreq(chassis_pack_get_1.robot_level, chassis_pack_get_1.buff_state);

    // PID计算

    float send_current;
    if (chassis_pack_get_1.is_shootable && toggle_controller.is_shoot && remote_controller.gimbal_action != GIMBAL_SMALL_BUFF_MODE && remote_controller.gimbal_action != GIMBAL_BIG_BUFF_MODE)
    {
        remote_controller.single_shoot_flag = FALSE; //连发情况也将单发标志位清零
        send_current = Toggle_Calculate(TOGGLE_SPEED, SIGN_ROTATE * toggle_controller.shoot_freq_speed);
    }
    else if (remote_controller.gimbal_action == GIMBAL_SMALL_BUFF_MODE || remote_controller.gimbal_action == GIMBAL_BIG_BUFF_MODE)
    {
        //大符模式，采用单发模式
        if (remote_controller.single_shoot_flag && chassis_pack_get_1.is_shootable) //触发单发射击标志
        {
            ToggleAddGrid(&toggle_controller.set_pos, 1);
            remote_controller.single_shoot_flag = FALSE;
        }
        send_current = Toggle_Calculate(TOGGLE_POS, toggle_controller.set_pos);
    }
    else
    {
        remote_controller.single_shoot_flag = FALSE; //不打弹情况将打击标志位清零
        send_current = Toggle_Calculate(TOGGLE_SPEED, 0.0f);
    }

    // 打包数据

    M2006_SendPack(dji_motors_send_data, C610_STD_ID_5_8, TOGGLE_MOTOR_CAN_ID - 0x200, send_current);
}

void Shoot_Test_Cal()
{
    BOMB_BAY_ON;

    // 摩擦轮
    for (int i = 0; i < 2; i++)
    {
        M3508_Decode(&friction_wheels.friction_motor_recv[i], &friction_wheels.friction_motor_msgs[i], ONLY_SPEED, 0.90);
    }

    setFrictionSpeed(chassis_pack_get_1.bullet_level);
    FrictionWheel_Set(-friction_wheels.set_speed_l, friction_wheels.set_speed_r);
    M3508_SendPack(dji_motors_send_data, C620_STD_ID_5_8, FRICTION_WHEEL_CAN_ID_1 - 0x200, friction_wheels.send_to_motor_current[LEFT_FRICTION_WHEEL], SEND_CURRENT);
    M3508_SendPack(dji_motors_send_data, C620_STD_ID_5_8, FRICTION_WHEEL_CAN_ID_2 - 0x200, friction_wheels.send_to_motor_current[RIGHT_FRICTION_WHEEL], SEND_CURRENT);

    // 拨盘
    M2006_Decode(&toggle_controller.toggle_recv, &toggle_controller.toggle_info, WITH_REDUCTION, 0.95);

    float send_current;

    send_current = Toggle_Calculate(TOGGLE_SPEED, -170.0f);

    // 打包数据
    M2006_SendPack(dji_motors_send_data, C610_STD_ID_5_8, TOGGLE_MOTOR_CAN_ID - 0x200, send_current);
}

void Shoot_Autoaim_Cal()
{
    // 弹舱盖

    // 拨盘

    // 摩擦轮
}

void Shoot_Supply_Cal()
{
    BOMB_BAY_ON;

    // 摩擦轮
    for (int i = 0; i < 2; i++)
    {
        M3508_Decode(&friction_wheels.friction_motor_recv[i], &friction_wheels.friction_motor_msgs[i], ONLY_SPEED, 0.90);
    }

    FrictionWheel_Set(0, 0);
    M3508_SendPack(dji_motors_send_data, C620_STD_ID_5_8, FRICTION_WHEEL_CAN_ID_1 - 0x200, friction_wheels.send_to_motor_current[LEFT_FRICTION_WHEEL], SEND_CURRENT);
    M3508_SendPack(dji_motors_send_data, C620_STD_ID_5_8, FRICTION_WHEEL_CAN_ID_2 - 0x200, friction_wheels.send_to_motor_current[RIGHT_FRICTION_WHEEL], SEND_CURRENT);

    // 拨盘
    M2006_Decode(&toggle_controller.toggle_recv, &toggle_controller.toggle_info, WITH_REDUCTION, 0.90);

    float send_current;

    send_current = Toggle_Calculate(TOGGLE_SPEED, 0.0f);

    // 打包数据
    M2006_SendPack(dji_motors_send_data, C610_STD_ID_5_8, TOGGLE_MOTOR_CAN_ID - 0x200, send_current);
}

/**
 * @brief 云台控制任务
 * @param[in] void
 */
void GimbalTask(void *pvParameters)
{
    portTickType xLastWakeTime;
    const portTickType xFrequency = 2; // 1kHZ

    memset(dji_motors_send_data, 0, 8);
    memset(dji_motors_send_data_2, 0, 8);
    FrictionWheel_Init();
    GimbalPidInit();
    TogglePidInit();

    /* 系统辨识以及测试 */
    SIInit(&SI_obeject, 10, 160.0f);
    // SawToothInit(&saw_tooth_wave, 50, 10, 1000, 20);

    vTaskDelay(2000);

    int index = 0;

    while (1)
    {
        xLastWakeTime = xTaskGetTickCount();

        switch (remote_controller.gimbal_action)
        {
        case GIMBAL_POWERDOWN: //掉电模式
            Gimbal_Powerdown_Cal();
            break;
        case GIMBAL_ACT_MODE: //云台运动模式
            Gimbal_Act_Cal();
            break;
        case GIMBAL_AUTO_AIM_MODE: //自瞄模式
            Gimbal_Autoaim_Cal();
            break;
        case GIMBAL_SMALL_BUFF_MODE:
            Gimbal_Small_Buff_Cal();
            break;
        case GIMBAL_BIG_BUFF_MODE:
            Gimbal_Big_Buff_Cal();
            break;
        case GIMBAL_SI_MODE:
            Gimbal_SI_Cal();
            break;
        case GIMBAL_TEST_MODE:
            Gimbal_Test_Cal();
            break;
        default:
            Gimbal_Powerdown_Cal();
            break;
        }

        switch (remote_controller.shoot_action)
        {
        case SHOOT_POWERDOWN_MODE: //掉电模式
            Shoot_Powerdown_Cal();
            break;
        case SHOOT_CHECK_MODE: //自检模式
            Shoot_Check_Cal();
            break;
        case SHOOT_FIRE_MODE: //开火模式
            Shoot_Fire_Cal();
            break;
        case SHOOT_TEST_MODE: //弹道测试模式
            Shoot_Test_Cal();
            break;
        case SHOOT_AUTO_AIM_MODE: //自瞄模式
            Shoot_Autoaim_Cal();
            break;
        case SHOOT_SUPPLY_MODE: //补给模式
            Shoot_Supply_Cal();
            break;
        default:
            Shoot_Powerdown_Cal();
            break;
        }

        CanSend(DJI_MOTORS_CAN, dji_motors_send_data, C620_STD_ID_5_8, 8);

        CanSend(CHASSIS_CAN_COMM_CANx, dji_motors_send_data_2, GM6020_STD_ID_1_4, 8);

        if (index % 2 == 0)
        {
            SendtoPC(); //将信息发送给上位机
        }
        index++;
        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

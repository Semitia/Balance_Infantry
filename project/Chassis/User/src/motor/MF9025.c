#include "MF9025.h"

MF9025 Wheels_Motor;

void limitWheelTorque(float *wheels)
{
    for (size_t i = 0; i < 2; i++)
    {
        wheels[i] = LIMIT_MAX_MIN(wheels[i], MAX_WHEEL_TORQUE, MIN_WHEEL_TORQUE);
    }
}

void MF9025_Decode(MF9025_Info *mf9025_info, MF9025_Recv *mf9025_recv)
{
    mf9025_info->temperature = mf9025_recv->temperature;
    float pos = mf9025_recv->pos * MF9025_POS_RATIO;
    mf9025_info->pos = ZeroCheck(&mf9025_info->pos_zero_check, pos, 360.0f);
    mf9025_info->rotate_speed = mf9025_recv->rotate_speed * MF9025_SPEED_RATIO;
    mf9025_info->current = mf9025_recv->current * MF9025_CURRENT_RATIO;
}

void WheelOffPack(MF9025_Broadcast *wheels)
{
    for (int8_t i = 0; i < 4; i++)
    {
        wheels->wheel_current[i] = 0;
    }
}

void WheelCurrentPack(MF9025_Broadcast *wheels, float *wheel_torque)
{
    for (int8_t i = 0; i < 2; i++)
    {
        wheels->wheel_current[i] = (int16_t)(wheel_torque[i] * MF9025_TORQUE2CURRENT);
        wheels->wheel_current[i + 2] = 0;
    }
}

void WheelCanSend(MF9025_Broadcast *wheels)
{
    global_debugger.mf9025_debugger[0].send_msgs_num++;
    global_debugger.mf9025_debugger[1].send_msgs_num++;

    CanSend(WHEEL_CAN, (int8_t *)wheels->wheel_current, WHEEL_BROCAST_CAN_ID, 8);
}

// 单电机发送，默认只发送两个ID的数据
void WheelCanSingleSend(MF9025_Broadcast *wheels)
{
    int8_t senddata[8] = {0};
    global_debugger.mf9025_debugger[0].send_msgs_num++;
    global_debugger.mf9025_debugger[1].send_msgs_num++;

    senddata[0] = 0xA1;
    memcpy(&senddata[4], (int8_t *)&wheels->wheel_current[0], 2);
    CanSend(WHEEL_CAN, senddata, 0x141, 8);

    memcpy(&senddata[4], (int8_t *)&wheels->wheel_current[1], 2);
    CanSend(WHEEL_CAN, senddata, 0x142, 8);
}

// 电机保护清除错误位
void WheelClearError()
{
    int8_t senddata[8] = {0};
    senddata[0] = 0x9B; // 清除标志位

    CanSend(WHEEL_CAN, senddata, 0x141, 8);
    CanSend(WHEEL_CAN, senddata, 0x142, 8);
}

void MF9025_Init(MF9025 *mf9025)
{
    int8_t init_finish = FALSE;
    MF9025_Broadcast wheels;

    WheelOffPack(&wheels);

    while (!init_finish)
    {
        init_finish = TRUE;

        WheelCanSend(&wheels);

        delay_ms(1);

        for (int8_t i = 0; i < 2; i++)
        {
            if (global_debugger.mf9025_debugger[i].recv_msgs_num < 30)
            {
                init_finish = FALSE;
            }
        }
    }

    // 正式初始化
    for (int8_t i = 0; i < 2; i++)
    {
        mf9025->init_pos[i] = mf9025->decoded_msgs[i].pos;
    }
}

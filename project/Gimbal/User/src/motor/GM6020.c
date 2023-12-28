/**
 ******************************************************************************
 * @file    GM6020.c
 * @brief   GM6020电机数据接收以及解码
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "GM6020.h"

/**
 * @brief GM6020电机反馈数据解码
 * @param[in] GM6020_recv 原始数据
 * @param[in] GM6020_decoded 解码后的数据
 */
void GM6020_Decode(GM6020_Recv *GM6020_recv, GM6020_Info *GM6020_decoded)
{
    //位置和速度转到输出轴
    float angle = GM6020_recv->angle * GM6020_ANGLE_RATIO;
    GM6020_decoded->speed = GM6020_recv->speed * GM6020_SPEED_RATIO;
    GM6020_decoded->angle = ZeroCheck(&GM6020_decoded->angle_zero_check, angle, 360.0f);
    //由于2006电机无电流反馈单位，故这里的单位不是A
    iir(&GM6020_decoded->torque_current, (float)GM6020_recv->torque_current, 0.70);
}

/**
 * @brief 封装6020电机数据
 * @param[in] can_data 需要发送的CAN数据
 * @param[in] std_id 发送的标准帧ID,0x200 , 0x1ff
 * @param[in] controller_id 电调ID,1,2,3,4,5,6,7,8
 * @param[in] control_current 控制电流
 */
void GM6020_SendPack(int8_t *can_data, uint32_t std_id, uint8_t motor_id, int16_t control_vol)
{
    control_vol = LIMIT_MAX_MIN(control_vol, GM6020_MAX_VOLTAGE, GM6020_MIN_VOLTAGE);

    if (std_id == GM6020_STD_ID_1_4)
    {
        if (motor_id >= GM6020_MOTOR_ID_1 && motor_id <= GM6020_MOTOR_ID_4)
        {
            int index = (motor_id - GM6020_MOTOR_ID_1) * 2;
            can_data[index] = (int8_t)((control_vol & 0xFF00) >> 8);
            can_data[index + 1] = (int8_t)(control_vol & 0x00FF);
        }
    }
    else if (std_id == GM6020_STD_ID_5_7)
    {
        if (motor_id >= GM6020_MOTOR_ID_5 && motor_id <= GM6020_MOTOR_ID_7)
        {
            int index = (motor_id - GM6020_MOTOR_ID_5) * 2;
            can_data[index] = (int8_t)((control_vol & 0xFF00) >> 8);
            can_data[index + 1] = (int8_t)(control_vol & 0x00FF);
        }
    }
}

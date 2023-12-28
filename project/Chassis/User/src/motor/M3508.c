/**
 ******************************************************************************
 * @file    M3508.c
 * @brief   M3508电机数据接收以及解码
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "M3508.h"

/**
 * @brief 3508电机反馈数据解码
 * @param[in] M3508_recv 原始数据
 * @param[in] M3508_decoded 解码后的数据
 */
void M3508_Decode(M3508_Recv *M3508_recv, M3508_Info *M3508_decoded, int8_t decode_type, float filte_value)
{
    if (decode_type == ONLY_SPEED)
    {
        float speed = M3508_recv->speed * M3508_SPEED_RATIO;
        iir(&M3508_decoded->speed, speed, filte_value);
    }
    else if (decode_type == ONLY_SPEED_WITH_REDUCTION)
    {
        float speed = M3508_recv->speed * M3508_SPEED_RATIO / M3508_REDUCTION_RATIO;
        iir(&M3508_decoded->speed, speed, filte_value);
    }
    else if (decode_type == ALL)
    {
        //位置和速度
        float angle = M3508_recv->angle * M3508_ANGLE_RATIO;
        float speed = M3508_recv->speed * M3508_SPEED_RATIO;
        iir(&M3508_decoded->speed, speed, filte_value);
        M3508_decoded->angle = ZeroCheck(&M3508_decoded->angle_zero_check, angle, 360.0f);
        M3508_decoded->torque_current = M3508_recv->torque_current / C620_CURRENT_SEND_TRANS;
    }
    else if (decode_type == ALL_WITH_REDUCTION)
    {
        //位置和速度转到输出轴
        float angle = M3508_recv->angle * M3508_ANGLE_RATIO;
        float speed = M3508_recv->speed * M3508_SPEED_RATIO / M3508_REDUCTION_RATIO;
        iir(&M3508_decoded->speed, speed, filte_value);
        M3508_decoded->angle = ZeroCheck(&M3508_decoded->angle_zero_check, angle, 360.0f) / M3508_REDUCTION_RATIO;
        M3508_decoded->torque_current = M3508_recv->torque_current / C620_CURRENT_SEND_TRANS;
    }
}

/**
 * @brief 封装3508电机数据
 * @param[in] can_data 需要发送的CAN数据
 * @param[in] std_id 发送的标准帧ID,0x200 , 0x1ff
 * @param[in] controller_id 电调ID,1,2,3,4,5,6,7,8
 * @param[in] control_current 控制电流
 */
void M3508_SendPack(int8_t *can_data, uint32_t std_id, uint8_t motor_id, float control_current, enum SEND_TYPE send_type)
{
    int16_t current_send;

    if (send_type == SEND_CURRENT)
    {
        current_send = (int16_t)LIMIT_MAX_MIN(control_current, C620_MAX_SEND_CURRENT, C620_MIN_SEND_CURRENT);
    }
    else if (send_type == RAW_CURRENT)
    {
        float current = LIMIT_MAX_MIN(control_current, C620_MAX_CURRENT, C620_MIN_CURRENT);
        current_send = (int16_t)(current * C620_CURRENT_SEND_TRANS);
    }
    else
    {
        current_send = 0;
    }

    if (std_id == C620_STD_ID_1_4)
    {
        if (motor_id >= C620_MOTOR_ID_1 && motor_id <= C620_MOTOR_ID_4)
        {
            int index = (motor_id - C620_MOTOR_ID_1) * 2;
            can_data[index] = (int8_t)((current_send & 0xFF00) >> 8);
            can_data[index + 1] = (int8_t)(current_send & 0x00FF);
        }
    }
    else if (std_id == C620_STD_ID_5_8)
    {
        if (motor_id >= C620_MOTOR_ID_5 && motor_id <= C620_MOTOR_ID_8)
        {
            int index = (motor_id - C620_MOTOR_ID_5) * 2;
            can_data[index] = (int8_t)((current_send & 0xFF00) >> 8);
            can_data[index + 1] = (int8_t)(current_send & 0x00FF);
        }
    }
}

/**
 ******************************************************************************
 * @file    M2006.c
 * @brief   M2006电机数据接收以及解码
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "M2006.h"

/**
 * @brief 2006电机反馈数据解码，
 * @param[in] m2006_recv 原始数据
 * @param[in] m2006_decoded 解码后的数据
 * @param[in] is_reduction 是否附带减速器
 */
void M2006_Decode(M2006_Recv *m2006_recv, M2006_Info *m2006_decoded, int8_t is_reduction, float filte_value)
{
    //位置和速度转到输出轴
    float angle = m2006_recv->angle * M2006_ANGLE_RATIO;
    if (is_reduction)
    {
        float speed = m2006_recv->speed * M2006_SPEED_RATIO / M2006_REDUCTION_RATIO;
        iir(&m2006_decoded->speed, speed, filte_value);
        m2006_decoded->angle = ZeroCheck(&m2006_decoded->angle_zero_check, angle, 360.0f) / M2006_REDUCTION_RATIO;
        m2006_decoded->torque_current = m2006_recv->torque_current / C610_CURRENT_SEND_TRANS;
    }
    else //无减速器
    {
        float speed = m2006_recv->speed * M2006_SPEED_RATIO;
        iir(&m2006_decoded->speed, speed, filte_value);
        m2006_decoded->angle = ZeroCheck(&m2006_decoded->angle_zero_check, angle, 360.0f);
        m2006_decoded->torque_current = m2006_recv->torque_current / C610_CURRENT_SEND_TRANS;
    }
}

/**
 * @brief 封装2006电机数据
 * @param[in] can_data 需要发送的CAN数据
 * @param[in] std_id 发送的标准帧ID,0x200 , 0x1ff
 * @param[in] controller_id 电调ID,1,2,3,4,5,6,7,8
 * @param[in] control_current 控制电流
 */
void M2006_SendPack(int8_t *can_data, uint32_t std_id, uint8_t motor_id, float control_current)
{
    int16_t current_send;
    float current = LIMIT_MAX_MIN(control_current, C610_MAX_CURRENT, C610_MIN_CURRENT);
    current_send = (int16_t)(current * C610_CURRENT_SEND_TRANS);

    if (std_id == C610_STD_ID_1_4)
    {
        if (motor_id >= C610_MOTOR_ID_1 && motor_id <= C610_MOTOR_ID_4)
        {
            int index = (motor_id - C610_MOTOR_ID_1) * 2;
            can_data[index] = (int8_t)((current_send & 0xFF00) >> 8);
            can_data[index + 1] = (int8_t)(current_send & 0x00FF);
        }
    }
    else if (std_id == C610_STD_ID_5_8)
    {
        if (motor_id >= C610_MOTOR_ID_5 && motor_id <= C610_MOTOR_ID_8)
        {
            int index = (motor_id - C610_MOTOR_ID_5) * 2;
            can_data[index] = (int8_t)((current_send & 0xFF00) >> 8);
            can_data[index + 1] = (int8_t)(current_send & 0x00FF);
        }
    }
}

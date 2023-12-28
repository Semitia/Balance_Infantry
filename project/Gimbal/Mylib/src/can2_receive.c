/**
 ******************************************************************************
 * @file    can2_receive.c
 * @brief   CAN2 接收函数
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "can2_receive.h"

/*  接收全局数据定义 */

/*  接收函数定义 */
void Can2_FIFO0_Receive0(CanRxMsg *rx_message0);
void Can2_FIFO0_Receive1(CanRxMsg *rx_message0);
void Can2_FIFO0_Receive2(CanRxMsg *rx_message0);
void Can2_FIFO0_Receive3(CanRxMsg *rx_message0);

void Can2_FIFO1_Receive0(CanRxMsg *rx_message1);
void Can2_FIFO1_Receive1(CanRxMsg *rx_message1);
void Can2_FIFO1_Receive2(CanRxMsg *rx_message1);
void Can2_FIFO1_Receive3(CanRxMsg *rx_message1);

/**
 * @brief CAN2 FIFO0数据接收
 * @param[in] rx_message0 CAN2 FIFO0接收数据
 */
void Can2Receive0(CanRxMsg rx_message0)
{
    /*  计时 */
    switch (rx_message0.StdId)
    {
    case CAN2_FIFO0_ID0:
        Can2_FIFO0_Receive0(&rx_message0);
        break;
    case CAN2_FIFO0_ID1:
        Can2_FIFO0_Receive1(&rx_message0);
        break;
    case CAN2_FIFO0_ID2:
        Can2_FIFO0_Receive2(&rx_message0);
        break;
    case CAN2_FIFO0_ID3:
        Can2_FIFO0_Receive3(&rx_message0);
        break;
    default:
        break;
    }
}

/**
 * @brief CAN2 FIFO1数据接收
 * @param[in] rx_message1 CAN2 FIFO1接收数据
 */
void Can2Receive1(CanRxMsg rx_message1)
{
    switch (rx_message1.StdId)
    {
    case CAN2_FIFO1_ID0:
        Can2_FIFO1_Receive0(&rx_message1);
        break;
    case CAN2_FIFO1_ID1:
        Can2_FIFO1_Receive1(&rx_message1);
        break;
    case CAN2_FIFO1_ID2:
        Can2_FIFO1_Receive2(&rx_message1);
        break;
    case CAN2_FIFO1_ID3:
        Can2_FIFO1_Receive3(&rx_message1);
        break;
    default:
        break;
    }
}

/*  接收数据传递到外部 */

/**
 * @brief CAN2 FIFO0 ID0 数据接收(0x100,GYRO)
 * @param[in] rx_message1 CAN2 FIFO0接收数据
 */
void Can2_FIFO0_Receive0(CanRxMsg *rx_message0)
{
    memcpy(&IMUReceive.Gyro[0], rx_message0->Data, 2);
    memcpy(&IMUReceive.Gyro[1], &rx_message0->Data[2], 2);
    memcpy(&IMUReceive.Gyro[2], &rx_message0->Data[4], 2);
    memcpy(&IMUReceive.temp_calib, &rx_message0->Data[6], 2);

    LossUpdate(&global_debugger.imu_debugger[0], 0.0025f);
}

/**
 * @brief CAN2 FIFO0 ID1 数据接收(0x101,Accel)
 * @param[in] rx_message1 CAN2 FIFO0接收数据
 */
void Can2_FIFO0_Receive1(CanRxMsg *rx_message0)
{
    memcpy(&IMUReceive.Acc[0], rx_message0->Data, 2);
    memcpy(&IMUReceive.Acc[1], &rx_message0->Data[2], 2);
    memcpy(&IMUReceive.Acc[2], &rx_message0->Data[4], 2);

    LossUpdate(&global_debugger.imu_debugger[1], 0.0025f);
}

/**
 * @brief CAN2 FIFO0 ID2 数据接收()
 * @param[in] rx_message1 CAN2 FIFO0接收数据
 */
void Can2_FIFO0_Receive2(CanRxMsg *rx_message0)
{
    // 拨弹电机数据接收
    toggle_controller.toggle_recv.angle = (rx_message0->Data[0] << 8) | (rx_message0->Data[1]);
    toggle_controller.toggle_recv.speed = (rx_message0->Data[2] << 8) | (rx_message0->Data[3]);
    toggle_controller.toggle_recv.torque_current = (rx_message0->Data[4] << 8) | (rx_message0->Data[5]);

    LossUpdate(&global_debugger.toggle_debugger, 0.0045f);
}

/**
 * @brief CAN2 FIFO0 ID3 数据接收()
 * @param[in] rx_message1 CAN2 FIFO0接收数据
 */
void Can2_FIFO0_Receive3(CanRxMsg *rx_message0)
{
}

/**
 * @brief CAN2 FIFO1 ID0 数据接收()
 * @param[in] rx_message1 CAN2 FIFO1接收数据
 */
void Can2_FIFO1_Receive0(CanRxMsg *rx_message1)
{
    // Pitch接收
    gimbal_controller.pitch_recv.angle = (rx_message1->Data[0] << 8) | (rx_message1->Data[1]);
    gimbal_controller.pitch_recv.speed = (rx_message1->Data[2] << 8) | (rx_message1->Data[3]);
    gimbal_controller.pitch_recv.torque_current = (rx_message1->Data[4] << 8) | (rx_message1->Data[5]);
    gimbal_controller.pitch_recv.temp = rx_message1->Data[6];

    LossUpdate(&global_debugger.gimbal_debugger[0], 0.0025f);
}

/**
 * @brief CAN2 FIFO0 ID1 数据接收()
 * @param[in] rx_message1 CAN2 FIFO1接收数据
 */
void Can2_FIFO1_Receive1(CanRxMsg *rx_message1)
{
    // 左摩擦轮接收
    friction_wheels.friction_motor_recv[LEFT_FRICTION_WHEEL].angle = (rx_message1->Data[0] << 8) | (rx_message1->Data[1]);
    friction_wheels.friction_motor_recv[LEFT_FRICTION_WHEEL].speed = (rx_message1->Data[2] << 8) | (rx_message1->Data[3]);
    friction_wheels.friction_motor_recv[LEFT_FRICTION_WHEEL].torque_current = (rx_message1->Data[4] << 8) | (rx_message1->Data[5]);
    friction_wheels.friction_motor_recv[LEFT_FRICTION_WHEEL].temp = (rx_message1->Data[6]);

    LossUpdate(&global_debugger.friction_debugger[LEFT_FRICTION_WHEEL], 0.0025f);
}

/**
 * @brief CAN2 FIFO0 ID2 数据接收()
 * @param[in] rx_message1 CAN2 FIFO1接收数据
 */
void Can2_FIFO1_Receive2(CanRxMsg *rx_message1)
{
    // 右摩擦轮接收
    friction_wheels.friction_motor_recv[RIGHT_FRICTION_WHEEL].angle = (rx_message1->Data[0] << 8) | (rx_message1->Data[1]);
    friction_wheels.friction_motor_recv[RIGHT_FRICTION_WHEEL].speed = (rx_message1->Data[2] << 8) | (rx_message1->Data[3]);
    friction_wheels.friction_motor_recv[RIGHT_FRICTION_WHEEL].torque_current = (rx_message1->Data[4] << 8) | (rx_message1->Data[5]);
    friction_wheels.friction_motor_recv[RIGHT_FRICTION_WHEEL].temp = (rx_message1->Data[6]);

    LossUpdate(&global_debugger.friction_debugger[RIGHT_FRICTION_WHEEL], 0.0025f);
}

/**
 * @brief CAN2 FIFO0 ID3 数据接收()
 * @param[in] rx_message1 CAN2 FIFO1接收数据
 */
void Can2_FIFO1_Receive3(CanRxMsg *rx_message1)
{
}

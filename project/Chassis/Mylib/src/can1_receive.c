/**
 ******************************************************************************
 * @file    can1_receive.c
 * @brief   CAN1 接收函数
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "can1_receive.h"
#include "remote_control.h"

/*  接收全局数据定义 */

/*  接收函数定义 */
void Can1_FIFO0_Receive0(CanRxMsg *rx_message0);
void Can1_FIFO0_Receive1(CanRxMsg *rx_message0);
void Can1_FIFO0_Receive2(CanRxMsg *rx_message0);
void Can1_FIFO0_Receive3(CanRxMsg *rx_message0);

void Can1_FIFO1_Receive0(CanRxMsg *rx_message1);
void Can1_FIFO1_Receive1(CanRxMsg *rx_message1);
void Can1_FIFO1_Receive2(CanRxMsg *rx_message1);
void Can1_FIFO1_Receive3(CanRxMsg *rx_message1);

/**
 * @brief CAN1 FIFO0数据接收
 * @param[in] rx_message0 CAN1 FIFO0接收数据
 */
void Can1Receive0(CanRxMsg rx_message0)
{
    switch (rx_message0.StdId)
    {
    case CAN1_FIFO0_ID0:
        Can1_FIFO0_Receive0(&rx_message0);
        break;
    case CAN1_FIFO0_ID1:
        Can1_FIFO0_Receive1(&rx_message0);
        break;
    case CAN1_FIFO0_ID2:
        Can1_FIFO0_Receive2(&rx_message0);
        break;
    case CAN1_FIFO0_ID3:
        Can1_FIFO0_Receive3(&rx_message0);
        break;
    default:
        break;
    }
}

/**
 * @brief CAN1 FIFO1数据接收
 * @param[in] rx_message1 CAN1 FIFO1接收数据
 */
void Can1Receive1(CanRxMsg rx_message1)
{
    switch (rx_message1.StdId)
    {
    case CAN1_FIFO1_ID0:
        Can1_FIFO1_Receive0(&rx_message1);
        break;
    case CAN1_FIFO1_ID1:
        Can1_FIFO1_Receive1(&rx_message1);
        break;
    case CAN1_FIFO1_ID2:
        Can1_FIFO1_Receive2(&rx_message1);
        break;
    case CAN1_FIFO1_ID3:
        Can1_FIFO1_Receive3(&rx_message1);
        break;
    default:
        break;
    }
}

/*  接收数据传递到外部 */

/**
 * @brief CAN1 FIFO0 ID0 数据接收(0x100,GYRO)
 * @param[in] rx_message1 CAN1 FIFO0接收数据
 */

void Can1_FIFO0_Receive0(CanRxMsg *rx_message0)
{
    //蓝牙数据接收
    // uint16_t blue_tooth_recv;
    // memcpy(&blue_tooth_recv, rx_message0->Data, 2);
    // Blue_Tooth_Deal(&blue_tooth_recv);
}

/**
 * @brief CAN1 FIFO0 ID1 数据接收(0x101,Accel)
 * @param[in] rx_message1 CAN1 FIFO0接收数据
 */
void Can1_FIFO0_Receive1(CanRxMsg *rx_message0)
{
    // //遥控器数据接收
    // static uint8_t data[6];
    // memcpy(data, rx_message0->Data, 6);
    // RemoteReceive(data);
}

/**
 * @brief CAN1 FIFO0 ID2 数据接收()
 * @param[in] rx_message1 CAN1 FIFO0接收数据
 */
void Can1_FIFO0_Receive2(CanRxMsg *rx_message0)
{
    memcpy(&IMUReceive.Gyro[0], rx_message0->Data, 2);
    memcpy(&IMUReceive.Gyro[1], &rx_message0->Data[2], 2);
    memcpy(&IMUReceive.Gyro[2], &rx_message0->Data[4], 2);

    LossUpdate(&global_debugger.imu_debugger[0], 0.0025f);
}

/**
 * @brief CAN1 FIFO0 ID3 数据接收()
 * @param[in] rx_message1 CAN1 FIFO0接收数据
 */
void Can1_FIFO0_Receive3(CanRxMsg *rx_message0)
{
    memcpy(&IMUReceive.Acc[0], rx_message0->Data, 2);
    memcpy(&IMUReceive.Acc[1], &rx_message0->Data[2], 2);
    memcpy(&IMUReceive.Acc[2], &rx_message0->Data[4], 2);

    LossUpdate(&global_debugger.imu_debugger[1], 0.0025f);
}

/**
 * @brief CAN1 FIFO1 ID0 数据接收()
 * @param[in] rx_message1 CAN1 FIFO1接收数据
 */
void Can1_FIFO1_Receive0(CanRxMsg *rx_message1)
{
    memcpy(&gimbal_receiver_pack1, rx_message1->Data, 8);

    Gimbal_msgs_Decode1();

    LossUpdate(&global_debugger.gimbal_comm_debugger[0], 0.0085f);
}

/**
 * @brief CAN1 FIFO0 ID1 数据接收()
 * @param[in] rx_message1 CAN1 FIFO1接收数据
 */
void Can1_FIFO1_Receive1(CanRxMsg *rx_message1)
{
    memcpy(&gimbal_receiver_pack2, rx_message1->Data, 8);

    // Gimbal_msgs_Decode2();

    LossUpdate(&global_debugger.gimbal_comm_debugger[1], 0.0015f);
}

/**
 * @brief CAN1 FIFO0 ID2 数据接收()
 * @param[in] rx_message1 CAN1 FIFO1接收数据
 */
void Can1_FIFO1_Receive2(CanRxMsg *rx_message1)
{
}

/**
 * @brief CAN1 FIFO0 ID3 数据接收()
 * @param[in] rx_message1 CAN1 FIFO1接收数据
 */
void Can1_FIFO1_Receive3(CanRxMsg *rx_message1)
{
}

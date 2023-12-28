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
    memcpy(&Wheels_Motor.recv_msgs[LEFT_WHEEL_ID], rx_message0->Data, 8);

    global_debugger.mf9025_debugger[LEFT_WHEEL_ID].recv_msgs_num++;
    global_debugger.mf9025_debugger[LEFT_WHEEL_ID].can_dt = GetDeltaT(&global_debugger.mf9025_debugger[LEFT_WHEEL_ID].last_can_cnt);
    if (global_debugger.mf9025_debugger[LEFT_WHEEL_ID].can_dt > 0.0017f)
    {
        global_debugger.mf9025_debugger[LEFT_WHEEL_ID].loss_num++;
    }
}

/**
 * @brief CAN2 FIFO0 ID1 数据接收(0x101,Accel)
 * @param[in] rx_message1 CAN2 FIFO0接收数据
 */
void Can2_FIFO0_Receive1(CanRxMsg *rx_message0)
{
    memcpy(&Wheels_Motor.recv_msgs[RIGHT_WHEEL_ID], rx_message0->Data, 8);

    global_debugger.mf9025_debugger[RIGHT_WHEEL_ID].recv_msgs_num++;
    global_debugger.mf9025_debugger[RIGHT_WHEEL_ID].can_dt = GetDeltaT(&global_debugger.mf9025_debugger[RIGHT_WHEEL_ID].last_can_cnt);
    if (global_debugger.mf9025_debugger[RIGHT_WHEEL_ID].can_dt > 0.0017f)
    {
        global_debugger.mf9025_debugger[RIGHT_WHEEL_ID].loss_num++;
    }
}

/**
 * @brief CAN2 FIFO0 ID2 数据接收()
 * @param[in] rx_message1 CAN2 FIFO0接收数据
 */
void Can2_FIFO0_Receive2(CanRxMsg *rx_message0)
{
}

/**
 * @brief CAN2 FIFO0 ID3 数据接收()
 * @param[in] rx_message1 CAN2 FIFO0接收数据
 */
void Can2_FIFO0_Receive3(CanRxMsg *rx_message0)
{
}

extern MF9025 Wheels_Motor;
/**
 * @brief CAN2 FIFO1 ID0 数据接收()
 * @param[in] rx_message1 CAN2 FIFO1接收数据
 */
void Can2_FIFO1_Receive0(CanRxMsg *rx_message1)
{
}

/**
 * @brief CAN2 FIFO0 ID1 数据接收()
 * @param[in] rx_message1 CAN2 FIFO1接收数据
 */
void Can2_FIFO1_Receive1(CanRxMsg *rx_message1)
{
}

/**
 * @brief CAN2 FIFO0 ID2 数据接收()
 * @param[in] rx_message1 CAN2 FIFO1接收数据
 */
void Can2_FIFO1_Receive2(CanRxMsg *rx_message1)
{
}

/**
 * @brief CAN2 FIFO0 ID3 数据接收()
 * @param[in] rx_message1 CAN2 FIFO1接收数据
 */
void Can2_FIFO1_Receive3(CanRxMsg *rx_message1)
{
}

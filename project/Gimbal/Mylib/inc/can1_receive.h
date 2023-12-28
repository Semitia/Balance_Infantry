#ifndef _CAN1_RECEIVE_H
#define _CAN1_RECEIVE_H

#include <stm32f4xx.h>
#include <stm32f4xx_conf.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "debug.h"
#include "GimbalEstimateTask.h"
#include "ToggleBullet.h"
#include "ChassisGet.h"
/*  Task */

// FIFO 0 接收ID
#define CAN1_FIFO0_ID0 0x000
#define CAN1_FIFO0_ID1 0x001
#define CAN1_FIFO0_ID2 0x002
#define CAN1_FIFO0_ID3 YAW_MOTOR_CAN_ID

// FIFO 1 接收ID
#define CAN1_FIFO1_ID0 0x000
#define CAN1_FIFO1_ID1 GET_FROM_CHASSIS_CAN_ID_1
#define CAN1_FIFO1_ID2 0x002
#define CAN1_FIFO1_ID3 0x003

void Can1Receive0(CanRxMsg rx_message0);
void Can1Receive1(CanRxMsg rx_message1);

#endif // !_CAN1_RECEIVE_H

#ifndef _CAN2_RECEIVE_H
#define _CAN2_RECEIVE_H

#include <stm32f4xx.h>
#include <stm32f4xx_conf.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/*  Task */
#include "ChasisEstimateTask.h"

#include "debug.h"
#include "MF9025.h"
#include "Gimbal.h"
#include "ToggleBullet.h"

// FIFO 0 接收ID
#define CAN2_FIFO0_ID0 LEFT_MF9025_CAN_ID
#define CAN2_FIFO0_ID1 RIGHT_MF9025_CAN_ID
#define CAN2_FIFO0_ID2 0x000
#define CAN2_FIFO0_ID3 0x001

// FIFO 1 接收ID
#define CAN2_FIFO1_ID0 0x002
#define CAN2_FIFO1_ID1 0x003
#define CAN2_FIFO1_ID2 0x000
#define CAN2_FIFO1_ID3 0x001

void Can2Receive0(CanRxMsg rx_message0);
void Can2Receive1(CanRxMsg rx_message1);

#endif // !_CAN2_RECEIVE_H

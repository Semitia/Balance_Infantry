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
#include "GimbalEstimateTask.h"

#include "can_config.h"
#include "FrictionWheel.h"
#include "Gimbal.h"
#include "debug.h"

// FIFO 0 接收ID
#define CAN2_FIFO0_ID0 GYRO_CAN_ID_1
#define CAN2_FIFO0_ID1 GYRO_CAN_ID_2
#define CAN2_FIFO0_ID2 TOGGLE_MOTOR_CAN_ID
#define CAN2_FIFO0_ID3 0x000

// FIFO 1 接收ID
#define CAN2_FIFO1_ID0 PITCH_MOTOR_CAN_ID
#define CAN2_FIFO1_ID1 FRICTION_WHEEL_CAN_ID_1
#define CAN2_FIFO1_ID2 FRICTION_WHEEL_CAN_ID_2
#define CAN2_FIFO1_ID3 0x000

void Can2Receive0(CanRxMsg rx_message0);
void Can2Receive1(CanRxMsg rx_message1);

#endif // !_CAN2_RECEIVE_H

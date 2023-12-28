#ifndef _CAN_CONFIG_H
#define _CAN_CONFIG_H

#include <stm32f4xx.h>
#include <stm32f4xx_conf.h>
#include <string.h>
#include <stdint.h>

//挂载在CAN2上的信息

#define LEFT_MF9025_CAN_ID 0x142
#define RIGHT_MF9025_CAN_ID 0x141
#define DJI_MOTORS_CAN CAN2

//挂载在CAN1上的信息
#define IMU_CAN_ID_1 0x100
#define IMU_CAN_ID_2 0x101
#define GIMBAL_COMM_CAN_ID_1 0x150 //和云台通信CAN ID1
#define GIMBAL_COMM_CAN_ID_2 0x151 //和云台通信CAN ID2
#define GIMBAL_CAN_COMM_CANx CAN1
#define SEND_TO_GIMBAL_CAN_ID_1 0x160

#endif // !_CAN_CONFIG_H

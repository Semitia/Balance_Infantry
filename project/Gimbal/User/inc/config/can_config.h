#ifndef _MOTOR_CONFIG_H
#define _MOTOR_CONFIG_H

#include <stm32f4xx.h>
#include <stm32f4xx_conf.h>
#include <string.h>
#include <stdint.h>

//挂载在CAN2上
#define PITCH_MOTOR_CAN_ID 0x207      // pitch轴ID
#define FRICTION_WHEEL_CAN_ID_1 0x205 // 摩擦轮ID1(左)
#define FRICTION_WHEEL_CAN_ID_2 0x208 // 摩擦轮ID2(右)
#define YAW_MOTOR_CAN_ID 0x206        // yaw轴电机
#define GYRO_CAN_ID_1 0x100           //角速度
#define GYRO_CAN_ID_2 0x101           //加速度计
#define DJI_MOTORS_CAN CAN2

#define TOGGLE_MOTOR_CAN_ID 0x206 //拨弹电机
#define SEND_TO_CHASSIS_CAN_ID_1 0x150
#define SEND_TO_CHASSIS_CAN_ID_2 0x151
#define CHASSIS_CAN_COMM_CANx CAN1
#define GET_FROM_CHASSIS_CAN_ID_1 0x160

#endif // !_MOTOR_CONFIG_H

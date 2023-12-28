#ifndef _GIMBAL_RECEIVE_H
#define _GIMBAL_RECEIVE_H

#include "stdint.h"

#include "remote_control.h"

#pragma pack(push, 1)

typedef struct GimbalReceivePack1
{
    uint16_t robot_state : 1;
    uint16_t control_type : 2;
    uint16_t control_mode_action : 4;
    uint16_t gimbal_mode : 3;
    uint16_t shoot_mode : 3;
    uint16_t leg_mode : 2;
    uint16_t is_pc_on : 1;
    uint8_t autoaim_id;   //自瞄ID
    int8_t robot_speed_x; // * 10 描述 x方向为云台正方向
    int8_t robot_speed_y; // * 10描述
    int8_t robot_speed_w;
} GimbalReceivePack1;

typedef struct GimbalReceivePack2 //云台yaw和pitch角度
{
    int16_t yaw_motor_angle; //云台yaw轴电机角度
    int16_t gimbal_pitch;    //底盘
    int16_t gimbal_yaw_speed;
    int16_t super_power;
} GimbalReceivePack2;

#pragma pack(pop)

extern GimbalReceivePack2 gimbal_receiver_pack2;
extern GimbalReceivePack1 gimbal_receiver_pack1;

extern int8_t gimbal_receive_1_update;
extern int8_t gimbal_receive_2_update;
extern int8_t gimbal_receive_3_update;

void Gimbal_msgs_Decode1(void);
void Gimbal_msgs_Decode2(void);

#define GIMBAL_FOLLOW_DEAD_ANGLE 1.0f
#define SIGN_MOTOR_FOLLOW -1.0f

#endif // !_GIMBAL_RECEIVE_H

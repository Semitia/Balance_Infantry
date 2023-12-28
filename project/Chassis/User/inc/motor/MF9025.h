#ifndef _MF_9025_H
#define _MF_9025_H

#include <stm32f4xx.h>
#include <stm32f4xx_conf.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "tools.h"
#include "can_send.h"
#include "ZeroCheck.h"

//默认左腿为0x141,右腿为0x142，使用前先通过线改轮子id
#define MF9025_TORQUE_CURRENT_RATIO 0.32f
#define MAX_WHEEL_TORQUE 10.0f
#define MIN_WHEEL_TORQUE -MAX_WHEEL_TORQUE
#define LEFT_WHEEL_ID 1
#define RIGHT_WHEEL_ID 0
#define MF9025_POS_RATIO (0.00549316f) // 度
#define MF9025_SPEED_RATIO 1.0f        // 度/s
#define MF9025_CURRENT_RATIO (33.0f / 2048.0f)
#define MF9025_TORQUE2CURRENT (2000 / 32.0f / MF9025_TORQUE_CURRENT_RATIO)

#define MAX_WHEEL_CURRENT (MAX_WHEEL_TORQUE / MF9025_TORQUE_CURRENT_RATIO * 2000 / 32.0f)
#define MIN_WHEEL_CURRENT -MAX_WHEEL_CURRENT

#define WHEEL_CAN CAN2
#define WHEEL_BROCAST_CAN_ID 0x280

/*  广播ID为0x280 */
typedef struct
{
    int16_t wheel_current[4];
} MF9025_Broadcast;

/* 接收数据定义  */
typedef struct
{
    uint8_t command_byte; //控制字节，转矩闭环控制为0xA1
    int8_t temperature;   //温度
    int16_t current;      //电流，*33/2048描述
    int16_t rotate_speed; //转速，1dps描述
    uint16_t pos;         // 16位编码器位置
} MF9025_Recv;

typedef struct
{
    int8_t temperature; //温度
    float current;      // A
    float rotate_speed; // rad/s
    float pos;          // rad
    ZeroCheck_Typedef pos_zero_check;
} MF9025_Info;

/*  总定义 */
typedef struct
{
    MF9025_Broadcast frame;      //广播帧
    MF9025_Recv recv_msgs[2];    //接收数据定义
    MF9025_Info decoded_msgs[2]; //解码后的数据定义

    float init_pos[2];
} MF9025;

void MF9025_Decode(MF9025_Info *mf9025_info, MF9025_Recv *mf9025_recv);
void WheelOffPack(MF9025_Broadcast *wheels);
void WheelCurrentPack(MF9025_Broadcast *wheels, float *wheel_torque);

void WheelCanSend(MF9025_Broadcast *wheels);
void MF9025_Init(MF9025 *mf9025);
void limitWheelTorque(float *wheels);

extern MF9025 Wheels_Motor;

#endif // !_MF_9025_H

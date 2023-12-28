#ifndef _GM6020_H
#define _GM6020_H

#include <stm32f4xx.h>
#include <stm32f4xx_conf.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "ZeroCheck.h"
#include "tools.h"
#include "my_filter.h"

//由于6020电机说明书上并没有表明电流以及电压的量纲，故实际计算的时候需要将整体作为处理对象

#define GM6020_ANGLE_RATIO 0.04394531f //将机械角度转为弧度 度
#define GM6020_SPEED_RATIO 6.0f        // 度/s

#define GM6020_MAX_VOLTAGE 25000 //最大发送电压值,最大值为30000
#define GM6020_MIN_VOLTAGE -GM6020_MAX_VOLTAGE

#define GM6020_MAX_CURRENT 19000
#define GM6020_MIN_CURRENT -GM6020_MAX_CURRENT

// #define GM6020_REDUCTION_RATIO 36.0f //减速比

#pragma pack(push, 1) //不进行字节对齐
// 接收到的2006数据
typedef struct GM6020_Recv //使用的是GM6020电调
{
    int16_t angle;
    int16_t speed;
    int16_t torque_current;
    int8_t temp;
    int8_t _;
} GM6020_Recv;

#pragma pack(pop)

enum GM6020_STD_ID
{
    GM6020_STD_ID_1_4 = 0X1ff, // ID为1-4的2006电机发送标准帧id
    GM6020_STD_ID_5_7 = 0x2ff
};

enum GM6020_MOTOR_ID
{
    GM6020_MOTOR_ID_1 = 1,
    GM6020_MOTOR_ID_2 = 2,
    GM6020_MOTOR_ID_3 = 3,
    GM6020_MOTOR_ID_4 = 4,
    GM6020_MOTOR_ID_5 = 5,
    GM6020_MOTOR_ID_6 = 6,
    GM6020_MOTOR_ID_7 = 7
};

// 解码后的2006数据
typedef struct GM6020_Info
{
    ZeroCheck_Typedef angle_zero_check;
    float angle; //转为输出轴 度
    float speed; //转为输出轴 度/s
    float torque_current;
} GM6020_Info;

void GM6020_Decode(GM6020_Recv *GM6020_recv, GM6020_Info *GM6020_decoded);
void GM6020_SendPack(int8_t *can_data, uint32_t std_id, uint8_t motor_id, int16_t control_vol);

#endif // !_GM6020_H

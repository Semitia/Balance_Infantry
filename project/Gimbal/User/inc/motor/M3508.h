#ifndef _M3508_H
#define _M3508_H

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

#define M3508_ANGLE_RATIO 0.04394531f //将机械角度转为度
#define M3508_SPEED_RATIO 6.0f        // 度/s

#define C620_CURRENT_SEND_TRANS 819.2f //将实际电流值转为发送值
#define C620_MAX_CURRENT 18.0f         //最大发送电流值,最大值为20.0000f
#define C620_MIN_CURRENT -C620_MAX_CURRENT
#define C620_MAX_SEND_CURRENT 10000.0f
#define C620_MIN_SEND_CURRENT -C620_MAX_SEND_CURRENT

#define M3508_REDUCTION_RATIO 19.0f

#pragma pack(push, 1) //不进行字节对齐
// 接收到的2006数据
typedef struct M3508_Recv //使用的是C620电调
{
    int16_t angle;
    int16_t speed;
    int16_t torque_current;
    int8_t temp; //温度
    int8_t _;
} M3508_Recv;

#pragma pack(pop)

enum C620_STD_ID
{
    C620_STD_ID_1_4 = 0X200, // ID为1-4的2006电机发送标准帧id
    C620_STD_ID_5_8 = 0x1ff
};

enum C620_MOTOR_ID
{
    C620_MOTOR_ID_1 = 1,
    C620_MOTOR_ID_2 = 2,
    C620_MOTOR_ID_3 = 3,
    C620_MOTOR_ID_4 = 4,
    C620_MOTOR_ID_5 = 5,
    C620_MOTOR_ID_6 = 6,
    C620_MOTOR_ID_7 = 7,
    C620_MOTOR_ID_8 = 8
};

enum SEND_TYPE
{
    RAW_CURRENT, //原始电流，即实际电流值
    SEND_CURRENT //发送的是直接电流值，不需要进行转化
};

enum DECODE_TYPE
{
    ONLY_SPEED = 0,
    ONLY_SPEED_WITH_REDUCTION, //带减速器的速度
    ALL_WITH_REDUCTION,        // 带减速器
    ALL                        //不带减速器
};

// 解码后的2006数据
typedef struct M3508_Info
{
    ZeroCheck_Typedef angle_zero_check;
    float angle; //转为输出轴弧度
    float speed; //转为输出轴 rad/s
    float torque_current;
} M3508_Info;

void M3508_Decode(M3508_Recv *M3508_recv, M3508_Info *M3508_decoded, int8_t decode_type, float filte_value);
void M3508_SendPack(int8_t *can_data, uint32_t std_id, uint8_t motor_id, float control_current, enum SEND_TYPE send_type);

#endif // !_M3508_H

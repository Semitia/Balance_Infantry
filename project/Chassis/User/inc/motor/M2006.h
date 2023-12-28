#ifndef _M2006_H
#define _M2006_H

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

#define M2006_ANGLE_RATIO 0.04394531f //将机械角度转为角度 度
#define M2006_SPEED_RATIO 6.0f        // 度/s

#define C610_CURRENT_SEND_TRANS 1000.0f //将实际电流值转为发送值,或者将接收到的值转为浮点数的实际电流
#define C610_MAX_CURRENT 8.0f           //最大发送电流值,最大值为10.0000f
#define C610_MIN_CURRENT -C610_MAX_CURRENT

#define M2006_REDUCTION_RATIO 36.0f //减速比

#pragma pack(push, 1) //不进行字节对齐
// 接收到的2006数据
typedef struct M2006_Recv //使用的是C610电调
{
    int16_t angle;
    int16_t speed;
    int16_t torque_current;
    int16_t _;
} M2006_Recv;

#pragma pack(pop)

enum C610_STD_ID
{
    C610_STD_ID_1_4 = 0X200, // ID为1-4的2006电机发送标准帧id
    C610_STD_ID_5_8 = 0x1ff
};

enum C610_MOTOR_ID
{
    C610_MOTOR_ID_1 = 1,
    C610_MOTOR_ID_2 = 2,
    C610_MOTOR_ID_3 = 3,
    C610_MOTOR_ID_4 = 4,
    C610_MOTOR_ID_5 = 5,
    C610_MOTOR_ID_6 = 6,
    C610_MOTOR_ID_7 = 7,
    C610_MOTOR_ID_8 = 8
};

enum REDUTION_OR_NOT
{
    WITHOUT_REDUCTION = 0, //不带减速器
    WITH_REDUCTION = 1,    //带减速器
};

// 解码后的2006数据
typedef struct M2006_Info
{
    ZeroCheck_Typedef angle_zero_check;
    float angle; //转为输出轴度
    float speed; //转为输出轴 度/s
    float torque_current;
} M2006_Info;

void M2006_Decode(M2006_Recv *m2006_recv, M2006_Info *m2006_decoded, int8_t is_reduction, float filte_value);
void M2006_SendPack(int8_t *can_data, uint32_t std_id, uint8_t motor_id, float control_current);
#endif // !_M2006_H

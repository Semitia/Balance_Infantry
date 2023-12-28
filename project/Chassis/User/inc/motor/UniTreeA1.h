#ifndef MOTOR_MSG
#define MOTOR_MSG

#include <stm32f4xx.h>
#include <stm32f4xx_conf.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <stdint.h>
#include "crc32.h"
#include "tools.h"

#define RESET_AND_CALIB_A1 1

/*        前
    左    ^     右
    0     |     3
    |----------|
    |          |
    |          |
    |          |
    1-----------2
         后
*/
#define LEFT_KNEE_LEFT 0   //左视图来看左边的关节电机
#define LEFT_KNEE_RIGHT 1  //左视图来看右边的关节电机
#define RIGHT_KNEE_LEFT 2  //右视图来看左边的关节电机
#define RIGHT_KNEE_RIGHT 3 //右视图来看右边的关节电机

#define LEFT_KNEE_RS485 1  // 1号RS485
#define RIGHT_KNEE_RS485 2 // 2号RS485

// 新调的车需要修改
#define LEFT_LEFT_MOTOR_ID 0
#define LEFT_RIGHT_MOTOR_ID 2
#define RIGHT_LEFT_MOTOR_ID 0
#define RIGHT_RIGHT_MOTOR_ID 2

#define REDUCTION_RATIO 9.1f //减速比
#define MAX_KNEE_TORQUE 28.0f
#define MIN_KNEE_TORQUE -MAX_KNEE_TORQUE

#define LEFT_KNEE_RS485_SEND() \
    MOTOR1_START_WRITE();      \
    DMA_Cmd(MOTOR1_RS485_SEND_DMAx_Streamx, ENABLE);
#define RIGHT_KNEE_RS485_SEND() \
    MOTOR2_START_WRITE();       \
    DMA_Cmd(MOTOR2_RS485_SEND_DMAx_Streamx, ENABLE);

#pragma pack(1)

// 32字节数据的多种形式
typedef union
{
    int32_t L;
    uint8_t u8[4];
    uint16_t u16[2];
    uint32_t u32;
    float F;
} COMData32;

// 定义 数据包头
typedef struct
{
    unsigned char start[2]; // 包头 0xFE,0xEE
    unsigned char motorID;  // 电机ID  0,1,2,3,0xBB 表示向所有电机广播
    unsigned char reserved; //预留位，可忽略
} COMHead;

#pragma pack()

#pragma pack(1)

/* 发送数据定义(不包含包头和校验码)  */
typedef struct
{
    uint8_t mode;      // 关节模式:0(停转)，5(开环转),10(闭环伺服)
    uint8_t ModifyBit; // 电机控制参数修改位，可忽略
    uint8_t ReadBit;   // 电机控制参数发送位，可忽略
    uint8_t reserved;  // 预留位，可忽略
    COMData32 Modify;  // 电机参数修改数据，可忽略

    int16_t T;   // 电机前馈力矩 x256描述
    int16_t W;   // 期望关节速度 x128描述
    int32_t Pos; // 期望关节位置 x 16384/6.2832, 14位编码器

    int16_t K_P; // 关节刚度系数 x2048 描述
    int16_t K_W; // 关节速度系数 x1024  描述

    uint8_t LowHzMotorCmdIndex; // 电机低频率控制命令的索引,可忽略
    uint8_t LowHzMotorCmdByte;  // 电机低频率控制命令的字节，可忽略

    COMData32 Res[1]; // 预留位，可忽略

} MasterComdV3;

/* 发送数据定义(包含包头和校验码)  */
typedef struct
{
    COMHead head;       //包头
    MasterComdV3 Mdata; //数据
    COMData32 CRCdata;  //校验码
} MasterComdDataV3;

#pragma pack()

#pragma pack(1)

/* 接收数据定义(不包含包头和校验码) */
typedef struct
{
    uint8_t mode;    // 当前关节模式，关节模式:0(停转)，5(开环转),10(闭环伺服)
    uint8_t ReadBit; // 表示电机内部控制参数是否修改成功，可忽略
    int8_t Temp;     // 电机当前平均温度
    uint8_t MError;  // 电机错误 标识

    COMData32 Read; // 读取的当前 电机 的控制数据，可忽略
    int16_t T;      // 当前实际电机输出力矩 * 256 描述

    int16_t W; // 当前实际电机速度（高速） *128 描述
    float LW;  // 当前实际电机速度（低速）(过了低通滤波)

    int16_t W2; // 关节编码器预留，可忽略
    float LW2;  // 关节编码器预留，可忽略

    int16_t Acc;    // 电机转子加速度  *1 描述
    int16_t OutAcc; // 关节编码器预留，可忽略

    int32_t Pos;  // 当前电机位置 *16384 / (2*pi)描述
    int32_t Pos2; // 关节编码器预留，可忽略

    int16_t gyro[3]; // 电机驱动板IMU角速度，x,y,z轴，用2000/(2^15)*2*pi/360描述
    int16_t acc[3];  // 电机驱动板IMU加速度，x,y,z,以 8 * 9.80665 / (2^15)描述

    int16_t Fgyro[3]; //足端imu预留，可忽略
    int16_t Facc[3];  //足端imu预留，可忽略
    int16_t Fmag[3];  //足端imu预留，可忽略
    uint8_t Ftemp;    //足端传感器温度，可忽略
    int16_t Force16;  //足端力传感器高八位，可忽略
    int8_t Force8;    //足端力传感器低八位，可忽略
    uint8_t FError;   // 足端传感器错误标识
    int8_t Res[1];    // 预留位，可忽略
} ServoComdV3;

/* 接收数据定义(包含包头和校验码) */
typedef struct
{
    COMHead head;      //包头
    ServoComdV3 Mdata; //数据
    COMData32 CRCdata; //校验码
} ServoComdDataV3;

#pragma pack()

typedef struct MOTOR_send
{
    MasterComdDataV3 motor_send_data; //发送数据

    // 发送数据源形式
    unsigned short id;   //电机ID，0xBB代表全部电机
    unsigned short mode; // 0:空闲, 5:开环转动, 10:闭环FOC控制

    // 实际给FOC的指令力矩为：K_P*delta_Pos + K_W*delta_W + T
    float T;   //期望关节的输出力矩（电机本身的力矩）（Nm）
    float W;   //期望关节速度（电机本身的速度）(rad/s)
    float Pos; //期望关节位置（rad）
    float K_P; //关节刚度系数
    float K_W; //关节速度系数

    //忽略
    COMData32 Res;

} MOTOR_send;

typedef struct MOTOR_recv
{
    //接收数据
    ServoComdDataV3 motor_recv_data;

    uint8_t correct; //接收数据是否完整（true完整，false不完整）

    //解读得出的电机数据
    unsigned char motor_id; //电机ID
    unsigned char mode;     // 0:空闲, 5:开环转动, 10:闭环FOC控制
    int Temp;               //温度
    unsigned char MError;   //错误码

    float T;   // 当前实际电机输出力矩
    float W;   // 当前实际电机速度（高速）
    float LW;  // 当前实际电机速度（低速）
    int Acc;   // 电机转子加速度
    float Pos; // 当前电机位置（主控0点修正，电机关节还是以编码器0点为准）

    float gyro[3]; // 电机驱动板6轴传感器数据
    float acc[3];
} MOTOR_recv;

typedef struct UniTreeA1
{
    MOTOR_recv a1_motor_recv[4];

    /*  初始化参数 */
    float init_pos[4];
} UniTreeA1;

void motorOffPack(MOTOR_send *motor_send, unsigned short id);
void motorDataPack(MOTOR_send *motor_send, unsigned short id, float T, float W, float Pos, float K_P, float K_W);
uint8_t extract_data(MOTOR_recv *motor_r);
void modify_data(MOTOR_send *motor_s);
void RS485Receive(uint8_t rs485_id);
void A1MotorInit(UniTreeA1 *knee_motor);
void limitKneeTorque(float *knee);

extern UniTreeA1 unitree_a1_motors;
extern int8_t A1MotorIDs[4];

#endif

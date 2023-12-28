// 遥控器选择
#ifndef _REMOTE_CONTROL_H
#define _REMOTE_CONTROL_H

#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "counter.h"
#include "debug.h"

enum CONTROL_TYPE
{
    DJI_REMOTE_CONTROL, // DJI遥控器
    BLUE_TOOTH,         // 蓝牙控制
    KEY_MOUSE,          // 键鼠
};
enum ROBOT_STATE
{
    OFFLINE_MODE,
    CONTROL_MODE,
};
enum CONTROL_MODE_ACTION
{
    NOT_CONTROL_MODE,
    SHRINK_LEG_MODE, // 倒地态，收腿复位
    RECOVER,         // 平衡态(倒地站立)
    BALANCE,         // 平衡态
    FOLLOW_GIMBAL,   // 云台跟随模式
    SELF_ROTATE,     // 自转态
    FLY_MODE,        // 飞坡模式(起飞)
    LAND_MODE,       // 落地模式
    FOLLOW_BESIDES,
    SLIDDED_MODE,   // 打滑模式
    JUMP_PRE_MODE,  // 起跳准备，下蹲
    JUMP_UP_MODE,   // 起跳开始状态，开始伸腿
    JUMP_DOWN_MODE, // 起跳结束，结束伸腿，收腿
    FELL_RUN_MODE   // 倒地控制模式
};

enum PowerControlState // 功率控制状态
{
    POWER_TO_BATTERY,    // 接电源
    POWER_TO_SuperPower, // 接电容
};

/* 底盘腿运动状态 */
enum LEG_ACTION
{
    NOT_STRETCH, // 原始状态下的不伸腿
    RAW_LENGTH,  // 正常运动时的小幅度伸腿
    HIGH_LENGTH  // 伸腿
};

enum GIMBAL_ACTION
{
    GIMBAL_POWERDOWN, // 云台掉电模式
    GIMBAL_ACT_MODE,
    GIMBAL_AUTO_AIM_MODE,
    GIMBAL_TEST_MODE,
    GIMBAL_SI_MODE,         // 云台系统辨识模式
    GIMBAL_SMALL_BUFF_MODE, // 打符模式
    GIMBAL_BIG_BUFF_MODE    // 大符
};

enum SHOOT_ACTION
{
    SHOOT_POWERDOWN_MODE, // 掉电模式
    SHOOT_CHECK_MODE,     // 自检模式
    SHOOT_TEST_MODE,      // 弹道测试模式
    SHOOT_FIRE_MODE,      // 射击模式
    SHOOT_AUTO_AIM_MODE,  // 自瞄模式
    SHOOT_SUPPLY_MODE
};

enum BLUE_TOOTH_KEY
{
    BLUE_TOOTH_OFFLINE_MODE = 0,
    BLUE_TOOTH_CONTROL_MODE = 1,
    BLUE_TOOTH_ROTATE_MODE = 2,
    BLUE_TOOTH_UP = 10,
    BLUE_TOOTH_LEFT = 11,
    BLUE_TOOTH_STABLE = 12,
    BLUE_TOOTH_RIGHT = 13,
    BLUE_TOOTH_DOWN = 14,
    BLUE_TOOTH_NO_ACTION = -1
};

#pragma pack(1)
/*遥控器结构体*/
typedef struct
{
    unsigned short ch[4]; // 摇杆
    unsigned short s[2];  // 拨杆
} Remote;

#define CH_MIDDLE 1024
#define CH_RANGE 660 // 范围为CH_MIDDLE - CH_RANGE ~ CH_MIDDLE + CH_RANGE

// 拨杆位置
enum SWPos
{
    Lost = 0,
    Up = 1,
    Mid = 3,
    Down = 2
};
enum WhichSW
{
    LEFT_SW = 1,
    RIGHT_SW = 0
};
enum WhichCH
{
    RIGHT_CH_LR = 0, // 右杆左右方向
    RIGHT_CH_UD = 1, // 右杆上下方向
    LEFT_CH_LR = 2,  // 左杆左右方向
    LEFT_CH_UD = 3,  // 左杆上下方向
};
/*鼠标结构体*/
typedef struct
{
    short x;
    short y;
    short z;
    unsigned char press_l;
    unsigned char press_r;
} Mouse;

#define KEY_B 0x8000
#define KEY_V 0x4000
#define KEY_C 0x2000
#define KEY_X 0x1000
#define KEY_Z 0x0800
#define KEY_G 0x0400
#define KEY_F 0x0200
#define KEY_R 0x0100
#define KEY_E 0x0080
#define KEY_Q 0x0040
#define KEY_CTRL 0x0020
#define KEY_SHIFT 0x0010
#define KEY_D 0x0008
#define KEY_A 0x0004
#define KEY_S 0x0002
#define KEY_W 0x0001

/*遥键鼠结构体综合*/
typedef struct
{
    Remote rc;
    Mouse mouse;
    uint16_t keyValue;

    // 以下是检测按键触发状态变量
    uint16_t last_keyValue;
    uint16_t keyChangeOn;  // 检测按键值从0转1改变
    uint16_t keyChangeOff; // 检测按键值从1转0转变
} RC_Ctl_t;

#pragma pack()

typedef struct RemoteController
{
    enum CONTROL_TYPE control_type; // 控制类型(遥控器，蓝牙等)
    enum ROBOT_STATE robot_state;   // 机器人状态(掉线模式，控制模式)
    enum ROBOT_STATE last_robot_state;
    enum GIMBAL_ACTION gimbal_action; // 云台
    enum GIMBAL_ACTION last_gimbal_action;
    enum SHOOT_ACTION shoot_action; // 打弹模式
    enum SHOOT_ACTION last_shoot_action;
    enum CONTROL_MODE_ACTION last_control_mode_action; // 底盘模式(平衡模式，倒地自救模式)
    enum CONTROL_MODE_ACTION control_mode_action;
    enum LEG_ACTION leg_action;
    enum PowerControlState super_power_state;

    enum BLUE_TOOTH_KEY blue_tooth_key;
    RC_Ctl_t dji_remote;
} RemoteController;

void setGimbalAction(enum GIMBAL_ACTION action);
void setShootAction(enum SHOOT_ACTION action);

void setControlMode(enum CONTROL_TYPE type);
void setRobotState(enum ROBOT_STATE state);
void setControlModeAction(enum CONTROL_MODE_ACTION action);
void initRemoteControl(enum CONTROL_TYPE type);
void setLegAction(enum LEG_ACTION action);
void setSuperPower(enum PowerControlState super_power_state);

// 蓝牙
void Blue_Tooth_Deal(uint16_t *);

// 遥控器
void RemoteReceive(volatile unsigned char rx_buffer[]);

void RC_Rst(void);

extern RemoteController remote_controller;

#endif // !_REMOTE_CONTROL_H

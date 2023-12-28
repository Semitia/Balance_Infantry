#ifndef _TOGGLE_BULLET_H
#define _TOGGLE_BULLET_H

#include "pid.h"
#include "M2006.h"

#define ONE_GRID_ANGLE 45.0f //度
#define SIGN_ROTATE -1.0f    //拨盘正转动方向

enum TOGGLE_CONTRL_MODE
{
    TOGGLE_SPEED, //拨弹速度控制(连发)
    TOGGLE_POS,   //拨弹单发控制(单发)
    TOGGLE_STOP
};

/* 拨弹控制器 */
typedef struct ToggleController
{
    PID_t toggle_pos_pid;
    PID_t toggle_speed_pid; //拨弹速度环
    M2006_Recv toggle_recv;
    M2006_Info toggle_info;

    float set_pos;
    float set_speed;
    int8_t is_shoot; //发射指令

    float shoot_freq_speed; //根据机器人等级设置的拨盘速度
} ToggleController;

extern ToggleController toggle_controller;

void TogglePidInit(void);
float Toggle_Calculate(enum TOGGLE_CONTRL_MODE control_mode, float set_point);
void ToggleAddGrid(float *set_point, int8_t N);
void selectShootFreq(uint8_t robot_level, uint8_t shoot_buff);

#endif // !_TOGGLE_BULLET_H

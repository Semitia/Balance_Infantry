#ifndef _TOGGLE_BULLET_H
#define _TOGGLE_BULLET_H

#include "pid.h"
#include "M2006.h"

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
} ToggleController;

extern ToggleController toggle_controller;

void TogglePidInit(void);
float Toggle_Calculate(enum TOGGLE_CONTRL_MODE control_mode, float set_point);
void ToggleAddGrid(float *set_point);

#endif // !_TOGGLE_BULLET_H

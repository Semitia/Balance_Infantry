#include "ToggleBullet.h"

ToggleController toggle_controller;

void TogglePidInit()
{
    PID_Init(&toggle_controller.toggle_pos_pid, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, NONE);
    PID_Init(&toggle_controller.toggle_speed_pid, C610_MAX_CURRENT, 0, 0, 0.07, 0.0012, 0, 0, 0, 0, 0, 1, NONE);
}

/**
 * @brief 拨弹电机控制计算
 * @param[in] control_mode 控制模式
 * @param[in] set_point 目的位置(位置控制)，目标速度(速度控制)
 */
float Toggle_Calculate(enum TOGGLE_CONTRL_MODE control_mode, float set_point)
{
    if (control_mode == TOGGLE_SPEED)
    {
        toggle_controller.set_pos = toggle_controller.toggle_info.angle;
        toggle_controller.set_speed = set_point;

        PID_Clear(&toggle_controller.toggle_pos_pid);
        return PID_Calculate(&toggle_controller.toggle_speed_pid, toggle_controller.toggle_info.speed, set_point);
    }
    else if (control_mode == TOGGLE_POS)
    {
        toggle_controller.set_speed = PID_Calculate(&toggle_controller.toggle_pos_pid, toggle_controller.toggle_info.angle, set_point);
        return PID_Calculate(&toggle_controller.toggle_speed_pid, toggle_controller.toggle_info.speed, toggle_controller.set_speed);
    }
    else
    {
        PID_Clear(&toggle_controller.toggle_pos_pid);
        PID_Clear(&toggle_controller.toggle_speed_pid);
        toggle_controller.set_pos = toggle_controller.toggle_info.angle;
        toggle_controller.set_speed = 0;
        return 0;
    }
}

/**
 * @brief 拨弹电机推动一格
 * @param[in] control_mode 控制模式
 */
void ToggleAddGrid(float *set_point)
{
    //设置值为当前值加上一格大小
    *set_point = toggle_controller.set_pos + 36.0f;
}

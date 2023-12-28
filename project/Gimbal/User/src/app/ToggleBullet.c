#include "ToggleBullet.h"

ToggleController toggle_controller;

/* 根据机器人等级以及增益来确定射频 */
void selectShootFreq(uint8_t robot_level, uint8_t shoot_buff)
{
    /* 分别为机器人的等级，以及有无增益 */
    static float shoot_freq[3][2] = {{400, 500}, {500, 600}, {600, 700}};
    uint8_t robot_level_ = robot_level > 3 ? 1 : (robot_level <= 0 ? 1 : robot_level);
    uint8_t shoot_buff_ = (shoot_buff & 0x02) >> 1; //取第二位冷却增益
    toggle_controller.shoot_freq_speed = shoot_freq[robot_level_ - 1][shoot_buff_];
}

void TogglePidInit()
{
    PID_Init(&toggle_controller.toggle_pos_pid, 800.0f, 0, 0, 26, 0, 0, 0, 0, 0, 0, 1, NONE);
    PID_Init(&toggle_controller.toggle_speed_pid, C610_MAX_CURRENT, 3.0f, 0, 0.026, 0.02, 0, 0, 0, 0, 0, 1, Integral_Limit);
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
 * @brief 拨弹电机推动N格
 * @param[in] control_mode 控制模式
 */
void ToggleAddGrid(float *set_point, int8_t N)
{
    //设置值为当前值加上一格大小
    *set_point = toggle_controller.set_pos + ONE_GRID_ANGLE * N * SIGN_ROTATE;
}

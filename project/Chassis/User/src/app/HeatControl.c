#include "HeatControl.h"

HeatController heat_controller;

void HeatControl(void)
{
    if (heat_controller.heat_update_flag) //热量更新
    {
        heat_controller.available_heat = LIMIT_MAX_MIN(heat_controller.HeatMax - heat_controller.CurHeat + heat_controller.HeatCool, heat_controller.HeatMax, 0);
        heat_controller.available_shoot = heat_controller.available_heat / ONE_BULLET_HEAT;
    }
    heat_controller.shoot_flag = heat_controller.available_shoot > heat_controller.shoot_count - heat_controller.last_shoot_count ? 1 : 0;
}

void HeatUpdate(void)
{
    heat_controller.HeatMax = referee_data.Game_Robot_State.shooter_id1_17mm_cooling_limit - ONE_BULLET_HEAT; //保留一颗子弹热量
    heat_controller.HeatCool = referee_data.Game_Robot_State.shooter_id1_17mm_cooling_rate / 10;
    heat_controller.CurHeat = referee_data.Power_Heat_Data.shooter_id1_17mm_cooling_heat;

    if (heat_controller.heat_count != heat_controller.last_heat_count)
    {
        heat_controller.last_heat_count = heat_controller.heat_count;
        heat_controller.heat_update_flag = 1;
        heat_controller.last_shoot_count = heat_controller.shoot_count;
    }

    if (heat_controller.CurHeat < HeatControlThreshold * heat_controller.HeatMax)
    {
        heat_controller.shoot_flag = 1;
    }
    else
    {
        HeatControl();
    }

    heat_controller.heat_update_flag = 0; //已处理完本次热量更新
}

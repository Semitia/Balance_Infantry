#ifndef _HEAT_CONTROL_H
#define _HEAT_CONTROL_H

#include "Referee.h"

#define ONE_BULLET_HEAT 10
#define HeatControlThreshold 0.8f
#define PredictControlThreshold 0.9f

typedef struct HeatController
{
    int heat_count;
    int shoot_count;
    int last_heat_count;  //上一次更新时的热量次数
    int last_shoot_count; //上一次更新热量时的发射子弹数

    short HeatMax;  //热量上限
    short HeatCool; //枪口冷却
    short CurHeat;  //当前热量
    short available_heat;
    short shoot_num_during_update; //热量更新中发射数
    short available_shoot;         //可发射数量
    char heat_update_flag;
    char shoot_flag; //打击标志位
} HeatController;

extern HeatController heat_controller;

void HeatUpdate(void);

#endif // !_HEAT_CONTROL_H

#ifndef _BOMB_BAY_H
#define _BOMB_BAY_H

#include "bsp_servos.h"

// #define BOMBBAY_SERVOS_ON_POS 1200 //弹舱盖舵机打开位置
// #define BOMBBAY_SERVOS_OFF_POS 470 //弹舱盖舵机关闭位置

#define BOMBBAY_SERVOS_ON_POS 700   //弹舱盖舵机打开位置
#define BOMBBAY_SERVOS_OFF_POS 3050 //弹舱盖舵机关闭位置

void BombBay_Set(int position);

#define BOMB_BAY_ON BombBay_Set(BOMBBAY_SERVOS_ON_POS);
#define BOMB_BAY_OFF BombBay_Set(BOMBBAY_SERVOS_OFF_POS);

enum BOMB_BAY_STATE
{
    BOMB_BAY_COVER_ON,
    BOMB_BAY_COVER_OFF
};

extern enum BOMB_BAY_STATE bomb_bay_state;

#endif

#ifndef _FRICTION_WHEEL_H
#define _FRICTION_WHEEL_H

#include "pid.h"
#include "M2006.h" //摩擦轮电机
#include "M3508.h"

#define LEFT_FRICTION_WHEEL 0
#define RIGHT_FRICTION_WHEEL 1

#define BULLET_17MM_15MS_SPEED_L 24500
#define BULLET_17MM_18MS_SPEED_L 27300
#define BULLET_17MM_30MS_SPEED_L 43000
#define BULLET_17MM_15MS_SPEED_R BULLET_17MM_15MS_SPEED_L
#define BULLET_17MM_18MS_SPEED_R BULLET_17MM_18MS_SPEED_L
#define BULLET_17MM_30MS_SPEED_R BULLET_17MM_30MS_SPEED_L

/*  摩擦轮结构体  */
typedef struct FrictionWheel_t
{
    PID_t PidFrictionSpeed[2];
    M3508_Recv friction_motor_recv[2];
    M3508_Info friction_motor_msgs[2];
    float send_to_motor_current[2];

    float set_speed_l;
    float set_speed_r;
} FrictionWheel_t;

extern FrictionWheel_t friction_wheels;

void FrictionWheel_Init(void);
void FrictionWheel_Set(float speed1, float speed2); // 度/s
void setFrictionSpeed(int8_t shoot_level);

#endif // !_FRICTION_WHEEL_H

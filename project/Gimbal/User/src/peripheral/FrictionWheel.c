#include "FrictionWheel.h"

FrictionWheel_t friction_wheels; //两个摩擦轮

void FrictionWheel_Init()
{
    PID_Init(&friction_wheels.PidFrictionSpeed[LEFT_FRICTION_WHEEL],
             C620_MAX_SEND_CURRENT, 7000.0f, 5, 1.6, 0.003, 0, 1000, 1000, 0, 0, 1, ChangingIntegrationRate);
    PID_Init(&friction_wheels.PidFrictionSpeed[RIGHT_FRICTION_WHEEL],
             C620_MAX_SEND_CURRENT, 7000.0f, 5, 1.6, 0.003, 0, 1000, 1000, 0, 0, 1, ChangingIntegrationRate);
}

/*  根据机器人等级获得摩擦轮转速 */
void setFrictionSpeed(int8_t shoot_level)
{
    friction_wheels.set_speed_l = shoot_level >= 2 ? BULLET_17MM_30MS_SPEED_L : (shoot_level == 1 ? BULLET_17MM_18MS_SPEED_L : BULLET_17MM_15MS_SPEED_L);
    friction_wheels.set_speed_r = shoot_level >= 2 ? BULLET_17MM_30MS_SPEED_R : (shoot_level == 1 ? BULLET_17MM_18MS_SPEED_R : BULLET_17MM_15MS_SPEED_R);
}

void FrictionWheel_Set(float speed1, float speed2) // 度/s
{
    friction_wheels.send_to_motor_current[LEFT_FRICTION_WHEEL] =
        PID_Calculate(&friction_wheels.PidFrictionSpeed[LEFT_FRICTION_WHEEL],
                      friction_wheels.friction_motor_msgs[LEFT_FRICTION_WHEEL].speed, speed1);

    friction_wheels.send_to_motor_current[RIGHT_FRICTION_WHEEL] =
        PID_Calculate(&friction_wheels.PidFrictionSpeed[RIGHT_FRICTION_WHEEL],
                      friction_wheels.friction_motor_msgs[RIGHT_FRICTION_WHEEL].speed, speed2);
}

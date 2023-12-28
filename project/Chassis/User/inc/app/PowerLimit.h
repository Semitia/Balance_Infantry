#ifndef _POWER_LIMIT_H
#define _POWER_LIMIT_H

#include "SuperPower.h"
#include "Referee.h"
#include "MF9025.h"

#include "remote_control.h"

#define CURRENT_NOT_LIMIT 5000
#define BASE_CURRENT_LIMIT 3000 //基础电流限幅
#define ADD_CURRENT_LIMIT 1000  //功率充分时增添的电流限制幅值
#define WARNING_BUFFER 20.0f    //警告的缓冲能量

//预测功率 P = I^2*R + K*W*I + P0
#define MF9025_R 0.06437f
#define MF9025_K 0.002871f
#define MF9025_P0 7.675f

#define NO_LIMIT_POWER 120.0f //无限制时的设置功率 /W
#define ADD_POWER 30          //当前功率较低时，期望功率可以上去
#define LOW_POWER 10          //缓冲功率衰减

typedef struct PowerLimiter
{
    float predict_power;
    float actual_ina260_power;
    float actual_referee_power;
    float predict_send_power; //发送转矩电流会造成电机的输出功率

    float set_power;         //预期最大功率值
    float referee_max_power; //从裁判系统读到的最大功率值
    float send_torque_lower_scale;
} PowerLimiter;

extern PowerLimiter power_limiter;

void PowerLimit(MF9025 *mf9025, float *send_torque);

#endif // !_POWER_LIMIT_H

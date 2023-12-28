#ifndef _SYSTEM_IDENTIFICATION_H
#define _SYSTEM_IDENTIFICATION_H

#include "SignalGenerator.h"

/* 系统辨识结构体  */
typedef struct SI_t
{
    SinFunction signal_function;
    float start_time;
    float time;
    float amplitude; //信号幅值
    float F;         //频率
    float out;
    float initial_value;
} SI_t;

void SIInit(SI_t *SI_object, float start_time, float amplitude);
float SIRun(SI_t *SI_object, float delta_t);

/* 连续阶跃函数  */
typedef struct ContinousStep
{
    StepFunction step_function;
    float start_time;
    float time;
    float interval_time;  //每个方波持续时间
    float amplitude;      //幅值
    float amplitude_step; //幅值步长
    float out;
    int16_t round_count;
} ContinousStep;

void ContinousStepInit(ContinousStep *Step, float start_time, float amplitude, float interval_time, float amplitude_step);
float ContinousStepRun(ContinousStep *Step, float delta_t);

#endif // !_SYSTEM_IDENTIFICATION_H

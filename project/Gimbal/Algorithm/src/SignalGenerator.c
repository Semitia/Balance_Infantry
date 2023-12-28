/**
 ******************************************************************************
 * @file	 SignalGenerator.c
 * @brief    信号发生器定义
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "SignalGenerator.h"

void StepInit(StepFunction *step, float initial_value, float step_amplitude, float start_time)
{
    step->start_time = start_time;
    step->time = 0;
    step->initial_value = initial_value;
    step->step_amplitude = step_amplitude;
}
float StepRun(StepFunction *step, float delta_t)
{
    step->time += delta_t;
    if (step->time > step->start_time)
    {
        return step->initial_value + step->step_amplitude;
    }
    else
    {
        return step->initial_value;
    }
}

void SinInit(SinFunction *sin_function, float amplitude, float start_time, uint16_t T)
{
    sin_function->amplitude = amplitude;
    sin_function->start_time = start_time;
    sin_function->cycle_count = 0;
    sin_function->T = T;
    sin_function->w = 2 * PI * 1000.0f / (float)T;
    sin_function->time = 0;
}

float SinRun(SinFunction *sin_function, float delta_t)
{
    sin_function->time += delta_t;
    if (sin_function->time < sin_function->start_time)
    {
        return 0.0f;
    }
    float angle = sin_function->w * (sin_function->time - sin_function->start_time);

    if (angle > 2 * PI * (sin_function->cycle_count + 1))
    {
        sin_function->cycle_count++;
    }

    return sin_function->amplitude * arm_sin_f32(angle);
}

void SawToothInit(SawToothWave *saw_tooth_wave, float amplitude, float start_time, uint16_t T, float initial_value)
{
    saw_tooth_wave->T = T;
    saw_tooth_wave->amplitude = amplitude;
    saw_tooth_wave->start_time = start_time;
    saw_tooth_wave->time = 0;
    saw_tooth_wave->cycle_count = 0;
    saw_tooth_wave->initial_value = initial_value;
    saw_tooth_wave->out = initial_value;
    saw_tooth_wave->slope = amplitude * 1000.0f / (float)T;
}

float SawWaveRun(SawToothWave *saw_tooth_wave, float delta_t)
{
    saw_tooth_wave->time += delta_t;
    if (saw_tooth_wave->time < saw_tooth_wave->start_time)
    {
        saw_tooth_wave->out = saw_tooth_wave->initial_value;
        return saw_tooth_wave->initial_value;
    }

    if ((saw_tooth_wave->time - saw_tooth_wave->start_time) > (saw_tooth_wave->cycle_count + 1) * (float)saw_tooth_wave->T / 1000.0f)
    {
        saw_tooth_wave->cycle_count++;
        saw_tooth_wave->out = saw_tooth_wave->initial_value;
    }

    saw_tooth_wave->out += saw_tooth_wave->slope * delta_t;
    return saw_tooth_wave->out;
}

#include "SystemIdentification.h"

void SIInit(SI_t *SI_object, float start_time, float amplitude)
{
    SI_object->amplitude = amplitude;
    SI_object->F = 1;
    SI_object->start_time = start_time;
    SI_object->time = 0;
    SI_object->out = 0;
    SI_object->initial_value = 0;
    SinInit(&SI_object->signal_function, amplitude, 0, (uint16_t)(1000.0f / SI_object->F));
}

float SIRun(SI_t *SI_object, float delta_t)
{
    if (delta_t > 0.01f)
    {
        delta_t = 0.001f;
    }
    //幅值区间
    static float F_set[] = {1, 24, 40, 120, 200, 250, 333, 500};
    //幅值间隔，长度为F_set长度-1
    static float F_space_set[] = {0.5, 2, 10, 80, 50, 333, 167};

    SI_object->time += delta_t;

    //时间没到或者频率已经超出范围
    if (SI_object->time < SI_object->start_time || SI_object->F - F_set[sizeof(F_set) / sizeof(float) - 1] > -0.1)
    {
        SI_object->out = SI_object->initial_value;
        return SI_object->out;
    }
    SI_object->out = SI_object->initial_value + SinRun(&SI_object->signal_function, delta_t);

    if (SI_object->signal_function.cycle_count >= 20) //经历一定周期之后
    {
        for (int i = 1; i < sizeof(F_set) / sizeof(float); i++)
        {
            if (SI_object->F < F_set[i])
            {
                SI_object->F += F_space_set[i - 1];
                SinInit(&SI_object->signal_function, SI_object->amplitude, 0, (uint16_t)(1000.0f / SI_object->F));
                break;
            }
        }
    }
    return SI_object->out;
}

void ContinousStepInit(ContinousStep *Step, float start_time, float amplitude, float interval_time, float amplitude_step)
{
    Step->amplitude = amplitude;
    Step->amplitude_step = amplitude_step;
    Step->out = 0;
    Step->interval_time = interval_time;
    Step->time = 0;
    Step->start_time = start_time;
    Step->round_count = 0;
    StepInit(&Step->step_function, 0.0f, amplitude, 0.0f);
}
float ContinousStepRun(ContinousStep *Step, float delta_t)
{
    Step->time += delta_t;
    if (Step->time < Step->start_time)
    {
        Step->out = 0.0f;
        return Step->out;
    }

    if (Step->time - Step->start_time > (Step->round_count + 1) * Step->interval_time)
    {
        Step->round_count++;
        Step->amplitude += Step->amplitude_step;
        StepInit(&Step->step_function, 0.0f, Step->amplitude, 0.0f);
    }

    Step->out = StepRun(&Step->step_function, delta_t);
    return Step->out;
}

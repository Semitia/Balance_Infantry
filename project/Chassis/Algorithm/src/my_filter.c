#include "my_filter.h"

void iir(float *raw_data, float new_data, float filter_value)
{
    *raw_data = *raw_data * filter_value + new_data * (1 - filter_value);
}

/*  一阶低通滤波器  */
void rc_filter(float *raw_data, float new_data, float delta_t, float LPF_RC)
{
    *raw_data = new_data * delta_t / (LPF_RC + delta_t) +
                *raw_data * LPF_RC / (LPF_RC + delta_t);
}

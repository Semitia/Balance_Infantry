#ifndef _FILTER_H
#define _FILTER_H

void iir(float *raw_data, float new_data, float filter_value);
void rc_filter(float *raw_data, float new_data, float delta_t, float LPF_RC);

#endif // !_FILTER_H

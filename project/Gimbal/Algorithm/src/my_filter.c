#include "my_filter.h"

void iir(float *raw_data, float new_data, float filter_value)
{
    *raw_data = *raw_data * filter_value + new_data * (1 - filter_value);
}

#include "PowerControlTask.h"

uint32_t test_cnt;
float test_t;
void PowerControlTask(void *pvParameters)
{
    portTickType xLastWakeTime;

    SuperPowerInit();

    while (1)
    {
        // GetDeltaT(&test_cnt);

        INA_READ();

        ADC_Filter();

        // test_t = GetDeltaT(&test_cnt);

        ChargeControl();

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}

#ifndef _POWER_CONTROL_TASK_H
#define _POWER_CONTROL_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "ina260.h"
#include "SuperPower.h"
#include "counter.h"

void PowerControlTask(void *pvParameters);

#endif // !_POWER_CONTROL_TASK_H

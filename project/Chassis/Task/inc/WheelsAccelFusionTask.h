#ifndef _WHEEL_ACCEL_FUSION_H
#define _WHEEL_ACCEL_FUSION_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "wheel_ins.h"
#include "ChasisController.h"

void WheelsAccelFusion_task(void *pvParameters);
#endif // !_WHEEL_ACCEL_FUSION_H

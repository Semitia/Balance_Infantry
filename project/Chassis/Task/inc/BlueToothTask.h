#ifndef _BLUE_TOOTH_TASK_H
#define _BLUE_TOOTH_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "bluetooth.h"
#include "ChasisControlTask.h"

#include "RobotAbnormalDetector.h"

void BlueToothTask(void *pvParameters);

#endif // !_BLUE_TOOTH_TASK_H

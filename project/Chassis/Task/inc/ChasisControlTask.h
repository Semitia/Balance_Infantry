#ifndef _CHASIS_CONTROL_H
#define _CHASIS_CONTROL_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "ChasisController.h"
#include "wheel_ins.h"

#include "iwdg.h"

#include "SuperPower.h"
#include "RobotAbnormalDetector.h"

void ChasisControl_task(void *pvParameters);

#endif // !_CHASIS_CONTROL_H

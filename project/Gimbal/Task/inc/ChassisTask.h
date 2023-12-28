#ifndef _CHASSIS_TASK_H
#define _CHASSIS_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "ChassisSolver.h"
#include "can_send.h"
#include "can_config.h"

#include "ChassisSend.h"

void ChassisTask(void *pvParameters);

#endif // !_CHASSIS_TASK_H

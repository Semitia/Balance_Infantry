#ifndef _ACTION_TASK_H
#define _ACTION_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "iwdg.h"
#include "GimbalReceive.h"

void Action_task(void *pvParameters);

#endif // !_ACTION_TASK_H

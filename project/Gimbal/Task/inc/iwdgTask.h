#ifndef _IWDG_TASK_H
#define _IWDG_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "iwdg.h"

#define IWDG_TASK_ON 0

void Iwdg_task(void *pvParameters);

#endif // !_IWDG_TASK_H

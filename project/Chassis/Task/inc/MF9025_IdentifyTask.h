#ifndef _MF9025_IDENTIFY_TASK_H
#define _MF9025_IDENTIFY_TASK_H

#define MF9025_IDENTIFY_ON 0

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "MF9025.h"
#include "RLS_Identification.h"
#include "counter.h"

#include "RobotAbnormalDetector.h"

void MF9025_IdentifyTask(void *pvParameters);

#endif // !_MF9025_IDENTIFY_TASK_H

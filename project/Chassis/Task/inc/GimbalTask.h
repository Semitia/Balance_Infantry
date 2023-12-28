#ifndef _GIMBAL_TASK_H
#define _GIMBAL_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "remote_control.h"
#include "can_config.h"
#include "GM6020.h"
#include "M2006.h"
#include "ToggleBullet.h"
#include "can_send.h"
#include "Gimbal.h"

#include "HeatControl.h"
#include "GimbalSend.h"

#include "SystemIdentification.h"

void GimbalTask(void *pvParameters);

#endif // !_GIMBAL_TASK_H

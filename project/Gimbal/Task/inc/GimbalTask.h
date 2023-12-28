#ifndef _GIMBAL_TASK_H
#define _GIMBAL_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "remote_control.h"
#include "Gimbal.h"
#include "FrictionWheel.h"
#include "can_config.h"
#include "can_send.h"

#include "M3508.h"
#include "ins.h"

#include "BombBay.h"
#include "SystemIdentification.h"
#include "SignalGenerator.h"

#include "pc_serial.h"

#define GYRO_PITCH_BIAS 0.0f

void GimbalTask(void *pvParameters);

#endif // !_GIMBAL_TASK_H

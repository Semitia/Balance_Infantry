#ifndef __GRAPHICS_SEND_TASK_H
#define __GRAPHICS_SEND_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "math.h"

#include "stdint.h"
#include "GimbalReceive.h"
#include "SuperPower.h"

#include "bsp_referee.h"
#include "Referee.h"

#include "algorithmOfCRC.h"
#include "remote_control.h"
#include "ins.h"
#include "ChasisController.h"

void Refereetask(void *pvParameters);

#endif

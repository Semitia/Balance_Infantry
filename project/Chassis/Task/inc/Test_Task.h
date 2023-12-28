#ifndef _TEST_TASK_H
#define _TEST_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "can1.h"
#include "can2.h"
#include "motor_rs485.h"
#include "pc_uart.h"
#include "wifi.h"
#include "bluetooth.h"
#include "led.h"
#include "ina260.h"

#include "ChasisController.h"

#define TEST_TASK_ON 0

void Test_task(void *pvParameters);

#endif // _TEST_TASK_H

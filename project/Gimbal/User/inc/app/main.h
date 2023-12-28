#ifndef __MAIN_H
#define __MAIN_H
/* Library */
#include <stm32f4xx.h>
#include <stm32f4xx_conf.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "arm_math.h"

/* Mylib */
#include "os_tick.h"
#include "can2.h"
#include "can1.h"
#include "counter.h"
#include "pc_uart.h"
#include "bluetooth.h"
#include "remote_control.h"
#include "wifi.h"
#include "iwdg.h"
#include "dji_remote.h"
#include "bsp_servos.h"
#include "imu_uart.h"

/* Algorithm */

/* Task */
#include "Start_Task.h"
#include "CPU_Task.h"

/* Os */
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

/* tools */
#include "tools.h"

/* init */
void BSP_Init(void);
void Robot_Init(void);
void startTask(void);

#endif

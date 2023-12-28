#ifndef _START_TASK_H
#define _START_TASK_H
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "os_tick.h"

#include "iwdg.h"

/* Task */
#include "GimbalEstimateTask.h"
#include "CPU_Task.h"
#include "Test_Task.h"
#include "Offline_Task.h"
#include "SDCardTask.h"
#include "BlueToothTask.h"
#include "iwdgTask.h"
#include "GimbalTask.h"
#include "ActionTask.h"
#include "ChassisTask.h"

/* function */

/*  根据创建的任务列举Task Handle */
enum TASK_LIST
{
    GIMBAL_ESTIMATE_TASK,
    CPU_TASK,
    TEST_TASK,
    OFFLINE_TASK,
    SDCARD_TASK,
    ACTION_TASK,
    GIMBAL_TASK,
    CHASSIS_TASK,
    BLUE_TOOTH_TASK,
    IWDG_TASK,
    TASK_NUM,
};

void startTask(void);

#endif // !_START_TASK_H

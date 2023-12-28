#ifndef _START_TASK_H
#define _START_TASK_H
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "os_tick.h"

#include "iwdg.h"

/* Task */
#include "ChasisEstimateTask.h"
#include "CPU_Task.h"
#include "Test_Task.h"
#include "ChasisControlTask.h"
#include "WheelsAccelFusionTask.h"
#include "Offline_Task.h"
#include "SDCardTask.h"
#include "MF9025_IdentifyTask.h"
#include "BlueToothTask.h"
#include "iwdgTask.h"
#include "ActionTask.h"
#include "GimbalTask.h"
#include "RefereeTask.h"
#include "PowerControlTask.h"

/* function */

/*  根据创建的任务列举Task Handle */
enum TASK_LIST
{
    CHASISESTIMATE_TASK,
    PC_TASK,
    CPU_TASK,
    TEST_TASK,
    CHASISCONTROL_TASK,
    OFFLINE_TASK,
    WHEELS_ACCEL_FUSION_TASK,
    SDCARD_TASK,
    MF9025_SYSTEM_IDENTIFY_TASK,
    BLUE_TOOTH_TASK,
    ACTION_TASK,
    IWDG_TASK,
    GIMBAL_TASK,
    REFEREE_TASK,
    POWER_CONTROL_TASK,
    TASK_NUM,
};

void startTask(void);

#endif // !_START_TASK_H

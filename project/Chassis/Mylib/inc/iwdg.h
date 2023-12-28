#ifndef __IWDG_H
#define __IWDG_H

#include "stm32f4xx_iwdg.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

void IWDG_Config(uint8_t prv, uint16_t rlv);
void IWDG_Feed(void);

// 给重要任务分配位，保证每一个重要任务都不掉线
#define TASK_BIT_0 (0x01 << 0)
#define TASK_BIT_1 (0x01 << 1)
#define TASK_BIT_2 (0x01 << 2)
#define TASK_BIT_3 (0x01 << 3)
#define TASK_BIT_4 (0x01 << 4)
#define TASK_BIT_5 (0x01 << 5)
#define TASK_BIT_6 (0x01 << 6)
#define TASK_BIT_7 (0x01 << 7)

#define CHASIS_ESTIMATE_BIT TASK_BIT_0 // 状态估计任务
#define CHASIS_CONTROL_BIT TASK_BIT_1  // 底盘控制任务
#define GIMBAL_TASK_BIT TASK_BIT_2
#define REFEREE_TASK_BIT TASK_BIT_3

#define TASK_BIT_ALL (CHASIS_ESTIMATE_BIT | CHASIS_CONTROL_BIT | GIMBAL_TASK_BIT | REFEREE_TASK_BIT)

extern EventGroupHandle_t xCreatedEventGroup; // 声明事件组

#endif

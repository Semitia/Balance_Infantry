/**
 ******************************************************************************
 * @file    Start_Task.c
 * @brief   初始化任务创建
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "Start_Task.h"

/*任务优先级数值越低，优先级越低*/

//初始化任务
#define START_TASK_PRIO 1       //任务优先级
#define START_TASK_STK_SIZE 128 //任务堆栈
static TaskHandle_t StartTask_Handler;

//底盘位姿估计任务
#define GimbalEstimate_task_PRIO 20      //任务优先级
#define GimbalEstimate_task_STK_SIZE 128 //任务堆栈

// CPU占用情况估计任务
#define CPU_TASK_PRIO 2       //任务优先级
#define CPU_TASK_STK_SIZE 128 // 任务堆栈

// 测试任务
#define TEST_TASK_PRIO 5
#define TEST_TASK_STK_SIZE 128 //任务堆栈

#define OFFLINE_TASK_PRIO 15
#define OFFLINE_TASK_STK_SIZE 128

#define SD_CARD_TASK_PRIO 20
#define SD_CARD_TASK_STK_SIZE 128

#define BLUE_TOOTH_TASK_PRIO 24
#define BLUE_TOOTH_TASK_STK_SIZE 128

#define IWDG_TASK_PRIO 3
#define IWDG_TASK_STK_SIZE 128

#define GIMBAL_TASK_PRIO 8
#define GIMBAL_TASK_STK_SIZE 128

#define ACTION_TASK_PRIO 7
#define ACTION_TASK_STK_SIZE 128

#define CHASSIS_TASK_PRIO 12
#define CHASSIS_TASK_STK_SIZE 128

#define PC_TASK_PRIO 10
#define PC_TASK_STK_SIZE 128

TaskHandle_t User_Tasks[TASK_NUM];

/**
 * @brief 初始化任务,创建其它任务
 * @param[in] void
 */
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();
#if TEST_TASK_ON
    xTaskCreate((TaskFunction_t)Test_task,               //任务函数
                (const char *)"Test_task",               //任务名称
                (uint16_t)TEST_TASK_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                            //传递给任务函数的参数
                (UBaseType_t)TEST_TASK_PRIO,             //任务优先级
                (TaskHandle_t *)&User_Tasks[TEST_TASK]); //任务句柄
#else

    //创建底盘位姿估计任务
    xTaskCreate((TaskFunction_t)GimbalEstimate_task,                //任务函数
                (const char *)"GimbalEstimate_task",                //任务名称
                (uint16_t)GimbalEstimate_task_STK_SIZE,             //任务堆栈大小
                (void *)NULL,                                       //传递给任务函数的参数
                (UBaseType_t)GimbalEstimate_task_PRIO,              //任务优先级
                (TaskHandle_t *)&User_Tasks[GIMBAL_ESTIMATE_TASK]); //任务句柄

    // //创建掉线检测任务
    xTaskCreate((TaskFunction_t)Offline_task,               //任务函数
                (const char *)"Offline_task",               //任务名称
                (uint16_t)OFFLINE_TASK_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                               //传递给任务函数的参数
                (UBaseType_t)OFFLINE_TASK_PRIO,             //任务优先级
                (TaskHandle_t *)&User_Tasks[OFFLINE_TASK]); //任务句柄

    // //创建云台控制任务
    xTaskCreate((TaskFunction_t)GimbalTask,                //任务函数
                (const char *)"GimbalTask",                //任务名称
                (uint16_t)GIMBAL_TASK_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                              //传递给任务函数的参数
                (UBaseType_t)GIMBAL_TASK_PRIO,             //任务优先级
                (TaskHandle_t *)&User_Tasks[GIMBAL_TASK]); //任务句柄

    xTaskCreate((TaskFunction_t)ActionTask,                //任务函数
                (const char *)"ActionTask",                //任务名称
                (uint16_t)ACTION_TASK_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                              //传递给任务函数的参数
                (UBaseType_t)ACTION_TASK_PRIO,             //任务优先级
                (TaskHandle_t *)&User_Tasks[ACTION_TASK]); //任务句柄

    xTaskCreate((TaskFunction_t)ChassisTask,                //任务函数
                (const char *)"ChassisTask",                //任务名称
                (uint16_t)CHASSIS_TASK_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                               //传递给任务函数的参数
                (UBaseType_t)CHASSIS_TASK_PRIO,             //任务优先级
                (TaskHandle_t *)&User_Tasks[CHASSIS_TASK]); //任务句柄

    //创建蓝牙传输任务
    // xTaskCreate((TaskFunction_t)BlueToothTask,              //任务函数
    //             (const char *)"BlueToothTask",              //任务名称
    //             (uint16_t)BLUE_TOOTH_TASK_STK_SIZE,         //任务堆栈大小
    //             (void *)NULL,                               //传递给任务函数的参数
    //             (UBaseType_t)BLUE_TOOTH_TASK_PRIO,          //任务优先级
    //             (TaskHandle_t *)&User_Tasks[OFFLINE_TASK]); //任务句柄

#if IWDG_TASK_ON
    xTaskCreate((TaskFunction_t)Iwdg_task,               //任务函数
                (const char *)"Iwdg_task",               //任务名称
                (uint16_t)IWDG_TASK_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                            //传递给任务函数的参数
                (UBaseType_t)IWDG_TASK_PRIO,             //任务优先级
                (TaskHandle_t *)&User_Tasks[IWDG_TASK]); //任务句柄
#endif // IWDG_TASK_ON

    // xTaskCreate((TaskFunction_t)SDCard_task,               //任务函数
    //             (const char *)"SDCard_task",               //任务名称
    //             (uint16_t)SD_CARD_TASK_STK_SIZE,           //任务堆栈大小
    //             (void *)NULL,                              //传递给任务函数的参数
    //             (UBaseType_t)SD_CARD_TASK_PRIO,            //任务优先级
    //             (TaskHandle_t *)&User_Tasks[SDCARD_TASK]); //任务句柄

    /* 最后创建  */
#ifdef DEBUG_MODE_FREERTOS
    xTaskCreate((TaskFunction_t)CPU_task,               //任务函数
                (const char *)"CPU_task",               //任务名称
                (uint16_t)CPU_TASK_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                           //传递给任务函数的参数
                (UBaseType_t)CPU_TASK_PRIO,             //任务优先级
                (TaskHandle_t *)&User_Tasks[CPU_TASK]); //任务句柄
#endif // DEBUG_MODE_FREERTOS
#endif // TEST_TASK_ON

    xCreatedEventGroup = xEventGroupCreate(); //创建任务组

    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}

/**
 * @brief 创建初始化任务
 * @param[in] void
 */
void startTask(void)
{
    xTaskCreate((TaskFunction_t)start_task,          //任务函数
                (const char *)"start_task",          //任务名称
                (uint16_t)START_TASK_STK_SIZE,       //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数
                (UBaseType_t)START_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&StartTask_Handler); //任务句柄
}

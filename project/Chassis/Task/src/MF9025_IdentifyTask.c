/**
 ******************************************************************************
 * @file    MF9025_IdentifyTask.c
 * @brief   MF9025转动惯量系统辨识程序，递归最小二乘法暂时没有调通(转动惯量收敛很慢)
 *          故使用了记录w以及T的办法，利用Matlab拟合转动惯量以及运动阻尼系数
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "MF9025_IdentifyTask.h"

#if MF9025_IDENTIFY_ON

// RLS
RLS rls_identification;
float x[2];
float w[3] = {0}; //分别为w_k w_k-1 w_k-2
float w_a;
float T[3] = {0}; //分别为T_k T_k-1 T_k-2
MF9025_Broadcast wheels;
float wheels_torque[2] = {0};
float t_;

//在线辨识失败，原因未知，忽略负载转矩再试一试
void MF9025_IdentifyTask(void *pvParameters)
{
    portTickType xLastWakeTime;

    //适当延时
    vTaskDelay(1000);

    WheelOffPack(&wheels);

    // RLS_Init(&rls_identification, 2, 1, 0.996);
    // rls_identification.x_data[0] = 2148.0;
    // rls_identification.x_data[1] = -2148.0;

    // rls_identification.P_data[0] = 1000;
    // rls_identification.P_data[3] = 1000;

    //    int identify_id = 0; //辨识的电机ID
    uint32_t last_cnt;
    GetDeltaT(&last_cnt);
    MF9025DetectInit();

    while (1)
    {
        t_ = GetDeltaTtoNow(&last_cnt);
        wheels_torque[0] = 0.25f * arm_sin_f32(t_);
        wheels_torque[1] = 0.25f * arm_sin_f32(t_);
        //更新w,T信息反馈
        // for (int i = 0; i < 2; i++)
        // {
        //     w[i + 1] = w[i];
        //     T[i + 1] = T[i];
        // }
        MF9025_Decode(&Wheels_Motor.decoded_msgs[LEFT_WHEEL_ID], &Wheels_Motor.recv_msgs[LEFT_WHEEL_ID]);
        MF9025_Decode(&Wheels_Motor.decoded_msgs[RIGHT_WHEEL_ID], &Wheels_Motor.recv_msgs[RIGHT_WHEEL_ID]);

        w[0] = Wheels_Motor.decoded_msgs[0].rotate_speed * ANGLE_TO_RAD_COEF;
        // w_a = (w[0] - w[1]) / 0.001f;
        T[0] = Wheels_Motor.decoded_msgs[0].current * MF9025_TORQUE_CURRENT_RATIO;
        w[1] = Wheels_Motor.decoded_msgs[1].rotate_speed * ANGLE_TO_RAD_COEF;
        T[1] = Wheels_Motor.decoded_msgs[1].current * MF9025_TORQUE_CURRENT_RATIO;

        MF9025StateDetect(w[0], w[1], T[0], T[1], 0.001);

        //更新矩阵信息
        // rls_identification.H_data[0] = 0.001f * T[1];
        // rls_identification.H_data[1] = 0.001f * T[2];

        // rls_identification.y_data[0] = -2 * w[1] + w[0] + w[2];

        // RLS_Update(&rls_identification);

        // x[0] = rls_identification.x_data[0];
        // x[1] = rls_identification.x_data[1];

        WheelCurrentPack(&wheels, wheels_torque);
        WheelCanSend(&wheels);

        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}

#endif // DEBUG

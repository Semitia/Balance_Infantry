#ifndef _WHEEL_INS_H
#define _WHEEL_INS_H

#include "kalman_filter.h"

/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

typedef struct
{
    uint8_t Initialized;
    KalmanFilter_t KF_Wheel_Accel;

    float Q1; //位置估计过程噪声
    float Q2; //速度估计过程噪声
    float R1; //速度测量噪声

    float x;
    float x_v;

    float dt; // 姿态更新周期

    //卡方检验
    mat ChiSquare;
    float ChiSquare_Data[1];      // 卡方检验检测函数
    float ChiSquareTestThreshold; // 卡方检验阈值

} Wheel_Accel_Fusion_t; //轮式里程计与加速度计数据融合

extern Wheel_Accel_Fusion_t Wheel_Accel_Fusion;
void KF_Wheel_Accel_Init(float process_noise1, float process_noise2, float measure_noise);
void KF_Wheel_Accel_Update(float xv, float xa, float dt);

#endif // !_WHEEL_INS_H

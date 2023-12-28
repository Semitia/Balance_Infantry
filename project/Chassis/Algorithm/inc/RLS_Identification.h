#ifndef _RLS_IDENTIFICATION
#define _RLS_IDENTIFICATION

#include <stm32f4xx.h>
#include <stm32f4xx_conf.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "arm_math.h"

/* 递推最小二乘法定义  */
typedef struct
{
    uint8_t xhatSize; //状态量
    uint8_t ySize;    //测量量
    float lambda;

    float *K_data;
    float *y_data;
    float *H_data;
    float *x_data;
    float *x_minus_data;
    float *P_data;
    float *I_data_xx;
    float *I_data_yy;
    float *H_T_data;
    float *P_minus_data;
    float *K_temp_1_data, *K_temp_2_data, *K_temp_3_data, *K_temp_4_data; //求解K中的中间变量
    float *P_temp_1_data, *P_temp_2_data;                                 //求解P中的中间变量
    float *x_temp_1_data, *x_temp_2_data, *x_temp_3_data;

    arm_matrix_instance_f32 K;
    arm_matrix_instance_f32 y;
    arm_matrix_instance_f32 H;
    arm_matrix_instance_f32 x;
    arm_matrix_instance_f32 x_minus;
    arm_matrix_instance_f32 P;
    arm_matrix_instance_f32 P_minus;
    arm_matrix_instance_f32 I_xx;
    arm_matrix_instance_f32 I_yy;

    arm_matrix_instance_f32 H_T; // H的转置

    arm_matrix_instance_f32 K_temp_1, K_temp_2, K_temp_3, K_temp_4; //求解K中的中间变量
    arm_matrix_instance_f32 P_temp_1, P_temp_2;                     //求解P中的中间变量
    arm_matrix_instance_f32 x_temp_1, x_temp_2, x_temp_3;

    int8_t MatStatus;

} RLS;

void RLS_Init(RLS *rls_identification, uint8_t x_hat_size, uint8_t y_size, float lambda);
void RLS_Update(RLS *rls_identification);

#endif // !_RLS_IDENTIFICATION

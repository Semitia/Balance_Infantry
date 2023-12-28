#ifndef _OBSERVER_H
#define _OBSERVER_H

#include <stm32f4xx.h>
#include <stm32f4xx_conf.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "arm_math.h"

typedef struct
{
    uint8_t xhatSize; //状态量
    uint8_t uSize;    //控制量
    uint8_t ySize;    //测量量

    float *A_data;
    float *x_data;
    float *B_data;
    float *u_data;
    float *C_data;
    float *L_data;
    float *y_data;

    arm_matrix_instance_f32 A;
    arm_matrix_instance_f32 x;
    arm_matrix_instance_f32 B;
    arm_matrix_instance_f32 u;
    arm_matrix_instance_f32 C;
    arm_matrix_instance_f32 L;
    arm_matrix_instance_f32 y;

    arm_matrix_instance_f32 Ax;
    arm_matrix_instance_f32 Bu;
    arm_matrix_instance_f32 Cx;
    arm_matrix_instance_f32 y_Cx;
    arm_matrix_instance_f32 L_y_Cx;
    arm_matrix_instance_f32 Ax_Bu;

    float *Ax_data;
    float *Bu_data;
    float *Cx_data;
    float *y_Cx_data;
    float *L_y_Cx_data;
    float *Ax_Bu_data;

    int8_t MatStatus;

} StateObserver_t;

void ObserverInit(StateObserver_t *observer, uint8_t xhatSize, uint8_t uSize, uint8_t ySize);
void ObserverUpdate(StateObserver_t *observer);
#endif // !_OBSERVER_H

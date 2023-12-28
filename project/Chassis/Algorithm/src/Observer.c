#include "Observer.h"

void ObserverInit(StateObserver_t *observer, uint8_t xhatSize, uint8_t uSize, uint8_t ySize)
{
    float sizeof_float = sizeof(float);
    observer->xhatSize = xhatSize;
    observer->uSize = uSize;
    observer->ySize = ySize;

    // A
    observer->A_data = (float *)malloc(sizeof_float * xhatSize * xhatSize);
    memset(observer->A_data, 0, sizeof_float * xhatSize * xhatSize);
    arm_mat_init_f32(&observer->A, xhatSize, xhatSize, (float *)observer->A_data);

    // B
    observer->B_data = (float *)malloc(sizeof_float * xhatSize * uSize);
    memset(observer->B_data, 0, sizeof_float * xhatSize * uSize);
    arm_mat_init_f32(&observer->B, xhatSize, uSize, (float *)observer->B_data);

    // C
    observer->C_data = (float *)malloc(sizeof_float * xhatSize * ySize);
    memset(observer->C_data, 0, sizeof_float * xhatSize * ySize);
    arm_mat_init_f32(&observer->C, ySize, xhatSize, (float *)observer->C_data);

    // x
    observer->x_data = (float *)malloc(sizeof_float * xhatSize);
    memset(observer->x_data, 0, sizeof_float * xhatSize);
    arm_mat_init_f32(&observer->x, xhatSize, 1, (float *)observer->x_data);

    // u
    observer->u_data = (float *)malloc(sizeof_float * uSize);
    memset(observer->u_data, 0, sizeof_float * uSize);
    arm_mat_init_f32(&observer->u, uSize, 1, (float *)observer->u_data);

    // L
    observer->L_data = (float *)malloc(sizeof_float * xhatSize * ySize);
    memset(observer->L_data, 0, sizeof_float * xhatSize * ySize);
    arm_mat_init_f32(&observer->L, xhatSize, ySize, (float *)observer->L_data);

    // y
    observer->y_data = (float *)malloc(sizeof_float * ySize);
    memset(observer->y_data, 0, sizeof_float * ySize);
    arm_mat_init_f32(&observer->y, ySize, 1, (float *)observer->y_data);

    // A * x
    observer->Ax_data = (float *)malloc(sizeof_float * xhatSize);
    arm_mat_init_f32(&observer->Ax, xhatSize, 1, (float *)observer->Ax_data);

    // B * u
    observer->Bu_data = (float *)malloc(sizeof_float * xhatSize);
    arm_mat_init_f32(&observer->Bu, xhatSize, 1, (float *)observer->Bu_data);

    // C * x
    observer->Cx_data = (float *)malloc(sizeof_float * ySize);
    memset(observer->Cx_data, 0, sizeof_float * ySize);
    arm_mat_init_f32(&observer->Cx, ySize, 1, (float *)observer->Cx_data);

    // y - Cx
    observer->y_Cx_data = (float *)malloc(sizeof_float * ySize);
    memset(observer->y_Cx_data, 0, sizeof_float * ySize);
    arm_mat_init_f32(&observer->y_Cx, ySize, 1, (float *)observer->y_Cx_data);

    // L(y - Cx)
    observer->L_y_Cx_data = (float *)malloc(sizeof_float * xhatSize);
    arm_mat_init_f32(&observer->L_y_Cx, xhatSize, 1, (float *)observer->L_y_Cx_data);

    // Ax + Bu
    observer->Ax_Bu_data = (float *)malloc(sizeof_float * xhatSize);
    arm_mat_init_f32(&observer->Ax_Bu, xhatSize, 1, (float *)observer->Ax_Bu_data);
}

void ObserverUpdate(StateObserver_t *observer)
{
    observer->MatStatus = arm_mat_mult_f32(&observer->A, &observer->x, &observer->Ax);

    observer->MatStatus = arm_mat_mult_f32(&observer->B, &observer->u, &observer->Bu);

    observer->MatStatus = arm_mat_mult_f32(&observer->C, &observer->x, &observer->Cx);

    observer->MatStatus = arm_mat_sub_f32(&observer->y, &observer->Cx, &observer->y_Cx);

    observer->MatStatus = arm_mat_mult_f32(&observer->L, &observer->y_Cx, &observer->L_y_Cx);

    observer->MatStatus = arm_mat_add_f32(&observer->Ax, &observer->Bu, &observer->Ax_Bu);

    observer->MatStatus = arm_mat_add_f32(&observer->Ax_Bu, &observer->L_y_Cx, &observer->x);
}

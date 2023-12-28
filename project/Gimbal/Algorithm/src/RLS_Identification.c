#include "RLS_Identification.h"

/**
 * @brief 递推最小二乘法初始化
 * @param[in] rls_identification
 * @param[in] x_hat_size 状态维度
 * @param[in] y_size 测量维度
 * @param[in] lambda 遗忘因子
 */
void RLS_Init(RLS *rls_identification, uint8_t x_hat_size, uint8_t y_size, float lambda)
{
    float sizeof_float = sizeof(float);
    rls_identification->xhatSize = x_hat_size;
    rls_identification->ySize = y_size;
    rls_identification->lambda = lambda;

    // x_hat
    rls_identification->x_data = (float *)malloc(sizeof_float * x_hat_size);
    memset(rls_identification->x_data, 0, sizeof_float * x_hat_size);
    arm_mat_init_f32(&rls_identification->x, x_hat_size, 1, (float *)rls_identification->x_data);

    // y
    rls_identification->y_data = (float *)malloc(sizeof_float * y_size);
    memset(rls_identification->y_data, 0, sizeof_float * y_size);
    arm_mat_init_f32(&rls_identification->y, y_size, 1, (float *)rls_identification->y_data);

    // P
    rls_identification->P_data = (float *)malloc(sizeof_float * x_hat_size * x_hat_size);
    memset(rls_identification->P_data, 0, sizeof_float * x_hat_size * x_hat_size);
    arm_mat_init_f32(&rls_identification->P, x_hat_size, x_hat_size, (float *)rls_identification->P_data);

    // H
    rls_identification->H_data = (float *)malloc(sizeof_float * x_hat_size * y_size);
    memset(rls_identification->H_data, 0, sizeof_float * x_hat_size * y_size);
    arm_mat_init_f32(&rls_identification->H, x_hat_size, y_size, (float *)rls_identification->H_data);

    // H^T
    rls_identification->H_T_data = (float *)malloc(sizeof_float * x_hat_size * y_size);
    memset(rls_identification->H_T_data, 0, sizeof_float * x_hat_size * y_size);
    arm_mat_init_f32(&rls_identification->H_T, y_size, x_hat_size, (float *)rls_identification->H_T_data);

    // K
    rls_identification->K_data = (float *)malloc(sizeof_float * x_hat_size * y_size);
    memset(rls_identification->K_data, 0, sizeof_float * x_hat_size * y_size);
    arm_mat_init_f32(&rls_identification->K, x_hat_size, y_size, (float *)rls_identification->K_data);

    // I
    rls_identification->I_data_xx = (float *)malloc(sizeof_float * x_hat_size * x_hat_size);
    memset(rls_identification->I_data_xx, 0, sizeof_float * x_hat_size * x_hat_size);
    arm_mat_init_f32(&rls_identification->I_xx, x_hat_size, x_hat_size, (float *)rls_identification->I_data_xx);

    for (int i = 0; i < x_hat_size; i++)
    {
        rls_identification->I_data_xx[i * x_hat_size + i] = 1;
    }

    rls_identification->I_data_yy = (float *)malloc(sizeof_float * y_size * y_size);
    memset(rls_identification->I_data_yy, 0, sizeof_float * y_size * y_size);
    arm_mat_init_f32(&rls_identification->I_yy, y_size, y_size, (float *)rls_identification->I_data_yy);

    for (int i = 0; i < y_size; i++)
    {
        rls_identification->I_data_yy[i * y_size + i] = 1;
    }

    //中间变量
    rls_identification->P_minus_data = (float *)malloc(sizeof_float * x_hat_size * x_hat_size);
    arm_mat_init_f32(&rls_identification->P_minus, x_hat_size, x_hat_size, (float *)rls_identification->P_minus_data);

    rls_identification->x_minus_data = (float *)malloc(sizeof_float * x_hat_size);
    arm_mat_init_f32(&rls_identification->x_minus, x_hat_size, 1, (float *)rls_identification->x_minus_data);

    rls_identification->K_temp_1_data = (float *)malloc(sizeof_float * x_hat_size * y_size);
    arm_mat_init_f32(&rls_identification->K_temp_1, x_hat_size, y_size, (float *)rls_identification->K_temp_1_data);

    rls_identification->K_temp_2_data = (float *)malloc(sizeof_float * y_size * y_size);
    arm_mat_init_f32(&rls_identification->K_temp_2, y_size, y_size, (float *)rls_identification->K_temp_2_data);

    rls_identification->K_temp_3_data = (float *)malloc(sizeof_float * y_size * y_size);
    arm_mat_init_f32(&rls_identification->K_temp_3, y_size, y_size, (float *)rls_identification->K_temp_3_data);

    rls_identification->K_temp_4_data = (float *)malloc(sizeof_float * y_size * y_size);
    arm_mat_init_f32(&rls_identification->K_temp_4, y_size, y_size, (float *)rls_identification->K_temp_4_data);

    rls_identification->P_temp_1_data = (float *)malloc(sizeof_float * x_hat_size * x_hat_size);
    arm_mat_init_f32(&rls_identification->P_temp_1, x_hat_size, x_hat_size, (float *)rls_identification->P_temp_1_data);

    rls_identification->P_temp_2_data = (float *)malloc(sizeof_float * x_hat_size * x_hat_size);
    arm_mat_init_f32(&rls_identification->P_temp_2, x_hat_size, x_hat_size, (float *)rls_identification->P_temp_2_data);

    rls_identification->x_temp_1_data = (float *)malloc(sizeof_float * y_size);
    arm_mat_init_f32(&rls_identification->x_temp_1, y_size, 1, (float *)rls_identification->x_temp_1_data);

    rls_identification->x_temp_2_data = (float *)malloc(sizeof_float * y_size * 1);
    arm_mat_init_f32(&rls_identification->x_temp_2, y_size, 1, (float *)rls_identification->x_temp_2_data);

    rls_identification->x_temp_3_data = (float *)malloc(sizeof_float * x_hat_size);
    arm_mat_init_f32(&rls_identification->x_temp_3, x_hat_size, 1, (float *)rls_identification->x_temp_3_data);
}

void RLS_Update(RLS *rls_identification)
{
    memcpy(rls_identification->P_minus_data, rls_identification->P_data, sizeof(float) * rls_identification->xhatSize * rls_identification->xhatSize);
    memcpy(rls_identification->x_minus_data, rls_identification->x_data, sizeof(float) * rls_identification->xhatSize);

    //求解K
    rls_identification->MatStatus =
        arm_mat_mult_f32(&rls_identification->P_minus, &rls_identification->H, &rls_identification->K_temp_1);
    rls_identification->MatStatus =
        arm_mat_trans_f32(&rls_identification->H, &rls_identification->H_T);
    rls_identification->MatStatus =
        arm_mat_mult_f32(&rls_identification->H_T, &rls_identification->K_temp_1, &rls_identification->K_temp_2);
    rls_identification->MatStatus =
        arm_mat_add_f32(&rls_identification->I_yy, &rls_identification->K_temp_2, &rls_identification->K_temp_3);
    rls_identification->MatStatus =
        arm_mat_inverse_f32(&rls_identification->K_temp_3, &rls_identification->K_temp_4);
    rls_identification->MatStatus =
        arm_mat_mult_f32(&rls_identification->K_temp_1, &rls_identification->K_temp_4, &rls_identification->K);

    //求解P
    rls_identification->MatStatus =
        arm_mat_mult_f32(&rls_identification->K, &rls_identification->H_T, &rls_identification->P_temp_1);

    rls_identification->MatStatus =
        arm_mat_sub_f32(&rls_identification->I_xx, &rls_identification->P_temp_1, &rls_identification->P_temp_2);

    rls_identification->MatStatus =
        arm_mat_mult_f32(&rls_identification->P_temp_2, &rls_identification->P_minus, &rls_identification->P);

    // for (int i = 0; i < rls_identification->xhatSize * rls_identification->xhatSize; i++)
    // {
    //     rls_identification->P_data[i] /= rls_identification->lambda;
    // }

    //求解x
    rls_identification->MatStatus =
        arm_mat_mult_f32(&rls_identification->H_T, &rls_identification->x_minus, &rls_identification->x_temp_1);

    rls_identification->MatStatus =
        arm_mat_sub_f32(&rls_identification->y, &rls_identification->x_temp_1, &rls_identification->x_temp_2);

    rls_identification->MatStatus =
        arm_mat_mult_f32(&rls_identification->K, &rls_identification->x_temp_2, &rls_identification->x_temp_3);

    rls_identification->MatStatus =
        arm_mat_add_f32(&rls_identification->x_minus, &rls_identification->x_temp_3, &rls_identification->x);
}

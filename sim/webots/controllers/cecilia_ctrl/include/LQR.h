#ifndef __LQR_H__
#define __LQR_H__

#include "matrix.h"
#include "sys.h"

/**
 * @brief LQR控制器结构体
 * @note 这里和Rhomberg观测器一起使用。
 *       为避免重复申请变量与计算，就干脆内嵌进去了
*/
typedef struct __LQR_t{
    Matrix_t x;             // state space
    Matrix_t x_hat;         // prediction of 
    Matrix_t xd;            // desire state space 
    Matrix_t u;             // control space
    Matrix_t A;             // 状态转移矩阵
    Matrix_t B;             // 控制转移矩阵
    Matrix_t C;             // 输出矩阵
    Matrix_t L;             // 观测器增益矩阵
    Matrix_t K;             // LQR增益矩阵
    double y_hat, y;        //
}LQR_t;


void calcLQR(LQR_t *L) {
    //观测器修正
    //L->x.matrix[4][0] += L->L.matrix[4][0] * (L->y - L->y_hat);

    L->u = mul_matrix(L->K, sub_matrix(L->xd,L->x));
    printf("u:\r\n");
    printf_matrix(&L->u);
    //观测器预测
    //L->x_hat = add_matrix(L->x, add_matrix(mul_matrix(L->A, sub_matrix(L->x, L->xd)), mul_matrix(L->B, L->u)));
    Matrix_t X_Xd = sub_matrix(L->x, L->xd);
    printf("X_Xd:\r\n");
    printf_matrix(&X_Xd);
    Matrix_t AX = mul_matrix(L->A, X_Xd);
    printf("AX:\r\n");
    printf_matrix(&AX);
    Matrix_t BU = mul_matrix(L->B, L->u);
    printf("BU:\r\n");
    printf_matrix(&BU);
    L->x_hat = add_matrix(L->x, add_matrix(AX, BU));
    printf("x_hat:\r\n");
    printf_matrix(&L->x_hat);
    L->x_hat = num_mul(0.032,L->x_hat);
    printf("x_hat * time:\r\n");
    printf_matrix(&L->x_hat);
    L->y_hat = mul_matrix(L->C, L->x_hat).matrix[0][0];
    printf("y_hat:%f\r\n", L->y_hat);
    return;
}

#endif // __LQR_H__
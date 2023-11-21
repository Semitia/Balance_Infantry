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
    L->u = mul_matrix(L->K, sub_matrix(L->xd,L->x));

    /* DEBUG */
    // printf("u\r\n");
    // printf_matrix(&L->u);
    return;
}

#endif // __LQR_H__
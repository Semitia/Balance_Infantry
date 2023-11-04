#ifndef __LQR_H__
#define __LQR_H__

#include "matrix.h"

/**
 * @brief LQR控制器结构体
 * @note 这里和Rhomberg观测器一起使用。
 *       为避免重复申请变量与计算，就干脆内嵌进去了
*/
typedef struct __LQR_t{
    Matrix_t x;             // actual state space
    Matrix_t x_hat;         // prediction of state space
    Matrix_t xd;            // desire state space 
    Matrix_t u;             // control space
    Matrix_t A;             // 状态转移矩阵
    Matrix_t B;             // 控制转移矩阵
    Matrix_t K;             // LQR增益矩阵
}LQR_t;

void calcLQR(LQR_t *L) {
    

    L->u = mul_matrix(L->K, sub_matrix(L->x,L->xd));
    return;
}

#endif // __LQR_H__
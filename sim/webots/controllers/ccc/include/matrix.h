#ifndef __MATRIX_H
#define __MATRIX_H

#include<math.h>
#include<stdio.h>
#include<stdlib.h>

#define INV 4

/**
 * @brief 
 * @param m 行数
 * @param n 列数
 * @param matrix 矩阵
 */
typedef struct __Matrix_t{
    int m;
    int n;
    double matrix[INV][INV];//matrix[Matrix_MX][Matrix_MX];
}Matrix_t;

Matrix_t get_I(int t);
Matrix_t inv_matrix(Matrix_t M);
Matrix_t tran_matrix(Matrix_t M);
Matrix_t matrix_init2(int m, int n);
void swap_col(int i, int j, Matrix_t *M);
void swap_row(int i, int j, Matrix_t *M);
void fprintf_matrix(Matrix_t *mat, FILE *fp);
Matrix_t add_matrix(Matrix_t m1, Matrix_t m2);
Matrix_t sub_matrix(Matrix_t m1, Matrix_t m2);
Matrix_t mul_matrix(Matrix_t m1, Matrix_t m2);
void add_row(int i, int j, double k, Matrix_t *M);
void add_col(int i, int j, double k, Matrix_t *M);
Matrix_t matrix_init(int m, int n, double matrix[][INV]);


/**
 * @brief printf the Matrix
 * @param m 
 */
void printf_matrix(Matrix_t *Mat)
{
    printf("%d x %d\r\n",Mat->m,Mat->n);
    for(int i=0; i<Mat->m; i++)
    {
        for(int j=0; j<Mat->n; j++)
        { printf("%.3f ",Mat->matrix[i][j]); }
        printf("\r\n");
    }
    printf("\r\n");
    return;
}

/**
 * @brief printf the matrix data to a file
 * 
 * @param mat 
 * @param fp 
 */
void fprintf_matrix(Matrix_t *mat, FILE *fp)
{
    int i,j;
    for(i=0; i<mat->m; i++)
    {
        for(j=0; j<mat->n; j++)
        { fprintf(fp,"%.3f\t",mat->matrix[i][j]); }
        fprintf(fp,"\n");
    }
    return;
}

/**
 * @brief 
 * 
 * @param m1 
 * @param m2 
 * @return Matrix_t 
 */
Matrix_t add_matrix(Matrix_t m1, Matrix_t m2)
{
    int i,j;
    for(i=0; i<m1.m; i++)
    {
        for(j=0; j<m1.n; j++)
        { m1.matrix[i][j] += m2.matrix[i][j]; }
    }
    return m1;
}

Matrix_t sub_matrix(Matrix_t m1, Matrix_t m2)
{
    int i,j;
    for(i=0; i<m1.m; i++)
    {
        for(j=0; j<m1.m; j++)
        { m1.matrix[i][j] -= m2.matrix[i][j]; }
    }
    return m1;
}

/**
 * @brief Matrix Mulplication
 * @param m1 
 * @param m2 
 * @return Matrix_t
 */
Matrix_t mul_matrix(Matrix_t m1, Matrix_t m2)
{
    //if (m1->n != m2->m) {return 0;}
    Matrix_t ans;
    ans.m = m1.m;
    ans.n = m2.n;

    for(int i=0; i < m1.m; i++)
    {
        for(int j=0; j < m2.n; j++)
        {
            float sum=0;
            for(int k=0; k < m1.n; k++)
            { sum += m1.matrix[i][k]*m2.matrix[k][j]; }
            ans.matrix[i][j] = sum;
        }
    }
    return ans;
}

/**
 * @brief 求逆矩阵
 * 
 * @param M 
 * @return the inverse matrix
 */
Matrix_t inv_matrix(Matrix_t M)
{
    //printf_matrix(&M);
    Matrix_t I;
    int t = M.m, j, k;
    double tem;
    I = get_I(t);
    for(j=0; j<t; j++)
    {
        //寻找首项非零的一行
        for(k=j; k<t; k++)
        { if(M.matrix[k][j] != 0) break; }
        swap_row(k, j, &M);
        swap_row(k, j, &I);
        //这一行归一
        tem = M.matrix[j][j];
        for(k=t-1; k>=0; k--)//for M, k>=j; but for I, k=0
        { 
            M.matrix[j][k] /= tem;
            I.matrix[j][k] /= tem;
        }
        //消去之后每一行的第j项
        for(k=j+1; k<t; k++)
        {
            //M.matrix[k][j] would be changed, I should be operated before M.
            add_row(j,k,-M.matrix[k][j],&I);           
            add_row(j,k,-M.matrix[k][j],&M);
        }
    }
    //从后往前消去每一行的第j项
    for(j=t-1; j>0; j--)
    {
        for(k=j-1; k>=0; k--)
        {
            tem = M.matrix[k][j];
            M.matrix[k][j] -= M.matrix[j][j]*tem;
            add_row(j, k, -tem, &I);
        }
    }
    //printf_matrix(&M);
    //printf_matrix(&I);
    return I;
}

/*
void test_inv(void)
{
    double te[3][3] = {{0,2,-1},{1,1,2},{-1,-1,-1}};
    Matrix_t test;
    test = matrix_init(3,3,te);
    inv_matrix(test);
    return;
}
*/

/**
 * @brief 返回转置矩阵
 * 
 * @param M 
 * @return Matrix_t 
 */
Matrix_t tran_matrix(Matrix_t M)
{
    int i,j;
    Matrix_t ans;
    ans.m = M.m, ans.n = M.n;
    for(i=0; i<M.m; i++)
    {
        for(j=0; j<M.n; j++)
        { ans.matrix[i][j] = M.matrix[j][i]; }
    }
    return ans;
}

/**
 * @brief 初始化矩阵
 * @param m 行数
 * @param n 列数
 * @param matrix 矩阵
*/
Matrix_t matrix_init(int m, int n, double matrix[][INV])
{
    int i,j;
    Matrix_t ans = {0};
    ans.m = m;
    ans.n = n;
    for(i=0; i<m; i++)
    {
        for(j=0; j<n; j++)
        { ans.matrix[i][j] = matrix[i][j]; }
    }
    return ans;
}

/**
 * @brief 初始化矩阵,无矩阵数据
 * @param m 行数
 * @param n 列数
*/
Matrix_t matrix_init2(int m, int n)
{
    Matrix_t ans = {0};
    ans.m = m;
    ans.n = n;
    return ans;
}

/**
 * @brief 交换行
 * 
 * @param i 
 * @param j 
 * @param M 
 */
void swap_row(int i, int j, Matrix_t *M)
{
    int k;
    double t;
    for(k=0; k<M->n; k++)
    {
        t = M->matrix[i][k];
        M->matrix[i][k] = M->matrix[j][k];
        M->matrix[j][k] = t;
    }
    return;
}

/**
 * @brief 交换列
 * 
 * @param i 
 * @param j 
 * @param M 
 */
void swap_col(int i, int j, Matrix_t *M)
{
    int k;
    double t;
    for(k=0; k<M->m; k++)
    {
        t = M->matrix[k][i];
        M->matrix[k][i] = M->matrix[k][j];
        M->matrix[k][j] = t;
    }
    return;
}

/**
 * @brief 第i行乘上k加到第j行
 * 
 * @param i 
 * @param j 
 * @param k 
 * @param M 
 */
void add_row(int i, int j, double k, Matrix_t *M)
{
    int t;
    for(t=0; t<M->n; t++)
    { M->matrix[j][t] += M->matrix[i][t]*k; }
    return;
}

void add_col(int i, int j, double k, Matrix_t *M)
{
    int t;
    for(t=0; t<M->m; t++)
    { M->matrix[t][j] += M->matrix[t][i]*k; }
    return;
}

Matrix_t get_I(int t)
{
    int i, j;
    Matrix_t ans;
    ans.m = t, ans.n = t;
    for(i=0; i<t; i++)
    { 
        for(j=0; j<t; j++)
        { ans.matrix[i][j] = 0; }
        ans.matrix[i][i] = 1;
    }
    return ans;
}

#endif

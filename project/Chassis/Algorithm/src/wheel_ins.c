#include "wheel_ins.h"

Wheel_Accel_Fusion_t Wheel_Accel_Fusion;

// 卡方检验
void KF_Wheel_Accel_Test(KalmanFilter_t *kf)
{
    kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT); // z|x => x|z
    kf->temp_matrix.numRows = kf->H.numRows;
    kf->temp_matrix.numCols = kf->Pminus.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix); // temp_matrix = H·P'(k)
    kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
    kf->temp_matrix1.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1); // temp_matrix1 = H·P'(k)·HT
    kf->S.numRows = kf->R.numRows;
    kf->S.numCols = kf->R.numCols;
    kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S); // S = H P'(k) HT + R
    kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);     // temp_matrix1 = inv(H·P'(k)·HT + R)

    kf->temp_vector.numRows = kf->H.numRows;
    kf->temp_vector.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->H, &kf->xhatminus, &kf->temp_vector); // temp_vector = H xhat'(k)
    kf->temp_vector1.numRows = kf->z.numRows;
    kf->temp_vector1.numCols = 1;
    kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1); // temp_vector1 = z(k) - H·xhat'(k)

    // chi-square test,卡方检验
    kf->temp_matrix.numRows = kf->temp_vector1.numRows;
    kf->temp_matrix.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix1, &kf->temp_vector1, &kf->temp_matrix); // temp_matrix = inv(H·P'(k)·HT + R)·(z(k) - h(xhat'(k)))
    kf->temp_vector.numRows = 1;
    kf->temp_vector.numCols = kf->temp_vector1.numRows;
    kf->MatStatus = Matrix_Transpose(&kf->temp_vector1, &kf->temp_vector); // temp_vector = z(k) - h(xhat'(k))'
    kf->MatStatus = Matrix_Multiply(&kf->temp_vector, &kf->temp_matrix, &Wheel_Accel_Fusion.ChiSquare);

    if (Wheel_Accel_Fusion.ChiSquare_Data[0] > Wheel_Accel_Fusion.ChiSquareTestThreshold)
    {
        kf->R_data[0] = 1e5; //此时设置测量噪声很大，迫使速度估计不发生很大变化
    }
}

/**
 * @brief 里程计与加速度计数据融合卡尔曼滤波初始化
 * @param[in] process_noise 位置过程噪声(叠加了加速度计)
 * @param[in] process_noise2 速度过程噪声(叠加了加速度计)
 * @param[in] measure_noise 速度测量噪声(里程计)
 */
void KF_Wheel_Accel_Init(float process_noise1, float process_noise2, float measure_noise)
{
    Wheel_Accel_Fusion.Initialized = 1;
    Wheel_Accel_Fusion.Q1 = process_noise1;
    Wheel_Accel_Fusion.Q2 = process_noise2;
    Wheel_Accel_Fusion.R1 = measure_noise;

    // 初始化矩阵维度信息
    Kalman_Filter_Init(&Wheel_Accel_Fusion.KF_Wheel_Accel, 2, 1, 1);

    Wheel_Accel_Fusion.KF_Wheel_Accel.UseAutoAdjustment = 0; // 不使用调整维度

    // // 姿态初始化
    Wheel_Accel_Fusion.KF_Wheel_Accel.xhat_data[0] = 0;
    Wheel_Accel_Fusion.KF_Wheel_Accel.xhat_data[1] = 0;

    //测量矩阵初始化
    Wheel_Accel_Fusion.KF_Wheel_Accel.H_data[0] = 0;
    Wheel_Accel_Fusion.KF_Wheel_Accel.H_data[1] = 1;

    //状态转移矩阵初始化
    Wheel_Accel_Fusion.KF_Wheel_Accel.F_data[0] = 1;
    Wheel_Accel_Fusion.KF_Wheel_Accel.F_data[3] = 1;

    //状态估计协方差初始化
    Wheel_Accel_Fusion.KF_Wheel_Accel.P_data[0] = 1;
    Wheel_Accel_Fusion.KF_Wheel_Accel.P_data[3] = 1;

    //卡方检验
    Matrix_Init(&Wheel_Accel_Fusion.ChiSquare, 1, 1, (float *)Wheel_Accel_Fusion.ChiSquare_Data);

    // Wheel_Accel_Fusion.KF_Wheel_Accel.User_Func2_f = KF_Wheel_Accel_Test;
    Wheel_Accel_Fusion.ChiSquareTestThreshold = 1e-3;
}

/**
 * @brief 传感器融合更新
 * @param[in]  x_v
 * @param[in]  x_a
 * @param[in]  dt
 */
void KF_Wheel_Accel_Update(float xv, float xa, float dt)
{
    if (!Wheel_Accel_Fusion.Initialized)
    {
        KF_Wheel_Accel_Init(10, 10, 1000);
    }
    Wheel_Accel_Fusion.dt = dt;

    // 改状态转移矩阵
    Wheel_Accel_Fusion.KF_Wheel_Accel.F_data[1] = dt;

    // 修改B矩阵
    Wheel_Accel_Fusion.KF_Wheel_Accel.B_data[0] = 0.5f * dt * dt;
    Wheel_Accel_Fusion.KF_Wheel_Accel.B_data[1] = dt;

    // 修改输入u
    Wheel_Accel_Fusion.KF_Wheel_Accel.ControlVector[0] = xa;

    // 修改测量向量
    Wheel_Accel_Fusion.KF_Wheel_Accel.MeasuredVector[0] = xv;

    // 修改过程噪声矩阵
    Wheel_Accel_Fusion.KF_Wheel_Accel.Q_data[0] = Wheel_Accel_Fusion.Q1 * dt;
    Wheel_Accel_Fusion.KF_Wheel_Accel.Q_data[3] = Wheel_Accel_Fusion.Q2 * dt;

    // 修改测量噪声矩阵
    Wheel_Accel_Fusion.KF_Wheel_Accel.R_data[0] = Wheel_Accel_Fusion.R1;

    Kalman_Filter_Update(&Wheel_Accel_Fusion.KF_Wheel_Accel);

    Wheel_Accel_Fusion.x = Wheel_Accel_Fusion.KF_Wheel_Accel.FilteredValue[0];
    Wheel_Accel_Fusion.x_v = Wheel_Accel_Fusion.KF_Wheel_Accel.FilteredValue[1];
}

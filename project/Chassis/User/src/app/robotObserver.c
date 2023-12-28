#include "robotObserver.h"

StateObserver_t robot_observer;

//状态观测器
void robotObserverInit(void)
{
    //注意由于离散化形式这里A = (I + delta_t * A),B = delta_t * B,L = delta_t * L
    // 7 * 7
    float A[49] = {
        1.0000, 0.0010, 0, 0, 0, 0, 0,
        0.1575, 1.0000, 0, 0, 0.0157, 0, 0.0157,
        0, 0, 1.0000, 0.0010, 0, 0, 0,
        -0.0231, 0, 0, 1.0000, 0.0003, 0, 0.0003,
        0, 0, 0, 0, 1.0000, 0.0010, 0,
        0.0652, 0, 0, 0, 0.0720, 1.0000, 0.0720,
        0, 0, 0, 0, 0, 0, 1.0000};
    // 7 * 3
    float B[21] = {
        0, 0, 0,
        -0.0232, 0.0104, 0,
        0, 0, 0.0000,
        0.0048, -0.0010, 0.0010,
        0, 0, 0,
        -0.0023, 0.0169, 0,
        0, 0, 0};
    // 4 * 7
    float C[28] = {
        1, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0};
    // 7 * 4
    float L[28] = {
        0.0104, -0.0021, 0.0062, 0.0034,
        0.1733, -0.0368, 0.1145, 0.0599,
        0.0000, 0.0086, 0.0016, -0.0000,
        -0.0233, 0.0003, 0.0105, 0.0006,
        -0.0030, -0.0100, 0.0299, 0.0265,
        -0.0013, -0.1713, 0.5342, 0.3056,
        -0.0044, -0.0098, 0.0322, 0.0094};

    ObserverInit(&robot_observer, 7, 3, 4);
    memcpy(robot_observer.A_data, A, 49 * 4);
    memcpy(robot_observer.B_data, B, 21 * 4);
    memcpy(robot_observer.C_data, C, 28 * 4);
    memcpy(robot_observer.L_data, L, 28 * 4);
}

/**
 * @brief 观测器更新(输入四个可观测状态)
 *
 * @param[in] theta
 * @param[in] x
 * @param[in] x_dot
 * @param[in] pitch
 */
void robotObserverUpdate(float theta, float x, float x_dot, float pitch)
{
    robot_observer.y_data[0] = theta;
    robot_observer.y_data[1] = x;
    robot_observer.y_data[2] = x_dot;
    robot_observer.y_data[3] = pitch;

    ObserverUpdate(&robot_observer);
}

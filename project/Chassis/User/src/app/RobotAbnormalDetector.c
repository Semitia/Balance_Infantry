#include "RobotAbnormalDetector.h"

RobotAbnormalDetector robot_abnormal_detector;

void MF9025DetectInit()
{
    OLS_Init(&robot_abnormal_detector.speed_derive, 200);
    TD_Init(&robot_abnormal_detector.speed_td_1, 400, 0.001);
    TD_Init(&robot_abnormal_detector.speed_td_2, 400, 0.001);
}

WHEELS_STATE MF9025StateDetect(float motor_speed_1, float motor_speed_2, float motor_current_1, float motor_current_2, float delta_t)
{
    //左轮滤波
    iir(&robot_abnormal_detector.filte_wheel_speed[0], motor_speed_1, 0.50);
    iir(&robot_abnormal_detector.filte_wheel_current[0], motor_current_1, 0.50);
    TD_Calculate(&robot_abnormal_detector.speed_td_1, motor_speed_1);
    robot_abnormal_detector.filte_wheel_accel[0] = robot_abnormal_detector.speed_td_1.dx;
    // robot_abnormal_detector.filte_wheel_accel[0] = OLS_Derivative(&robot_abnormal_detector.speed_derive, delta_t, robot_abnormal_detector.filte_wheel_speed[0]);

    //右轮滤波
    iir(&robot_abnormal_detector.filte_wheel_speed[1], motor_speed_2, 0.50);
    iir(&robot_abnormal_detector.filte_wheel_current[1], motor_current_2, 0.50);
    TD_Calculate(&robot_abnormal_detector.speed_td_2, motor_speed_2);
    robot_abnormal_detector.filte_wheel_accel[1] = robot_abnormal_detector.speed_td_2.dx;

    // //负载估计
    robot_abnormal_detector.estimate_wheel_load[0] = (robot_abnormal_detector.filte_wheel_current[0] - MF9025_J * robot_abnormal_detector.filte_wheel_accel[0] - MF9025_B * robot_abnormal_detector.filte_wheel_speed[0]) * 1000.0f;
    robot_abnormal_detector.estimate_wheel_load[1] = (robot_abnormal_detector.filte_wheel_current[1] - MF9025_J * robot_abnormal_detector.filte_wheel_accel[1] - MF9025_B * robot_abnormal_detector.filte_wheel_speed[1]) * 1000.0f;

    return MF9025_NORMAL;
}

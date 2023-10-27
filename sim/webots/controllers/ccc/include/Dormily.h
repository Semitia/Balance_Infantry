#ifndef __DORMILY_H__
#define __DORMILY_H__

#include "LQR.h"
#include "sys.h"
#include "display.h"

/**
 * @brief Dormily_t
 * @note 机器人结构体，目前只实现运动控制功能
*/
typedef struct __Dormily_t {
    WbDeviceTag imu;                        // 机器人IMU
    WbDeviceTag gyro;                             // 机器人陀螺仪
    WbDeviceTag wheels[2];                  // 机器人轮子
    WbDeviceTag pos_ss[2];                  // 机器人电机位置传感器

    double pitch, roll, yaw;                // 机器人姿态
    double pitch_vel, roll_vel, yaw_vel;    // 机器人姿态角速度
    double vx, vy, w;                       // 机器人速度
    double px, py, pz;                      // 机器人位置
    LQR_t LQR;                              // LQR控制器

} Dormily_t;

/**
 * @brief Dormily_init
 * @note 初始化机器人结构体
 * @param D 机器人结构体指针
*/
void Dormily_init(Dormily_t *D) {
    //imu
    D->imu = wb_robot_get_device("imu");
    wb_inertial_unit_enable(D->imu, TIME_STEP);
    D->gyro= wb_robot_get_device("gyro");
    wb_inertial_unit_enable(D->gyro, TIME_STEP);

    //motor
    D->wheels[0] = wb_robot_get_device("motor1");
    D->wheels[1] = wb_robot_get_device("motor2");
    wb_motor_set_position(D->wheels[0], INFINITY);
    wb_motor_set_position(D->wheels[1], INFINITY);

    D->pos_ss[0] = wb_motor_get_position_sensor(D->wheels[0]);
    D->pos_ss[1] = wb_motor_get_position_sensor(D->wheels[1]);
    wb_position_sensor_enable(D->pos_ss[0], TIME_STEP);
    wb_position_sensor_enable(D->pos_ss[1], TIME_STEP);

    // LQR
    double A[4][INV] = {{0,1,0,0},{432.353,0,0,0},{0,0,0,1},{-4.9,0,0,0}},
        B[4][INV] = {{0},{-7.353},{0},{0.25}},
        K[1][INV] = {{-126.0532, -6.9019, -3.1623, -7.3078}};

    D->LQR.x = matrix_init2(4, 1);
    D->LQR.u = matrix_init2(2, 1);
    D->LQR.A = matrix_init(4, 4, A);
    D->LQR.B = matrix_init(4, 1, B);
    D->LQR.K = matrix_init(1, 4, K);
}

/**
 * @brief set motors' torque
 * @param D robot
 * @param torque1 left wheel's torque
 * @param torque2 right wheel's torque
*/
void setMotorTorque(Dormily_t *D, double torque1, double torque2) {
    wb_motor_set_torque(D->wheels[0], torque1);
    wb_motor_set_torque(D->wheels[1], torque2);
    return;
}

/**
 * @brief update state
*/
void updateState(Dormily_t *D) {
    //imu
    const double *imu_data = wb_inertial_unit_get_roll_pitch_yaw(D->imu);
    D->pitch = imu_data[0];
    D->roll = imu_data[1];
    D->yaw = imu_data[2];
    const double *gyro_data = wb_gyro_get_values(D->gyro);
    D->pitch_vel = gyro_data[0];
    D->roll_vel = gyro_data[1];
    D->yaw_vel = gyro_data[2];
    //motor
    D->vx = wb_motor_get_velocity(D->wheels[0]);
    D->vy = wb_motor_get_velocity(D->wheels[1]);
    D->w = (D->vy - D->vx) / 0.1;
    D->px = wb_position_sensor_get_value(D->pos_ss[0]);
    D->py = wb_position_sensor_get_value(D->pos_ss[1]);

    //lqr
    D->LQR.x.matrix[0][0] = D->pitch;
    D->LQR.x.matrix[1][0] = D->pitch_vel;
    D->LQR.x.matrix[2][0] = D->px;
    D->LQR.x.matrix[3][0] = D->vx;
    return;
}

/**
 * @brief control loop for Dormily
*/
void dormilyCtrl(Dormily_t *D) {
    
}

/**
 * @brief balance control
*/
void balanceCtrl(Dormily_t *D){

    calcLQR(&D->LQR);
    return;
}

#endif // __DORMILY_H__
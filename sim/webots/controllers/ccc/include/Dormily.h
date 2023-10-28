#ifndef __DORMILY_H__
#define __DORMILY_H__

#include "LQR.h"
#include "sys.h"
#include "display.h"

#define WHEEL_RADIUS 0.03

/**
 * @brief Dormily_t
 * @note 机器人结构体，目前只实现运动控制功能
*/
typedef struct __Dormily_t {
    WbDeviceTag imu;                        // 机器人IMU
    WbDeviceTag gyro;                       // 机器人陀螺仪
    WbDeviceTag wheels[2];                  // 机器人轮子
    double motor_mx_torque[2];                 // 机器人电机最大扭矩
    WbDeviceTag pos_ss[2];                  // 机器人电机位置传感器
    double pos_bias[2];                     // 机器人电机位置传感器初始值

    double pitch, roll, yaw;                // 机器人姿态 rad
    double pitch_vel, roll_vel, yaw_vel;    // 机器人姿态角速度 rad/s
    double wheel_vel[2], wheel_pos[2];      // 机器人轮子速度和位置 rad/s. rad
    double vx, vy, w;                       // 机器人速度 m/s, m/s, rad/s
    double px, py, pz;                      // 机器人位置 m, m, m
    LQR_t LQR;                              // LQR控制器
    Display_t display;                      // 显示器
} Dormily_t;

/**
 * @brief Dormily_init
 * @note 初始化机器人结构体
 * @param D 机器人结构体指针
*/
void Dormily_init(Dormily_t *D) {
    //display
    displayInit(&D->display);

    //imu
    D->imu = wb_robot_get_device("imu");
    wb_inertial_unit_enable(D->imu, TIME_STEP);
    D->gyro= wb_robot_get_device("gyro");
    wb_gyro_enable(D->gyro, TIME_STEP);

    //motor
    D->wheels[0] = wb_robot_get_device("motor1");
    D->wheels[1] = wb_robot_get_device("motor2");
    D->motor_mx_torque[0] = wb_motor_get_max_torque(D->wheels[0]);
    D->motor_mx_torque[1] = wb_motor_get_max_torque(D->wheels[1]);
    wb_motor_enable_torque_feedback(D->wheels[0], TIME_STEP);
    wb_motor_enable_torque_feedback(D->wheels[1], TIME_STEP);
    printf("motor1 max torque: %f, motor2 max torque: %f\n", D->motor_mx_torque[0], D->motor_mx_torque[1]);
    // wb_motor_set_position(D->wheels[0], INFINITY);
    // wb_motor_set_position(D->wheels[1], INFINITY);

    //position sensor
    D->pos_ss[0] = wb_robot_get_device("position_sensor1");
    D->pos_ss[1] = wb_robot_get_device("position_sensor2");
    wb_position_sensor_enable(D->pos_ss[0], TIME_STEP);
    wb_position_sensor_enable(D->pos_ss[1], TIME_STEP);
    wb_robot_step(TIME_STEP);                                           //要先等待一个时间步，否则会出现nan
    D->pos_bias[0] = wb_position_sensor_get_value(D->pos_ss[0]);
    D->pos_bias[1] = wb_position_sensor_get_value(D->pos_ss[1]);
    printf("pos_bias1:%f, pos_bias2:%f\n", D->pos_bias[0], D->pos_bias[1]);

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
    D->wheel_vel[0] = wb_motor_get_velocity(D->wheels[0]);
    D->wheel_vel[1] = wb_motor_get_velocity(D->wheels[1]);
    D->vx = (D->wheel_vel[0] + D->wheel_vel[1]) / 2 * WHEEL_RADIUS;
    D->wheel_pos[0] = wb_position_sensor_get_value(D->pos_ss[0]) - D->pos_bias[0];
    D->wheel_pos[1] = wb_position_sensor_get_value(D->pos_ss[1]) - D->pos_bias[1];
    D->px = (D->wheel_pos[0] + D->wheel_pos[1]) / 2 * WHEEL_RADIUS;
    
    //lqr
    D->LQR.x.matrix[0][0] = D->pitch;
    D->LQR.x.matrix[1][0] = D->pitch_vel;
    D->LQR.x.matrix[2][0] = D->px;
    D->LQR.x.matrix[3][0] = D->vx;
    return;
}

/**
 * @brief 绘制机器人数据波形
*/
void drawData(Dormily_t *D) {
    //D->display.data[0] = D->pitch;
    //D->display.data[1] = D->pitch_vel;
    //printf( "pitch: %.3f, vel: %.3f\n", D->pitch, D->pitch_vel);
    D->display.data[0] = D->wheel_vel[0];
    D->display.data[1] = D->wheel_pos[0];
    //printf( "wheel vel: %.3f, pos: %.3f\n", D->wheel_vel[0], D->wheel_pos[0]); 
    addDisData(&D->display);
    updateDis(&D->display);
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
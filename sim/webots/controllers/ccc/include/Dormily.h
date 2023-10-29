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
    double motor_mx_torque[2];              // 机器人电机最大扭矩
    double motor_torque_out[2];             // 机器人输出电机扭矩
    WbDeviceTag pos_ss[2];                  // 机器人电机位置传感器
    double pos_bias[2];                     // 机器人电机位置传感器初始值

    double pitch, roll, yaw;                // 机器人姿态 rad
    double pitch_vel, roll_vel, yaw_vel;    // 机器人姿态角速度 rad/s
    double wheel_vel[2], wheel_pos[2], wheel_last_pos[2];      // 机器人轮子速度和位置 rad/s. rad
    double vx, vy, w;                       // 机器人速度 m/s, m/s, rad/s
    double tar_vx, tar_vy, tar_w;           // 机器人目标速度 m/s, m/s, rad/s
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
    //重置机器人物理姿态
    // const double INITIAL_TRANS[3] = {0, 0, 0.08};
    // const double INITIAL_ROT[4] = {0, 0, 1, 1.5708};
    // WbNodeRef robot_node = wb_supervisor_node_get_from_def("Dormily");
    // WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
    // WbFieldRef rot_field = wb_supervisor_node_get_field(robot_node, "rotation");
    // wb_supervisor_field_set_sf_vec3f(trans_field, INITIAL_TRANS);
    // wb_supervisor_field_set_sf_rotation(rot_field, INITIAL_ROT);
    // wb_supervisor_node_reset_physics(robot_node);

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
        K[1][INV] = {{-150.0532, -5.9019, -76.6623, -80.3078}};
    D->LQR.x = matrix_init2(4, 1);//pitch , pitch_vel, px, vx
    D->LQR.xd = matrix_init2(4, 1);
    D->LQR.u = matrix_init2(1, 1);
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
    if(torque1 > D->motor_mx_torque[0]) torque1 = D->motor_mx_torque[0];
    if(torque1 < -D->motor_mx_torque[0]) torque1 = -D->motor_mx_torque[0];
    if(torque2 > D->motor_mx_torque[1]) torque2 = D->motor_mx_torque[1];
    if(torque2 < -D->motor_mx_torque[1]) torque2 = -D->motor_mx_torque[1];
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
    D->wheel_pos[0] = wb_position_sensor_get_value(D->pos_ss[0]) - D->pos_bias[0];
    D->wheel_pos[1] = wb_position_sensor_get_value(D->pos_ss[1]) - D->pos_bias[1];
    D->wheel_vel[0] = (D->wheel_pos[0] - D->wheel_last_pos[0])/TIME_STEP*1000;                       //wb_motor_get_velocity 返回的是目标值，默认为10，我真的栓Q
    D->wheel_vel[1] = (D->wheel_pos[1] - D->wheel_last_pos[1])/TIME_STEP*1000;
    D->vx = (D->wheel_vel[0] + D->wheel_vel[1]) / 2 * WHEEL_RADIUS;
    D->px = (D->wheel_pos[0] + D->wheel_pos[1]) / 2 * WHEEL_RADIUS;
    D->wheel_last_pos[0] = D->wheel_pos[0];
    D->wheel_last_pos[1] = D->wheel_pos[1];

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
    D->display.data[0] = D->pitch;
    D->display.data[1] = D->pitch_vel;
    D->display.data[2] = D->px;
    D->display.data[3] = D->vx;
    D->display.data[4] = D->motor_torque_out[0];

    channelEnable(&D->display, 0, 0x00ff00, PI/5);        //绿色，pitch
    // channelEnable(&D->display, 1, 0xffff00, 10);        //黄色, pitch_vel
    channelEnable(&D->display, 2, 0xffffff, 2);         //白色, px
    channelEnable(&D->display, 3, 0xff0000, 1);         //红色. vx
    // channelEnable(&D->display, 4, 0x0000ff, 3);         //蓝色, motor_torque_out
    addDisData(&D->display);
    updateDis(&D->display);
    printf("pitch:%f, pitch_vel:%f, px:%f, vx:%f, torque:%f\n", D->pitch, D->pitch_vel, D->px, D->vx, D->motor_torque_out[0]);
    return;
}

/**
 * @brief balance control
*/
void balanceCtrl(Dormily_t *D){

    calcLQR(&D->LQR);
    return;
}

/**
 * @brief control loop for Dormily
*/
void dormilyCtrl(Dormily_t *D) {
    D->LQR.xd.matrix[0][0] = 0.2 *  D->tar_vx*D->tar_vx;
    D->LQR.xd.matrix[3][0] = D->tar_vx;
    D->LQR.xd.matrix[2][0] = D->px;
    balanceCtrl(D);
    D->motor_torque_out[0] = -D->LQR.u.matrix[0][0] * WHEEL_RADIUS;
    D->motor_torque_out[1] = -D->LQR.u.matrix[0][0] * WHEEL_RADIUS;
    setMotorTorque(D, D->motor_torque_out[0], D->motor_torque_out[1]);
    return;
}

#endif // __DORMILY_H__


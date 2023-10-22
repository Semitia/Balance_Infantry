/*
 * @Author: DYH
 * @Date: 2021-01-24 21:46:46
 * @LastEditTime: 2021-08-09 22:08:23
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \Two_wheels_robot\controllers\include\balence.h
 */

#ifndef WB_BALENCE_H
#define WB_BALENCE_H

#define WB_USING_C_API
#include "types.h"

#ifndef WB_MATLAB_LOADLIBRARY
#include <math.h>
#endif

#ifdef __cplusplus
extern "C"
{
#endif

#define roll 0
#define pitch 1
#define yaw 2

#define x_length 0
#define y_length 1
#define z_length 2

#define X_DIR 0
#define Y_DIR 1
#define Z_DIR 2

// define the ID of the motor and the posotion sensor
#define MOTOR_NUM 6

#define KNEE_RBM 0
#define KNEE_RFM 1
#define KNEE_LBM 2
#define KNEE_LFM 3
#define WHEEL_R 4
#define WHEEL_L 5
#define SPRING_R 6 //虚拟扭簧
#define SPRING_L 7

#define NORMAL 5
#define JUMP_UP 1
#define JUMP_DOWN 2
#define JUMP_PRE 0
#define BREAKDOWN 3
#define SQART 4
#define JUMP_FLOPE 6 //飞坡

#include "PID.h"
#include "motor.h"
#include "position_sensor.h"
#include "inertial_unit.h"
#include "accelerometer.h"
#include "gyro.h"
#include "keyboard.h"
#include "mouse.h"
#include "types.h"
#include "camera.h"
#include "robot.h"

  //第二部分: 定义变量
  //电机参数
  typedef struct motor_feature
  {
    WbDeviceTag ID;
    const char *name;
    double MAX_TORQUE;
    double torque;
    double torque_fb;
    double torque_tgt;
    double omg;
    double angle;
    double angle_last;
  } MOTOR;

  //编码器参数
  typedef struct position_sensor_feature
  {
    WbDeviceTag ID;
    const char *name;
    double resolusion;
    double position; // rad
    double position_last;
    double w; // rad/s
    double w_last;
  } POSITION_SENSOR;

  // imu参数
  typedef struct imu_feature
  {
    WbDeviceTag ID;
    const char *name;
    double angle_value[3]; // rad
  } IMU;

  // accelerometer参数
  typedef struct accelerometer_feature
  {
    WbDeviceTag accelerometer_ID;
    const char *accelerometer_name;
    double accelerometer_value[3];
  } ACCE;

  // gyro参数
  typedef struct gyro_feature
  {
    WbDeviceTag gyro_ID;
    const char *gyro_name;
    double gyro_value[3];
  } GYRO;

  //相机参数
  typedef struct camera_feature
  {
    WbDeviceTag camera_ID;
    const char *camera_name;
  } CAMERA;

  //显示屏参数
  typedef struct display
  {
    WbDeviceTag ID;
    const char *name;
  } DISPLAY;

  //机器人参数
  typedef struct balence_robot
  {
    /* data */
    double radius_of_wheel;  // m
    double height_of_center; // m
    double mass_body;        // kg
    double mass_wheel;       // kg
    double DofJ_12;
    double DofJ_23;
    double DofJ_34;
    double DofJ_25;           // 1~5,5个轴之间的距离
    double velocity;          // m/s
    double w_yaw_target;      // rad/s
    double displacement;      //机器人质心的位移
    double displacement_last; //上一个时间戳, 机器人质心的位移
    double velocity_target;   //机器人的目标速度
    int module;
    bool camera_state;
    MOTOR motor[6];
    POSITION_SENSOR position_sensor[6];
    IMU imu;
    ACCE acce;
    GYRO gyro;
    CAMERA camera;
    DISPLAY display;
  } Balence_robot;

  // initializing function
  void robot_init();
  void motor_init(double angle_set);
  void position_sensor_init();
  void imu_init();
  void acce_init();
  void gyro_init();
  void key_init();
  void camera_init();
  void display_init();
  void pid_init();

  // agrithm function
  // PID函数名称中的PID均为大写
  void joint_PID_cal();
  void balance_LQR_cal();
  void balance_PID_cal();
  int state_judge();

  void joint_spring_cal();

  // detecting and based calculating function
  void joint_detect();
  void angle_detect();
  void gyro_detect();
  void velocity_detect();

  // papametar seting function
  void torque_set();
  void para_print();
  void reveive_velocity_z();
  void display(double variable, double MAX, double MIN, int color, int order, char *value);
  void all_display();

#ifdef __cplusplus
}
#endif

#endif /* WB_BALENCE_H */

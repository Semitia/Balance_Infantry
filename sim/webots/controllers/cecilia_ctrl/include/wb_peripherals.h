#ifndef __WB_PERIPHERALS_H__
#define __WB_PERIPHERALS_H__

#include "sys.h"
#include "display.h"

/* 电机 */
typedef struct __Motor_t
{
    WbDeviceTag ID;
    const char *name;
    bool angle_limit;           // 是否角度限制
    double MAX_TORQUE;
    double MAX_ANGLE;
    double MIN_ANGLE;
    double torque_fb;           // torque feedback
    double torque_tgt;          // torque target
    double omg;                 // 角速度 rad/s
    double angle;
} Motor_t;

/* 轮子编码器 */
typedef struct __PosSensor_t
{
    WbDeviceTag ID;
    const char *name;
    double resolusion;              
    double position;            // rad
    double position_last;
    double position_bias;
    double w; // rad/s
    double w_last;
} PosSensor_t;

/* IMU */
typedef struct __IMU_t
{
    WbDeviceTag ID;
    const char *name;
    double angle_value[3]; // rad
} IMU_t;

/* 加速度计 */
typedef struct __Acce_t
{
    WbDeviceTag accelerometer_ID;
    const char *accelerometer_name;
    double accelerometer_value[3];
} Acce_t;

/* 角速度计 */
typedef struct __Gyro_t
{
    WbDeviceTag gyro_ID;
    const char *gyro_name;
    double gyro_value[3];
} Gyro_t;

/* 相机 */
typedef struct __Camera_t
{
    WbDeviceTag camera_ID;
    const char *camera_name;
} Camera_t;

/**
 * @brief motorInit 
 *        初始化电机结构体
 * @param motor 电机结构体指针
 * @param name 电机名字
 * @param angle_set 电机初始角度
*/
void motorInit(Motor_t *motor, const char *name, double angle_set)
{
    motor->name = name;
    motor->ID = wb_robot_get_device(motor->name);
    assert(motor->ID); //如果motor->ID为0, 则程序停止
    motor->MAX_TORQUE = wb_motor_get_max_torque(motor->ID);
    motor->MAX_ANGLE = wb_motor_get_max_position(motor->ID);
    motor->MIN_ANGLE = wb_motor_get_min_position(motor->ID);
    wb_motor_enable_torque_feedback(motor->ID, (int)TIME_STEP);
    //wb_motor_set_position(motor->ID, angle_set);

    printf("motor %s 's MAX_TORQUE: %f\n", motor->name, motor->MAX_TORQUE);
}

/**
 * @brief posSensorInit
 *       初始化编码器结构体
 * @param pos_sensor 编码器结构体指针
 * @param name 编码器名字
*/
void posSensorInit(PosSensor_t *pos_sensor, const char *name)
{
    pos_sensor->name = name;
    pos_sensor->ID = wb_robot_get_device(pos_sensor->name);
    assert(pos_sensor->ID);
    wb_position_sensor_enable(pos_sensor->ID, (int)TIME_STEP);
    pos_sensor->position = 0;
    pos_sensor->position_last = 0;
    pos_sensor->w = 0;
    pos_sensor->w_last = 0;
    wb_robot_step(TIME_STEP);
    pos_sensor->position_bias = wb_position_sensor_get_value(pos_sensor->ID);
    return;
}

#define roll 0
#define pitch 1
#define yaw 2
/**
 * @brief imuInit
 *        初始化IMU结构体
 * @param imu IMU结构体指针
 * @param name IMU名字
*/
void imuInit(IMU_t *imu, const char *name)
{
    imu->name = name;
    imu->ID = wb_robot_get_device(imu->name);
    wb_inertial_unit_enable(imu->ID, (int)TIME_STEP);
    imu->angle_value[yaw] = 0;
    imu->angle_value[pitch] = 0;
    imu->angle_value[roll] = 0;
}

void acceInit(Acce_t *acce, const char *name)
{
    acce->accelerometer_name = name;
    acce->accelerometer_ID = wb_robot_get_device(acce->accelerometer_name);
    wb_accelerometer_enable(acce->accelerometer_ID, (int)TIME_STEP);
}
void gyroInit(Gyro_t *gyro, const char *name)
{
    gyro->gyro_name = name;
    gyro->gyro_ID = wb_robot_get_device(gyro->gyro_name);
    wb_gyro_enable(gyro->gyro_ID, (int)TIME_STEP);
}
void cameraInit(Camera_t *camera, const char *name)
{
    camera->camera_name = name;
    camera->camera_ID = wb_robot_get_device(camera->camera_name);
    wb_camera_enable(camera->camera_ID, (int)TIME_STEP);
}

/**
 * @brief updateIMU 更新IMU数据
 * @param imu IMU结构体指针
*/
void updateIMU(IMU_t *imu)
{
    assert(imu->ID);
    imu->angle_value[roll] = wb_inertial_unit_get_roll_pitch_yaw(imu->ID)[roll];
    imu->angle_value[pitch] = wb_inertial_unit_get_roll_pitch_yaw(imu->ID)[pitch];
    imu->angle_value[yaw] = wb_inertial_unit_get_roll_pitch_yaw(imu->ID)[yaw];
    return;
}

void updateGyro(Gyro_t *gyro)
{
    assert(gyro->gyro_ID);
    gyro->gyro_value[roll] = wb_gyro_get_values(gyro->gyro_ID)[roll];
    gyro->gyro_value[pitch] = wb_gyro_get_values(gyro->gyro_ID)[pitch];
    gyro->gyro_value[yaw] = wb_gyro_get_values(gyro->gyro_ID)[yaw];
    return;
}

void updatePosSensor(PosSensor_t *pos_sensor) {
    assert(pos_sensor->ID);
    pos_sensor->position = wb_position_sensor_get_value(pos_sensor->ID);//这里要用绝对角度
    pos_sensor->w = (pos_sensor->position - pos_sensor->position_last)/T;
    pos_sensor->position_last = pos_sensor->position;
    return;
}

void updateMotor(PosSensor_t *pos_sensor, Motor_t *motor)
{
    updatePosSensor(pos_sensor);
    assert(motor->ID);
    //motor->angle = pos_sensor->position;
    //motor->omg = pos_sensor->w;
    motor->torque_fb = wb_motor_get_torque_feedback(motor->ID);
    return;
}

#define X 0
#define Y 1
#define Z 2
/**
 * @brief updateAcce 更新加速度计数据
 * @param acce 加速度计结构体指针
*/
void updateAcce(Acce_t *acce)
{
    assert(acce->accelerometer_ID);
    acce->accelerometer_value[X] = wb_accelerometer_get_values(acce->accelerometer_ID)[X];
    acce->accelerometer_value[Y] = wb_accelerometer_get_values(acce->accelerometer_ID)[Y];
    acce->accelerometer_value[Z] = wb_accelerometer_get_values(acce->accelerometer_ID)[Z];
    return;
}

void setTorque(Motor_t *motor) {
    motor->torque_tgt = limit(motor->torque_tgt, -motor->MAX_TORQUE, motor->MAX_TORQUE);
    wb_motor_set_torque(motor->ID, motor->torque_tgt);
    return;
}

#endif
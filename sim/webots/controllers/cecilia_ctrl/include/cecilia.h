#ifndef __CECILIA_H__
#define __CECILIA_H__

#include "LQR.h"
#include "wb_peripherals.h"

#define WHEEL_RADIUS 0.03

//各个电机名字
typedef enum __MotorName_e {
    KNEE_RBM = 0,
    KNEE_RFM,
    KNEE_LRM,       
    KNEE_LFM,       
    WHEEL_R,        //右轮
    WHEEL_L,
    MOTOR_NUM
} MotorName_e;
//robot mode
typedef enum __Mode_e {
    JUMP_PRE = 0,       //预备跳跃
    JUNP_UP,            //跳跃
    JUMP_DOWN,          //落地
    BREAKDOWN,          //倒地
    SQART,              //蹲下
    NORMAL,             //正常
    JUMP_FLOPE,         //跳跃后倒地
    MODE_NUM
} Mode_e;

/**
 * @brief Dormily_t
 * @note 机器人结构体，目前只实现运动控制功能
*/
typedef struct __Cecilia_t {
    double wheel_radius;                    // 机器人轮子半径 m
    double center_height;                   // 机器人中心高度 m
    double body_mass;                       // 机器人质量 kg
    double wheel_mass;                      // 机器人轮子质量 kg
    double DofJ_12;             
    double DofJ_23;
    double DofJ_34;
    double DofJ_25;           // 1~5,5个轴之间的距离
    double velocity;          // m/s
    double w_yaw_target;      // rad/s
    double displacement;      //机器人质心的位移
    double displacement_last; //上一个时间戳, 机器人质心的位移
    double velocity_target;   //机器人的目标速度
    bool camera_state;
    int mode;
    
    PosSensor_t PosSensor[MOTOR_NUM];       // 机器人编码器
    Motor_t Motor[MOTOR_NUM];               // 机器人电机
    Display_t Display;                      // 显示器
    Camera_t Camera;                        // 相机
    Gyro_t Gyro;                            // 陀螺仪
    Acce_t Acce;                            // 加速度计
    LQR_t LQR;                              // LQR控制器
    IMU_t IMU;                              // IMU
} Cecilia_t;


void ceciliaInit(Cecilia_t *Ce) {
    int i;
    Ce->center_height = 0;
    Ce->body_mass = 30;
    Ce->wheel_mass = 0.5;
    Ce->wheel_radius = 0.065;
    Ce->camera_state = false;

    const char *MOTOR_NAMES[] = {"RBM", "RFM", "LBM", "LFM", "wheel1", "wheel2"};
    const char *POS_SENSOR_NAMES[] = {"RBS", "RFS", "LBS", "LFS", "Swheel1", "Swheel2"};
    for(i=0; i < MOTOR_NUM; i++) 
        motorInit(&Ce->Motor[i], MOTOR_NAMES[i], 0); 
    for(i=0; i < MOTOR_NUM; i++) 
        posSensorInit(&Ce->PosSensor[i], POS_SENSOR_NAMES[i]);
    
    imuInit(&Ce->IMU, "imu");
    acceInit(&Ce->Acce, "Acce");
    gyroInit(&Ce->Gyro, "gyro");
    cameraInit(&Ce->Camera, "camera");
    displayInit(&Ce->Display);
}

void cecliaCtrl(Cecilia_t *Ce) {

}

/**
 * @brief update state
*/
void updateState(Cecilia_t *Ce) {
    int i;
    for(i=0; i < MOTOR_NUM; i++) 
        updateMotor(&Ce->PosSensor[i], &Ce->Motor[i]);
    updateIMU(&Ce->IMU);
    updateGyro(&Ce->Gyro);
    updateAcce(&Ce->Acce);
    // 更新状态向量

    return;
}

void peripherals_check(Cecilia_t *Ce) {
    int i;
    Ce->Motor[0].torque_tgt = 20;
    Ce->Motor[1].torque_tgt = -20;
    Ce->Motor[2].torque_tgt = 20;
    Ce->Motor[3].torque_tgt = -20;
    updateState(Ce);
    for(i=0; i < MOTOR_NUM; i++) {
        setTorque(&Ce->Motor[i]);
        //printf("Motor %d: %f, %f\r\n", i, Ce->PosSensor[i].position, Ce->PosSensor[i].w);
    }
    //printf("IMU: %f, %f, %f\r\n", Ce->IMU.angle_value[roll], Ce->IMU.angle_value[pitch], Ce->IMU.angle_value[yaw]);
    //printf("Gyro: %f, %f, %f\r\n", Ce->Gyro.gyro_value[roll], Ce->Gyro.gyro_value[pitch], Ce->Gyro.gyro_value[yaw]);
    //printf("Acce: %f, %f, %f\r\n", Ce->Acce.accelerometer_value[roll], Ce->Acce.accelerometer_value[pitch], Ce->Acce.accelerometer_value[yaw]);
    return;
}


//     // LQR
//     double A[5][INV] = {{0,1,0,0,0},{432.353,0,0,0,-423.353},{0,0,0,1,0},{-4.9,0,0,0,4.9},{0,0,0,0,0}},
//         B[5][INV] = {{0},{-7.353},{0},{0.25},{0}},
//         K[1][INV] = {{-150.0532, -5.9019, -76.6623, -80.3078,150.0532}},
//         C[1][INV] = {{1,0,0,0,0}},
//         L[5][INV] = {{-0.0000},{-0.0000},{-0.0000},{-0.0000},{-0.01}};
//     D->LQR.x = matrix_init2(5, 1);//pitch , pitch_vel, px, vx, alpha
//     D->LQR.xd = matrix_init2(5, 1);
//     D->LQR.u = matrix_init2(1, 1);
//     D->LQR.A = matrix_init(5, 5, A);
//     D->LQR.B = matrix_init(5, 1, B);
//     D->LQR.C = matrix_init(1, 5, C);
//     D->LQR.K = matrix_init(1, 5, K);
//     D->LQR.L = matrix_init(5, 1, L);
// }

/**
 * @brief 绘制机器人数据波形
*/
void drawData(Cecilia_t *Ce) {
    Ce->Display.data[0] = Ce->IMU.angle_value[pitch];

    channelEnable(&Ce->Display, 0, 0x00ff00, PI/5);        //绿色，pitch
    // channelEnable(&Ce->Display, 1, 0xffff00, 10);        //黄色, pitch_vel
    // channelEnable(&Ce->Display, 2, 0xffffff, PI/12);           //白色, alpha
    // channelEnable(&Ce->Display, 3, 0xff0000, 1);         //红色. vx
    // channelEnable(&Ce->Display, 4, 0x0000ff, 3);         //蓝色, motor_torque_out
    addDisData(&Ce->Display);
    updateDis(&Ce->Display);
    printf("pitch: %f\r\n", Ce->IMU.angle_value[pitch]);
    return;
}

// /**
//  * @brief balance control
// */
// void balanceCtrl(Dormily_t *D){

//     calcLQR(&D->LQR);
//     return;
// }

// /**
//  * @brief control loop for Dormily
// */
// void dormilyCtrl(Dormily_t *D) {
//     D->LQR.xd.matrix[0][0] = 0.2 *  D->tar_vx*D->tar_vx;
//     D->LQR.xd.matrix[3][0] = D->tar_vx;
//     D->LQR.xd.matrix[2][0] = D->px;
//     balanceCtrl(D);
//     D->motor_torque_out[0] = D->LQR.u.matrix[0][0] * WHEEL_RADIUS;
//     D->motor_torque_out[1] = D->LQR.u.matrix[0][0] * WHEEL_RADIUS;
//     setMotorTorque(D, D->motor_torque_out[0], D->motor_torque_out[1]);
//     return;
// }

#endif // __DORMILY_H__


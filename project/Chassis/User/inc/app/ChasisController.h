#ifndef _CHASIS_CONTROLLER_H
#define _CHASIS_CONTROLLER_H

#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "arm_math.h"
#include "arm_atan2_f32.h"

#include "tools.h"
#include "motor_rs485.h"
#include "UniTreeA1.h"
#include "counter.h"
#include "MF9025.h"
#include "ins.h"
#include "user_lib.h"

#include "remote_control.h"
#include "pid.h"

#include "wheel_ins.h"

#include "Offline_Task.h"

#include "robotObserver.h" //龙贝格观测器

#include "M2006.h"
#include "can_config.h"
#include "TD.h"

#include "SuperPower.h"
#include "PowerLimit.h"

// #define ALL_SEND_ZERO_TORQUE //决定是否是测试模式

/*
    先任意定正方向
    左视图 :
LEFT_LEFT   --------    LEFT_RIGHT
           /        \
          /          \
          \          /
            \       /
              \ O/
    右视图 :
RIGHT_LEFT   --------    RIGHT_RIGHT
           /        \
          /          \
          \          /
            \       /
              \ O /
    所有轮毂电机与关节电机以逆时针为正(分别以左右视图，由于电机孔位一定，方向一般可忽略)
*/

enum CONTROL_STATE
{
    STATE_THETA = 0,
    STATE_THETA_V,
    STATE_X,
    STATE_X_V,
    STATE_PITCH,
    STATE_PITCH_V
};

/*   调整参数  */
#define WHEEL_RADIUS 0.11f
#define LP_THETA_V 0.60f // 对角速度的低通滤波 //一般取0.91-0.99之间
#define LP_PITCH_V 0.60f // 一般取0.91-0.99之间
#define LP_X_V 0.60f     // 一般取0.91-0.99之间
#define LP_X 0.0f

#define LP_X_V_RC 0.0018
#define LP_THETA_V_RC 0.0018
#define LP_PITCH_V_RC 0.0018
#define LP_YAW_V_RC 0.0018

#define PITCH_DIFF_ORDER 20 // 若采用最小二乘法提取信号微分，选择样本数
#define THETA_DIFF_ORDER 40

#define MECHANICAL_PITCH 0.0f // pitch角机械中值 度
#define MECHANICAL_ROLL 0.0f
#define FORBIDDEN_ANGLE 0.4155628949f // 限位，弧度，23.81度

#define SIGN_PITCH -1 // PITCH头向上为正
#define SIGN_YAW 1    // YAW逆时针为正
#define SIGN_ROLL -1  // 左高为正

#define SPEED_MAX 2.2f   // 最大速度
#define SPEED_W_MAX 8.0f // 最大角速度

#define MAX_L 0.32f         // 最大腿长
#define MIN_L 0.11f         // 最短腿长
#define MAX_LEG_SPEED 0.08f // 最大伸腿速度 m/s

#define WHEEL_WEIGHT 1.28f

#define MAX_X 2.0f // 最大x误差
#define MAX_YAW_ANGLE 1.0f

#define MIN_FORCE 10.0f // 最小支持力
#define MAX_FORCE 35.0f // 从跳跃态到着地过程转换的最大力

#define MAX_VMC_TORQUE 9.0f // 每个腿的限制虚拟杆力矩
#define MIN_VMC_TORQUE -MAX_VMC_TORQUE

#define MAX_VMC_FORCE 350 // 每个腿的限制虚拟杆支持力
#define MIN_VMC_FORCE -MAX_VMC_FORCE

#define LAND_TO_BALANCE_TIME 2 // 经过多长时间后从落地态变回平衡态

#define GRAVITY_FEED_FORWARD 60.0f // mg / 2

#define GIMBAL_FOLLOW_ZERO 3480          // 底盘跟随机械零点
#define GIMBAL_FOLLOW_BESIDES_ZEROS 5528 // 侧跟随零点
#define GIMBAL_MOTOR_SIGN 1              // 云台电机方向，以逆时针为正
// #define GIMBAL_FOLLOW_K 0.001f           //底盘跟随系数

#define ROBOT_WIDTH 0.28f // 车身宽

/* 运动学解算参数 (主要用于裁判系统UI绘制)*/
typedef struct MotionSolver
{
    float x_B;
    float y_B;
    float x_D;
    float y_D;
    float x_C;
    float y_C;
} MotionSolver;

/* 底盘跟随方向  */
typedef enum
{
    CHASSIS_FRONT = 1,
    CHASSIS_BACK = -1,
} chassis_direction_e;

typedef struct Sensors
{
    // MF9025
    float wheel_pos[2];   // rad
    float wheel_speed[2]; // rad/s
    float wheel_torque[2];

    // A1
    float knee_angle[4]; // rad
    float torque_feed_back[4];

    // IMU
    float pitch; // rad
    float yaw;
    float roll;

} Sensors;

typedef struct ExcuteTorque
{
    // MF9025
    float wheels_torque[2];
    MF9025_Broadcast wheels_send;

    // A1
    MOTOR_send knee_send[4];
    float knee_torque[4];

} ExcuteTorque;

typedef struct BalanceInfantry
{
    /*  传感器信息 */
    Sensors sensors_info;

    /*  执行机构 */
    ExcuteTorque excute_info;

    // IMU指针
    INS_t *INS;

    // 里程计与加速度计数据融合指针
    Wheel_Accel_Fusion_t *Wheel_Accel_Fusion;

    // 轮毂电机指针
    MF9025 *Wheels_Motor;

    // 关节电机指针
    UniTreeA1 *unitree_a1_motors;

    // 用于计算速度
    float pitch_last;

    // 脚部机构长度,m
    float L_left[6];
    float L_right[6];

    // 五连杆机构角度 rad
    float phi_left[5];
    float phi_right[5];

    // 指LQR算法中腿部姿态角以及角速度
    float theta_left;
    float theta_left_w;
    float theta_right;
    float theta_right_w;

    float K_coef[12][3];    // 变腿长控制系数
    float K[2][6];          // LQR算法中增益矩阵
    float state_vector[6];  // theta,theta_dot,x,x_dot,phi,phi_dot
    float target_vector[6]; // 目标向量

    float virtual_torque_l[3]; // 轮毂，关节虚拟力矩，力为沿着虚拟杆的力，单位为N
    float virtual_torque_r[3];

    float output_torque_l[3]; // 输出力矩，轮毂 + 关节*2，未区分左右轮
    float output_torque_r[3];

    float J_l[2][2]; // VMC算法参数
    float J_r[2][2];

    /* 移动控制  */
    float target_v;
    float target_x_v;
    float target_y_v;

    /* 转向控制  */
    PID_t turn_pid;
    PID_t turn_speed_pid;
    float target_yaw;
    float target_yaw_v;
    float set_yaw_v;
    float yaw_v;
    float last_yaw_angle;
    float target_pid_yaw_v;

    /* 腿长控制  */
    PID_t leg_control_pid_l;
    PID_t leg_control_pid_r;
    // Feedforward_t leg_control_forward_l;
    // Feedforward_t leg_control_forward_r;
    float target_L;
    float target_L_left;
    float target_L_right;
    float extra_leg_force_l;
    float extra_leg_force_r;
    float leg_speed;

    /*  roll角控制 */
    PID_t roll_control_pid;

    /* 双腿协同  */
    PID_t legs_cooperate_pid;
    float extra_torque_l;
    float extra_torque_r;

    uint32_t last_cnt;
    float delta_t; // 前后两帧时间差

    // debug
    float LQR_Output[2][6];

    // 最小二乘法提取信号微分
    // Ordinary_Least_Squares_t theta_left_diff;
    // Ordinary_Least_Squares_t theta_right_diff;
    // Ordinary_Least_Squares_t pitch_diff;

    // 跟踪微分器
    TD_t pitch_td;
    TD_t theta_left_td;
    TD_t theta_right_td;

    /* 离地检测 */
    float wheels_f_torque_feedback_l[2];
    float wheels_f_torque_feedback_r[2];
    float last_l_theta_l;
    float last_l_theta_r;
    float L_theta_v_l;
    float L_theta_v_r;
    float last_l_theta_v_l;
    float last_l_theta_v_r;
    float L_theta_a_l;
    float L_theta_a_r;
    float FN_l;
    float FN_r;
    int8_t is_get_enough_force; // 是否有足够的支持力

    // 落地态区域平衡的时间
    uint32_t land_start_time;
    float land_to_now_time;

    // 平衡态开始时间
    uint32_t balance_start_time;
    float balance_to_now_time;
    uint8_t fly_detect_symbol;

    // 倒地收腿PID
    PID_t shrink_leg_pid_l;
    PID_t shrink_leg_pid_r;

    StateObserver_t *robot_observer;
    float state_observe[7];

    chassis_direction_e chassis_direction;
    float err_angle;
    float cos_dir;
    float sin_dir;

    MotionSolver motion_solver_l;
    MotionSolver motion_solver_r;

    int8_t control_lose_flag;
    float slid_angle;

    // 打滑检测
    int8_t is_slidded;

    // 斜坡角度检测
    float slope_angle;

    // 期望x误差限制
    float limit_err_x_min;
    float limit_err_x_max;

    // 异常检测
    int16_t abnormal_count; // 异常次数计数
    int8_t is_abnormal;     // 是否正在异常状态

    // 收腿状态判断
    int8_t is_shrink_finish_1; // 收腿结束状态1，在该状态后，给轮毂电机和关节电机发0减速，不要让关节或者轮毂运动速度影响恢复站立的初状态
    int8_t is_shrink_finish_2; // 收腿结束状态2，完全收腿结束，切换状态
    int16_t shrink_finish_count_2;

} BalanceInfantry;

void YawInit(BalanceInfantry *balance_infantry);

void SensorsInit(BalanceInfantry *balance_infantry);

void BalanceInfantryInit(BalanceInfantry *balance_infantry);

void get_control_info(BalanceInfantry *balance_infantry);

void get_sensor_info(BalanceInfantry *balance_infantry);

void main_control(BalanceInfantry *balance_infantry);

void execute_control(ExcuteTorque *excute_info);

void accel_odom_fusion(BalanceInfantry *balance_infantry);

// void shrink_leg(BalanceInfantry *balance_infantry);

float ramp_control(float ref, float set, float accel);

float angle_z_err_get(float target_ang, float zeros_angle);

void shrink_leg_finish_judge(BalanceInfantry *balance_infantry);

int8_t recover_finish_judge(void);

void slid_angle_solve(BalanceInfantry *balance_infantry);

void AbnormalDetect(BalanceInfantry *balance_infantry);

void InitLandPid(BalanceInfantry *balance_infantry);
void InitFlyPid(BalanceInfantry *balance_infantry);
void InitRawLegPid(BalanceInfantry *balance_infantry);
void InitJumpPrePid(BalanceInfantry *balance_infantry);
void InitJumpUpPid(BalanceInfantry *balance_infantry);
void InitJumpDownPid(BalanceInfantry *balance_infantry);

extern BalanceInfantry balance_infantry;

#endif // !_CHASIS_CONTROLLER_H

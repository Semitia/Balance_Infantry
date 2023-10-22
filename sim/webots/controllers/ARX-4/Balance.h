#include "pid.h"
typedef struct JUMP //跳跃参数
{
    float pre_time;
    float launch_time;
    float fall_time;
    float jump_start_time;

    float stance_height;  // Desired leg extension before the jump [m]
    float jump_extension; // Maximum leg extension in [m]
    float fall_extension; // Desired leg extension during fall [m]

    float normal_height; // normal state leg height

    Pid_Typedef jump_pid[3];

} JUMP;
typedef struct Balance
{
    float L_left[6];   //脚部机构长度
    float phi_left[5]; //角度
    float theta_left;
    float theta_left_w;

    float L_right[6];
    float phi_right[6];
    float theta_right;
    float theta_right_w;

    float K[2][6];          // LQR
    float state_vector[6];  // x,x_dot,theta,theta_dot,phi,phi_dot
    float target_vector[6]; // 目标x

    float output_torque_l[3];  //输出力矩，轮毂 + 关节*2，未区分左右轮
    float virtual_torque_l[3]; //轮毂，关节虚拟力矩，力为沿着虚拟杆的力
    float output_torque_r[3];
    float virtual_torque_r[3];

    float J_l[2][2];
    float J_r[2][2];

    float target_yaw;
    Pid_Typedef turn_pid;

    float target_L;
    Pid_Typedef leg_pid;
    float extra_leg_force_l;
    float extra_leg_force_r;

    Pid_Typedef leg_cooperate_pid;
    float extra_torque_l;
    float extra_torque_r;

    Pid_Typedef lr_balance_pid;
    float extra_balance_force_l;
    float extra_balance_force_r;

    float force_l;
    float force_r;

    float K_coef[12][3];
    int robot_type;
    int last_robot_type;

    JUMP jump_params
} Balance;

// void MotionSolve(); //运动学解算
// void LQR_Solve();

// void VMC_Solve(); //虚拟力到关节力
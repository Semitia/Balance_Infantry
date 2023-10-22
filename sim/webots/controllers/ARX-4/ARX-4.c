/*
 * File:          ARX-4.c
 * Date:          2021/5/3
 * Description:
 * Author:        DYH
 * Modifications:
 */

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "include/arx.h"
#include "include/print.h"
#include "include/PID.h"
#include "include/display.h"
#include "include/motor.h"
#include "include/position_sensor.h"
#include "include/inertial_unit.h"
#include "include/accelerometer.h"
#include "include/gyro.h"
#include "include/keyboard.h"
#include "include/mouse.h"
#include "include/types.h"
#include "include/camera.h"
#include "include/robot.h"
#include "include/mathFuch.h"
#include "include/display.h"
#include "pid.h"

#include "Balance.h"

#define TIME_STEP 4

Balence_robot arx;

double time = 0; //时实时间
int status = 0;  //腿长,用来插值

// //自己撸的数据可视化屏幕
double display_abs[10][100] = {0};
double display_ratio[10][100] = {0};
char display_value[10][20] = {0}; //数值

Balance balance_infantry;
bool is_pos_init = false;

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv)
{
  //系统初始化
  wb_robot_init();

  //机器人初始化
  robot_init();

  control_init();

  while (wb_robot_step(TIME_STEP) != -1)
  {
    //上位机控制
    reveive_velocity_z();

    //速度角度检测, 数据存入全局变量中
    velocity_detect();

    angle_detect(); //角度, IMU

    gyro_detect(); //角速度

    acc_detect(); //加速度计

    get_info_from_sensor();
    set_target_vector();

    jump_state_trans();

    print_state();

    // printf("robot type: %d", balance_infantry.robot_type);
    printf("feed_back: %f, %f", arx.motor[WHEEL_L].torque_fb, arx.motor[WHEEL_R].torque_fb);

    MotionSolve(balance_infantry.L_left, balance_infantry.phi_left, &balance_infantry.theta_left, &balance_infantry.theta_left_w);     //没有问题
    MotionSolve(balance_infantry.L_right, balance_infantry.phi_right, &balance_infantry.theta_right, &balance_infantry.theta_right_w); //没有问题

    get_info_from_solve(balance_infantry.theta_left, balance_infantry.theta_left_w);
    updateK(balance_infantry.L_left[0]);
    jump_change_k();
    LQR_Solve(balance_infantry.virtual_torque_l);
    leg_control(balance_infantry.L_left[0], balance_infantry.target_L, &balance_infantry.extra_leg_force_l);

    // printf("theta_right: %f  , w: %f", balance_infantry.theta_right, balance_infantry.theta_right_w);
    get_info_from_solve(balance_infantry.theta_right, balance_infantry.theta_right_w);
    updateK(balance_infantry.L_right[0]);
    jump_change_k();
    LQR_Solve(balance_infantry.virtual_torque_r);
    leg_control(balance_infantry.L_right[0], balance_infantry.target_L, &balance_infantry.extra_leg_force_r);

    leg_cooperate(balance_infantry.theta_left, balance_infantry.theta_right, &balance_infantry.extra_torque_l, &balance_infantry.extra_torque_r);
    roll_control(arx.imu.angle_value[roll], &balance_infantry.extra_balance_force_l, &balance_infantry.extra_balance_force_r);
    // printf("phi_0_l:  %f , phi_0_r: %f \n", balance_infantry.theta_left, balance_infantry.theta_right);
    //腿部角度
    force_calc(balance_infantry.extra_balance_force_l, balance_infantry.extra_leg_force_l, &balance_infantry.force_l);
    force_calc(balance_infantry.extra_balance_force_r, balance_infantry.extra_leg_force_r, &balance_infantry.force_r);

    VMC_Solve(balance_infantry.L_left, balance_infantry.phi_left, balance_infantry.virtual_torque_l, balance_infantry.output_torque_l, &balance_infantry.force_l, &balance_infantry.extra_torque_l, balance_infantry.J_l);
    VMC_Solve(balance_infantry.L_right, balance_infantry.phi_right, balance_infantry.virtual_torque_r, balance_infantry.output_torque_r, &balance_infantry.force_r, &balance_infantry.extra_torque_r, balance_infantry.J_r);
    // printf("r torque: %f", balance_infantry.L_right[0])
    // printf("extra_force_r: %f extra_force_l: %f\n", balance_infantry.extra_leg_force_r, balance_infantry.extra_leg_force_l);
    // printf("set L: %f , now L_r: %f , L_l: %f \n", balance_infantry.target_L, balance_infantry.L_right[0], balance_infantry.L_left[0]);

    // fly_slope();

    torque_get();

    printf("time: %f \n", wb_robot_get_time());

    //转向控制
    turn_solve();

    torque_set();
  };

  /* Enter your cleanup code here */
  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
//运动学逆解，已知轮子位置，求解角度期望
void reverse_motion_solve(float *x_c, float *y_c, float *l, float *phi_14)
{
  // for (int i = 0; i < 6; i++)
  // {
  //   printf("L: %f \n", l[i]);
  // }
  float A_0 = pow(*x_c, 2) + pow(*y_c, 2) + l[1] * l[1] - l[2] * l[2];
  float B_0 = -2 * (*x_c) * l[1];
  float C_0 = -2 * (*y_c) * l[1];
  printf("TEMP:  %f,%f,%f \n", *x_c, *y_c, C_0);
  float M_0 = -C_0 + sqrt(C_0 * C_0 + B_0 * B_0 - A_0 * A_0);
  float N_0 = A_0 - B_0;
  phi_14[0] = 2 * atan2(M_0, N_0);

  float x_E = l[5];
  float y_E = 0;
  float A_1 = pow(*x_c - x_E, 2) + pow(*y_c - y_E, 2) + l[4] * l[4] - l[3] * l[3];
  float B_1 = -2 * (*x_c - x_E) * l[4];
  float C_1 = -2 * (*y_c - y_E) * l[4];
  float M_1 = -C_1 - sqrt(C_1 * C_1 + B_1 * B_1 - A_1 * A_1);
  float N_1 = A_1 - B_1;

  float T = M_1 / N_1;
  printf("T: %f", T);

  // float angle = atan2(M_1, N_1);
  // if (angle > pi / 2)
  // {
  //   angle = angle - pi;
  // }
  // else if (angle < -pi / 2)
  // {
  //   angle += pi;
  // }
  printf("M_1:  %f  N_1: %f", M_1, N_1);
  phi_14[1] = 2 * atan2(-M_1, -N_1);

  // printf("phi[1]:  %f,phi[4]: %f", phi_14[0], phi_14[1]);
}
void print_state()
{
  if (balance_infantry.robot_type == JUMP_UP)
  {
    printf("jump up !!! \n");
  }
  else if (balance_infantry.robot_type == JUMP_DOWN)
  {
    printf("jump down !!! \n");
  }
  else if (balance_infantry.robot_type == JUMP_PRE)
  {
    printf("jump pre !!! \n");
  }
  else if (balance_infantry.robot_type == NORMAL)
  {
    printf("normal !!! \n");
  }
}
void jump_change_k()
{
  if (balance_infantry.robot_type == JUMP_UP || balance_infantry.robot_type == JUMP_DOWN || balance_infantry.robot_type == JUMP_FLOPE)
  {
    for (int i = 0; i < 2; i++)
    {
      for (int j = 0; j < 6; j++)
      {
        if (i == 0 || j == 2 || j == 3 || j == 4 || j == 5)
        {
          balance_infantry.K[i][j] = 0;
        }
      }
    }
    // balance_infantry.K[1][1] = ;
    // balance_infantry.K[1][0] = 40;
    // balance_infantry.K[1][1] = 6;
  }
}
void jump_state_trans() //跳跃状态转移
{
  //跳跃代码参考自Doggo

  float jump_to_now_time = wb_robot_get_time() - balance_infantry.jump_params.jump_start_time;

  if (balance_infantry.last_robot_type == NORMAL && balance_infantry.robot_type == JUMP_PRE)
  {
    //跳跃准备开始计时
    balance_infantry.jump_params.jump_start_time = (float)wb_robot_get_time();
    balance_infantry.last_robot_type = JUMP_PRE;
    // balance_infantry.target_L = balance_infantry.jump_params.stance_height;
    printf("start jump!! \n");
  }
  else if (balance_infantry.robot_type == JUMP_PRE && jump_to_now_time > balance_infantry.jump_params.pre_time)
  {
    //切换到跳跃状态
    balance_infantry.robot_type = JUMP_UP;
    balance_infantry.last_robot_type = JUMP_UP;
    balance_infantry.target_L = balance_infantry.jump_params.jump_extension;
  }
  else if (balance_infantry.robot_type == JUMP_UP && jump_to_now_time > balance_infantry.jump_params.pre_time + balance_infantry.jump_params.launch_time)
  {
    //跳跃结束，切换到下降状态
    balance_infantry.robot_type = JUMP_DOWN;
    balance_infantry.last_robot_type = JUMP_DOWN;
    balance_infantry.target_L = balance_infantry.jump_params.fall_extension;
  }
  else if (balance_infantry.robot_type == JUMP_DOWN &&
           jump_to_now_time > balance_infantry.jump_params.fall_time +
                                  balance_infantry.jump_params.launch_time + balance_infantry.jump_params.pre_time)
  {
    //下降缓冲结束，切换为正常状态
    balance_infantry.robot_type = NORMAL;
    balance_infantry.last_robot_type = NORMAL;
    balance_infantry.target_L = balance_infantry.jump_params.normal_height;
    is_pos_init = false;
  }
}
void force_calc(float extra_balance_force, float extra_leg_force, float *output_force)
{
  //平衡时的计算
  *output_force = -(10 + 1.8 + extra_leg_force + extra_balance_force) * 9.81;
}
// void ExecuteJump()
// {
//   float t = millis() / 1000.0f - start_time_; // Seconds since jump was commanded

//   if (t < prep_time)
//   {
//     float x = 0;
//     float y = stance_height;
//     float theta, gamma;
//     CartesianToThetaGamma(x, y, 1.0, theta, gamma);

//     // Use gains with small stiffness and lots of damping
//     struct LegGain gains = {50, 1.0, 50, 1.0};
//     CommandAllLegs(theta, gamma, gains);
//     // Serial << "Prep: +" << t << "s, y: " << y;
//   }
//   else if (t >= prep_time && t < prep_time + launch_time)
//   {
//     float x = 0;
//     float y = jump_extension;
//     float theta, gamma;
//     CartesianToThetaGamma(x, y, 1.0, theta, gamma);

//     // Use high stiffness and low damping to execute the jump
//     struct LegGain gains = {240, 0.5, 240, 0.2};
//     CommandAllLegs(theta, gamma, gains);
//     // Serial << "Jump: +" << t << "s, y: " << y;
//   }
//   else if (t >= prep_time + launch_time && t < prep_time + launch_time + fall_time)
//   {
//     float x = 0;
//     float y = fall_extension;
//     float theta, gamma;
//     CartesianToThetaGamma(x, y, 1.0, theta, gamma);

//     // Use low stiffness and lots of damping to handle the fall
//     struct LegGain gains = {50, 1.0, 50, 1.0};

//     CommandAllLegs(theta, gamma, gains);
//     // Serial << "Retract: +" << t << "s, y: " << y;
//   }
//   else
//   {
//     state = STOP;
//     Serial.println("Jump Complete.");
//   }
//   // Serial << '\n';
// }

void fly_slope() //加速度计配合imu解算机器人加速度
{
  float x_a = arx.acce.accelerometer_value[X_DIR];
  float y_a = arx.acce.accelerometer_value[Z_DIR];
  float z_a = arx.acce.accelerometer_value[Y_DIR];

  float yaw_angle = arx.imu.angle_value[yaw];
  float pitch_angle = arx.imu.angle_value[pitch];
  float roll_angle = arx.imu.angle_value[roll];

  // float a_x = cos(yaw_angle) * cos(pitch_angle) * x_a +
  //             sin(yaw_angle) * cos(pitch_angle) * y_a -
  //             sin(pitch_angle) * z_a;
  // float a_y = (cos(yaw_angle) * sin(pitch_angle) * sin(roll_angle) - sin(yaw_angle) * cos(roll_angle)) * x_a +
  //             (sin(yaw_angle) * sin(pitch_angle) * sin(roll_angle) + cos(yaw_angle) * cos(roll_angle)) * y_a +
  //             cos(pitch_angle) * sin(roll_angle) * z_a;
  float a_z = (cos(yaw_angle) * sin(pitch_angle) * cos(roll_angle) + sin(yaw_angle) * sin(roll_angle)) * x_a +
              (sin(yaw_angle) * sin(pitch_angle) * cos(roll_angle) - cos(yaw_angle) * sin(roll_angle)) * y_a +
              cos(pitch_angle) * cos(roll_angle) * z_a;

  // printf("acc info: %f,%f,%f", x_a, y_a, z_a);
  // printf("a_z:  %f", a_z - 9.81); //计算加速度

  float f_and_torque_l[2], f_and_torque_r[2]; //顺序为力矩，力
  float joint_motor_l[2] = {(float)arx.motor[KNEE_LBM].torque_fb, (float)arx.motor[KNEE_LFM].torque_fb};
  float joint_motor_r[2] = {(float)arx.motor[KNEE_RBM].torque_fb, (float)arx.motor[KNEE_RFM].torque_fb};

  get_feedback_f_torque(joint_motor_l, balance_infantry.J_l, f_and_torque_l);
  get_feedback_f_torque(joint_motor_r, balance_infantry.J_r, f_and_torque_r);

  // printf("joint_motor: %f %f \n", arx.motor[KNEE_LBM].torque_fb, arx.motor[KNEE_RBM].torque_fb);
  // printf("feedback %f  , %f \n", f_and_torque_l[0], f_and_torque_l[1]);
}
void get_feedback_f_torque(float *joint_motor, float J[][2], float *feedback) //通过转矩反馈反解算力
{
  float J_inv[2][2];
  float coef = J[0][0] * J[1][1] - J[0][1] * J[1][0];
  J_inv[0][0] = J[1][1] / coef;
  J_inv[0][1] = -J[0][1] / coef;
  J_inv[1][0] = -J[1][0] / coef;
  J_inv[1][1] = J[0][0] / coef;

  for (int i = 0; i < 2; i++)
  {
    feedback[i] = 0;
    for (int j = 0; j < 2; j++)
    {
      feedback[i] += J_inv[i][j] * joint_motor[j];
    }
  }
}

void leg_cooperate(float angle_left, float angle_right, float *output_l, float *output_r)
{
  balance_infantry.leg_cooperate_pid.ActualValue = angle_left - angle_right;
  balance_infantry.leg_cooperate_pid.SetPoint = 0;
  float output = PID_Calc(&balance_infantry.leg_cooperate_pid);
  *output_l = output;
  *output_r = -output;

  // printf("angle diff: %f ,", angle_left - angle_right);
  // printf("extra torque: %f \n", output);
}
void roll_control(float roll_angle, float *output_l, float *output_r)
{
  balance_infantry.lr_balance_pid.ActualValue = roll_angle;
  balance_infantry.lr_balance_pid.SetPoint = 0;
  float output = PID_Calc(&balance_infantry.lr_balance_pid);
  *output_l = output;
  *output_r = -output;

  // printf("roll : %f ", roll_angle);
  // printf("extra force: %f\n", output);
}

void leg_control(float L0, float L_target, float *output) //计算额外力
{
  if (balance_infantry.robot_type != JUMP_UP && balance_infantry.robot_type != JUMP_DOWN)
  {
    balance_infantry.leg_pid.ActualValue = L0;
    balance_infantry.leg_pid.SetPoint = L_target;
    *output = PID_Calc(&balance_infantry.leg_pid);
    printf("L_target : %f , L: %f forward: %f pid: %f\n", L_target, L0, 200 * (L_target - L0), *output);

    *output += 200 * (L_target - L0); //前馈
  }
  else
  {
    balance_infantry.jump_params.jump_pid[JUMP_UP].ActualValue = L0;
    balance_infantry.jump_params.jump_pid[JUMP_UP].SetPoint = L_target;
    *output = PID_Calc(&balance_infantry.jump_params.jump_pid);
    *output += 800 * (L_target - L0); //前馈
  }
}

void torque_get()
{
  arx.motor[WHEEL_L].torque = balance_infantry.output_torque_l[0];
  arx.motor[WHEEL_R].torque = balance_infantry.output_torque_r[0];
  arx.motor[KNEE_LBM].torque = balance_infantry.output_torque_l[1];
  arx.motor[KNEE_RBM].torque = balance_infantry.output_torque_r[1];
  arx.motor[KNEE_LFM].torque = balance_infantry.output_torque_l[2];
  arx.motor[KNEE_RFM].torque = balance_infantry.output_torque_r[2];

  printf("set torque: %f %f", balance_infantry.output_torque_l[1], balance_infantry.output_torque_l[2]);
}

void set_target_vector()
{
  if (!is_pos_init)
  {
    balance_infantry.target_vector[2] = (arx.position_sensor[WHEEL_L].position + arx.position_sensor[WHEEL_R].position) * arx.radius_of_wheel / 2;
    is_pos_init = true;
  }

  // printf("left_wheel pos: %f,right_wheel pos: %f", arx.position_sensor[WHEEL_L].position, arx.position_sensor[WHEEL_R].position);
}

void updateK(float L0)
{
  // if (balance_infantry.robot_type == NORMAL || balance_infantry.robot_type == JUMP_PRE || balance_infantry.robot_type == JUMP_DOWN)
  // {
  // printf("normal state!!!!!!!!");
  float pow_2 = L0 * L0;
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      balance_infantry.K[i][j] = balance_infantry.K_coef[i * 6 + j][0] * pow_2 + L0 * balance_infantry.K_coef[i * 6 + j][1] + balance_infantry.K_coef[i * 6 + j][2];
    }
  }
  // }
  // else
  // {
  //   printf("jump up !!!!!!!!!!!!!!!!!!!!!!!!!!! \n");
  //   float pow_2 = L0 * L0;
  //   //先全部清零
  //   // for (int i = 0; i < 2; i++)
  //   // {
  //   //   for (int j = 0; j < 6; j++)
  //   //   {
  //   //     balance_infantry.K[i][j] = 0;
  //   //   }
  //   // }

  //   // for (int j = 0; j < 4; j++)
  //   // {
  //   //   balance_infantry.K[1][j] = balance_infantry.K_coef[1 * 6 + j][0] * pow_2 + L0 * balance_infantry.K_coef[1 * 6 + j][1] + balance_infantry.K_coef[1 * 6 + j][2];
  //   // }
  // }
}

void turn_solve()
{
  balance_infantry.turn_pid.ActualValue = arx.imu.angle_value[yaw];
  balance_infantry.turn_pid.SetPoint = balance_infantry.target_yaw;
  float turn_out = PID_Calc(&balance_infantry.turn_pid);

  // printf("turn out: %f , gyro: %f ,set: %f\n", turn_out, arx.imu.angle_value[yaw], balance_infantry.target_yaw);

  arx.motor[WHEEL_L].torque += turn_out;
  arx.motor[WHEEL_R].torque -= turn_out;
}

void get_info_from_solve(float theta, float theta_w)
{
  // TODO: 分开左右腿计算
  balance_infantry.state_vector[0] = theta;
  balance_infantry.state_vector[1] = theta_w;
}

void get_info_from_sensor()
{
  balance_infantry.state_vector[4] = arx.imu.angle_value[pitch];

  // //需满足纯滚动条件
  float Z_x_middle =
      (arx.position_sensor[WHEEL_L].position + arx.position_sensor[WHEEL_R].position) * arx.radius_of_wheel / 2;
  float w = (arx.position_sensor[WHEEL_L].w + arx.position_sensor[WHEEL_R].w) * arx.radius_of_wheel / 2;
  balance_infantry.state_vector[2] = Z_x_middle;                                         //位移
  balance_infantry.state_vector[3] = balance_infantry.state_vector[3] * 0.95 + w * 0.05; //线速度

  balance_infantry.state_vector[5] = arx.gyro.gyro_value[pitch];

  balance_infantry.phi_left[1] = M_PI - arx.position_sensor[KNEE_LBM].position;
  balance_infantry.phi_left[4] = -arx.position_sensor[KNEE_LFM].position;

  balance_infantry.phi_right[1] = M_PI - arx.position_sensor[KNEE_RBM].position;
  balance_infantry.phi_right[4] = -arx.position_sensor[KNEE_RFM].position;
}

void VMC_Solve(float *l, float *phi, float *virtual_torque, float *output_torque, float *force, float *extra_torque, float J[][2])
{
  float sigma_1 = sin(phi[2] - phi[3]);
  float sigma_2 = sin(phi[3] - phi[4]);
  float sigma_3 = sin(phi[1] - phi[2]);

  J[0][0] = -l[1] * cos(phi[0] - phi[3]) * sigma_3 / l[0] / sigma_1;
  J[0][1] = -l[1] * sin(phi[0] - phi[3]) * sigma_3 / sigma_1;
  J[1][0] = -l[4] * cos(phi[0] - phi[2]) * sigma_2 / l[0] / sigma_1;
  J[1][1] = -l[4] * sin(phi[0] - phi[2]) * sigma_2 / sigma_1;

  output_torque[0] = -virtual_torque[0];

  virtual_torque[1] = -(virtual_torque[1] + *extra_torque);
  // extra为额外力，由滚转角和期望腿长计算而得，以上为正方向
  // printf("force_:  %f \n", *extra_balance_force);
  virtual_torque[2] = *force * cos(phi[0] - M_PI / 2); //单个杆承受的力
  printf("set force: %f  l_0 : %f\n ", *force, l[0]);
  // printf("phi_0: %f \n", phi[0]);
  // printf("extra force: %f \n", *extra_force);
  printf("virtual force %f \n ", virtual_torque[1]);
  // printf("L: %f", l[0]);

  for (int i = 1; i < 3; i++)
  {
    output_torque[i] = 0;
  }
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      output_torque[i + 1] += J[i][j] * virtual_torque[j + 1];
    }
  }
}

void LQR_Solve(float *virtual_torque)
{
  for (int i = 0; i < 2; i++)
  {
    virtual_torque[i] = 0;
  }
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      virtual_torque[i] += balance_infantry.K[i][j] * (balance_infantry.target_vector[j] - balance_infantry.state_vector[j]);
    }
  }

  for (int i = 0; i < 2; i++)
  {
    virtual_torque[i] /= 2;
  }

  // for (int i = 0; i < 6; i++)
  // {
  //   printf("state vector: %f   ", balance_infantry.state_vector[i]);
  // }
  // printf("target vector: %f", balance_infantry.target_vector[2]);
  // printf("\n");
}

void control_init()
{
  //五连杆机构参数,L(0)需要哦计算得到
  balance_infantry.L_left[1] = 0.15;
  balance_infantry.L_left[2] = 0.3;
  balance_infantry.L_left[3] = 0.3;
  balance_infantry.L_left[4] = 0.15;
  balance_infantry.L_left[5] = 0.15;

  balance_infantry.phi_left[0] = M_PI / 2;
  balance_infantry.theta_left = 0;
  balance_infantry.theta_left_w = 0;

  balance_infantry.L_right[1] = 0.15;
  balance_infantry.L_right[2] = 0.3;
  balance_infantry.L_right[3] = 0.3;
  balance_infantry.L_right[4] = 0.15;
  balance_infantry.L_right[5] = 0.15;

  balance_infantry.phi_right[0] = M_PI / 2;
  balance_infantry.theta_right = 0; //与垂直线的夹角
  balance_infantry.theta_right_w = 0;

  // LQR参数
  // -52.5563   -7.3815  -21.2552  -17.8601   21.9583    1.4259
  // 24.9652    4.3519   13.8877   11.3103  134.4296    5.2532

  // balance_infantry.K[0][0] = -52.5563;
  // balance_infantry.K[0][1] = -7.3815;
  // balance_infantry.K[0][2] = -21.2552;
  // balance_infantry.K[0][3] = -17.8601;
  // balance_infantry.K[0][4] = 21.9583;
  // balance_infantry.K[0][5] = 1.4259;
  // balance_infantry.K[1][0] = 24.9652;
  // balance_infantry.K[1][1] = 4.3519;
  // balance_infantry.K[1][2] = 13.8877;
  // balance_infantry.K[1][3] = 11.3103;
  // balance_infantry.K[1][4] = 134.4296;
  // balance_infantry.K[1][5] = 5.2532;

  float K_params[12][3] = {{92.9716, -157.9021, -28.3685},
                           {-10.3298, -28.6164, -2.2200},
                           {24.7399, -15.9543, -19.2435},
                           {21.2118, -23.6639, -14.4412},
                           {159.4265, -125.2661, 38.7084},
                           {9.7165, -7.6421, 2.4482},
                           {-0.8049, -15.1622, 27.5487},
                           {-8.1683, 7.1489, 3.3689},
                           {100.8302, -79.2252, 24.4813},
                           {63.9201, -51.6109, 18.2611},
                           {-156.4691, 100.9039, 121.7065},
                           {-9.0445, 5.8748, 4.5106}};
  for (int j = 0; j < 12; j++)
  {
    for (int i = 0; i < 3; i++)
    {
      balance_infantry.K_coef[j][i] = K_params[j][i];
    }
  }

  for (int i = 0; i < 6; i++)
  {
    balance_infantry.target_vector[i] = 0;
  }

  //转向
  balance_infantry.target_yaw = 0;
  balance_infantry.turn_pid.P = 40;
  balance_infantry.turn_pid.I = 0;
  balance_infantry.turn_pid.D = 0;
  balance_infantry.turn_pid.RC_DF = 0;
  balance_infantry.turn_pid.OutMax = 5;

  //腿长
  balance_infantry.target_L = 0.184;
  balance_infantry.leg_pid.P = 5;
  balance_infantry.leg_pid.I = 0;
  balance_infantry.leg_pid.D = 0;
  balance_infantry.leg_pid.RC_DF = 0;
  balance_infantry.leg_pid.OutMax = 15;

  balance_infantry.extra_leg_force_l = 0;
  balance_infantry.extra_leg_force_r = 0;

  //双腿协同
  balance_infantry.extra_torque_l = 0;
  balance_infantry.extra_torque_r = 0;
  balance_infantry.leg_cooperate_pid.P = 10;
  balance_infantry.leg_cooperate_pid.I = 0;
  balance_infantry.leg_cooperate_pid.D = 0;
  balance_infantry.leg_cooperate_pid.RC_DF = 0;
  balance_infantry.leg_cooperate_pid.OutMax = 10;

  // 左右平衡
  balance_infantry.extra_balance_force_l = 0;
  balance_infantry.extra_balance_force_r = 0;
  balance_infantry.lr_balance_pid.P = 100;
  balance_infantry.lr_balance_pid.I = 0;
  balance_infantry.lr_balance_pid.D = 0;
  balance_infantry.lr_balance_pid.RC_DF = 0;
  balance_infantry.lr_balance_pid.OutMax = 10;

  balance_infantry.robot_type = NORMAL;
  balance_infantry.last_robot_type = NORMAL;

  balance_infantry.jump_params.pre_time = 0.2f;
  balance_infantry.jump_params.launch_time = 0.15f;
  balance_infantry.jump_params.fall_time = 0.22;
  balance_infantry.jump_params.jump_start_time = 0.0f;

  balance_infantry.jump_params.stance_height = 0.081f;
  balance_infantry.jump_params.jump_extension = 0.249f;
  balance_infantry.jump_params.fall_extension = 0.13f;
  balance_infantry.jump_params.normal_height = 0.184f;

  balance_infantry.jump_params.jump_pid[JUMP_PRE].P = 400;
  balance_infantry.jump_params.jump_pid[JUMP_PRE].D = 400; //关节控制PID

  balance_infantry.jump_params.jump_pid[JUMP_UP].P = 2000;
  balance_infantry.jump_params.jump_pid[JUMP_UP].D = 20;

  balance_infantry.jump_params.jump_pid[JUMP_DOWN].P = 400;
  balance_infantry.jump_params.jump_pid[JUMP_DOWN].D = 400;
}

void MotionSolve(float *L, float *phi, float *theta, float *theta_w) //运动学解算
{
  float x_B = L[1] * cos(phi[1]);
  float y_B = L[1] * sin(phi[1]);
  float x_D = L[4] * cos(phi[4]) + L[5];
  float y_D = L[4] * sin(phi[4]);
  float B_D_distance_2 = pow(x_D - x_B, 2) + pow(y_D - y_B, 2);
  float A_0 = 2 * L[2] * (x_D - x_B);
  float B_0 = 2 * L[2] * (y_D - y_B);
  float C_0 = pow(L[2], 2) + B_D_distance_2 - pow(L[3], 2);

  float temp = (B_0 + sqrt(pow(A_0, 2) + pow(B_0, 2) - pow(C_0, 2)));
  phi[2] = 2 * atan2(temp, A_0 + C_0);

  float x_C = x_B + L[2] * cos(phi[2]);
  float y_C = y_B + L[2] * sin(phi[2]);

  phi[3] = atan2(y_C - y_D, x_C - x_D);

  float top_half_x = L[5] / 2;
  x_C -= top_half_x;

  L[0] = sqrt(x_C * x_C + y_C * y_C);

  phi[0] = atan2(y_C, x_C);

  float temp_ = -(M_PI / 2 - phi[0] + balance_infantry.state_vector[4]);

  *theta_w = *theta_w * 0.95 + (temp_ - (*theta)) * 1000 / TIME_STEP * 0.05;
  *theta = temp_;
}

//初始化函数
void robot_init()
{
  arx.height_of_center = 0;
  arx.mass_body = 30;
  arx.mass_wheel = 0.5;
  arx.radius_of_wheel = 0.065;

  arx.camera_state = 0;

  arx.module = NORMAL;

  arx.displacement = 0;
  arx.displacement_last = 0;
  arx.velocity_target = 0;

  //最初的标定
  motor_init(0);
  position_sensor_init();
  imu_init();
  acce_init();
  gyro_init();
  camera_init();
  display_init();
  key_init();
}

void motor_init(double angle_set)
{
  //每加一个电机, (1)在宏定义中修改MOTOR_NUM 赋值, (2)增加代号的数量, (3)在这里写上name
  arx.motor[0].name = "RBM";
  arx.motor[1].name = "RFM";
  arx.motor[2].name = "LBM";
  arx.motor[3].name = "LFM";
  arx.motor[4].name = "wheel1"; //右
  arx.motor[5].name = "wheel2"; //左
  //虚拟扭簧
  // arx.motor[6].name = "Spring1"; //右
  // arx.motor[7].name = "Spring2"; //左

  int i;
  for (i = 0; i < MOTOR_NUM; i++)
  {
    //获取电机ID
    arx.motor[i].ID = wb_robot_get_device(arx.motor[i].name);
    assert(arx.motor[i].ID);
    //获取最大扭矩
    arx.motor[i].MAX_TORQUE = wb_motor_get_max_torque(arx.motor[i].ID);
    //使能扭矩反馈
    wb_motor_enable_torque_feedback(arx.motor[i].ID, (int)TIME_STEP);
    //归零
    arx.motor[i].torque = 0;
    arx.motor[i].omg = 0;
    arx.motor[i].angle = angle_set;
    // printf("get motor %s succeed: %d\n", arx.motor[i].name, arx.motor[i].ID);
  }

  // for (int i = 0; i < 4; i++)
  // {
  //   wb_motor_set_available_torque(arx.motor[i].name, 35);
  // }
  // for (int i = 4; i < 6; i++)
  // {
  //   wb_motor_set_available_torque(arx.motor[i].name, 10);
  // }
};

void position_sensor_init()
{
  arx.position_sensor[0].name = "RBS";
  arx.position_sensor[1].name = "RFS";
  arx.position_sensor[2].name = "LBS";
  arx.position_sensor[3].name = "LFS";
  arx.position_sensor[4].name = "Swheel1";
  arx.position_sensor[5].name = "Swheel2";
  // arx.position_sensor[6].name = "SSpring1";
  // arx.position_sensor[7].name = "SSpring2";

  int i;
  for (i = 0; i < MOTOR_NUM; i++)
  {
    arx.position_sensor[i].ID = wb_robot_get_device(arx.position_sensor[i].name);
    assert(arx.position_sensor[i].ID);
    wb_position_sensor_enable(arx.position_sensor[i].ID, (int)TIME_STEP);
    // printf("get position senser %s succeed: %d\n", arx.position_sensor[i].name, arx.position_sensor[i].ID);

    arx.position_sensor[i].position = 0;
    arx.position_sensor[i].position_last = 0;
    arx.position_sensor[i].w = 0;
    arx.position_sensor[i].w_last = 0;
  }
};

void imu_init()
{
  arx.imu.name = "imu";
  arx.imu.ID = wb_robot_get_device(arx.imu.name);
  wb_inertial_unit_enable(arx.imu.ID, (int)TIME_STEP);
  arx.imu.angle_value[yaw] = 0;
  arx.imu.angle_value[pitch] = 0;
  arx.imu.angle_value[roll] = 0;
  // printf("imu over!\n");
};

void acce_init()
{
  arx.acce.accelerometer_name = "Acce";
  arx.acce.accelerometer_ID = wb_robot_get_device(arx.acce.accelerometer_name);
  wb_accelerometer_enable(arx.acce.accelerometer_ID, (int)TIME_STEP);
  // printf("accelerometer over!\n");
}

void gyro_init()
{
  arx.gyro.gyro_ID = wb_robot_get_device("gyro");
  wb_gyro_enable(arx.gyro.gyro_ID, (int)TIME_STEP);
  arx.gyro.gyro_value[yaw] = 0;
  arx.gyro.gyro_value[pitch] = 0;
  arx.gyro.gyro_value[roll] = 0;
  // printf("gyro over!\n");
};

void camera_init()
{
  arx.camera.camera_ID = wb_robot_get_device("camera");
  wb_camera_enable(arx.camera.camera_ID, (int)TIME_STEP);
  // printf("camera over!\n");
};

void key_init()
{
  wb_keyboard_enable((int)TIME_STEP);
};

void display_init()
{
  arx.display.ID = wb_robot_get_device("display");
  // printf("display over!\n");
}

//传感器检测函数
void angle_detect()
{
  assert(arx.imu.ID);
  arx.imu.angle_value[roll] = wb_inertial_unit_get_roll_pitch_yaw(arx.imu.ID)[roll];
  arx.imu.angle_value[pitch] = wb_inertial_unit_get_roll_pitch_yaw(arx.imu.ID)[pitch];
  arx.imu.angle_value[yaw] = wb_inertial_unit_get_roll_pitch_yaw(arx.imu.ID)[yaw];

  // IMU/GYRO信息
  // printf("0---0yaw: %.3f\t 0pitch: %.3f\t 0roll: %.3f\t \n",
  //        dgr(arx.imu.angle_value[yaw]), dgr(arx.imu.angle_value[pitch]), dgr(arx.imu.angle_value[roll]));
};

void gyro_detect()
{
  assert(arx.gyro.gyro_ID);
  arx.gyro.gyro_value[roll] = wb_gyro_get_values(arx.gyro.gyro_ID)[roll];
  arx.gyro.gyro_value[pitch] = wb_gyro_get_values(arx.gyro.gyro_ID)[pitch];
  arx.gyro.gyro_value[yaw] = wb_gyro_get_values(arx.gyro.gyro_ID)[yaw];
  // printf("w---Wyaw: %.3f\t Wpitch: %.3f\t Wroll: %.3f\t \n",
  //        dgr(arx.gyro.gyro_value[yaw]), dgr(arx.gyro.gyro_value[pitch]), dgr(arx.gyro.gyro_value[roll]));
};

void velocity_detect() //轮询所有电机传感器
{
  int i;
  for (i = 0; i < MOTOR_NUM; i++)
  {
    assert(arx.position_sensor[i].ID);
    arx.position_sensor[i].position = wb_position_sensor_get_value(arx.position_sensor[i].ID);                              // GET THE POSITION
    arx.position_sensor[i].w = (arx.position_sensor[i].position - arx.position_sensor[i].position_last) * 1000 / TIME_STEP; // CAL THE ANGULAR V
    arx.position_sensor[i].position_last = arx.position_sensor[i].position;

    // printf("%s motor, pos: %f \n", arx.position_sensor[i].name, arx.position_sensor[i].position);

    arx.motor[i].torque_fb = wb_motor_get_force_feedback(arx.motor[i].ID);
    printf(" torque fb: %f", arx.motor[i].torque_fb);
  }

  arx.position_sensor[WHEEL_R].position = -arx.position_sensor[WHEEL_R].position;
  arx.position_sensor[WHEEL_R].w = -arx.position_sensor[WHEEL_R].w;
};

//上位机函数
void reveive_velocity_z()
{
  int key = -1;

  key = wb_keyboard_get_key();
  printf("key: %d \n", key);
  if (key != -1)
  {
    // update var according to 'key' value
    switch (key)
    {
    case 87: // w
      balance_infantry.target_vector[2] += 0.005;
      break;
    case 83: // s
      balance_infantry.target_vector[2] -= 0.005;
      break;
    case 65: // a
      balance_infantry.target_yaw += 0.005;
      break;
    case 68: // d
      balance_infantry.target_yaw -= 0.005;
      break;
    case 72: // h
      balance_infantry.target_L += 0.01;
      break;
    case 76: // l
      balance_infantry.target_L -= 0.01;
      break;
    case 74:                                  // J
      balance_infantry.robot_type = JUMP_PRE; //准备跳跃
      break;
    case 70: // f
      balance_infantry.robot_type = JUMP_FLOPE;
      break;
    }
  }
};

void acc_detect()
{
  assert(arx.acce.accelerometer_ID);
  arx.acce.accelerometer_value[X_DIR] = wb_accelerometer_get_values(arx.acce.accelerometer_ID)[X_DIR];
  arx.acce.accelerometer_value[Y_DIR] = wb_accelerometer_get_values(arx.acce.accelerometer_ID)[Y_DIR];
  arx.acce.accelerometer_value[Z_DIR] = wb_accelerometer_get_values(arx.acce.accelerometer_ID)[Z_DIR];
}
//施加扭矩
void torque_set()
{
  for (int i = 0; i < 4; i++)
  {
    if (arx.motor[i].torque > 35)
      arx.motor[i].torque = 35;
    else if (arx.motor[i].torque < -35)
      arx.motor[i].torque = -35;
  }

  if (arx.motor[WHEEL_L].torque < -10)
    arx.motor[WHEEL_L].torque = -10;
  if (arx.motor[WHEEL_L].torque > 10)
    arx.motor[WHEEL_L].torque = 10;
  if (arx.motor[WHEEL_R].torque < -10)
    arx.motor[WHEEL_R].torque = -10;
  if (arx.motor[WHEEL_R].torque > 10)
    arx.motor[WHEEL_R].torque = 10;

  wb_motor_set_torque(arx.motor[KNEE_RBM].ID, arx.motor[KNEE_RBM].torque);
  wb_motor_set_torque(arx.motor[KNEE_RFM].ID, arx.motor[KNEE_RFM].torque);
  wb_motor_set_torque(arx.motor[KNEE_LBM].ID, arx.motor[KNEE_LBM].torque);
  wb_motor_set_torque(arx.motor[KNEE_LFM].ID, arx.motor[KNEE_LFM].torque);

  // printf("module: %d", arx.module);

  // printf("torque feedback: %.3f\t %.3f\t %.3f\t %.3f\n", wb_motor_get_torque_feedback(arx.motor[0].ID),
  //        wb_motor_get_torque_feedback(arx.motor[1].ID),
  //        wb_motor_get_torque_feedback(arx.motor[2].ID),
  //        wb_motor_get_torque_feedback(arx.motor[3].ID));

  // if (time > 0) //&& arx.module != JUMP_UP && arx.module != JUMP_DOWN)
  // {
  // printf("set wheel torque")
  wb_motor_set_torque(arx.motor[WHEEL_L].ID, -arx.motor[WHEEL_L].torque);
  wb_motor_set_torque(arx.motor[WHEEL_R].ID, arx.motor[WHEEL_R].torque);
  // wb_motor_set_torque(arx.motor[WHEEL_L].ID, -10);
  // wb_motor_set_torque(arx.motor[WHEEL_R].ID, 10);
  // }
}

/**********************************************************************************************************
 *函 数 名: PID_Calc
 *功能说明: PID+各种优化
 *形    参: PID_Struct *P  PID参数结构体
 *        ActualValue    PID计算反馈量
 *返 回 值: PID反馈计算输出值
 **********************************************************************************************************/
float PID_Calc(Pid_Typedef *P)
{
  P->LastError = P->PreError;
  P->PreError = P->SetPoint - P->ActualValue;
  if ((ABS(P->PreError) < P->DeadZone)) //死区控制
  {
    P->PreError = 0.0f;
  }

  //微分先行
  float DM = P->D * (P->Out - P->Out_last); //微分先行

  //变速积分   (积分分离)
  if (ABS(P->PreError) < P->I_L)
  {

    P->SumError += (P->PreError + P->LastError) / 2;
    P->SumError = LIMIT_MAX_MIN(P->SumError, P->IMax, -P->IMax);
  }
  else if (ABS(P->PreError) < P->I_U)
  {
    //梯形积分
    P->SumError += (P->PreError + P->LastError) / 2 * (P->PreError - P->I_L) / (P->I_U - P->I_L);
    P->SumError = LIMIT_MAX_MIN(P->SumError, P->IMax, -P->IMax);
  }

  P->POut = P->P * P->PreError;

  P->IOut = P->I * P->SumError;

  //不完全微分
  P->DOut_last = P->DOut;
  P->DOut = DM * P->RC_DF + P->DOut_last * (1 - P->RC_DF);

  P->Out_last = P->Out;
  P->Out = LIMIT_MAX_MIN(P->POut + P->IOut + P->DOut, P->OutMax, -P->OutMax);

  return P->Out;
}

/**********************************************************************************************************
 *函 数 名: FeedForward_Calc
 *功能说明: 前馈算法
 *形    参: PID_Struct *P  PID参数结构体
 *        ActualValue    PID计算反馈量（当前真实检测值）
 *返 回 值: PID反馈计算输出值
 **********************************************************************************************************/
float FeedForward_Calc(FeedForward_Typedef *FF)
{
  FF->Out = FF->Now_DeltIn * FF->K1 + (FF->Now_DeltIn - FF->Last_DeltIn) * FF->K2;
  FF->Last_DeltIn = FF->Now_DeltIn;
  return LIMIT_MAX_MIN(FF->Out, FF->OutMax, -FF->OutMax);
}

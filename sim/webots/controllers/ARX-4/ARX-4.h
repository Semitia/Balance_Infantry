#ifndef __ARX_4_H__
#define __ARX_4_H__

void torque_get();
void acc_detect();
void roll_control(float roll_angle, float *output_l, float *output_r);
void velocity_detect();
void reveive_velocity_z();
void gyro_detect();
void angle_detect();
void MotionSolve(float *L, float *phi, float *theta, float *theta_w);
void LQR_Solve(float *virtual_torque);
void VMC_Solve(float *l, float *phi, float *virtual_torque, float *output_torque, float *force, float *extra_torque, float J[][2]);
void get_info_from_sensor();
void get_info_from_solve(float theta, float theta_w);
void turn_solve();
void updateK(float L0);
void set_target_vector();
void leg_control(float L0, float L_target, float *output);
void roll_control(float roll_angle, float *output_l, float *output_r);
void leg_cooperate(float angle_left, float angle_right, float *output_l, float *output_r);
void get_feedback_f_torque(float *joint_motor, float J[][2], float *feedback);
void fly_slope();
void force_calc(float extra_balance_force, float extra_leg_force, float *output_force);
void jump_state_trans();
void jump_change_k();
void reverse_motion_solve(float *x_c, float *y_c, float *l, float *phi_14);
void print_state();

#endif



#include "ChassisSend.h"

ChassisSendPack1 chassis_send_pack1;
ChassisSendPack2 chassis_send_pack2;

void Pack_InfantryMode()
{
    chassis_send_pack1.robot_state = remote_controller.robot_state;
    chassis_send_pack1.control_type = remote_controller.control_type;
    chassis_send_pack1.control_mode_action = remote_controller.control_mode_action;
    chassis_send_pack1.gimbal_mode = remote_controller.gimbal_action;
    chassis_send_pack1.shoot_mode = remote_controller.shoot_action;
    chassis_send_pack1.leg_mode = remote_controller.leg_action;
    chassis_send_pack1.is_pc_on = offline_detector.pc_state == PC_ON;
    chassis_send_pack1.autoaim_id = pc_recv_data.enemy_id;
    chassis_send_pack1.robot_speed_x = (int8_t)(chassis_solver.chassis_speed_x * 10.0f);
    chassis_send_pack1.robot_speed_y = (int8_t)(chassis_solver.chassis_speed_y * 10.0f);
    chassis_send_pack1.robot_speed_w = (int8_t)(chassis_solver.chassis_speed_w * 10.0f);
    // chassis_send_pack1.left_ch_ud_value = remote_controller.dji_remote.rc.ch[LEFT_CH_UD];
    // chassis_send_pack1.right_ch_lr_value = remote_controller.dji_remote.rc.ch[RIGHT_CH_LR];
    // chassis_send_pack1.right_ch_ud_value = remote_controller.dji_remote.rc.ch[RIGHT_CH_UD];
}

void Pack_Yaw()
{
    chassis_send_pack2.yaw_motor_angle = gimbal_controller.yaw_recv.angle;
    chassis_send_pack2.gimbal_pitch = (int16_t)(gimbal_controller.gyro_pitch_angle * 100.0f);
    chassis_send_pack2.gimbal_yaw_speed = (int16_t)(gimbal_controller.gyro_yaw_speed * 100.0f);
    chassis_send_pack2.super_power = remote_controller.super_power_state;
    // chassis_send_pack2.right_ch_ud_value = remote_controller.dji_remote.rc.ch[RIGHT_CH_UD];
    // chassis_send_pack2.right_ch_lr_value = remote_controller.dji_remote.rc.ch[RIGHT_CH_LR];
}

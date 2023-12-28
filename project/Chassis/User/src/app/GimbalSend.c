#include "GimbalSend.h"

GimbalSendPack_1 gimbal_pack_send_1;

void GimbalSendPack()
{
    gimbal_pack_send_1.is_shootable = heat_controller.shoot_flag;
    gimbal_pack_send_1.robot_color = referee_data.Game_Robot_State.robot_id < 10 ? 1 : 0;
    gimbal_pack_send_1.robot_level = referee_data.Game_Robot_State.robot_level;
    gimbal_pack_send_1.bullet_level = referee_data.Game_Robot_State.shooter_id1_17mm_speed_limit <= 15 ? 0 : (referee_data.Game_Robot_State.shooter_id1_17mm_speed_limit <= 22 ? 1 : 2);
    gimbal_pack_send_1.buff_state = referee_data.Buff_Musk.power_rune_buff & 0x0F;
}

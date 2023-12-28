/**
 ******************************************************************************
 * @file    RefereeTask.c
 * @brief   ??????????????λ????????,?????????????
 ******************************************************************************
 * @attention
 ******************************************************************************
 */
#include "RefereeTask.h"

#define Robot_ID_Current Robot_ID_Blue_Infantry4

/* ??????????? */
uint16_t y01 = 455;
uint16_t y02 = 420;
uint16_t y03 = 280;
uint16_t y04 = 230;

uint16_t camera_view_y1 = 100;
uint16_t camera_view_y2 = 860;
uint16_t camera_view_x1 = 320;
uint16_t camera_view_x2 = 1600;

// ?????????????
#define LEG_PLACE_BIAS_X 1630
#define LEG_PLACE_BIAS_Y 644
#define LEG_RATIO 400 // ???????????????????

// ????????н????
#define GIM_CHASSIS_ANGLE_LINE_LEN 70

void drawFiveLeg(UI_Graph5_t *UI_Graph5, uint8_t GraphOperate)
{
	int16_t x0 = LEG_PLACE_BIAS_X, y0 = LEG_PLACE_BIAS_Y;
	int16_t x1, y1, x2, y2, x3, y3, x4, y4;
	float cos_phi = arm_cos_f32(balance_infantry.state_vector[STATE_PITCH]);
	float sin_phi = arm_sin_f32(balance_infantry.state_vector[STATE_PITCH]);

	x1 = LEG_PLACE_BIAS_X + (int16_t)(balance_infantry.L_left[1] * cos_phi * LEG_RATIO);
	y1 = LEG_PLACE_BIAS_Y + (int16_t)(balance_infantry.L_left[1] * sin_phi * LEG_RATIO);
	x2 = LEG_PLACE_BIAS_X + (int16_t)((balance_infantry.motion_solver_l.x_D * cos_phi + balance_infantry.motion_solver_l.y_D * sin_phi) * LEG_RATIO);
	y2 = LEG_PLACE_BIAS_Y + (int16_t)((balance_infantry.motion_solver_l.x_D * sin_phi - balance_infantry.motion_solver_l.y_D * cos_phi) * LEG_RATIO);
	x3 = LEG_PLACE_BIAS_X + (int16_t)((balance_infantry.motion_solver_l.x_C * cos_phi + balance_infantry.motion_solver_l.y_C * sin_phi) * LEG_RATIO);
	y3 = LEG_PLACE_BIAS_Y + (int16_t)((balance_infantry.motion_solver_l.x_C * sin_phi - balance_infantry.motion_solver_l.y_C * cos_phi) * LEG_RATIO);
	x4 = LEG_PLACE_BIAS_X + (int16_t)((balance_infantry.motion_solver_l.x_B * cos_phi + balance_infantry.motion_solver_l.y_B * sin_phi) * LEG_RATIO);
	y4 = LEG_PLACE_BIAS_Y + (int16_t)((balance_infantry.motion_solver_l.x_B * sin_phi - balance_infantry.motion_solver_l.y_B * cos_phi) * LEG_RATIO);

	// ????????
	UI_Draw_Line(&UI_Graph5->Graphic[0], "500", GraphOperate, 3, UI_Color_Pink, 5, x0, y0, x1, y1);

	// ????????
	UI_Draw_Line(&UI_Graph5->Graphic[1], "501", GraphOperate, 3, UI_Color_Pink, 5, x1, y1, x2, y2);

	// ????????
	UI_Draw_Line(&UI_Graph5->Graphic[2], "502", GraphOperate, 3, UI_Color_Pink, 5, x2, y2, x3, y3);

	// // ????????
	UI_Draw_Line(&UI_Graph5->Graphic[3], "503", GraphOperate, 3, UI_Color_Pink, 5, x3, y3, x4, y4);

	// // ????????
	UI_Draw_Line(&UI_Graph5->Graphic[4], "504", GraphOperate, 3, UI_Color_Pink, 5, x4, y4, x0, y0);
}

/**
 * @brief ??ν??????????(????????????)
 * @param[in] void
 */
void Refereetask(void *pvParameters)
{
	portTickType xLastWakeTime;
	uint16_t UI_PushUp_Counter = 261;

	static char chassis_state[9][8] = {"OFFLINE", "SHRINK", "RECOVER", "BALANCE", "FOLLOW", "ROTATE", "JUMP", "LAND", "BESIDES"};
	static char gimbal_state[7][8] = {"OFFLINE", "ACT", "AUTOAIM", "TEST", "SI", "SMA_BUFF", "BIG_BUFF"};

	int8_t is_fric_offline = 0, is_pc_offline = 0, is_map_offline;
	uint16_t UI_PushUp_Counter_500;
	uint16_t UI_PushUp_Counter_20;

	while (1)
	{
		Referee_UnpackFifoData();

		/* UI???? */
		UI_PushUp_Counter++;
		UI_PushUp_Counter_500 = UI_PushUp_Counter % 500;
		UI_PushUp_Counter_20 = UI_PushUp_Counter % 20;
		// ???UI????(?????)
		if (UI_PushUp_Counter_500 == 37) // ??????????
		{
			UI_Draw_String(&referee_data.UI_String.String, "001", UI_Graph_Add, 2, UI_Color_Green, 15, 8, 3, 1600, 800, "  FRIC ");
			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
		}
		else if (UI_PushUp_Counter_500 == 107) // PC?????
		{
			UI_Draw_String(&referee_data.UI_String.String, "002", UI_Graph_Add, 2, UI_Color_Green, 15, 8, 3, 1600, 830, "  PC   ");
			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
		}
		else if (UI_PushUp_Counter_500 == 157) // ??????????
		{
			UI_Draw_String(&referee_data.UI_String.String, "003", UI_Graph_Add, 2, UI_Color_Green, 15, 8, 3, 1600, 860, "  BAY  ");
			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
		}
		else if (UI_PushUp_Counter_500 == 211) // ???????????
		{
			UI_Draw_String(&referee_data.UI_String.String, "100", UI_Graph_Add, 2, UI_Color_Orange, 15, 8, 3, 300, 745, chassis_state[remote_controller.control_mode_action]);
			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
		}
		else if (UI_PushUp_Counter_500 == 263) // ??????????
		{
			UI_Draw_String(&referee_data.UI_String.String, "101", UI_Graph_Add, 2, UI_Color_Purple, 15, 8, 3, 300, 765, gimbal_state[remote_controller.gimbal_action]);
			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
		}
		else if (UI_PushUp_Counter_500 == 293) // ????ID?????
		{
			UI_Draw_String(&referee_data.UI_String.String, "102", UI_Graph_Add, 2, UI_Color_Green, 15, 8, 3, 600, 880, "ID:   ");
			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
		}
		else if (UI_PushUp_Counter_500 == 331) // ?????????????
		{
			UI_Draw_String(&referee_data.UI_String.String, "200", UI_Graph_Add, 2, UI_Color_Green, 15, 10, 3, 1600, 770, "CAP:    %");
			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
		}
		else if (UI_PushUp_Counter_500 == 383) // PITCH?????????
		{
			UI_Draw_String(&referee_data.UI_String.String, "201", UI_Graph_Add, 2, UI_Color_Green, 15, 8, 3, 1280, 800, "PITCH: ");
			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
		}
		else if (UI_PushUp_Counter_500 == 439) // ???UI??????pitch????????????,?????????ID
		{
			// pitch???
			UI_Draw_Float(&referee_data.UI_Graph7.Graphic[0], "300", UI_Graph_Add, 3, UI_Color_Pink, 15, 2, 4, 1380, 800, gimbal_receiver_pack2.gimbal_pitch / 100.0f); // Pith????

			// ???????
			UI_Draw_Float(&referee_data.UI_Graph7.Graphic[1], "301", UI_Graph_Add, 3, UI_Color_Pink, 15, 2, 4, 1655, 770, super_power.actual_vol * 4.3478f);

			// ?????????
			UI_Draw_Line(&referee_data.UI_Graph7.Graphic[2], "302", UI_Graph_Add, 3, UI_Color_Orange, 5, 1620, 790, 1690, 790);

			// ?????????
			UI_Draw_Line(&referee_data.UI_Graph7.Graphic[3], "303", UI_Graph_Add, 3, UI_Color_Orange, 5, 1620, 850, 1690, 850);

			// PC????
			UI_Draw_Line(&referee_data.UI_Graph7.Graphic[4], "304", UI_Graph_Add, 3, UI_Color_Orange, 5, 1620, 820, 1690, 820);

			// ????ID???
			UI_Draw_Int(&referee_data.UI_Graph7.Graphic[5], "305", UI_Graph_Add, 3, UI_Color_Cyan, 20, 2, 640, 880, 0);

			// ???????????н???
			UI_Draw_Line(&referee_data.UI_Graph7.Graphic[6], "306", UI_Graph_Add, 3, UI_Color_Orange, 5, 1620, 400, 1690, 400);

			UI_PushUp_Graphs(7, &referee_data.UI_Graph7, referee_data.Game_Robot_State.robot_id);
		}
		else if (UI_PushUp_Counter_500 == 499) // ???????
		{
			drawFiveLeg(&referee_data.UI_Graph5, UI_Graph_Add);

			UI_PushUp_Graphs(5, &referee_data.UI_Graph5, referee_data.Game_Robot_State.robot_id);
		}

		if (UI_PushUp_Counter_20 == 7)
		{
			drawFiveLeg(&referee_data.UI_Graph5, UI_Graph_Change);

			UI_PushUp_Graphs(5, &referee_data.UI_Graph5, referee_data.Game_Robot_State.robot_id);
		}
		else if (UI_PushUp_Counter_20 == 16)
		{
			UI_Draw_String(&referee_data.UI_String.String, "100", UI_Graph_Change, 2, UI_Color_Orange, 15, 8, 3, 300, 745, chassis_state[remote_controller.control_mode_action]);
			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
		}
		else if (UI_PushUp_Counter_20 == 19)
		{
			UI_Draw_String(&referee_data.UI_String.String, "101", UI_Graph_Change, 2, UI_Color_Purple, 15, 8, 3, 300, 765, gimbal_state[remote_controller.control_mode_action]);
			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
		}
		else if (UI_PushUp_Counter_20 == 0) // ???UI???? ???
		{
			// pitch???
			UI_Draw_Float(&referee_data.UI_Graph7.Graphic[0], "300", UI_Graph_Change, 3, UI_Color_Pink, 15, 2, 4, 1380, 800, gimbal_receiver_pack2.gimbal_pitch / 100.0f); // Pith????

			/* ???????? */
			UI_Draw_Float(&referee_data.UI_Graph7.Graphic[1], "301", UI_Graph_Change, 3, UI_Color_Pink, 15, 2, 4, 1655, 770, super_power.actual_vol * 4.3478f);

			// ?????????,??????????????????
			is_fric_offline = remote_controller.shoot_action == SHOOT_POWERDOWN_MODE || remote_controller.shoot_action == SHOOT_SUPPLY_MODE;
			UI_Draw_Line(&referee_data.UI_Graph7.Graphic[2], "302", UI_Graph_Change, 3, UI_Color_Green + is_fric_offline, 5, 1620, 790, 1690, 790);

			// ?????????????????????????
			is_map_offline = (remote_controller.shoot_action == SHOOT_SUPPLY_MODE || remote_controller.shoot_action == SHOOT_TEST_MODE) ? 0 : 1;
			UI_Draw_Line(&referee_data.UI_Graph7.Graphic[3], "303", UI_Graph_Change, 3, UI_Color_Green + is_map_offline, 5, 1620, 850, 1690, 850);

			// PC????
			is_pc_offline = !gimbal_receiver_pack1.is_pc_on;
			UI_Draw_Line(&referee_data.UI_Graph7.Graphic[4], "304", UI_Graph_Change, 3, UI_Color_Green + is_pc_offline, 5, 1620, 820, 1690, 820);

			// ????ID???
			UI_Draw_Int(&referee_data.UI_Graph7.Graphic[5], "305", UI_Graph_Change, 3, UI_Color_Cyan, 20, 2, 640, 880, gimbal_receiver_pack1.autoaim_id);

			//  ??????????н?
			UI_Draw_Line(&referee_data.UI_Graph7.Graphic[6], "306", UI_Graph_Change, 3, UI_Color_Orange, 5, 1620, 400, (uint16_t)(1620 - (int16_t)(GIM_CHASSIS_ANGLE_LINE_LEN * balance_infantry.sin_dir)), (uint16_t)(400 + (int16_t)(GIM_CHASSIS_ANGLE_LINE_LEN * balance_infantry.cos_dir)));

			UI_PushUp_Graphs(7, &referee_data.UI_Graph7, referee_data.Game_Robot_State.robot_id);
		}

		xEventGroupSetBits(xCreatedEventGroup, REFEREE_TASK_BIT); // 标志位置一

		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
	}
}

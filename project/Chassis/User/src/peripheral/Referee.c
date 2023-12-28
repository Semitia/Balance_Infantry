/**
 ******************************************************************************
 * @file    referee.c
 * @author  Karolance Future
 * @version V1.0.0
 * @date    2022/03/21
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

/* Private includes ----------------------------------------------------------*/
#include "Referee.h"
#include "protocol.h"
#include "string.h"
#include "bsp_referee.h"

/* Private define ------------------------------------------------------------*/
Referee_t referee_data;
RefereeDataUpdate referee_data_updater;

/* Private variables ---------------------------------------------------------*/

int8_t fullDataBuffer[MAX_REFEREE_DATA_LEN];
/* Functions -----------------------------------------------------------------*/

void Referee_StructInit(void)
{
	memset(&referee_data.Referee_Receive_Header, 0, sizeof(referee_data.Referee_Receive_Header));

	memset(&referee_data.Game_Status, 0, sizeof(referee_data.Game_Status));
	memset(&referee_data.Game_Result, 0, sizeof(referee_data.Game_Result));
	memset(&referee_data.Game_Robot_HP, 0, sizeof(referee_data.Game_Robot_HP));

	memset(&referee_data.Event_Data, 0, sizeof(referee_data.Event_Data));
	memset(&referee_data.Supply_Projectile_Action, 0, sizeof(referee_data.Supply_Projectile_Action));
	memset(&referee_data.Supply_Projectile_Booking, 0, sizeof(referee_data.Supply_Projectile_Booking));
	memset(&referee_data.Referee_Warning, 0, sizeof(referee_data.Referee_Warning));
	memset(&referee_data.Dart_Remaining_Time, 0, sizeof(referee_data.Dart_Remaining_Time));

	memset(&referee_data.Game_Robot_State, 0, sizeof(referee_data.Game_Robot_State));
	memset(&referee_data.Power_Heat_Data, 0, sizeof(referee_data.Power_Heat_Data));
	memset(&referee_data.Game_Robot_Pos, 0, sizeof(referee_data.Game_Robot_Pos));
	memset(&referee_data.Buff_Musk, 0, sizeof(referee_data.Buff_Musk));
	memset(&referee_data.Aerial_Robot_Energy, 0, sizeof(referee_data.Aerial_Robot_Energy));
	memset(&referee_data.Robot_Hurt, 0, sizeof(referee_data.Robot_Hurt));
	memset(&referee_data.Shoot_Data, 0, sizeof(referee_data.Shoot_Data));
	memset(&referee_data.Bullet_Remaining, 0, sizeof(referee_data.Bullet_Remaining));
	memset(&referee_data.RFID_Status, 0, sizeof(referee_data.RFID_Status));
	memset(&referee_data.Dart_Client_Cmd, 0, sizeof(referee_data.Dart_Client_Cmd));

	memset(&referee_data.Student_Interactive_Header_Data, 0, sizeof(referee_data.Student_Interactive_Header_Data));
	memset(&referee_data.Robot_Interactive_Data, 0, sizeof(referee_data.Robot_Interactive_Data));
	memset(&referee_data.Robot_Command, 0, sizeof(referee_data.Robot_Command));
	memset(&referee_data.Client_Map_Command, 0, sizeof(referee_data.Client_Map_Command));
}

/*==============================================================================
			  #####     ϵͳ   ݽ        #####
  ==============================================================================
	[..]   ò    ṩ   º   :
		  (+)     ϵͳ       ݽ ѹ     Referee_UnpackFifoData
	  (+)     ϵͳ       ݴ       Referee_SolveFifoData
*/

void Referee_UnpackFifoData()
{
	//      һȦ   ڣ   ֹ         δ      µĿ
	referee_data.decoder.receive_data_len = REFEREE_RECVBUF_SIZE - DMA_GetCurrDataCounter(REFEREE_RECV_DMAx_Streamx) + referee_data.decoder.judgementFullCount * REFEREE_RECVBUF_SIZE; //  ȡ   е ǰ Ѿ    յĳ

	//      Ȧ
	if (referee_data.decoder.receive_data_len - referee_data.decoder.decode_data_len > 2 * REFEREE_RECVBUF_SIZE)
	{
		referee_data.decoder.decode_data_len = referee_data.decoder.receive_data_len - 2 * REFEREE_RECVBUF_SIZE;
	}
	int read_arr = referee_data.decoder.decode_data_len % REFEREE_RECVBUF_SIZE; //  ǰӦ ö ȡ   ֽ   judgeDataBuffer  λ
	u8 byte;

	while (referee_data.decoder.receive_data_len > referee_data.decoder.decode_data_len + 1) //  û    ʱ
	{
		byte = Refereebuffer[read_arr]; //     ֽ ȡ

		switch (referee_data.decoder.judgementStep)
		{
		case STEP_HEADER_SOF: // ֡ͷ
			if (byte == 0xA5) //  ʼλ
			{
				fullDataBuffer[referee_data.decoder.index++] = byte;
				referee_data.decoder.judgementStep = STEP_LENGTH_LOW;
			}
			else
			{
				referee_data.decoder.index = 0; //      ʼλ  ֱ
			}
			break;
		case STEP_LENGTH_LOW:
		{
			referee_data.decoder.data_len = byte;
			fullDataBuffer[referee_data.decoder.index++] = byte;
			referee_data.decoder.judgementStep = STEP_LENGTH_HIGH;
		}
		break;
		case STEP_LENGTH_HIGH:
		{
			referee_data.decoder.data_len |= (byte << 8);
			fullDataBuffer[referee_data.decoder.index++] = byte;
			if (referee_data.decoder.data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
			{
				referee_data.decoder.judgementStep = STEP_FRAME_SEQ;
			}
			else //   ȴ
			{
				referee_data.decoder.judgementStep = STEP_HEADER_SOF;
				referee_data.decoder.index = 0;
			}
		}
		break;
		case STEP_FRAME_SEQ:
		{
			fullDataBuffer[referee_data.decoder.index++] = byte;
			referee_data.decoder.judgementStep = STEP_HEADER_CRC8;
		}
		break;
		case STEP_HEADER_CRC8:
		{
			fullDataBuffer[referee_data.decoder.index++] = byte;
			if (referee_data.decoder.index == REF_PROTOCOL_HEADER_SIZE)
			{
				if (Verify_CRC8_Check_Sum((unsigned char *)fullDataBuffer, REF_PROTOCOL_HEADER_SIZE))
				{
					referee_data.decoder.judgementStep = STEP_DATA_CRC16;
				}
				else
				{
					referee_data.decoder.judgementStep = STEP_HEADER_SOF;
					referee_data.decoder.index = 0;
				}
			}
		}
		break;
		case STEP_DATA_CRC16:
		{
			if (referee_data.decoder.index < (REF_HEADER_CRC_CMDID_LEN + referee_data.decoder.data_len))
			{
				fullDataBuffer[referee_data.decoder.index++] = byte;
			}
			if (referee_data.decoder.index >= (REF_HEADER_CRC_CMDID_LEN + referee_data.decoder.data_len))
			{
				//      ص   ʼλ
				referee_data.decoder.judgementStep = STEP_HEADER_SOF;
				referee_data.decoder.index = 0;
				if (Verify_CRC16_Check_Sum((unsigned char *)fullDataBuffer, REF_HEADER_CRC_CMDID_LEN + referee_data.decoder.data_len))
				{
					Referee_SolveFifoData((unsigned char *)fullDataBuffer);
				}
			}
		}
		break;

		default:
			referee_data.decoder.judgementStep = STEP_HEADER_SOF; //  ͷ  ʼ
			referee_data.decoder.index = 0;						  //   fullDataBuffer
			break;
		}

		referee_data.decoder.decode_data_len++; //  Ϊ   ֽ  Ѿ   ȡ  λ
		read_arr = read_arr + 1 >= REFEREE_RECVBUF_SIZE ? 0 : read_arr + 1;
		//   ½   λ
		referee_data.decoder.receive_data_len = REFEREE_RECVBUF_SIZE - DMA_GetCurrDataCounter(REFEREE_RECV_DMAx_Streamx) + referee_data.decoder.judgementFullCount * REFEREE_RECVBUF_SIZE; //  ȡ   е ǰ Ѿ    յĳ
	}
	if (referee_data.decoder.receive_data_len % REFEREE_RECVBUF_SIZE > (REFEREE_RECVBUF_SIZE / 3) &&
		referee_data.decoder.receive_data_len % REFEREE_RECVBUF_SIZE < (2 * REFEREE_RECVBUF_SIZE / 3)) //  ֹjudgementFullCount                         ܻᵼ ° Ȧ
	{
		referee_data.decoder.decode_data_len -= REFEREE_RECVBUF_SIZE * referee_data.decoder.judgementFullCount;
		referee_data.decoder.judgementFullCount = 0;
	}
}

void Referee_SolveFifoData(uint8_t *frame)
{
	uint16_t cmd_id = 0;
	uint8_t index = 0;

	memcpy(&referee_data.Referee_Receive_Header, frame, sizeof(frame_header_struct_t));
	index += sizeof(frame_header_struct_t);
	memcpy(&cmd_id, frame + index, sizeof(uint16_t));
	index += sizeof(uint16_t);

	switch (cmd_id)
	{
	case GAME_STATE_CMD_ID:
		memcpy(&referee_data.Game_Status, frame + index, sizeof(ext_game_status_t));
		break;
	case GAME_RESULT_CMD_ID:
		memcpy(&referee_data.Game_Result, frame + index, sizeof(ext_game_result_t));
		break;
	case GAME_ROBOT_HP_CMD_ID:
		memcpy(&referee_data.Game_Robot_HP, frame + index, sizeof(ext_game_robot_HP_t));
		break;
	case DART_FLYING_STATE_CMD_ID:
		break;

	case FIELD_EVENTS_CMD_ID:
		memcpy(&referee_data.Event_Data, frame + index, sizeof(ext_event_data_t));
		break;
	case SUPPLY_PROJECTILE_ACTION_CMD_ID:
		memcpy(&referee_data.Supply_Projectile_Action, frame + index, sizeof(ext_supply_projectile_action_t));
		break;
	case SUPPLY_PROJECTILE_BOOKING_CMD_ID:
		memcpy(&referee_data.Supply_Projectile_Booking, frame + index, sizeof(ext_supply_projectile_booking_t));
		break;
	case REFEREE_WARNING_CMD_ID:
		memcpy(&referee_data.Referee_Warning, frame + index, sizeof(ext_referee_warning_t));
		break;
	case DART_REMAINING_TIME_CMD_ID:
		memcpy(&referee_data.Dart_Remaining_Time, frame + index, sizeof(ext_dart_remaining_time_t));
		break;

	case ROBOT_STATE_CMD_ID:
		referee_data_updater.is_max_power_data_update = TRUE;
		memcpy(&referee_data.Game_Robot_State, frame + index, sizeof(ext_game_robot_state_t));
		break;
	case POWER_HEAT_DATA_CMD_ID:
		referee_data_updater.is_power_data_update = TRUE;
		heat_controller.heat_count++;
		memcpy(&referee_data.Power_Heat_Data, frame + index, sizeof(ext_power_heat_data_t));
		break;
	case ROBOT_POS_CMD_ID:
		memcpy(&referee_data.Game_Robot_Pos, frame + index, sizeof(ext_game_robot_pos_t));
		break;
	case BUFF_MUSK_CMD_ID:
		memcpy(&referee_data.Buff_Musk, frame + index, sizeof(ext_buff_musk_t));
		break;
	case AERIAL_ROBOT_ENERGY_CMD_ID:
		memcpy(&referee_data.Aerial_Robot_Energy, frame + index, sizeof(aerial_robot_energy_t));
		break;
	case ROBOT_HURT_CMD_ID:
		memcpy(&referee_data.Robot_Hurt, frame + index, sizeof(ext_robot_hurt_t));
		break;
	case SHOOT_DATA_CMD_ID:
		heat_controller.shoot_count++;
		memcpy(&referee_data.Shoot_Data, frame + index, sizeof(ext_shoot_data_t));
		break;
	case BULLET_REMAINING_CMD_ID:
		memcpy(&referee_data.Bullet_Remaining, frame + index, sizeof(ext_bullet_remaining_t));
		break;
	case ROBOT_RFID_STATE_CMD_ID:
		memcpy(&referee_data.RFID_Status, frame + index, sizeof(ext_rfid_status_t));
		break;
	case DART_CLIENT_CMD_ID:
		memcpy(&referee_data.Dart_Client_Cmd, frame + index, sizeof(ext_dart_client_cmd_t));
		break;

	case STUDENT_INTERACTIVE_DATA_CMD_ID:
		memcpy(&referee_data.Robot_Interactive_Data, frame + index, sizeof(robot_interactive_data_t));
		break;
	case ROBOT_COMMAND_CMD_ID:
		memcpy(&referee_data.Robot_Command, frame + index, sizeof(ext_robot_command_t));
		break;
	case CLIENT_MAP_COMMAND_CMD_ID:
		memcpy(&referee_data.Client_Map_Command, frame + index, sizeof(ext_client_map_command_t));
		break;

	default:
		break;
	}
}

/*==============================================================================
			  ##### UI    ͼ λ  ƺ    #####
  ==============================================================================
*/

/**
 * @brief     ֱ
 * @param[in] Graph UIͼ     ݽṹ  ָ
 * @param[in] GraphName ͼ       Ϊ ͻ  ˵
 * @param[in] GraphOperate UIͼ β      ӦUI_Graph_XXX  4 ֲ
 * @param[in] Layer   UIͼ  ͼ   [0,9]
 * @param[in] Color UIͼ    ɫ   ӦUI_Color_XXX  9    ɫ
 * @param[in] Width  ߿
 * @param[in] StartX   ʼ    X
 * @param[in] StartY   ʼ    Y
 * @param[in] EndX   ֹ    X
 * @param[in] EndY   ֹ    Y
 */
void UI_Draw_Line(graphic_data_struct_t *Graph,
				  char GraphName[3],
				  uint8_t GraphOperate,
				  uint8_t Layer,
				  uint8_t Color,
				  uint16_t Width,
				  uint16_t StartX,
				  uint16_t StartY,
				  uint16_t EndX,
				  uint16_t EndY)
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye = GraphOperate;
	Graph->graphic_tpye = UI_Graph_Line;
	Graph->layer = Layer;
	Graph->color = Color;
	Graph->width = Width;
	Graph->start_x = StartX;
	Graph->start_y = StartY;
	Graph->end_x = EndX;
	Graph->end_y = EndY;
}

/**
 * @brief    ƾ
 * @param[in] Graph UIͼ     ݽṹ  ָ
 * @param[in] GraphName ͼ       Ϊ ͻ  ˵
 * @param[in] GraphOperate UIͼ β      ӦUI_Graph_XXX  4 ֲ
 * @param[in] Layer   UIͼ  ͼ   [0,9]
 * @param[in] Color UIͼ    ɫ   ӦUI_Color_XXX  9    ɫ
 * @param[in] Width  ߿
 * @param[in] StartX   ʼ    X
 * @param[in] StartY   ʼ    Y
 * @param[in] EndX   ֹ    X
 * @param[in] EndY   ֹ    Y
 */
void UI_Draw_Rectangle(graphic_data_struct_t *Graph,
					   char GraphName[3],
					   uint8_t GraphOperate,
					   uint8_t Layer,
					   uint8_t Color,
					   uint16_t Width,
					   uint16_t StartX,
					   uint16_t StartY,
					   uint16_t EndX,
					   uint16_t EndY)
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye = GraphOperate;
	Graph->graphic_tpye = UI_Graph_Rectangle;
	Graph->layer = Layer;
	Graph->color = Color;
	Graph->width = Width;
	Graph->start_x = StartX;
	Graph->start_y = StartY;
	Graph->end_x = EndX;
	Graph->end_y = EndY;
}

/**
 * @brief     Բ
 * @param[in] Graph UIͼ     ݽṹ  ָ
 * @param[in] GraphName ͼ       Ϊ ͻ  ˵
 * @param[in] GraphOperate UIͼ β      ӦUI_Graph_XXX  4 ֲ
 * @param[in] Layer   UIͼ  ͼ   [0,9]
 * @param[in] Color UIͼ    ɫ   ӦUI_Color_XXX  9    ɫ
 * @param[in] Width  ߿
 * @param[in] CenterX Բ      X
 * @param[in] CenterY Բ      Y
 * @param[in] Radius  뾶
 */
void UI_Draw_Circle(graphic_data_struct_t *Graph,
					char GraphName[3],
					uint8_t GraphOperate,
					uint8_t Layer,
					uint8_t Color,
					uint16_t Width,
					uint16_t CenterX,
					uint16_t CenterY,
					uint16_t Radius)
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye = GraphOperate;
	Graph->graphic_tpye = UI_Graph_Circle;
	Graph->layer = Layer;
	Graph->color = Color;
	Graph->width = Width;
	Graph->start_x = CenterX;
	Graph->start_y = CenterY;
	Graph->radius = Radius;
}

/**
 * @brief       Բ
 * @param[in] Graph UIͼ     ݽṹ  ָ
 * @param[in] GraphName ͼ       Ϊ ͻ  ˵
 * @param[in] GraphOperate UIͼ β      ӦUI_Graph_XXX  4 ֲ
 * @param[in] Layer   UIͼ  ͼ   [0,9]
 * @param[in] Color UIͼ    ɫ   ӦUI_Color_XXX  9    ɫ
 * @param[in] Width  ߿
 * @param[in] CenterX Բ      X
 * @param[in] CenterY Բ      Y
 * @param[in] XHalfAxis X   ᳤
 * @param[in] YHalfAxis Y   ᳤
 */
void UI_Draw_Ellipse(graphic_data_struct_t *Graph,
					 char GraphName[3],
					 uint8_t GraphOperate,
					 uint8_t Layer,
					 uint8_t Color,
					 uint16_t Width,
					 uint16_t CenterX,
					 uint16_t CenterY,
					 uint16_t XHalfAxis,
					 uint16_t YHalfAxis)
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye = GraphOperate;
	Graph->graphic_tpye = UI_Graph_Ellipse;
	Graph->layer = Layer;
	Graph->color = Color;
	Graph->width = Width;
	Graph->start_x = CenterX;
	Graph->start_y = CenterY;
	Graph->end_x = XHalfAxis;
	Graph->end_y = YHalfAxis;
}

/**
 * @brief     Բ
 * @param[in] Graph UIͼ     ݽṹ  ָ
 * @param[in] GraphName ͼ       Ϊ ͻ  ˵
 * @param[in] GraphOperate UIͼ β      ӦUI_Graph_XXX  4 ֲ
 * @param[in] Layer   UIͼ  ͼ   [0,9]
 * @param[in] Color UIͼ    ɫ   ӦUI_Color_XXX  9    ɫ
 * @param[in] StartAngle   ʼ Ƕ  [0,360]
 * @param[in] EndAngle   ֹ Ƕ  [0,360]
 * @param[in] Width  ߿
 * @param[in] CenterX Բ      X
 * @param[in] CenterY Բ      Y
 * @param[in] XHalfAxis X   ᳤
 * @param[in] YHalfAxis Y   ᳤
 */
void UI_Draw_Arc(graphic_data_struct_t *Graph,
				 char GraphName[3],
				 uint8_t GraphOperate,
				 uint8_t Layer,
				 uint8_t Color,
				 uint16_t StartAngle,
				 uint16_t EndAngle,
				 uint16_t Width,
				 uint16_t CenterX,
				 uint16_t CenterY,
				 uint16_t XHalfAxis,
				 uint16_t YHalfAxis)
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye = GraphOperate;
	Graph->graphic_tpye = UI_Graph_Arc;
	Graph->layer = Layer;
	Graph->color = Color;
	Graph->start_angle = StartAngle;
	Graph->end_angle = EndAngle;
	Graph->width = Width;
	Graph->start_x = CenterX;
	Graph->start_y = CenterY;
	Graph->end_x = XHalfAxis;
	Graph->end_y = YHalfAxis;
}

/**
 * @brief    Ƹ
 * @param[in] Graph UIͼ     ݽṹ  ָ
 * @param[in] GraphName ͼ       Ϊ ͻ  ˵
 * @param[in] GraphOperate UIͼ β      ӦUI_Graph_XXX  4 ֲ
 * @param[in] Layer   UIͼ  ͼ   [0,9]
 * @param[in] Color UIͼ    ɫ   ӦUI_Color_XXX  9    ɫ
 * @param[in] NumberSize      С
 * @param[in] Significant   Чλ
 * @param[in] Width  ߿
 * @param[in] StartX   ʼ    X
 * @param[in] StartY   ʼ    Y
 * @param[in] FloatData
 */
void UI_Draw_Float(graphic_data_struct_t *Graph,
				   char GraphName[3],
				   uint8_t GraphOperate,
				   uint8_t Layer,
				   uint8_t Color,
				   uint16_t NumberSize,
				   uint16_t Significant,
				   uint16_t Width,
				   uint16_t StartX,
				   uint16_t StartY,
				   float FloatData)
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye = GraphOperate;
	Graph->graphic_tpye = UI_Graph_Float;
	Graph->layer = Layer;
	Graph->color = Color;
	Graph->start_angle = NumberSize;
	Graph->end_angle = Significant;
	Graph->width = Width;
	Graph->start_x = StartX;
	Graph->start_y = StartY;
	int32_t IntData = FloatData * 1000;
	Graph->radius = (IntData & 0x000003ff) >> 0;
	Graph->end_x = ((IntData & 0x001ffc00) >> 10);
	Graph->end_y = ((IntData & 0xffe00000) >> 21);
}

/**
 * @brief
 * @param[in] Graph UIͼ     ݽṹ  ָ
 * @param[in] GraphName ͼ       Ϊ ͻ  ˵
 * @param[in] GraphOperate UIͼ β      ӦUI_Graph_XXX  4 ֲ
 * @param[in] Layer   UIͼ  ͼ   [0,9]
 * @param[in] Color UIͼ    ɫ   ӦUI_Color_XXX  9    ɫ
 * @param[in] NumberSize      С
 * @param[in] Width  ߿
 * @param[in] StartX   ʼ    X
 * @param[in] StartY   ʼ    Y
 * @param[in] IntData
 */
void UI_Draw_Int(graphic_data_struct_t *Graph,
				 char GraphName[3],
				 uint8_t GraphOperate,
				 uint8_t Layer,
				 uint8_t Color,
				 uint16_t NumberSize,
				 uint16_t Width,
				 uint16_t StartX,
				 uint16_t StartY,
				 int32_t IntData)
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye = GraphOperate;
	Graph->graphic_tpye = UI_Graph_Int;
	Graph->layer = Layer;
	Graph->color = Color;
	Graph->start_angle = NumberSize;
	Graph->width = Width;
	Graph->start_x = StartX;
	Graph->start_y = StartY;
	Graph->radius = (IntData & 0x000003ff) >> 0;
	Graph->end_x = (IntData & 0x001ffc00) >> 10;
	Graph->end_y = (IntData & 0xffe00000) >> 21;
}

/**
 * @brief      ַ
 * @param[in] String UIͼ     ݽṹ  ָ
 * @param[in] StringName ͼ       Ϊ ͻ  ˵
 * @param[in] StringOperate UIͼ β      ӦUI_Graph_XXX  4 ֲ
 * @param[in] Layer   UIͼ  ͼ   [0,9]
 * @param[in] Color UIͼ    ɫ   ӦUI_Color_XXX  9    ɫ
 * @param[in] CharSize      С
 * @param[in] StringLength  ַ
 * @param[in] Width  ߿
 * @param[in] StartX   ʼ    X
 * @param[in] StartY   ʼ    Y
 * @param[in] StringData  ַ
 */
void UI_Draw_String(string_data_struct_t *String,
					char StringName[3],
					uint8_t StringOperate,
					uint8_t Layer,
					uint8_t Color,
					uint16_t CharSize,
					uint16_t StringLength,
					uint16_t Width,
					uint16_t StartX,
					uint16_t StartY,
					char *StringData)
{
	String->string_name[0] = StringName[0];
	String->string_name[1] = StringName[1];
	String->string_name[2] = StringName[2];
	String->operate_tpye = StringOperate;
	String->graphic_tpye = UI_Graph_String;
	String->layer = Layer;
	String->color = Color;
	String->start_angle = CharSize;
	String->end_angle = StringLength;
	String->width = Width;
	String->start_x = StartX;
	String->start_y = StartY;
	memset(String->stringdata, 0, 30);
	for (int i = 0; i < StringLength; i++)
		String->stringdata[i] = *StringData++;
}

/*==============================================================================
			  ##### UI    ͼ     ͺ    #####
  ==============================================================================
*/

/**
 * @brief     ͼ
 * @param[in] Counter 1,2,5,7
 * @param[in] Graphs   Counter  һ µ UI_Graphx ṹ  ͷָ
 * @param[in] RobotID
 */
void UI_PushUp_Graphs(uint8_t Counter, void *Graphs, uint8_t RobotID)
{
	UI_Graph1_t *Graph = (UI_Graph1_t *)Graphs; //    ֻ  һ      ͼ

	/*     frame_header */
	Graph->Referee_Transmit_Header.SOF = HEADER_SOF;
	if (Counter == 1)
		Graph->Referee_Transmit_Header.data_length = 6 + 1 * 15;
	else if (Counter == 2)
		Graph->Referee_Transmit_Header.data_length = 6 + 2 * 15;
	else if (Counter == 5)
		Graph->Referee_Transmit_Header.data_length = 6 + 5 * 15;
	else if (Counter == 7)
		Graph->Referee_Transmit_Header.data_length = 6 + 7 * 15;
	Graph->Referee_Transmit_Header.seq = Graph->Referee_Transmit_Header.seq + 1;
	Append_CRC8_Check_Sum((uint8_t *)(&Graph->Referee_Transmit_Header), sizeof(frame_header_struct_t));

	/*     cmd_id */
	Graph->CMD_ID = STUDENT_INTERACTIVE_DATA_CMD_ID;

	/*     student_interactive_header */
	if (Counter == 1)
		Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw1;
	else if (Counter == 2)
		Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw2;
	else if (Counter == 5)
		Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw5;
	else if (Counter == 7)
		Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw7;
	Graph->Interactive_Header.sender_ID = RobotID;		   //  ǰ      ID
	Graph->Interactive_Header.receiver_ID = RobotID + 256; //  Ӧ      ID

	/*     frame_tail   CRC16 */
	if (Counter == 1)
	{
		UI_Graph1_t *Graph1 = (UI_Graph1_t *)Graphs;
		Append_CRC16_Check_Sum((uint8_t *)Graph1, sizeof(UI_Graph1_t));
	}
	else if (Counter == 2)
	{
		UI_Graph2_t *Graph2 = (UI_Graph2_t *)Graphs;
		Append_CRC16_Check_Sum((uint8_t *)Graph2, sizeof(UI_Graph2_t));
	}
	else if (Counter == 5)
	{
		UI_Graph5_t *Graph5 = (UI_Graph5_t *)Graphs;
		Append_CRC16_Check_Sum((uint8_t *)Graph5, sizeof(UI_Graph5_t));
	}
	else if (Counter == 7)
	{
		UI_Graph7_t *Graph7 = (UI_Graph7_t *)Graphs;
		Append_CRC16_Check_Sum((uint8_t *)Graph7, sizeof(UI_Graph7_t));
	}

	// /* ʹ ô   PushUp      ϵͳ */
	if (Counter == 1)
		REFEREE_SendBytes((uint8_t *)Graph, sizeof(UI_Graph1_t));
	else if (Counter == 2)
		REFEREE_SendBytes((uint8_t *)Graph, sizeof(UI_Graph2_t));
	else if (Counter == 5)
		REFEREE_SendBytes((uint8_t *)Graph, sizeof(UI_Graph5_t));
	else if (Counter == 7)
		REFEREE_SendBytes((uint8_t *)Graph, sizeof(UI_Graph7_t));
}

/**
 * @brief      ַ
 * @param[in] String
 * @param[in] RobotID
 */
void UI_PushUp_String(UI_String_t *String, uint8_t RobotID)
{
	/*     frame_header */
	String->Referee_Transmit_Header.SOF = HEADER_SOF;
	String->Referee_Transmit_Header.data_length = 6 + 45;
	String->Referee_Transmit_Header.seq = String->Referee_Transmit_Header.seq + 1;
	Append_CRC8_Check_Sum((uint8_t *)(&String->Referee_Transmit_Header), sizeof(frame_header_struct_t));

	/*     cmd_id */
	String->CMD_ID = STUDENT_INTERACTIVE_DATA_CMD_ID;

	/*     student_interactive_header */
	String->Interactive_Header.data_cmd_id = UI_DataID_DrawChar;
	String->Interactive_Header.sender_ID = RobotID;			//  ǰ      ID
	String->Interactive_Header.receiver_ID = RobotID + 256; //  Ӧ      ID

	/*     frame_tail   CRC16 */
	Append_CRC16_Check_Sum((uint8_t *)String, sizeof(UI_String_t));

	/* ʹ ô   PushUp      ϵͳ */
	REFEREE_SendBytes((uint8_t *)String, sizeof(UI_String_t));
}

/**
 * @brief ɾ  ͼ
 * @param[in] Delete
 * @param[in] RobotID
 */
void UI_PushUp_Delete(UI_Delete_t *Delete, uint8_t RobotID)
{
	/*     frame_header */
	Delete->Referee_Transmit_Header.SOF = HEADER_SOF;
	Delete->Referee_Transmit_Header.data_length = 6 + 2;
	Delete->Referee_Transmit_Header.seq = Delete->Referee_Transmit_Header.seq + 1;
	Append_CRC8_Check_Sum((uint8_t *)(&Delete->Referee_Transmit_Header), sizeof(frame_header_struct_t));

	/*     cmd_id */
	Delete->CMD_ID = STUDENT_INTERACTIVE_DATA_CMD_ID;

	/*     student_interactive_header */
	Delete->Interactive_Header.data_cmd_id = UI_DataID_Delete;
	Delete->Interactive_Header.sender_ID = RobotID;			//  ǰ      ID
	Delete->Interactive_Header.receiver_ID = RobotID + 256; //  Ӧ      ID

	/*     frame_tail   CRC16 */
	Append_CRC16_Check_Sum((uint8_t *)Delete, sizeof(UI_Delete_t));

	/* ʹ ô   PushUp      ϵͳ */
	REFEREE_SendBytes((uint8_t *)Delete, sizeof(UI_Delete_t));
}

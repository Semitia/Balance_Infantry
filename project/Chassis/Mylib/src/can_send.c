#include "can_send.h"

void CanSend(CAN_TypeDef *CANx, int8_t *data, uint32_t std_id, uint8_t data_length)
{
  CanTxMsg tx_message;
  tx_message.IDE = CAN_ID_STD;
  tx_message.RTR = CAN_RTR_DATA;
  tx_message.DLC = data_length;
  tx_message.StdId = std_id;

  memcpy(tx_message.Data, data, data_length);
  CAN_Transmit(CANx, &tx_message);
}

// 上位机可视化测试用
// extern INS_t INS;
// uint64_t count_can = 0;

// /* 陀螺仪数据CAN发送 */
// void SendIMUByCan(void)
// {
//   IMU_Out imu_out;

//   imu_out.pitch = INS.Pitch;
//   imu_out.roll = INS.Roll;

//   count_can++;

//   if (count_can % 2 == 0)
//   {
//     CanSend(CAN2, (int8_t *)(&imu_out), 0x110, sizeof(imu_out));
//   }
// }

#ifndef CAN_SEND_H
#define CAN_SEND_H

#include "can1.h"
#include "can2.h"

/* CAN发送结构体定义  */
#pragma pack(push, 1)

typedef struct IMU_Out
{
    float pitch;
    float roll;
} IMU_Out;

#pragma pack(pop)

void CanSend(CAN_TypeDef *CANx, int8_t *data, uint32_t std_id, uint8_t data_length);

#endif // !CAN_SEND_H

/**
 ******************************************************************************
 * @file    pc_uart.c
 * @brief   serial数据接发
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "pc_serial.h"

/* 全局变量 */
static unsigned char temptemp[2 * PC_RECVBUF_SIZE]; //用于PC数据处理
unsigned char tempPC[PC_RECVBUF_SIZE];

void PCReceive(unsigned char *PCbuffer)
{
    memcpy(temptemp + PC_RECVBUF_SIZE, PCbuffer, PC_RECVBUF_SIZE);
    for (uint8_t PackPoint = 0; PackPoint < PC_RECVBUF_SIZE + 1; PackPoint++) //防止错位，不一定数组元素的第一个就为
    {
        if (temptemp[PackPoint] == '!')
        {
            memcpy(tempPC, temptemp + PackPoint, PC_RECVBUF_SIZE);
            if (Verify_CRC8_Check_Sum(tempPC, PC_RECVBUF_SIZE))
            {
                //数据解码
            }
            break;
        }
    }
    memcpy(temptemp, temptemp + PC_RECVBUF_SIZE, PC_RECVBUF_SIZE);
}

/*  接收数据定义  */
extern unsigned char SendToPC_Buff[PC_SENDBUF_SIZE];
int32_t PC_TX_num = 0;
extern INS_t INS;

/**
 * @brief 在这里写发送数据的封装
 * @param[in] void
 */
void SendtoPCPack(unsigned char *buff)
{
    PCSendData pc_send_data;
    pc_send_data.yaw = INS.YawTotalAngle * 100;
    pc_send_data.pitch = INS.Pitch * 100;
    pc_send_data.roll = INS.Roll * 100;

    memcpy(buff, &pc_send_data.yaw, PC_SENDBUF_SIZE - 2);
}

/**
 * @brief 发送数据调用
 * @param[in] void
 */
void SendtoPC(void)
{
    if (PC_TX_num % TX_DOWN_SAMPLING_TATE == 0)
    {
        SendToPC_Buff[0] = '!';

        SendtoPCPack(&SendToPC_Buff[1]);

        SendToPC_Buff[PC_SENDBUF_SIZE - 1] = '#';
        Append_CRC8_Check_Sum(SendToPC_Buff, PC_SENDBUF_SIZE);
//        DMA_Cmd(PC_UART_SEND_DMAx_Streamx, ENABLE);
    }
    PC_TX_num++;
}

/**
 ******************************************************************************
 * @file    pc_uart.c
 * @brief   serial数据接发
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "pc_serial.h"
#include "Gimbal.h"
#include "arm_atan2_f32.h"
#include "debug.h"

#include "SignalGenerator.h"

PCRecvData pc_recv_data;
PCSendData pc_send_data;

void PCSolve(void)
{
    LossUpdate(&global_debugger.pc_receive_debugger, 0.02);
}

void PCReceive(unsigned char *PCbuffer)
{
    if (PCbuffer[0] == '!' && Verify_CRC16_Check_Sum(PCbuffer, PC_RECVBUF_SIZE))
    {
        //数据解码
        memcpy(&pc_recv_data, PCbuffer, PC_RECVBUF_SIZE);
        PCSolve();
    }
}

/**
 * @brief 在这里写发送数据的封装
 * @param[in] void
 */
void SendtoPCPack(unsigned char *buff)
{
    static SinFunction sin_function;
    static int8_t is_init = 0;
    if (!is_init)
    {
        SinInit(&sin_function, 80, 10, 1000);
        is_init = 1;
    }

    pc_send_data.start_flag = '!';
    pc_send_data.robot_color = chassis_pack_get_1.robot_color;
    pc_send_data.shoot_level = chassis_pack_get_1.bullet_level;
    pc_send_data.which_balance = 0;
    pc_send_data.change_priority_flag = 0;
    pc_send_data.frame_id++;
    pc_send_data.pitch = (short)(gimbal_controller.gyro_pitch_angle * 100.0f);
    pc_send_data.yaw = gimbal_controller.gyro_yaw_angle;

    Append_CRC16_Check_Sum((unsigned char *)&pc_send_data, PC_SENDBUF_SIZE);
    memcpy(buff, (void *)&pc_send_data, PC_SENDBUF_SIZE);
}

/**
 * @brief 发送数据调用
 * @param[in] void
 */
void SendtoPC(void)
{
    SendtoPCPack(SendToPC_Buff);

    DMA_Cmd(PC_UART_SEND_DMAx_Streamx, ENABLE);
}

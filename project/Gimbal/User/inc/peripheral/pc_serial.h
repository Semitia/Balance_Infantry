#ifndef _PC_SERIAL_H
#define _PC_SERIAL_H

#include "pc_uart.h"
#include "algorithmOfCRC.h"
#include "ins.h"

#include "FreeRTOS.h"
#include "task.h"

#include "ChassisGet.h"

enum AUTOAIM_MODE
{
    AUTO_AIM = 0,
    SMALL_BUFF,
    BIG_BUFF
};

void PCReceive(unsigned char *PCbuffer);
void SendtoPC(void);

extern PCRecvData pc_recv_data;
extern PCSendData pc_send_data;

#endif // !_PC_SERIAL_H

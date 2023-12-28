#ifndef _PC_SERIAL_H
#define _PC_SERIAL_H

#include "pc_uart.h"
#include "algorithmOfCRC.h"
#include "ins.h"

#define TX_DOWN_SAMPLING_TATE 4

void PCReceive(unsigned char *PCbuffer);
void SendtoPC(void);

#endif // !_PC_SERIAL_H

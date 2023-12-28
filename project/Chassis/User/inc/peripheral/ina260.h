#ifndef __INA260_H
#define __INA260_H

#include "stm32f4xx.h"
#include "i2c.h"
#include "my_filter.h"

#define INA260_1_ID 0x45 // 1000101
#define INA260_2_ID 0x44 // 1000100
#define REG_SETTING 0x00
#define REG_CURRENT 0x01
#define REG_VOLTAGE 0x02
#define REG_POWER 0x03
#define LSB_CURRENT 1.25f // mA
#define LSB_VOLTAGE 1.25f // mV
#define LSB_POWER 10.0f	  // mW

typedef struct INA260
{
	unsigned char Configuration[2]; // INA260
	float Current;
	float Voltage;
	float Power;
} INA260;

void INA_READ_Current(void);
void INA_READ_Vol(void);
void INA_READ_Power(void);
short INA260_Read(u8 address, u8 reg);
void INA_READ(void);

extern INA260 INA260_1; //输入
extern INA260 INA260_2; //输出

#endif

#ifndef __I2C_H
#define __I2C_H

#include <stm32f4xx.h>
#include <stm32f4xx_conf.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "pin_affine.h"
#include "tools.h"

#define IIC_SCL PBout(10) // SCL
#define IIC_SDA PBout(11) // SDA
#define READ_SDA PBin(11) //

#define IIC_RCC_AHBxPeriphClockCmd RCC_AHB1PeriphClockCmd
#define IIC_RCC_AHBxPeriph_GPIOx RCC_AHB1Periph_GPIOB
#define IIC_SCL_GPIO_Pin_x GPIO_Pin_10
#define IIC_SDA_GPIO_Pin_x GPIO_Pin_11
#define IIC_GPIOx GPIOB

enum SDA_ACTION
{
    SDA_IN,
    SDA_OUT
};

void i2c_init(void);
void IIC_Start(void);
void IIC_Stop(void);
unsigned char IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);
void IIC_Send_Byte(unsigned char txd);
unsigned char IIC_Read_Byte(void);

#endif

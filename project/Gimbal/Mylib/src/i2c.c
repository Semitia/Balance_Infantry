#include "i2c.h"
void i2c_init(void)
{
    GPIO_InitTypeDef GPIO_Init_Structure;
    IIC_RCC_AHBxPeriphClockCmd(IIC_RCC_AHBxPeriph_GPIOx, ENABLE);
    GPIO_Init_Structure.GPIO_Pin = IIC_SCL_GPIO_Pin_x | IIC_SDA_GPIO_Pin_x;
    GPIO_Init_Structure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init_Structure.GPIO_OType = GPIO_OType_PP;
    GPIO_Init_Structure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init_Structure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(IIC_GPIOx, &GPIO_Init_Structure);
    GPIO_SetBits(IIC_GPIOx, IIC_SCL_GPIO_Pin_x | IIC_SDA_GPIO_Pin_x);
}

void SDA_OUT()
{
    GPIO_InitTypeDef GPIO_Init_Structure;
    IIC_RCC_AHBxPeriphClockCmd(IIC_RCC_AHBxPeriph_GPIOx, ENABLE);
    GPIO_Init_Structure.GPIO_Pin = IIC_SDA_GPIO_Pin_x;
    GPIO_Init_Structure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init_Structure.GPIO_OType = GPIO_OType_PP;
    GPIO_Init_Structure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init_Structure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(IIC_GPIOx, &GPIO_Init_Structure);
}

void SDA_IN()
{
    GPIO_InitTypeDef GPIO_Init_Structure;
    IIC_RCC_AHBxPeriphClockCmd(IIC_RCC_AHBxPeriph_GPIOx, ENABLE);
    GPIO_Init_Structure.GPIO_Pin = IIC_SDA_GPIO_Pin_x;
    GPIO_Init_Structure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init_Structure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(IIC_GPIOx, &GPIO_Init_Structure);
}

//产生IIC起始信号
void IIC_Start(void)
{
    SDA_OUT(); // sda线输出
    IIC_SDA = 1;
    IIC_SCL = 1;
    delay_us(4);
    IIC_SDA = 0; // START:when CLK is high,DATA change form high to low
    delay_us(4);
    IIC_SCL = 0; //钳住I2C总线，准备发送或接收数据
}

//产生IIC停止信号
void IIC_Stop(void)
{
    SDA_OUT(); // sda线输出
    IIC_SCL = 0;
    IIC_SDA = 0; // STOP:when CLK is high DATA change form low to high
    delay_us(4);
    IIC_SCL = 1;
    IIC_SDA = 1; //发送I2C总线结束信号
    delay_us(4);
}

//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
unsigned char IIC_Wait_Ack(void)
{
    unsigned char ucErrTime = 0;
    IIC_SDA = 1;
    delay_us(1);
    SDA_IN(); // SDA设置为输入
    IIC_SCL = 1;
    delay_us(1);
    while (READ_SDA)
    {
        ucErrTime++;
        if (ucErrTime > 250)
        {
            // IIC_Stop();
            return 1;
        }
    }
    IIC_SCL = 0; //时钟输出0
    return 0;
}

void IIC_Ack(void)
{
    IIC_SCL = 0;
    SDA_OUT();
    IIC_SDA = 0;
    delay_us(2);
    IIC_SCL = 1;
    delay_us(2);
    IIC_SCL = 0;
}

void IIC_NAck(void)
{
    IIC_SCL = 0;
    SDA_OUT();
    IIC_SDA = 1;
    delay_us(2);
    IIC_SCL = 1;
    delay_us(2);
    IIC_SCL = 0;
}

void IIC_Send_Byte(u8 txd)
{
    unsigned char t;
    SDA_OUT();
    IIC_SCL = 0; //拉低时钟开始数据传输
    for (t = 0; t < 8; t++)
    {
        // IIC_SDA=(txd&0x80)>>7;
        if ((txd & 0x80) >> 7)
            IIC_SDA = 1;
        else
            IIC_SDA = 0;
        txd <<= 1;
        delay_us(2); //对TEA5767这三个延时都是必须的
        IIC_SCL = 1;
        delay_us(2);
        IIC_SCL = 0;
        delay_us(2);
    }
}

unsigned char IIC_Read_Byte(void)
{
    unsigned char i, receive = 0;
    SDA_IN(); // SDA设置为输入
    for (i = 0; i < 8; i++)
    {
        IIC_SCL = 0;
        delay_us(2);
        IIC_SCL = 1;
        receive <<= 1;
        if (READ_SDA)
            receive++;
        delay_us(2);
    }
    return receive;
}

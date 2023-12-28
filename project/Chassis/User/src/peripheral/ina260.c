#include "ina260.h"

#define WAITING_TIME 0.05 // us  发送数据后的接收时间

INA260 INA260_1; // 输入
INA260 INA260_2; // 输出

/**
 * @brief 读取电流
 * @param[in] void
 */
void INA_READ_Current()
{
	float Cur = 0.0f;
	Cur = INA260_Read(INA260_1_ID << 1, REG_CURRENT);
	INA260_1.Current = LSB_CURRENT * Cur;
	Cur = INA260_Read(INA260_2_ID << 1, REG_CURRENT);
	INA260_2.Current = LSB_CURRENT * Cur;
}

/**
 * @brief 读取电压
 * @param[in] void
 */
void INA_READ_Vol()
{
	float Vol = 0.0f;
	Vol = INA260_Read(INA260_1_ID << 1, REG_VOLTAGE);
	INA260_1.Voltage = LSB_VOLTAGE * Vol;
	Vol = INA260_Read(INA260_2_ID << 1, REG_VOLTAGE);
	INA260_2.Voltage = LSB_VOLTAGE * Vol;
}
/**
 * @brief 读取输入输出功率
 * @param[in] void
 */
void INA_READ_Power()
{
	unsigned short power = 0;
	power = INA260_Read(INA260_1_ID << 1, REG_POWER);
	INA260_1.Power = LSB_POWER * power;

	power = INA260_Read(INA260_2_ID << 1, REG_POWER);
	INA260_2.Power = LSB_POWER * power;
}

/**
 * @brief 通过iic中读取寄存器中的值
 * @param[in] address INA260地址
 * @param[in] reg 读取寄存器
 */
short INA260_Read(u8 address, u8 reg)
{
	u8 templ = 0, temph = 0;
	short temp = 0;
	IIC_Start();
	IIC_Send_Byte(address); // 发送低地址
	IIC_Wait_Ack();
	IIC_Send_Byte(reg); // 发送低地址
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(address + 1); // 进入接收模式
	IIC_Wait_Ack();
	delay_us_f(WAITING_TIME); // 增加此代码通信成功！！！
	temph = IIC_Read_Byte();  // 读寄存器 3
	IIC_Ack();
	templ = IIC_Read_Byte(); // 读寄存器 3
	IIC_NAck();
	IIC_Stop(); // 产生一个停止条件
	temp = (short)temph << 8 | templ;
	return temp;
}

void INA_READ()
{
	// INA_READ_Current();

	// 电压变化较慢，不需要读电压，可以从裁判系统读
	// INA_READ_Vol();

	INA_READ_Power();
}

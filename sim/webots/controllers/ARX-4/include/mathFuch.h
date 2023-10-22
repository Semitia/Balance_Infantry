/*
 * @Author: DTH
 * @Date: 2021-07-05 14:10:59
 * @LastEditTime: 2021-07-19 14:29:11
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \controllers\2_wheels_robot_controller\include\mathFuch.h
 */

#ifndef MATH_FUNC_H
#define MATH_FUNC_H

#include "math.h"
#include "arx.h"

#define pi 3.1415926

#ifdef __cplusplus
extern "C" {
#endif


double dgr(double rad);
double rad(double dgr);

double Lsin_rad(double angle_rad, double side_known, double side_unknown);
double Lsin_dgr(double angle_dgr, double side_known, double side_unknown);

double Lcos_rad(double angle_rad, double side_one, double side_ano);
double Lcos_dgr(double angle_dgr, double side_one, double side_ano);

double Lcos_side(double side_one, double side_ano, double side_op);

double knee_angle_calc(double active, double lower_leg_h, double driven, double D_body, double active_angle);

char* itoa(int num,char* str,int radix);//字符串转化函数

double dgr(double rad){
  return rad * 180 / 3.14159;
} 

double rad(double dgr){
  return dgr * 3.14159 / 180;
} 

double Lsin_rad(double angle_rad, double side_op, double side_side){
    double sin_angle_unknown = sin(angle_rad) * side_op / side_side;
    if(-1 <= sin_angle_unknown && sin_angle_unknown <= 1)
        return asin(sin_angle_unknown);
    else
        printf("参数有误\n");
        return 0;
}

double Lsin_dgr(double angle_dgr, double side_op, double side_side){
    double angle_rad = rad(angle_dgr);
    double sin_angle_unknown = sin(angle_rad) * side_op / side_side;
    if(-1 <= sin_angle_unknown && sin_angle_unknown <= 1)
        return dgr(asin(sin_angle_unknown));
    else
        printf("参数有误\n");
        return 0;   
}

double Lcos_rad(double angle_rad, double side_one, double side_ano){
    double side_third_2 = side_one * side_one + side_ano * side_ano - 2 * side_one * side_ano * cos(angle_rad);
    return sqrt(side_third_2);
}

double Lcos_dgr(double angle_dgr, double side_one, double side_ano){
    double angle_rad = rad(angle_dgr);
    double side_third_2 = side_one * side_one + side_ano * side_ano - 2 * side_one * side_ano * cos(angle_rad);
    return sqrt(side_third_2);
}

double Lcos_side(double side_one, double side_ano, double side_op){
    double cos_angle = (side_one * side_one + side_ano * side_ano - side_op * side_op) / (2 * side_one * side_ano);
    if(-1 <= cos_angle && cos_angle <= 1)
        return acos(cos_angle);
    else
        printf("参数有误\n");
        return 0; 
}

//弧度制
double knee_angle_calc(double active, double lower_leg_h, double driven, double D_body, double active_angle){
    double l1 = active;
    double l2 = lower_leg_h;
    double l3 = driven;
    double l4 = D_body;
    double a1 = active_angle;

    double L = Lcos_rad(a1, l1, l4);
    double a2 = Lcos_side(l2, l3, L);
    double a3 = Lsin_rad(a2, l3, L);
    double a4 = Lsin_rad(a1, l4, l1);
    double fi = pi - a3 - a4;

    return fi;
}

char* itoa(int num,char* str,int radix)
{
	char index[]="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";//索引表
	unsigned unum;//存放要转换的整数的绝对值,转换的整数可能是负数
	int i=0,j,k;//i用来指示设置字符串相应位，转换之后i其实就是字符串的长度；转换后顺序是逆序的，有正负的情况，k用来指示调整顺序的开始位置;j用来指示调整顺序时的交换。
 
	//获取要转换的整数的绝对值
	if(radix==10&&num<0)//要转换成十进制数并且是负数
	{
		unum=(unsigned)-num;//将num的绝对值赋给unum
		str[i++]='-';//在字符串最前面设置为'-'号，并且索引加1
	}
	else unum=(unsigned)num;//若是num为正，直接赋值给unum
 
	//转换部分，注意转换后是逆序的
	do
	{
		str[i++]=index[unum%(unsigned)radix];//取unum的最后一位，并设置为str对应位，指示索引加1
		unum/=radix;//unum去掉最后一位
 
	}while(unum);//直至unum为0退出循环
 
	str[i]='\0';//在字符串最后添加'\0'字符，c语言字符串以'\0'结束。
 
	//将顺序调整过来
	if(str[0]=='-') k=1;//如果是负数，符号不用调整，从符号后面开始调整
	else k=0;//不是负数，全部都要调整
 
	char temp;//临时变量，交换两个值时用到
	for(j=k;j<=(i-1)/2;j++)//头尾一一对称交换，i其实就是字符串的长度，索引最大值比长度少1
	{
		temp=str[j];//头部赋值给临时变量
		str[j]=str[i-1+k-j];//尾部赋值给头部
		str[i-1+k-j]=temp;//将临时变量的值(其实就是之前的头部值)赋给尾部
	}
 
	return str;//返回转换后的字符串
}


#ifdef __cplusplus
}
#endif

#endif /* MATH_FUNC_H */

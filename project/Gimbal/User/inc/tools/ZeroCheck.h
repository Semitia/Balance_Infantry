#ifndef __ZEROCHECK_H
#define __ZEROCHECK_H

typedef struct
{
	float Circle;	   //ת��Ȧ��
	float CountCycle;  //ת��һȦ���ܼ�������
	float LastValue;   //����������һ�ε�ֵ
	float ActualValue; //����������ǰֵ
	float PreError;	   //������жϲ�ֵ
} ZeroCheck_Typedef;

float ZeroCheck(ZeroCheck_Typedef *Zero, float value, float CountCycle);

#endif

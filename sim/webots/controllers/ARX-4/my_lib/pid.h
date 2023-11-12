#ifndef __PID_H
#define __PID_H

#define LIMIT_MAX_MIN(x, max, min) (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))
typedef struct
{
	float K1;
	float K2;
	float Last_DeltIn;
	float Now_DeltIn;
	float Out;
	float OutMax;
} FeedForward_Typedef;

/*   PID����-----ZN������
		 Td/T = Kd  T/Ti = Ki
	 PID:   Ti >= 4Td    Kp = 0.6Ku   Td = 0.125Tu
	 PI:    Kp = 0.45Ku  Ti = 0.83Tu
	 P:     Kp = 0.5Ku
*/

typedef struct PID
{
	float SetPoint; //�趨Ŀ��ֵ

	float ActualValue; //ʵ��ֵ

	float DeadZone;
	float P; //��������
	float I; //���ֳ���
	float D; //΢�ֳ���

	float LastError; //ǰ�����
	float PreError;	 //��ǰ���
	float SumError;	 //�������

	float IMax; //��������

	float POut;		 //�������
	float IOut;		 //�������
	float DOut;		 //΢�����
	float DOut_last; //��һ��΢�����
	float OutMax;	 //�޷�
	float Out;		 //�����
	float Out_last;	 //��һ�����

	float I_U; //���ٻ�������
	float I_L; //���ٻ�������

	float RC_DF; //����ȫ΢���˲�ϵ��

} Pid_Typedef;

float PID_Calc(Pid_Typedef *P);

float FeedForward_Calc(FeedForward_Typedef *FF);
#endif

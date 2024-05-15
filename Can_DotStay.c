
#include "Ano_Math.h"
#include "Can_DotStay.h"
#include "Ano_ProgramCtrl_User.h"
#include "Drv_OpenMV.h"

#define XK210_Kp 0.4f
#define XK210_Ki 0.f
#define XK210_Kd 0.1f

#define YK210_Kp 0.4f
#define YK210_Ki 0.f
#define YK210_Kd 0.1f

#define K210_LENGTHPERPIXEL 0.0026f
extern _pc_user_st pc_user;

struct XK210_Value
{
	float ErrorLastL;
	float ErrorLast;
	float Error;
	float	Integral;
	float IntegralLast;
	float OutputDelta;
	float Output;
};
struct XK210_Value XK210;

struct YK210_Value
{
	float ErrorLastL;
	float ErrorLast;
	float Error;
	float	Integral;
	float IntegralLast;
	float OutputDelta;
	float Output;
};
struct YK210_Value YK210;

void XK210PID_Init(void)
{
	XK210.Error = 0.f;
	XK210.ErrorLast = 0.f;
	XK210.ErrorLastL = 0.f;
	XK210.Integral = 0.f;
	XK210.IntegralLast = 0.f;
	XK210.OutputDelta = 0.f;
	XK210.Output = 0.f;
}

void YK210PID_Init(void)
{
	YK210.Error = 0.f;
	YK210.ErrorLast = 0.f;
	YK210.ErrorLastL = 0.f;
	YK210.Integral = 0.f;
	YK210.IntegralLast = 0.f;
	YK210.OutputDelta = 0.f;
	YK210.Output = 0.f;
}

//�����Ӿ����͵�������Ϣ����Ŀ�����������ת��Ϊʵ�ʷɿ�����
short int K210DataProcess(short int Dot_Position, unsigned int KeepHeight_cm)
{
	//ʹ�ù�ʽ��������x,y
	Dot_Position = K210_LENGTHPERPIXEL * KeepHeight_cm * Dot_Position;
	return Dot_Position;
}
 
char XK2100rderNum = 0;
short int LastDot_X = 0;
float XK210PID_Calculation(short int Dot_X, unsigned int KeepHeight_cm)
{
	Dot_X = K210DataProcess(Dot_X, KeepHeight_cm);//ʵ�ʸ߶�
	LastDot_X = Dot_X;
	
	switch(XK2100rderNum)
	{
		case 0:
		{
			//��ʼ��X����
			XK210PID_Init();
			XK2100rderNum = 1;
		}break;
		case 1:
		{
			//PID����
			XK210.OutputDelta = XK210_Kp * (XK210.Error - XK210.ErrorLast) + 
			XK210_Ki * (XK210.Integral - XK210.IntegralLast) +
			XK210_Kd * (XK210.Error - 2 * XK210.ErrorLast + XK210.ErrorLastL);
			
			XK210.ErrorLastL = XK210.ErrorLast;
			XK210.ErrorLast = XK210.Error;
			
			XK210.Output += XK210.OutputDelta;
			XK210.Error = 0 - Dot_X;//����Ŀ�����ɿأ�����������x�����λ�ã��ɿص�����Ϊ��0��0��
			
			XK210.IntegralLast = XK210.Integral;
			XK210.Integral += XK210.Error;
		}break;
	}
	return XK210.Output;
}



char YK2100rderNum = 0;
short int LastDot_Y = 0;
float YK210PID_Calculation(short int Dot_Y, unsigned int KeepHeight_cm)
{
	Dot_Y = K210DataProcess(Dot_Y, KeepHeight_cm);//ʵ�ʸ߶�
	LastDot_Y = Dot_Y;
	
	switch(YK2100rderNum)
	{
		case 0:
		{
			//��ʼ��Y����
			YK210PID_Init();
			YK2100rderNum = 1;
		}break;
		case 1:
		{
			//PID����
			YK210.OutputDelta = YK210_Kp * (YK210.Error - YK210.ErrorLast) + 
			YK210_Ki * (YK210.Integral - YK210.IntegralLast) +
			YK210_Kd * (YK210.Error - 2 * YK210.ErrorLast + YK210.ErrorLastL);
			
			YK210.ErrorLastL = YK210.ErrorLast;
			YK210.ErrorLast = YK210.Error;
			
			YK210.Output += YK210.OutputDelta;
			YK210.Error = 0 - Dot_Y;//����Ŀ�����ɿأ�����������y�����λ�ã��ɿص�����Ϊ��0��0��
			
			YK210.IntegralLast = YK210.Integral;
			YK210.Integral += YK210.Error;
		}break;
	}
	return YK210.Output;
}


//�Ӿ�ƫ�����
/*
��������XK210_Stay
����˵�����Ӿ�K210��ƫ�Ƕ�
������Dot_X,���Ӿ�����
����ֵ����
���ڣ�2022��7��17��  Cop:2023.12.17
*/
float XSpeed = 0;
void XK210_Stay(unsigned char Dot_Special_Point, short int Dot_X, unsigned int KeepHeight_cm)
{
	if(Can_K210.flag == Dot_Special_Point)//��־λһ��
	{
		XSpeed = XK210PID_Calculation(Dot_X, KeepHeight_cm);//����x���ǰ���ٶ�
		
		if(ABS(XSpeed) < 0.5f)//�ٶȽӽ�0��ֱ����Ϊ0
		{
			XSpeed = 0;
		}
		
		XSpeed = LIMIT(XSpeed, -10, 10);//����
		pc_user.vel_cmps_set_h[0] = XSpeed;//�����ٶȸ�ֵ
		
	}
	else
	{
		XK210PID_Init();//������־λ��һ�£����и�ֵ
		pc_user.vel_cmps_set_h[0] = 0;//x���ٶ���Ϊ0
	}
}

/*
��������XK210_Stay
����˵�����Ӿ�K210��ƫ����
������Dot_Y,���Ӿ�����
����ֵ����
���ڣ�2022��7��17�� cop:2023.12.17
*/

float YSpeed = 0;
void YK210_Stay(unsigned char Dot_Special_Point, short int Dot_Y, unsigned int KeepHeight_cm)
{
	if(Can_K210.flag == Dot_Special_Point)//��־λһ��
	{
		YSpeed = YK210PID_Calculation(Dot_Y, KeepHeight_cm);
		
		if(ABS(YSpeed) < 0.5f)//�ٶȽӽ�0��ֱ����Ϊ0
		{
			YSpeed = 0;
		}
		
		YSpeed = LIMIT(YSpeed, -10, 10);//����
		pc_user.vel_cmps_set_h[1] = YSpeed;
		
	}
	else
	{
		YK210PID_Init();
		pc_user.vel_cmps_set_h[1] = 0;
	}
}

void K210_Stay(unsigned char Dot_Speical_Point, short int Dot_X, short int Dot_Y, unsigned int KeepHeight_cm)
{
	XK210_Stay(Dot_Speical_Point, Dot_X, KeepHeight_cm);
	YK210_Stay(Dot_Speical_Point, Dot_Y, KeepHeight_cm);
}




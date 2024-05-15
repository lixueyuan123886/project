#include "Ano_Math.h"
#include "Ano_RC.h"
#include "Ano_ProgramCtrl_User.h"

#include "AutoTakeOff.h"
#include "Ano_FlightCtrl.h"


#define TAKEOFFSPEED_KP 0.3f  //���PID����ϵ��
#define TAKEOFFSPEED_KD 0.12f //΢��ϵ��

#define KEEPHEIGHTSPEED_KP 0.5f //����PID����ϵ��
#define KEEPHEIGHTSPEED_KD 0.15f  //

extern _pc_user_st pc_user;

unsigned char Is_AutoTakeOff = 0;
unsigned char KHFlag = 0;
unsigned char OffFlag = 0;
unsigned char OffProgramUnlock_sta = 0;





struct AutoTakeOffSpeedCalcuStruct
{
	float ErrorLastL;
	float Errorlast;
	float Error;
	float OutputDelta;
	//float AutoTakeOffSpeed;
};
struct AutoTakeOffSpeedCalcuStruct AutoTakeOffSpeedCalcuPID; 
float AutoTakeOffSpeed = 0;

struct KeepHeightSpeedCalcuStruct
{
	float ErrorLastL;
	float ErrorLast;
	float Error;
	float OutputDelta;
	float KeepHeight_TargetHeight_cm;
};

//�������

float KeepHeightSpeed = 0;
struct KeepHeightSpeedCalcuStruct	KeepHeightSpeedCalcuPID;

void AutoTakeOffSpeedCalcu(float TargetHeight_cm);
void KeepHeightSpeedCalcu(unsigned int Height);
void PID_Init(void);

//�޸ģ������ע�� 2023.11.29 ����Undefined symbol 
extern unsigned char LandProgramUnlock_sta;

extern unsigned char Is_AutoLand;
extern unsigned char Basic1Flag;
extern unsigned char Advanced2Flag;
unsigned int RC_Timer_ms = 0;

void Remote_Control(unsigned char dT_ms)
{
	//if(CH_N[6]){}ͨ��debug�鿴ң����������CH_N�Ķ�Ӧ��ϵ�������鿴��ʱʹ�õ�ң����SWC��Ӧ�ɿص�CH_N[6]
	if (CH_N[5] < -300)
	{
		OffProgramUnlock_sta = 0; // ??0������?��?��??��?e����3��D��?�¨�?��???��? 
		Is_AutoTakeOff = No; // ??No������?��?2?��?��?��??��?e����
		LandProgramUnlock_sta = 0;
		Is_AutoLand = No; 
	}
	else if (CH_N[5] < 200)
	{
		OffProgramUnlock_sta = 1; 
		Is_AutoTakeOff = Yes; 
	}
	else
	{	
		LandProgramUnlock_sta = 1;
		Is_AutoLand = Yes; 

	}	
}

/*
��������AutoTakeOff
����˵��:һ���������
����������ʱ��
*/
unsigned int Timer = 0;

unsigned char AutoTakeOff_ConfirmCount = 0;
void AutoTakeOff(unsigned char dT_ms)
{

	if (flag.unlock_sta == 0) // flag.unlock_sta?a??��??��?a??��?����?����??a??��??��?-?-��?
	{
		flag.taking_off = 0;
		flag.auto_take_off_land = AUTO_TAKE_OFF_NULL;
		PID_Init();
	}
	else
	{
		if (OffProgramUnlock_sta == 1) // ?�������̨�?D????����?����??��?e����?��
		{
			if (Timer <= 2000)
			{		
				Timer += dT_ms; 
			}
			
			if (Timer > 2000) // 2so��?e����
			{
				Timer += dT_ms;
								
				if (Is_AutoTakeOff == Yes && KHFlag == 0) // Is_AutoTakeOff?a??��??��?������???�¨����̨�?D��??3?�̡�?������?��a?����??��?e������??��?-?-
				{
					flag.taking_off = 1;
					flag.auto_take_off_land = AUTO_TAKE_OFF;
					AutoTakeOffSpeedCalcu(OFF_CM);
					Program_Ctrl_User_Set_Zcmps(AutoTakeOffSpeed);
					
				}
				
				
				if (ABS(OFF_CM - RELATIVE_HEIGHT_CM) <= 10)
				{
					if (AutoTakeOff_ConfirmCount <= 10)
					{
						AutoTakeOff_ConfirmCount ++;
					}
					else
					{
						AutoTakeOff_ConfirmCount = 0;
						KHFlag = 1;
					}					
					//flag.auto_take_off_land = AUTO_TAKE_OFF_FINISH;											
				}
			
				if (Timer > (2000 + 1400)) // ���1.4�����ҡ�˿��ƣ����˳�һ�����
				{
					Timer = 3500;
					if (ABS(fs.speed_set_h_norm[Z]) > 0.1f) // �������ͨ��ҡ�˿��Ʒɻ������˳�һ����ɳ���
					{
						flag.taking_off = 0;
						flag.auto_take_off_land = AUTO_TAKE_OFF_FINISH;
						return ;
					}
				}
 		}

	}
}
};
	



/*
�������ʱ���ٶ�
������Ŀ��߶ȣ��ٶ�
*/

void AutoTakeOffSpeedCalcu(float TargetHeight_cm)
{

	AutoTakeOffSpeedCalcuPID.ErrorLastL = AutoTakeOffSpeedCalcuPID.Errorlast;
	AutoTakeOffSpeedCalcuPID.Errorlast = AutoTakeOffSpeedCalcuPID.Error;
	AutoTakeOffSpeedCalcuPID.Error = TargetHeight_cm - RELATIVE_HEIGHT_CM;
	
	AutoTakeOffSpeedCalcuPID.OutputDelta = TAKEOFFSPEED_KP * (AutoTakeOffSpeedCalcuPID.Error - AutoTakeOffSpeedCalcuPID.Errorlast) 
	+ TAKEOFFSPEED_KD * (AutoTakeOffSpeedCalcuPID.Error - 2.f * AutoTakeOffSpeedCalcuPID.Errorlast 
	+ AutoTakeOffSpeedCalcuPID.ErrorLastL);
	
	AutoTakeOffSpeed += AutoTakeOffSpeedCalcuPID.OutputDelta;
	AutoTakeOffSpeed = LIMIT(AutoTakeOffSpeed, -AUTOTAKEOFFSPEEDMAX, AUTOTAKEOFFSPEEDMAX);
};


/*
���ܣ�����������
��������������߶�
*/
void KeepHeight(unsigned int Height)
{

	KeepHeightSpeedCalcu(Height);                     	//�����ٶ�                                        
	Program_Ctrl_User_Set_Zcmps(KeepHeightSpeed);  //ʹ�ó̿ظ�ֵ�ٶ�
	
}



/*
���㶨�߹����е��ٶ�
������Ŀ��߶�
*/
void KeepHeightSpeedCalcu(unsigned int Height)
{
	KeepHeightSpeedCalcuPID.KeepHeight_TargetHeight_cm = Height;
	KeepHeightSpeedCalcuPID.ErrorLastL = KeepHeightSpeedCalcuPID.ErrorLast;
	KeepHeightSpeedCalcuPID.ErrorLast = KeepHeightSpeedCalcuPID.Error;
	KeepHeightSpeedCalcuPID.Error = KeepHeightSpeedCalcuPID.KeepHeight_TargetHeight_cm - RELATIVE_HEIGHT_CM;
	
	KeepHeightSpeedCalcuPID.OutputDelta = KEEPHEIGHTSPEED_KP * (KeepHeightSpeedCalcuPID.Error - KeepHeightSpeedCalcuPID.ErrorLast)
	+ KEEPHEIGHTSPEED_KD * (KeepHeightSpeedCalcuPID.Error - KeepHeightSpeedCalcuPID.ErrorLast * 2.0 
	+ KeepHeightSpeedCalcuPID.ErrorLastL);
	
	KeepHeightSpeed += KeepHeightSpeedCalcuPID.OutputDelta;
	KeepHeightSpeed = LIMIT(KeepHeightSpeed, -KEEPHEIGHTSPEEDMAX, KEEPHEIGHTSPEEDMAX);
};


//��ɺͶ���PID�ṹ���ʼ��
void PID_Init(void)
{
	AutoTakeOffSpeedCalcuPID.Error = 0;
	AutoTakeOffSpeedCalcuPID.Errorlast = 0;
	AutoTakeOffSpeedCalcuPID.ErrorLastL = 0;
	AutoTakeOffSpeedCalcuPID.OutputDelta = 0;
	//AutoTakeOffSpeedCalcuPID.AutoTakeOffSpeed = 0;
	
	KeepHeightSpeedCalcuPID.Error = 0;
	KeepHeightSpeedCalcuPID.ErrorLast = 0;
	KeepHeightSpeedCalcuPID.ErrorLastL = 0;
	KeepHeightSpeedCalcuPID.OutputDelta = 0;
	KeepHeightSpeedCalcuPID.KeepHeight_TargetHeight_cm = 0;
	KeepHeightSpeed = 0;
};

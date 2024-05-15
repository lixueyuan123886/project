#include "FlightCtrlTask.h"
#include "Ano_FlightCtrl.h"
#include "Ano_RC.h"
#include "Ano_ProgramCtrl_User.h"
#include "Drv_OpenMV.h"
#include "Ano_Math.h"
#include "Ano_Imu.h"
#include "Ano_OF.h"
#include "Drv_icm20602.h"
#include "Ano_Sensor_Basic.h"

#include "Drv_OpenMV.h"
#include "Can_DotStay.h"
#include "AutoTakeOff.h"
#include "AutoLand.h"
#include "Can_DataTransmission.h"

#define CAR 1
#define TAW_CTRL_NUM 98
extern _ano_of_st ano_of;

extern unsigned char KHFlag;
extern unsigned char LandProgramUnlock_sta;
extern unsigned char Is_AutoLand;

unsigned int TaskConfirmCount = 0;//���״̬����
unsigned char SFlag = 1;
unsigned char OrdinalNum = 0;//����ִ�н׶Σ���ɡ����񡢽���
unsigned int TaskTimer_ms = 0;//�������ʱ�䣬�˴�Ϊ����
float Dheight = 0.f;//Ŀ��߶���ʵ�ʸ߶ȵĲ�ֵ

unsigned char DT_data[20] = {0};


void FlightCtrlTask(unsigned char dT_ms)
{
	DT_data[4] = (short)(TaskTimer_ms);
	DT_data[5] = (short)(TaskTimer_ms) >> 8;
	DT_data[6] = (short)(OrdinalNum);	
	DT_data[7] = (short)(OrdinalNum) >> 8;
	DT_data[8] = (short)(jsdata.valid_of_alt_cm);	
	DT_data[9] = (short)(jsdata.valid_of_alt_cm) >> 8;	
	DT_data[10] = (short)(pc_user.vel_cmps_set_z);
	DT_data[11] = (short)(pc_user.vel_cmps_set_z) >> 8;
	DT_data[12] = (short)(Dheight);
	DT_data[13] = (short)(Dheight) >> 8;
	Can_shortDataTransfer(DT_data, 0xF1);
	
	Dheight = OFF_CM - RELATIVE_HEIGHT_CM;//�߶Ȳ�ֵ
	if(flag.unlock_sta == 1)
	{
		
		Remote_Control(dT_ms);
		AutoLand(dT_ms);
		if(KHFlag == 1 && LandProgramUnlock_sta == 0)
		{
			KeepHeight(OFF_CM);
		};
		
		switch(OrdinalNum)
		{
			case 0:
			{
				if (SFlag == 1)
				{
					SFlag = 0;
					
					TaskConfirmCount = 0;
				}
				AutoTakeOff(dT_ms);
				if(KHFlag == 1)
				{

					if (TaskConfirmCount <= 10)
					{
						TaskConfirmCount ++;
					}
					else
					{
						TaskConfirmCount = 0;
						TaskTimer_ms = 0;					
						SFlag = 1;
						
						OrdinalNum += 1;						
					}
				};
			}break;
			
//���߲���
			case 1: //��ͣ5s����
			{
				if (SFlag == 1)
				{
					SFlag = 0;
					
					TaskTimer_ms = 0;
				}
				if(TaskTimer_ms <= 10000 )//��Ӵ��룬�����ɻ���ĳһ�߶ȷ�Χ��ʱ��ʱ����
				{
					if(Dheight <= 10.f && Dheight >= -10.f){
						TaskTimer_ms += dT_ms; 
					}
				}
				else
				{
					TaskTimer_ms = 0;
					OrdinalNum = 99;
					
				}
			}break;
			
//K210���㲿��
//			case 1:
//			{
//				if(SFlag == 1)
//				{
//					SFlag = 0;
//					TaskTimer_ms = 0;
//					
//				}
//				
//				K210_Stay(CAR, Can_K210.x, Can_K210.y, OFF_CM);
//				
//				//�������С���Ͽո�����ͣ��5�����
//				if (ABS(Can_K210.x) <= 10 && ABS(Can_K210.y) <= 10)
//				{
//					if (TaskTimer_ms <= 5000)
//					{
//						TaskTimer_ms += dT_ms;
//					}
//					else
//					{
//						TaskTimer_ms = 0;
//						SFlag = 0;
//						
//						OrdinalNum = 99;
//					}
//				}				
//				
//			}break;	 
			case 99:
			{
				LandProgramUnlock_sta = 1;
				Is_AutoLand = Yes;
			}break;
		};
		
	};
};


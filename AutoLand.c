#include "Ano_ProgramCtrl_User.h"
#include "Ano_FlightCtrl.h"

#include "FlightCtrlTask.h"
#include "AutoLand.h"
//#include "Yang_OFMeasure.h"

#define RELATIVE_HEIGHT_CM      (jsdata.valid_of_alt_cm)  //��Ը߶�

extern unsigned char OrdinalNum;

unsigned char LandProgramUnlock_sta = 0;
unsigned char Is_AutoLand = 0;
/*
��������һ������
����˵�����ɻ�һ������
������dT_ms�����ڼ�ʱ
����ֵ����
���ڣ�2022��6��17��
*/
unsigned int LandTimer = 0;
unsigned char Lookflag;
void AutoLand(unsigned char dT_ms)
{	
	static unsigned char AutoLand_Num;
	Lookflag = AutoLand_Num;
	if (flag.unlock_sta == 0) // flag.unlock_staΪ�棬���������ɻ����������򡭡���
	{

	}
	else
	{
		if (LandProgramUnlock_sta == 1 && Is_AutoLand == 1) // ��ͨ���п��Ʒɻ�һ������
		{			
			OrdinalNum = 200;
						
			if (RELATIVE_HEIGHT_CM > 17)
			{

				switch(AutoLand_Num)
				{
					case 0:
					{						
						pc_user.pal_dps_set = 0; 		// ȡ����ת
						pc_user.vel_cmps_set_h[0] = 0;  // ȡ��ƽ��
						pc_user.vel_cmps_set_h[1] = 0;
												
						AutoLand_Num = 1;
					}break;
					case 1:
					{									
						Program_Ctrl_User_Set_Zcmps(-25);						
					}break;
				}
			}
			else
			{
				LandTimer += dT_ms;

				if (LandTimer >= 1000)
				{
					LandTimer = 0;
					flag.unlock_cmd = 0;
				}
			}
					
		}		
	}
}


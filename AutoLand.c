#include "Ano_ProgramCtrl_User.h"
#include "Ano_FlightCtrl.h"

#include "FlightCtrlTask.h"
#include "AutoLand.h"
//#include "Yang_OFMeasure.h"

#define RELATIVE_HEIGHT_CM      (jsdata.valid_of_alt_cm)  //相对高度

extern unsigned char OrdinalNum;

unsigned char LandProgramUnlock_sta = 0;
unsigned char Is_AutoLand = 0;
/*
函数名：一键降落
功能说明：飞机一键降落
参数：dT_ms，用于计时
返回值：无
日期：2022年6月17日
*/
unsigned int LandTimer = 0;
unsigned char Lookflag;
void AutoLand(unsigned char dT_ms)
{	
	static unsigned char AutoLand_Num;
	Lookflag = AutoLand_Num;
	if (flag.unlock_sta == 0) // flag.unlock_sta为真，即解锁；飞机若解锁，则……；
	{

	}
	else
	{
		if (LandProgramUnlock_sta == 1 && Is_AutoLand == 1) // 在通道中控制飞机一键降落
		{			
			OrdinalNum = 200;
						
			if (RELATIVE_HEIGHT_CM > 17)
			{

				switch(AutoLand_Num)
				{
					case 0:
					{						
						pc_user.pal_dps_set = 0; 		// 取消旋转
						pc_user.vel_cmps_set_h[0] = 0;  // 取消平动
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


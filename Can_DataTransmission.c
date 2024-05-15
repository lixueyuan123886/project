

#include "Drv_usart.h"



unsigned char DataReceive[5] = {0};
unsigned char Usart_Data4Car[3] = {0};

void Can_DataAnalysis(unsigned char Byte)
{	
	static unsigned char i;
	unsigned char sum_data = 0;
	if (i == 0)
	{
		if (Byte == 0xAA)
		{
			DataReceive[i] = Byte;
			i ++;
		}
	}
	else if (i == 1)
	{
		if (Byte == 0xFF)
		{
			DataReceive[i] = Byte;
			i ++;
		}
	}
	else if (i >= 2 && i <= 3)
	{
		DataReceive[i] = Byte;
		i ++;
	}
	else if (i == 4)
	{
		for (i = 0; i < 4; i ++)
		{
			sum_data += DataReceive[i];
		}
		i = 4;
		
		if (sum_data == Byte)
		{
			DataReceive[i] = Byte;
			
			if (DataReceive[2] == 0xF1)		// ��ɱ�־λ
			{
				Usart_Data4Car[0] = DataReceive[3];
			}
			else if (DataReceive[2] == 0xF2)	// ��Դ��־λ
			{
				Usart_Data4Car[1] = DataReceive[3];
			}
			else if (DataReceive[2] == 0xF3)	// һ����ɽ���λ
			{
				Usart_Data4Car[2] = DataReceive[3];
			}
		}
		
		i = 0;
	}
}


//�����������ݴ��
// int16_t������ô�棬short i; Data[4] = i; Data[5] = i >> 8;
// ����λ����������λ��������Data[4]��Data[5]��˵����ӦID������λ1��ѡint16
// Data[4]~Data[13]������λ

void Can_shortDataTransfer(unsigned char *Data, unsigned char ID)
{
	unsigned char i = 0;
	unsigned char sum_check = 0;
	unsigned char add_check = 0;
	
	Data[0] = 0xAA;
	Data[1] = 0xFF;
	Data[2] = ID;			
	Data[3] = 10; 			// 1 char = 1 Byte, 1 short = 2 Byte 
	for (i = 0; i < 14; i ++)
	{
		sum_check += Data[i];
		add_check += sum_check;
	}	
	Data[14] = sum_check;
	Data[15] = add_check;
	
	//UT5����������Ϣ
	Uart5_Send(Data, 16);
}






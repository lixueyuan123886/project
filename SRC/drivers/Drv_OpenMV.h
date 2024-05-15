#ifndef __DRV_OPENMV_H
#define __DRV_OPENMV_H

//==����

#include "Ano_FcData.h"

//==����
typedef struct
{
	//
	u8 color_flag;
	u8 sta;
	s16 pos_x;
	s16 pos_y;
	u8 dT_ms;

}_openmv_color_block_st;

typedef struct
{
	//
	u8 sta;	
	s16 angle;
	s16 deviation;
	u8 p_flag;
	s16 pos_x;
	s16 pos_y;
	u8 dT_ms;

}_openmv_line_tracking_st;

typedef struct
{
	u8 offline;
	u8 mode_cmd;
	u8 mode_sta;
	//
	_openmv_color_block_st cb;
	_openmv_line_tracking_st lt;
}_openmv_data_st;
//==��������
extern _openmv_data_st opmv;


//==�Զ��� + ��������
typedef struct
{
	u8 flag;
	s16 angle;
	s16 distance;
	u8 cross_flag;
	s16 cross_x;
	s16 cross_y;
}_Can_opmv_data_st;
extern _Can_opmv_data_st Can_opmv;
typedef struct
{
	u8 flag;
	s16 angle;
	s16 distance;
	s16 x;
	s16 y;
}_Can_K210_data_st;
extern _Can_K210_data_st Can_K210;
//==��������

//static
static void OpenMV_Data_Analysis(u8 *buf_data,u8 len);
static void OpenMV_Check_Reset(void);

//public
void OpenMV_Offline_Check(u8 dT_ms);
void OpenMV_Byte_Get(u8 bytedata);


#endif


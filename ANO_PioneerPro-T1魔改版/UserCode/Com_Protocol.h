#ifndef _COM_PROTOCOL_H_
#define _COM_PROTOCOL_H_
#include "include.h"
extern int user_x ,user_y;
extern int user_control_num;
extern u8 A_RxBuffer[256];
extern u8 stm32_RxBuffer[20];
extern u8 stm32_RxBuffer_Count;
extern u8 stm32_full_tr;
extern u16 begin_pole_angle;//杆的起始角度
extern u16 end_pole_angle;//杆的终止角度
extern u16 mid_pole_angle;//杆的中间角度
extern u16 mid_pre_pole_angle;
extern u16 begin_pole_distance;//杆的起始距离
extern u16 begin_pre_pole_distance;//杆的终止距离
extern u8 openmv_angle;
extern int openmv_true_angle;
extern u8 openmv_distance;
extern int openmv_true_distance;
extern u8 if_find_yellow;
//void User_DT_Data_Receive_Anl(u8 *data_buf,u8 num);
void User_DT_Data_Receive_Prepare(u8 data);
void stm32_DT_Data_Receive(u8 data);
void stm32_DT_Data_Receive_Prepare(u8 *RxBuffer);
//void User_DT_Data_Receive_Anl_Task();
#endif

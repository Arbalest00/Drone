#include "Drv_Uart.h"
#include "stm32f4xx.h"
#ifndef _MY_PROTOCOL_H_
#define _MY_PROTOCOL_H_
///////////////////////////////////////变量
struct sdata
{
	s16 com_x;//x方向指令
	s16 com_y;//y方向指令
	s16 com_z;//z方向指令
	s16 com_yaw;//yaw指令
	u8 task_sta;//
	u8 next_task_sign;//阶段切换指令
};
struct PID_inc
{
	u16 target;
	u16 actual;
	u16 p;
	u16 i;
	u16 d;
	s32 err_current;
	s32 err_last;
	s32 err_previous;
};
extern u8 RxBuffer[256];
extern u8 LidarBuffer[256];
extern u8 pi_receive_done_sign;
extern u8 lidar_receive_done_sign;
extern u8 task_mode;
extern s16 CSPX,CSPY;
///////////////////////////////////////结构体
struct lidar_data
{
	u16 lidar_speed;
	u16 start_angle;
	u16 end_angle;
	u16 point_dis[12];
	u16 point_credit[12];
	u16 timestamp;
	u8 crc;
};
///////////////////////////////////////函数
void my_spcal(s16,s16);
void pi_receive( u8 );
void pi_send();
void lidar_receive(u8);
void lidar_cal(struct lidar_data *);
void Send_str_by_len(USART_TypeDef * USARTx,u8 *s,u16 len);
void PID_height_init();
s16 height_set(u32 height,u16 height_set);
#endif
	
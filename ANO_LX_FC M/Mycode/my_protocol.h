#include "Drv_Uart.h"
#ifndef _MY_PROTOCOL_H_
#define _MY_PROTOCOL_H_
///////////////////////////////////////变量
struct sdata
{
	u8 com_x;//x方向指令
	u8 com_y;//y方向指令
	u8 com_z;//z方向指令
	u8 com_yaw;//yaw指令
	u8 task_sta;//0为开环飞 1为树莓派速度控制
	u8 next_task_sign;//阶段切换指令
};
struct PID_inc
{
	u8 target;
	u8 actual;
	u8 p;
	u8 i;
	u8 d;
	u8 err_current;
	u8 err_last;
	u8 err_previous;
};
extern u8 RxBuffer[256];
extern u8 LidarBuffer[256];
extern u8 pi_receive_done_sign;
extern u8 lidar_receive_done_sign;
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
void pi_receive( u8 );
void lidar_receive(u8);
void lidar_cal(struct lidar_data *);
void Send_str_by_len(USART_TypeDef * USARTx,u8 *s,u16 len);
void PID_height_init();
u16 height_set(u32 height,u16 height_set);
#endif
	
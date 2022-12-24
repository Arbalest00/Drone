#include "Drv_Uart.h"
#ifndef _MY_PROTOCOL_H_
#define _MY_PROTOCOL_H_
///////////////////////////////////////变量
extern u8 com_x;
extern u8 com_y;
extern u8 com_z;
extern u8 com_yaw;
extern u8 task_sta;
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
#endif
	
#include "Drv_Uart.h"
#ifndef _MY_PROTOCOL_H_
#define _MY_PROTOCOL_H_
///////////////////////////////////////����
struct sdata
{
	u8 com_x;//x����ָ��
	u8 com_y;//y����ָ��
	u8 com_z;//z����ָ��
	u8 com_yaw;//yawָ��
	u8 task_sta;//0Ϊ������ 1Ϊ��ݮ���ٶȿ���
	u8 next_task_sign;//�׶��л�ָ��
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
///////////////////////////////////////�ṹ��
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
///////////////////////////////////////����
void pi_receive( u8 );
void lidar_receive(u8);
void lidar_cal(struct lidar_data *);
void Send_str_by_len(USART_TypeDef * USARTx,u8 *s,u16 len);
void PID_height_init();
u16 height_set(u32 height,u16 height_set);
#endif
	
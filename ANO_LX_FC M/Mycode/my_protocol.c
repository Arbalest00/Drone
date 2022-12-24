#include "my_protocol.h"
u8 com_x=0;
u8 com_y=0;
u8 com_z=0;
u8 com_yaw=0;
u8 task_sta=0;
u8 RxBuffer[256];//树莓派数据缓存
u8 LidarBuffer[256];//激光雷达数据缓存
u8 pi_receive_done_sign=0;//树莓派接收完成标志位
u8 lidar_receive_done_sign=0;//激光雷达接收完成标志位
u8 task_sta_t=0;//任务状态标志位
void Send_str_by_len(USART_TypeDef * USARTx,u8 *s,u16 len)//串口发送函数
{
	u16 i=0;
	while(i<len)
	{
		while(USART_GetFlagStatus(USARTx,USART_FLAG_TC )==RESET);
		USART_SendData(USARTx,*s);
		s++;
		i++;
	}
}
void pi_receive(u8 data)//树莓派接受协议 串口2
{
	static u8 state_1 = 0;
	if(state_1==0&&data==0xAA)	//帧头0xAA
	{
		state_1=1;
		RxBuffer[0]=data;
	}
	else if(state_1==1)	//任务模式
	{
		state_1=2;
		RxBuffer[1]=data;
		switch(RxBuffer[1])
		{
			case 0x00:
				task_sta_t=0;
			break;
			case 0x01:
				task_sta_t=1;
			break;
			case 0x02:
				task_sta_t=2;
			break;
			default:
				task_sta_t=0;
		}
	}
	else if(state_1==2)		//x移动指令
	{
		state_1=3;
		RxBuffer[2]=data;
	}
	else if(state_1==3)		//y移动指令
	{
		state_1=4;
		RxBuffer[3]=data;
	}
	else if(state_1==4)    //z移动指令
	{
		state_1=5;
		RxBuffer[4]=data;
	}
	else if(state_1==5)    //z移动指令
	{
		state_1=6;
		RxBuffer[5]=data;
	}
	else if(state_1==6&&data==0xFF)   //帧尾
	{
		state_1 = 0;
		RxBuffer[6]=data;
		task_sta=task_sta_t;
		com_x=RxBuffer[2];
		com_y=RxBuffer[3];
		com_z=RxBuffer[4];
		com_yaw=RxBuffer[5];
		pi_receive_done_sign=1;

	}
	else
		state_1 = 0;
}
void lidar_receive(u8 data)//激光雷达LDS=08接收 串口3
{
	static u8 lidar_state=0;
	static u8 i=0;
	if(lidar_state==0&&data==0x54)
	{
		LidarBuffer[i]=data;
		lidar_state=1;
		i++;
	}
	if(lidar_state==1&&i==46)
	{
		LidarBuffer[i]=data;
		lidar_state=0;
		i=0;
	}
	else if(lidar_state==1)
	{
		LidarBuffer[i]=data;
		i++;
	}
}
void lidar_cal(struct lidar_data * lidar)//lidar缓存数据计算
{
		lidar->lidar_speed=(int)LidarBuffer[3]*16+(int)LidarBuffer[2];
		lidar->start_angle=(int)LidarBuffer[5]*16+(int)LidarBuffer[4];
		lidar->end_angle=(int)LidarBuffer[43]*16+(int)LidarBuffer[42];
		lidar->timestamp=(int)LidarBuffer[45]*16+(int)LidarBuffer[44];
		lidar->crc=(int)LidarBuffer[46];
		for(int f=0;f<11;f+=1)
		{
			lidar->point_dis[f]=(int)LidarBuffer[7+f*3]*16+(int)LidarBuffer[6+f*3];
			lidar->point_credit[f]=(int)LidarBuffer[8+f*3];
		}
}

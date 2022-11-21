#include "my_protocol.h"
#include "Drv_usart.h"
u8 com_x=0;
u8 com_y=0;
u8 com_z=0;
u8 com_yaw=0;
u8 task_sta=0;
u8 RxBuffer[256];
u8 receive_done_sign=0;
u8 task_sta_t=0;
void pi_receive(u8 data)
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
		receive_done_sign=1;

	}
	else
		state_1 = 0;
}
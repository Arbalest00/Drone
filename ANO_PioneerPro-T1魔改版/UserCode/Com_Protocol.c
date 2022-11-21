# include "Com_Protocol.h"
int user_x=240 ,user_y=240;
int user_control_num=0;
u8 A_RxBuffer[256];
u8 stm32_RxBuffer[20];
u8 stm32_RxBuffer_Count = 1;
u8 stm32_full_tr = 0;
u16 begin_pole_angle=0;//�˵���ʼ�Ƕ�
u16 end_pole_angle=0;//�˵���ֹ�Ƕ�
u16 mid_pole_angle = 45;//�˵��м�Ƕ�
u16 mid_pre_pole_angle = 45;
u16 begin_pole_distance=0;//�˵���ʼ����
u16 begin_pre_pole_distance=0;//�˵���ֹ����
u8 openmv_angle=0;
int openmv_true_angle=0;
u8 openmv_distance=0;
int openmv_true_distance=0;
u8 if_find_yellow=0;
void stm32_DT_Data_Receive(u8 data)//���պ������ŵ�����5���жϽ���
{
	static u8 state_2 = 0;
	if(state_2==0&&data==0x3A)	//֡ͷ0x3A
	{
		state_2=1;
		stm32_RxBuffer[0]=data;
	}
	else if(state_2==1)//���ݰ���ȡ
	{
		if(stm32_RxBuffer_Count<7)
		{
			stm32_RxBuffer[stm32_RxBuffer_Count]=data;
			stm32_RxBuffer_Count++;
			if(stm32_RxBuffer_Count>=7)
			{
				stm32_RxBuffer_Count=1;
				state_2=2;
			}
		}
	}
	else if(state_2==2&&data==0xff)		//֡β0xff
	{
		stm32_RxBuffer[7]=data;
		stm32_full_tr = 1;
		state_2 = 0;
//		USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
	}
	else
	{
		state_2 = 0;
		stm32_full_tr = 0;
//		USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
	}
}
void stm32_DT_Data_Receive_Prepare(u8 *RxBuffer) //���ݰ������������ŵ�Ano_Scheduler.c��ѭ��ִ��
{
	if(stm32_full_tr == 1)
	{
		begin_pole_angle = RxBuffer[1];
		end_pole_angle = RxBuffer[2];
		mid_pole_angle = (begin_pole_angle + end_pole_angle)/2;
		begin_pole_distance = (RxBuffer[3]<<8|RxBuffer[4]);
		if(begin_pole_distance>1600)
		{
			begin_pole_distance = begin_pre_pole_distance;
			mid_pole_angle = mid_pre_pole_angle;
		}
		openmv_angle = RxBuffer[5];
		if(openmv_angle>90)
		{
			openmv_true_angle=openmv_angle-180;
		}
		else
		{
			openmv_true_angle=openmv_angle;
		}
		openmv_distance = RxBuffer[6];
		openmv_true_distance = openmv_distance-60;
		if_find_yellow = RxBuffer[7];
		mid_pre_pole_angle = mid_pole_angle;
		begin_pre_pole_distance = begin_pole_distance;
		stm32_full_tr = 0;
	}
}

void User_DT_Data_Receive_Prepare(u8 data)
{
	static u8 state_1 = 0;
	if(state_1==0&&data==0xAA)	//֡ͷ0xAA
	{
		state_1=1;
		A_RxBuffer[0]=data;
	}
	else if(state_1==1)	//����Դ��0x01��ʾ����������ݮ��
	{
		state_1=2;
		A_RxBuffer[1]=data;
		switch(data)
		{
			case 0x00:
				user_control_num=0;//ֹͣ
			break;
			case 0x01:
				user_control_num=1;//����ʶ��
			break;
			case 0x02:
				user_control_num=2;
			break;
			default:
				user_control_num=0;
		}
	}
	else if(state_1==2)		//x�������λ
	{
		state_1=3;
		A_RxBuffer[2]=data;
	}
	else if(state_1==3)		//x�������λ
	{
		state_1=4;
		A_RxBuffer[3]=data;
	}
	else if(state_1==4)		//y�������λ
	{
		state_1 = 5;
		A_RxBuffer[4]=data;
	}
	else if(state_1==5)   //y�������λ
	{
		A_RxBuffer[5]=data;
		state_1 = 6;
	}
	else if(state_1==6&&data==0xFF)   //֡β
	{
		state_1 = 0;
		A_RxBuffer[6]=data;
		user_x = A_RxBuffer[3]*256+A_RxBuffer[2];
		user_y = A_RxBuffer[5]*256+A_RxBuffer[4];
	
	}
	else
		state_1 = 0;
}

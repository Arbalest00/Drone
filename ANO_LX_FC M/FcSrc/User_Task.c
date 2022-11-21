#include "User_Task.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"
#include "ANO_DT_LX.h"
#define speed_x 10
#define speed_y 10
#define speed_z 10
#define speed_yaw 10
u8 eme_stop=0;
void tar_setdata(u16 x,u16 y, u16 z,u16 yaw)//ʵʱ����֡�����ٶ�
	{
		rt_tar.st_data.vel_x=x;
		rt_tar.st_data.vel_y=y;
		rt_tar.st_data.vel_z=z;
		rt_tar.st_data.yaw_dps=yaw;
		dt.fun[0x41].WTS = 1;
	}
void UserTask_OneKeyCmd(void)
{
    //////////////////////////////////////////////////////////////////////
    //һ�����/��������
    //////////////////////////////////////////////////////////////////////
    //�þ�̬������¼һ�����/����ָ���Ѿ�ִ�С�
    static u8 one_key_takeoff_f = 1, one_key_land_f = 1, one_key_mission_f = 0;
    static u8 mission_step;
    //�ж���ң���źŲ�ִ��
    if (rc_in.no_signal == 0)
    {
        //�жϵ�6ͨ������λ�� 1300<CH_6<1700
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1700 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 2200)
        {
            //��û��ִ��
            if (one_key_takeoff_f == 0)
            {
                //����Ѿ�ִ��
                one_key_takeoff_f =
                    //ִ��һ����� 
                    OneKey_Takeoff(100); //������λ�����ף� 0��Ĭ����λ�����õĸ߶ȡ�
            }
        }
        else
        {
            //��λ��ǣ��Ա��ٴ�ִ��
            one_key_takeoff_f = 0;
        }
        //
        //�жϵ�6ͨ������λ�� 800<CH_6<1200
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 800 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1200)
        {
            //��û��ִ��
            if (one_key_land_f == 0)
            {
                //����Ѿ�ִ��
                one_key_land_f =
                    //ִ��һ������
                    OneKey_Land();
            }
        }
        else
        {
            //��λ��ǣ��Ա��ٴ�ִ��
            one_key_land_f = 0;
        }
	}
    ////////////////////////////////////////////////////////////////////////
	if (rc_in.rc_ch.st_data.ch_[ch_8_aux4] > 1700 &&rc_in.rc_ch.st_data.ch_[ch_8_aux4] < 2200) 
	{
		if (eme_stop == 0) 
		{
			eme_stop = 1;
			//ִ��һ������
			FC_Lock();
			pwm_to_esc.pwm_m1 = 0;
			pwm_to_esc.pwm_m2 = 0;
			pwm_to_esc.pwm_m3 = 0;
			pwm_to_esc.pwm_m4 = 0;
		}
	} 
	else 
	{
		eme_stop = 0;
	}
	if(rc_in.rc_ch.st_data.ch_[ch_7_aux3]>1700 && rc_in.rc_ch.st_data.ch_[ch_7_aux3]<2200)
		{
			//��û��ִ��
			if(one_key_mission_f ==0)
			{
				//����Ѿ�ִ��
				one_key_mission_f = 1;
				//��ʼ����
				mission_step = 0;
			}
		}
		else
		{
			//��λ��ǣ��Ա��ٴ�ִ��
			one_key_mission_f = 0;		
		}
		//
		if(one_key_mission_f==1)
		{
			static u16 time_dly_cnt_ms;
			//
			switch(mission_step)
			{
				case 0:
				{
					//reset
					time_dly_cnt_ms = 0;
					mission_step +=1;
				}
				break;
				case 1:
				{
					//�л��̿�ģʽ
					mission_step +=1;
				}
				break;
				case 2:
				{
					//����
					mission_step +=1;
				}
				break;
				case 3:
				{
					//��1��
					if(time_dly_cnt_ms<1000)
					{
						time_dly_cnt_ms+=20;//ms
					}
					else
					{
						time_dly_cnt_ms = 0;
						mission_step += 1;
					}
				}
				break;
				case 4:
				{
					tar_setdata(speed_x,speed_y,0,0);
					mission_step += 1;//������λ�����ף� 0��Ĭ����λ�����õĸ߶ȡ�
				}
				break;
				case 5:
				{
					//��10��
					if(time_dly_cnt_ms<3000)
					{
						time_dly_cnt_ms+=20;//ms
					}
					else
					{
						tar_setdata(0,0,0,0);
						time_dly_cnt_ms = 0;
						mission_step += 1;
					}					
				}
				break;
				case 6:
				{
					//ǰ��1��
					tar_setdata(-speed_x,-speed_y,0,0);
				}
				break;	
				case 7:
				{
					//��10��
					if(time_dly_cnt_ms<1000)
					{
						time_dly_cnt_ms+=20;//ms
					}
					else
					{
						time_dly_cnt_ms = 0;
						mission_step += 1;
					}	
				}
				break;
				case 8:
				{
					//����1��
					mission_step += Horizontal_Move(50,10,90);
				}
				break;
				case 9:
				{
					//��10��
					if(time_dly_cnt_ms<1000)
					{
						time_dly_cnt_ms+=20;//ms
					}
					else
					{
						time_dly_cnt_ms = 0;
						mission_step += 1;
					}						
				}
				break;
				case 10:
				{
					//ִ��һ������
					OneKey_Land();					
				}
				break;	
				case 11:
				{
					
				}
				break;
				case 12:
				{
				
				}
				break;
				case 13:
				{
					
				}
				break;
				case 14:
				{
					
				}
				break;				
				default:break;
			}
		}
		else
		{
			mission_step = 0;
		}
}
#include "Ano_LocCtrl.h"
#include "Drv_Gps.h"
#include "Ano_Imu.h"
#include "Ano_FlightCtrl.h"
#include "Ano_OF.h"
#include "Ano_Parameter.h"
#include "pid.h"

//位置速度环控制参数
_PID_arg_st loc_arg_1[2] ; 

//位置速度环控制数据
_PID_val_st loc_val_1[2] ; 

//位置速度环修正控制参数
_PID_arg_st loc_arg_1_fix[2] ; 

//位置速度环修正控制数据
_PID_val_st loc_val_1_fix[2] ; 

static u8 mode_f[2];

/*角度环PID参数初始化*/
void Loc_1level_PID_Init()
{
	//GPS
	if(mode_f[1] == 2)
	{
		//normal
		loc_arg_1[X].kp = Ano_Parame.set.pid_gps_loc_1level[KP];//0.22f  ;
		loc_arg_1[X].ki = 0  ;
		loc_arg_1[X].kd_ex = 0.00f ;
		loc_arg_1[X].kd_fb = Ano_Parame.set.pid_gps_loc_1level[KD];
		loc_arg_1[X].k_ff = 0.02f;
		
		loc_arg_1[Y] = loc_arg_1[X];
		//fix	
		loc_arg_1_fix[X].kp = 0.0f  ;
		loc_arg_1_fix[X].ki = Ano_Parame.set.pid_loc_1level[KI] ;
		loc_arg_1_fix[X].kd_ex = 0.00f;
		loc_arg_1_fix[X].kd_fb = 0.00f;
		loc_arg_1_fix[X].k_ff = 0.0f;
		
		loc_arg_1_fix[Y] = loc_arg_1_fix[X];	
	}
	//OF
	else if(mode_f[1] == 1)
	{
		//normal
		loc_arg_1[X].kp = Ano_Parame.set.pid_loc_1level[KP];//0.22f  ;
		loc_arg_1[X].ki = 0.0f  ;
		loc_arg_1[X].kd_ex = 0.00f ;
		loc_arg_1[X].kd_fb = Ano_Parame.set.pid_loc_1level[KD];
		loc_arg_1[X].k_ff = 0.02f;
		
		loc_arg_1[Y] = loc_arg_1[X];
		//fix	
		loc_arg_1_fix[X].kp = 0.0f  ;
		loc_arg_1_fix[X].ki = Ano_Parame.set.pid_loc_1level[KI] ;
		loc_arg_1_fix[X].kd_ex = 0.00f;
		loc_arg_1_fix[X].kd_fb = 0.00f;
		loc_arg_1_fix[X].k_ff = 0.0f;
		
		loc_arg_1_fix[Y] = loc_arg_1_fix[X];	
	}
	else
	{
		//null
	}
	

}

_loc_ctrl_st loc_ctrl_1;
static float fb_speed_fix[2];

float vel_fb_d_lpf[2];
/*位置速度环*/
void Loc_1level_Ctrl(u16 dT_ms,s16 *CH_N)
{
	static float loc_hand_exp_vel[2]={0};
	static unsigned short waite_gps_loc_cnt = 0;
	float ne_pos_control[2];
	unsigned char vel_diff = 5;
	float loc_gps_h_out[2];
	float loc_gps_w_out[2];
	
	if(switchs.of_flow_on && (!switchs.gps_on))
	{
		mode_f[1] = 1;
		if(mode_f[1] != mode_f[0])
		{
			Loc_1level_PID_Init();
			mode_f[0] = mode_f[1];
		}
		////
		loc_ctrl_1.exp[X] = fs.speed_set_h[X];
		loc_ctrl_1.exp[Y] = fs.speed_set_h[Y];
		//
		LPF_1_(5.0f,dT_ms*1e-3f,imu_data.h_acc[X],vel_fb_d_lpf[X]);
		LPF_1_(5.0f,dT_ms*1e-3f,imu_data.h_acc[Y],vel_fb_d_lpf[Y]);		

		loc_ctrl_1.fb[X] = OF_DX2 + 0.02f *vel_fb_d_lpf[X];
		loc_ctrl_1.fb[Y] = OF_DY2 + 0.02f *vel_fb_d_lpf[Y];
		
		fb_speed_fix[0] = OF_DX2FIX;
		fb_speed_fix[1] = OF_DY2FIX;
		
		for(u8 i =0;i<2;i++)
		{
			PID_calculate( dT_ms*1e-3f,            //周期（单位：秒）
										loc_ctrl_1.exp[i] ,				//前馈值
										loc_ctrl_1.exp[i] ,				//期望值（设定值）
										loc_ctrl_1.fb[i] ,			//反馈值（）
										&loc_arg_1[i], //PID参数结构体
										&loc_val_1[i],	//PID数据结构体
										50,//积分误差限幅
										10 *flag.taking_off			//integration limit，积分限幅
										 )	;	
			
			//fix
			PID_calculate( dT_ms*1e-3f,            //周期（单位：秒）
										loc_ctrl_1.exp[i] ,				//前馈值
										loc_ctrl_1.exp[i] ,				//期望值（设定值）
										fb_speed_fix[i] ,			//反馈值（）
										&loc_arg_1_fix[i], //PID参数结构体
										&loc_val_1_fix[i],	//PID数据结构体
										50,//积分误差限幅
										10 *flag.taking_off			//integration limit，积分限幅
										 )	;	
			
			loc_ctrl_1.out[i] = loc_val_1[i].out + loc_val_1_fix[i].out;	//(PD)+(I)	
		}		


	}
	else if (switchs.gps_on)
	{
		mode_f[1] = 2;
		if(mode_f[1] != mode_f[0])
		{
			Loc_1level_PID_Init();
			mode_f[0] = mode_f[1];
		}
		////
		for(u8 j = 0; j < 2; j++)
		{
			if (fs.speed_set_h[j] != 0)				//判读是否动控制摇杆
			{
				if (ABS(loc_hand_exp_vel[j]) < ABS(fs.speed_set_h[j]))	//判断速度是否达到期望速度
				{
					if (loc_hand_exp_vel[j]*fs.speed_set_h[j] < 0)	//判断摇杆方向和期望方向是否相同
					{
						if (fs.speed_set_h[j] > 0)					//判断摇杆方向
						{
							fs.speed_set_h[j] = MAX_SPEED;			//限制最大速度
						}
						else 
						{
							fs.speed_set_h[j] = -MAX_SPEED;
						}
					}
					loc_hand_exp_vel[j] += 0.5f*dT_ms*fs.speed_set_h[j]/MAX_SPEED;		//计算期望速度
				}
				else												
				{
					if (loc_hand_exp_vel[j] > 0)					//回杆响应按最大加速度响应
					{
						loc_hand_exp_vel[j] -= vel_diff;
					}
					else
					{
						loc_hand_exp_vel[j] += vel_diff;
					}
				}
			}
			else
			{
				if (loc_hand_exp_vel[j] > vel_diff)					//回杆响应按最大加速度响应
				{
					loc_hand_exp_vel[j] -= vel_diff;
				}
				else if (loc_hand_exp_vel[j] <= -vel_diff)
				{
					loc_hand_exp_vel[j] += vel_diff;
				}
				else
				{
					loc_hand_exp_vel[j] = 0;
				}
			}
		}
		if (loc_hand_exp_vel[X] || loc_hand_exp_vel[Y])				//判断是否有手动期望速度
		{
			ne_pos_control[0] = 0;									//位置控制量清零
			ne_pos_control[1] = 0;
			waite_gps_loc_cnt = 50;									//期望速度归零之后等待500ms
		}
		else
		{
			if (waite_gps_loc_cnt > 0)
			{
				ne_pos_control[0] = 0;					//位置控制量清零
				ne_pos_control[1] = 0;
				waite_gps_loc_cnt--;
				if (waite_gps_loc_cnt == 0)				//估计位置已经稳定 记录期望位置以及对位置进行控制
				{
					Gps_information.hope_latitude = Gps_information.latitude_offset;		//期望位置等于当前位置
					Gps_information.hope_longitude = Gps_information.longitude_offset;
				}
			}
			else
			{
				Gps_information.hope_latitude_err = Gps_information.hope_latitude - Gps_information.latitude_offset;		//纬度误差
				Gps_information.hope_longitude_err = Gps_information.hope_longitude - Gps_information.longitude_offset;		//经度误差
				length_limit(&(Gps_information.hope_latitude_err), &(Gps_information.hope_longitude_err), MAX_SPEED*1.2f, ne_pos_control);	//控制模长限制
			}
		}
		
		
		loc_ctrl_1.exp[X] =  ne_pos_control[0]*Ano_Parame.set.pid_gps_loc_2level[KP] + loc_hand_exp_vel[X]*imu_data.hx_vec[0] - loc_hand_exp_vel[Y]*imu_data.hx_vec[1];		//期望速度（航向坐标转换到世界坐标NED）
		loc_ctrl_1.exp[Y] = -ne_pos_control[1]*Ano_Parame.set.pid_gps_loc_2level[KP] + loc_hand_exp_vel[X]*imu_data.hx_vec[1] + loc_hand_exp_vel[Y]*imu_data.hx_vec[0];		

		loc_ctrl_1.fb[X] =  (Gps_information.last_N_vel) + (wcx_acc_use*980/4096);			//速度反馈+加速度提前
		loc_ctrl_1.fb[Y] = -(Gps_information.last_E_vel) + (wcy_acc_use*980/4096);
		
		fb_speed_fix[X] =  (Gps_information.last_N_vel);
		fb_speed_fix[Y] = -(Gps_information.last_E_vel);
		
		for(u8 i =0;i<2;i++)
		{
			PID_calculate( dT_ms*1e-3f,            //周期（单位：秒）
										loc_ctrl_1.exp[i] ,				//前馈值
										loc_ctrl_1.exp[i] ,				//期望值（设定值）
										loc_ctrl_1.fb[i] ,			//反馈值（）
										&loc_arg_1[i], //PID参数结构体
										&loc_val_1[i],	//PID数据结构体
										50,//积分误差限幅
										10 *flag.taking_off			//integration limit，积分限幅
										 )	;		
			
			//fix
			PID_calculate( dT_ms*1e-3f,            //周期（单位：秒）
										loc_ctrl_1.exp[i] ,				//前馈值
										loc_ctrl_1.exp[i] ,				//期望值（设定值）
										fb_speed_fix[i] ,			//反馈值（）
										&loc_arg_1_fix[i], //PID参数结构体
										&loc_val_1_fix[i],	//PID数据结构体
										50,//积分误差限幅
										10 *flag.taking_off			//integration limit，积分限幅
										 )	;	
			
			if (!flag.taking_off)
			{
				loc_val_1_fix[i].err_i = 0;
			}				
		}
		//
		loc_gps_w_out[0] = loc_val_1[0].out + loc_val_1_fix[0].out;//(PD)+(I)
		loc_gps_w_out[1] = loc_val_1[1].out + loc_val_1_fix[1].out;//(PD)+(I)
		w2h_2d_trans(loc_gps_w_out, imu_data.hx_vec, loc_gps_h_out);	//世界坐标（NED）控制结果转换到航向坐标下
		loc_ctrl_1.out[X] = loc_gps_h_out[0];
		loc_ctrl_1.out[Y] = loc_gps_h_out[1];
	}
	else
	{
		mode_f[1] = 3;
		if(mode_f[1] != mode_f[0])
		{
			Loc_1level_PID_Init();
			mode_f[0] = mode_f[1];
		}
		////
		loc_ctrl_1.out[X] = (float)MAX_ANGLE/MAX_SPEED *fs.speed_set_h[X] ;
		loc_ctrl_1.out[Y] = (float)MAX_ANGLE/MAX_SPEED *fs.speed_set_h[Y] ;
	}
}

_loc_ctrl_st loc_ctrl_2;
pid_struct_t loc_ctrl_2_pid[2],loc_ctrl_line_fix[2],loc_ctrl_height,loc_ctrl_pic_fix[2],loc_ctrl_yaw_fix;
float loc_ctrl_2_x_speed,loc_ctrl_2_y_speed;
void loc_ctrl_2_pid_init()
{
	for(int i=0;i<2;i++)
	{
	pid_init(&loc_ctrl_2_pid[i],
	         0.05,//kp
	         0,  //ki
	         0.6,//kd
	         0,  //i_max
	         2  //out_max
	        );//定圆的pid
	
	pid_init(&loc_ctrl_line_fix[0],
					 1.5,
					 0,
	         2.5,
	         0,
	         15
	        );//飞行过程中修正其他偏移的pid
	pid_init(&loc_ctrl_line_fix[1],
					 1.1,
					 0,
	         2,
	         0,
	         15
	        );//飞行过程中修正其他偏移的pid
	pid_init(&loc_ctrl_pic_fix[i],
		       0.1,
		       0,
		       0.6,
		       0,
		       3
					 );//识别图标后定图标的pid
	}
	pid_init(&loc_ctrl_height,
	         0.7,
	         0,
	         0.7,
	         0,
	         10
	        );//定高pid
	pid_init(&loc_ctrl_yaw_fix,
	         1,
	         0,
	         1,
	         0,
	         10);//定yaw轴pid 
}
float loc_ctrl_2_pid_calc(float loc_ctrl_2_exp,int16_t loc_ctrl_2_fb,int i)//定圆pid计算
{
	return pid_calc(&loc_ctrl_2_pid[i],
					        loc_ctrl_2_exp,
					        loc_ctrl_2_fb
	               );
}
float loc_ctrl_fix_pid_calc(float loc_ctrl_2_exp,int16_t loc_ctrl_2_fb,int i)//修正pid
{
	return pid_calc(&loc_ctrl_pic_fix[i],
					        loc_ctrl_2_exp,
					        loc_ctrl_2_fb
	               );
}
float loc_ctrl_pic_pid_calc(float loc_ctrl_2_exp,int16_t loc_ctrl_2_fb,int i)//定图标pid
{
		return pid_calc(&loc_ctrl_line_fix[i],
					         loc_ctrl_2_exp,
					         loc_ctrl_2_fb
		             );
}
float loc_ctrl_yaw_pid_calc(float loc_ctrl_2_exp,int16_t loc_ctrl_2_fb)//定yaw轴pid
{
		return pid_calc(&loc_ctrl_yaw_fix,
					         loc_ctrl_2_exp,
					         loc_ctrl_2_fb
		             );
}
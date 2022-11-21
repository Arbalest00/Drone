#include "Ano_ProgramCtrl_User.h"
#include "Ano_MotionCal.h"
#include <stdio.h>
#include "Ano_OF.h"
#include "Ano_DT.h"
#include "Ano_FlightCtrl.h"
#include "Ano_FlyCtrl.h"
#include "Ano_Math.h"
#include "Drv_Led.h"
#include "Drv_usart.h"
#include "Ano_RC.h"
#include "Ano_LocCtrl.h"
#include "pid.h"
#include "Com_Protocol.h"
#include "Ano_Imu.h"
#include "my_protocol.h"
float myspeed_x=10;
float myspeed_y=10;
float myspeed_z=10;
float myheight=100;
float myspeed_yaw=15;
u8 pi_sign=0;
//此处开始为原程序定义
static int16_t point_x[12] = { 50,200,275,350,350,275,125,125, 50,200,200,350};
static int16_t point_y[12] = {275,125,200,0  ,275, 50, 50,200,125,  0,275,125};
// #define USER_TASK_NUM 1  //用户任务数
int16_t line_err;
void UserCtrlReset(void);
void Program_Ctrl_User_Set_HX(float hx);
void Program_Ctrl_User_Set_HY(float hy);
void Program_Ctrl_User_Set_YAW(float yaw_pal);
void move_on_back(float x);
void move_left_right(float y);
void turn_left_right(float angle);
int heartbeat = 0;
u8 heart_err = 0;
u8 height_try_hold = 0;
_pc_user_st pc_user;
static u8 loc_ctrl_2_hold = 0;
_user_cntrl_word user_cntrl_word;  //用户控制字
u8 getchar_test1[5] = {0,0};
static u8 cnt=0;
float move_speed_y = 16;
float move_speed_x = 16;
float speed_x = 0;
float speed_y = 0;
float speed_yaw = 0;
float speed_fix_x = 0;
float speed_fix_y = 0;

/*-----------------------------------------------------------------------------*/
//用户任务
 u8 pi_stage=1;
//魔改开始
void move_by_pi(u32 dt_us)//
{
	Program_Ctrl_User_Set_ZHeight(myheight);
	pi_sign=1;
	switch(com_x)
	{
		case 0x00:Program_Ctrl_User_Set_HX(0);
			break;
		case 0x01:Program_Ctrl_User_Set_HX(myspeed_x);
			break;
		case 0x02:Program_Ctrl_User_Set_HX(-myspeed_x);
			break;
		default:Program_Ctrl_User_Set_HX(0);
			break;
	}
	switch(com_y)
	{

		case 0x00:Program_Ctrl_User_Set_HY(0);
			break;
		case 0x01:Program_Ctrl_User_Set_HY(myspeed_y);
			break;
		case 0x02:Program_Ctrl_User_Set_HY(-myspeed_y);
			break;
		default:Program_Ctrl_User_Set_HY(0);
			break;
	}
	switch(com_z)
	{
		case 0x00:Program_Ctrl_User_Set_Zcmps(0);
			break;
		case 0x01:Program_Ctrl_User_Set_Zcmps(0);
			break;
		case 0x02:Program_Ctrl_User_Set_Zcmps(0);
			break;
		default:Program_Ctrl_User_Set_Zcmps(0);
			break;
	}	
	switch(com_yaw)
	{
		case 0x00:Program_Ctrl_User_Set_YAWdps(0);
			break;
		case 0x01:Program_Ctrl_User_Set_YAWdps(-myspeed_yaw);
			break;
		case 0x02:Program_Ctrl_User_Set_YAWdps(myspeed_yaw);
			break;
		default:Program_Ctrl_User_Set_YAWdps(0);
			break;
	}	
	
}
void move_by_pi_2(u32 dt_us)//
{
	Program_Ctrl_User_Set_ZHeight(myheight);
	pi_sign=1;
	switch(com_x)
	{
		case 0x00:Program_Ctrl_User_Set_HX(0);
			break;
		case 0x01:Program_Ctrl_User_Set_HX(myspeed_x);
			break;
		case 0x02:Program_Ctrl_User_Set_HX(-myspeed_x);
			break;
		default:Program_Ctrl_User_Set_HX(0);
			break;
	}
	switch(com_y)
	{

		case 0x00:Program_Ctrl_User_Set_HY(0);
			break;
		case 0x01:Program_Ctrl_User_Set_HY(-myspeed_y);
			break;
		case 0x02:Program_Ctrl_User_Set_HY(myspeed_y);
			break;
		default:Program_Ctrl_User_Set_HY(0);
			break;
	}
	switch(com_z)
	{
		case 0x00:Program_Ctrl_User_Set_Zcmps(0);
			break;
		case 0x01:Program_Ctrl_User_Set_Zcmps(0);
			break;
		case 0x02:Program_Ctrl_User_Set_Zcmps(0);
			break;
		default:Program_Ctrl_User_Set_Zcmps(0);
			break;
	}	
	switch(com_yaw)
	{
		case 0x00:Program_Ctrl_User_Set_YAWdps(0);
			break;
		case 0x01:Program_Ctrl_User_Set_YAWdps(-myspeed_yaw);
			break;
		case 0x02:Program_Ctrl_User_Set_YAWdps(myspeed_yaw);
			break;
		default:Program_Ctrl_User_Set_YAWdps(0);
			break;
	}	
	
}
void mode_change(u32 dT_us)
{
	pi_stage=2;
	Program_Ctrl_User_Set_HXYcmps(0,0);
	Program_Ctrl_User_Set_ZHeight(myheight);
	Send_str_by_len(UART5,"111",3);
}
void loc_re(u32 dT_us)
{
	Program_Ctrl_User_Set_HXYcmps(myspeed_x,-myspeed_y);
	Program_Ctrl_User_Set_Zcmps(myspeed_z);
}
void move_on(u32 dT_us)
{
  Program_Ctrl_User_Set_HXYcmps(myspeed_x,0);
}
void move_on_t(u32 dT_us)
{
	Program_Ctrl_User_Set_ZHeight(myheight);
  Program_Ctrl_User_Set_HX(myspeed_x);
}
void move_back(u32 dT_us)
{
  Program_Ctrl_User_Set_HXYcmps(-myspeed_x,0);
}
void move_back_t(u32 dT_us)
{
	Program_Ctrl_User_Set_ZHeight(myheight);
  Program_Ctrl_User_Set_HX(-myspeed_x);
}
void move_left(u32 dT_us)
{
  Program_Ctrl_User_Set_HXYcmps(0,myspeed_y);
}
void move_left_t(u32 dT_us)
{
	Program_Ctrl_User_Set_ZHeight(myheight);
	Program_Ctrl_User_Set_HY(myspeed_y);
}
void move_right(u32 dT_us)
{
  Program_Ctrl_User_Set_HXYcmps(0,-myspeed_y);
}
void move_right_t(u32 dT_us)
{
	Program_Ctrl_User_Set_ZHeight(myheight);
  Program_Ctrl_User_Set_HY(-myspeed_y);
}
void turn_left(u32 dT_us)
{
  Program_Ctrl_User_Set_YAWdps(-15/2.0f);
}
void turn_right(u32 dT_us)
{
  Program_Ctrl_User_Set_YAWdps(45/2.0f);
}
void fly_high(u32 dT_us)
{
	Program_Ctrl_User_Set_Zcmps(myspeed_z);
}
void fly_low(u32 dT_us)
{
	Program_Ctrl_User_Set_Zcmps(-myspeed_z);
}
void myset_height(u32 dt_ms) 
{
	Program_Ctrl_User_Set_ZHeight(myheight);
}
void flyinit(u32 dt_ms) 
{
	Program_Ctrl_User_Set_Zcmps(0.0f);
	Program_Ctrl_User_Set_HXYcmps(0,0);
}

void mywait(u32 dt_ms)
{
	Program_Ctrl_User_Set_HXYcmps(0,0);
	Program_Ctrl_User_Set_ZHeight(myheight);
}
void turn_left_right(float angle){Program_Ctrl_User_Set_YAW(angle);}
/*------------------------------------------------------------------------------*/
/*static user_task_t user_task[] =  //程控，屏蔽测试
    {
		{myset_height,4000,0,0},
		{mywait,2000,0,0},
		{move_on_t,4000,0,0},
		{move_back_t,4000,0,0},
		{move_left_t,4000,0,0},
		{move_right_t,4000,0,0},
		{mywait,2000,0,0},
};*/
static user_task_t user_task[] = 
    {
		{mywait,1000,0,0},
		{move_by_pi,100,0,0},
		{mode_change,2000,0,0},
		{move_left_t,3000,0,0},
		{mywait,500,0,0},
		{move_by_pi_2,100,0,0},
		{mode_change,1000,0,0},
};//巡线任务列表
const u8 USER_TASK_NUM = sizeof(user_task) / sizeof(user_task_t);
//顺序执行用户任务
static u8 task_index = 0;     //用于记录正在执行的任务
static u32 running_time = 0;  //任务已运行时间
u8 height_sign=0;
void User_Ctrl(u32 dT_ms) {
//  sendchar_test();
//	user_task[2].run_time = 10000;
	if (user_cntrl_word.takeoff_en == 1 && switchs.of_flow_on) {
			FlyCtrlReset();
			one_key_take_off();
			user_cntrl_word.takeoff_en=0;
			height_sign=1;
  } else if (user_cntrl_word.land_en) {
    //			printf("land\r\n");
    UserCtrlReset();
    one_key_land();
    user_cntrl_word.land_en = 0;
//    user_cntrl_word.user_task_running = 0;  //终止任务
	}
	if(height_sign==1)
	{
		if(flag.taking_off == 1)Program_Ctrl_User_Set_ZHeight(myheight);
	}
	if(user_cntrl_word.task_en==1)
	{
		FlyCtrlReset();
        UserCtrlReset();
		user_cntrl_word.user_task_running = 1;  //用来防止纯手动模式下进入usertask
		user_cntrl_word.break_out = 0;  //为任务的跳出做准备
		task_index = 0;
		height_sign=0;
		running_time = 0;  //任务已运行时间
		for (int i = 0; i < USER_TASK_NUM; i++) {
			user_task[i].end_flag = 0;  //清空所有任务结束标志位
		}
	user_cntrl_word.task_en=0;
	}
//		else if (flag.auto_take_off_land == AUTO_TAKE_OFF_FINISH &&
//             user_cntrl_word.user_task_running) {
     else if (user_cntrl_word.user_task_running) {
    if (task_index < USER_TASK_NUM) {
      if (user_task[task_index].start_flag == 1)  //执行第task_index个任务
      {
        user_task[task_index].task_func(dT_ms);
       	if(pi_sign==0)running_time += dT_ms;		  //记录累计时间
		  else if(task_sta==2&&pi_sign==1)
		  {
			   user_task[task_index].start_flag = 0;
			  user_task[task_index].end_flag = 1;
			  pi_sign=0;
		  }
      }

      if (user_task[task_index].end_flag == 1)  //准备开始下一个任务
      {
//        if(task_index == 1){height_try_hold = 1;}
        task_index++;
        running_time = 0;
        FlyCtrlReset();
        UserCtrlReset();
				
      } else if (user_cntrl_word.break_out == 1)  //任务的强制结束
      {
        task_index++;
        running_time = 0;
        user_cntrl_word.break_out = 0;
        user_cntrl_word.stop = 0;
        FlyCtrlReset();
        UserCtrlReset();
				
      }

      if (running_time == 0)  //启动第task_index个任务
      {
        user_task[task_index].start_flag = 1;
      } else if (running_time >=
                 user_task[task_index].run_time)  //判断当前任务是否完成
      {
        user_task[task_index].start_flag = 0;
        user_task[task_index].end_flag = 1;
      }
    } else {
      ANO_DT_SendString(" > All task finished!");
			height_try_hold = 0;
      user_cntrl_word.land_en = 1;
      user_cntrl_word.user_task_running = 0;
      UserCtrlReset();
      FlyCtrlReset();
      task_index = 0;
      running_time = 0;
    }
  }
}

//=====1、航向水平坐标系程控位移功能接口函数=====
/**********************************************************************************************************
 *函 数 名: Program_Ctrl_User_Set_HXY
 *功能说明: 程控功能，航向水平坐标系下位移设定（实时控制）
 *形    参:
 *X速度（厘米每秒，正为前进，负为后退），Y速度（厘米每秒，正为左移，负为右移） 
 *返 回 值: 无
 **********************************************************************************************************/
void Program_Ctrl_User_Set_HX(float hx)
{
	pc_user.vel_cmps_set_h[0] = hx;
	length_limit(&pc_user.vel_cmps_set_h[0], &pc_user.vel_cmps_set_h[1],
               MAX_PC_XYVEL_CMPS, pc_user.vel_cmps_set_h);
}
void Program_Ctrl_User_Set_HY(float hy)
{
//	speed_y = loc_ctrl_2_pid_calc(hy,INTEG_Y,Y);
	pc_user.vel_cmps_set_h[1] = hy;
  //限制XY速度模长
  length_limit(&pc_user.vel_cmps_set_h[0], &pc_user.vel_cmps_set_h[1],
               MAX_PC_XYVEL_CMPS, pc_user.vel_cmps_set_h);
}
//=====1、航向水平坐标系程控速度功能接口函数=====
/**********************************************************************************************************
 *函 数 名: Program_Ctrl_User_Set_HXYcmps
 *功能说明: 程控功能，航向水平坐标系下速度设定（实时控制）
 *形    参:
 *X速度（厘米每秒，正为前进，负为后退，Y速度（厘米每秒，正为左移，负为右移） 返
 *回 值: 无
 **********************************************************************************************************/
void Program_Ctrl_User_Set_HXYcmps(float hx_vel_cmps, float hy_vel_cmps) {
  //
  pc_user.vel_cmps_set_h[0] = hx_vel_cmps;
  pc_user.vel_cmps_set_h[1] = hy_vel_cmps;
  //限制XY速度模长
  length_limit(&pc_user.vel_cmps_set_h[0], &pc_user.vel_cmps_set_h[1],
               MAX_PC_XYVEL_CMPS, pc_user.vel_cmps_set_h);
}

//=====2、无头模式参考坐标系程控功速度能接口函数（暂无，稍后开发，或者参考上位机程控功能）=====
//
//
//

//=====3、通用程控速度功能接口函数=====
/**********************************************************************************************************
 *函 数 名: Program_Ctrl_User_Set_WHZcmps
 *功能说明: 程控功能，上升下降速度设定（实时控制）
 *形    参: 速度（厘米每秒，正为上升，负为下降）
 *返 回 值: 无
 **********************************************************************************************************/
void Program_Ctrl_User_Set_Zcmps(float z_vel_cmps) {
  //
  pc_user.vel_cmps_set_z = z_vel_cmps;
  //限幅
  pc_user.vel_cmps_set_z =
      LIMIT(pc_user.vel_cmps_set_z, -MAX_PC_ZVEL_CMPS, MAX_PC_ZVEL_CMPS);
}
/**********************************************************************************************************
 *函 数 名: Program_Ctrl_User_Set_YAWdps
 *功能说明: 程控功能，航向速度设定（实时控制）
 *形    参: 速度（度每秒，正为右转，负为左转）
 *返 回 值: 无
 **********************************************************************************************************/
void Program_Ctrl_User_Set_YAW(float yaw_pal)
{
	pc_user.yaw_set = yaw_pal;
	pc_user.engage_yaw_set = 1;
}
void Program_Ctrl_User_Set_YAWdps(float yaw_pal_dps) {
  //
  pc_user.pal_dps_set = yaw_pal_dps;
  //限幅
  pc_user.pal_dps_set =
      LIMIT(pc_user.pal_dps_set, -MAX_PC_PAL_DPS, MAX_PC_PAL_DPS);
}

/**********************************************************************************************************
 *函 数 名: Program_Ctrl_User_Set_ZHeight
 *功能说明: 程控功能，定高高度设定
 *形    参: 高度（厘米）
 *返 回 值: 无
 **********************************************************************************************************/
void Program_Ctrl_User_Set_ZHeight(float z_height) {
  //
//  pc_user.height_set = z_height;
//  //限幅
//  pc_user.height_set = LIMIT(pc_user.height_set, 40, MAX_PC_HEIGHT);
//  pc_user.engage_height_set = 1;
	pc_user.height_speed_set = pid_calc(&loc_ctrl_height,z_height,OF_ALT);
	Program_Ctrl_User_Set_Zcmps(pc_user.height_speed_set);
}

void UserCtrlReset() {
  Program_Ctrl_User_Set_HXYcmps(0, 0);
  Program_Ctrl_User_Set_YAWdps(0);
  Program_Ctrl_User_Set_Zcmps(0);
	INTEG_X = 0;
	INTEG_Y = 0;
	speed_x = 0;
	speed_y = 0;
	speed_fix_x = 0;
	speed_fix_y = 0;
	speed_yaw = 0;
}

static uint8_t _user_data_temp[50]={0,0,0,0};
static u8 _user_data_cnt = 4;
static u8 user_ctrl_data_ok = 0;
void user_task_1ms()
{
	if(loc_ctrl_2_hold == 1)
	{
		INTEG_X += OF_DX2FIX*0.001;
		INTEG_Y += OF_DY2FIX*0.001;
	}
	else
	{
		INTEG_X = 0;
		INTEG_Y = 0;
	}
}
void getchar_test(uint8_t data)
{
	getchar_test1[cnt++] = data;
	if(cnt == 5)
	{cnt = 0;}
}
#ifndef __ANO_PROGRAMCTRL_USER_H
#define __ANO_PROGRAMCTRL_USER_H

//==引用
//#include "sysconfig.h"
#include "Ano_FcData.h"
#include "include.h"
#define MAX_PC_XYVEL_CMPS 200
#define MAX_PC_ZVEL_CMPS 150
#define MAX_PC_PAL_DPS 100
#define MAX_PC_HEIGHT 300
extern u8 pi_stage;
extern int heartbeat;
extern u8 heart_err;
extern u8 height_try_hold;
extern u32 running_time;
extern u8 openmv_mode;
extern u16 openmv_pwmval;
extern u16 house_pwmval;
extern u8 point_1;
extern u8 point_2;
extern u8 if_play;
extern int16_t diff_x_1,diff_y_1,diff_x_2,diff_y_2,diff_x_3,diff_y_3;
extern u32 time_x_1,time_y_1,time_x_2,time_y_2,time_x_3,time_y_3;
void Program_Ctrl_User_Set_HXYcmps(float hx_vel_cmps, float hy_vel_cmps);
void Program_Ctrl_User_Set_YAWdps(float yaw_pal_dps);
void Program_Ctrl_User_Set_Zcmps(float z_vel_cmps);
void Program_Ctrl_User_Set_ZHeight(float z_height);
void User_Ctrl(u32 dT_ms);  //
void user_task_1ms();
void Program_Ctrl_User_Set_HXY(float hx,float hy);
void height_try_schedule(u32 dT_ms);
//==定义
typedef struct {
  //
  float vel_cmps_set_h[2];
  float vel_cmps_set_w[2];
  float vel_cmps_set_ref[2];
  //
  float vel_cmps_set_z;
  float pal_dps_set;
  //
  u8 engage_height_set;
  float height_set;
	u8 engage_yaw_set;
	float yaw_set;
	float height_speed_set;
} _pc_user_st;
extern _pc_user_st pc_user;

//==数据声明

typedef struct {
  u8 takeoff_en;
  u8 mode;
  u8 break_out;  //专门用于结束当前任务
  u8 stop;       //专门用于结束当前任务
  u8 land_en;
  u8 user_task_running;
  u8 user_task_end;
  u8 turn_left_en;
  u8 turn_right_en;
  u8 rc_lock;  //锁定遥控器
  u8 task_en;
} _user_cntrl_word;

typedef struct  //用户任务结构体
{
  void (*task_func)(u32 dT_us);
  u32 run_time;  //要执行的时间ms
  u8 start_flag;
  u8 end_flag;
} user_task_t;
extern _user_cntrl_word user_cntrl_word;
//==函数声明

// static

// public
void getchar_test(uint8_t data);
void AnoUserCtrl_GetOneByte(uint8_t data);
//void AnoUserCtrl_Process(void);
void height_try(u32 dT_us);
void UserCtrl_Data_Analyse(u8 *user_data);
void heartbeat_decide();
#endif

#ifndef _MY_PROTOCOL_H_
#define _MY_PROTOCOL_H_
#include "include.h"
extern u8 com_x;
extern u8 com_y;
extern u8 com_z;
extern u8 com_yaw;
extern u8 task_sta;
extern u8 RxBuffer[256];
extern u8 receive_done_sign;
void pi_receive( u8 );
#endif

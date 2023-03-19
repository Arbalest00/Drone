#ifndef __USER_TASK_H
#define __USER_TASK_H

#include "SysConfig.h"
#include "stm32f4xx.h"
#include "my_protocol.h"
void UserTask_OneKeyCmd(void);
extern struct sdata received_data;
extern u8 height_set_sign;
#endif

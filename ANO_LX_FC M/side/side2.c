/*
#include "User_Task.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"
#include "ANO_DT_LX.h"
#include "Drv_AnoOf.h"
#include "math.h"
#define speed_x 30
#define speed_y 30
#define speed_z 30
#define speed_yaw 30
u16 pid_speed=0;
u8 mission_flag=0;
u8 mission_done_flag=0;
s16 cnmcos[1280]={31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,27.00,27.00,27.00,27.00,27.00,27.00,27.00,27.00,27.00,27.00,26.00,26.00,26.00,26.00,26.00,26.00,26.00,26.00,26.00,26.00,25.00,25.00,25.00,25.00,25.00,25.00,25.00,25.00,25.00,24.00,24.00,24.00,24.00,24.00,24.00,24.00,24.00,23.00,23.00,23.00,23.00,23.00,23.00,23.00,23.00,22.00,22.00,22.00,22.00,22.00,22.00,22.00,22.00,21.00,21.00,21.00,21.00,21.00,21.00,21.00,20.00,20.00,20.00,20.00,20.00,20.00,20.00,19.00,19.00,19.00,19.00,19.00,19.00,18.00,18.00,18.00,18.00,18.00,18.00,18.00,17.00,17.00,17.00,17.00,17.00,17.00,16.00,16.00,16.00,16.00,16.00,16.00,15.00,15.00,15.00,15.00,15.00,15.00,14.00,14.00,14.00,14.00,14.00,14.00,13.00,13.00,13.00,13.00,13.00,13.00,12.00,12.00,12.00,12.00,12.00,12.00,11.00,11.00,11.00,11.00,11.00,10.00,10.00,10.00,10.00,10.00,10.00,9.00,9.00,9.00,9.00,9.00,8.00,8.00,8.00,8.00,8.00,8.00,7.00,7.00,7.00,7.00,7.00,6.00,6.00,6.00,6.00,6.00,5.00,5.00,5.00,5.00,5.00,5.00,4.00,4.00,4.00,4.00,4.00,3.00,3.00,3.00,3.00,3.00,2.00,2.00,2.00,2.00,2.00,2.00,1.00,1.00,1.00,1.00,1.00,0.00,0.00,0.00,-1.00,-1.00,-1.00,-1.00,-1.00,-2.00,-2.00,-2.00,-2.00,-2.00,-3.00,-3.00,-3.00,-3.00,-3.00,-4.00,-4.00,-4.00,-4.00,-4.00,-4.00,-5.00,-5.00,-5.00,-5.00,-5.00,-6.00,-6.00,-6.00,-6.00,-6.00,-7.00,-7.00,-7.00,-7.00,-7.00,-7.00,-8.00,-8.00,-8.00,-8.00,-8.00,-9.00,-9.00,-9.00,-9.00,-9.00,-9.00,-10.00,-10.00,-10.00,-10.00,-10.00,-11.00,-11.00,-11.00,-11.00,-11.00,-11.00,-12.00,-12.00,-12.00,-12.00,-12.00,-13.00,-13.00,-13.00,-13.00,-13.00,-13.00,-14.00,-14.00,-14.00,-14.00,-14.00,-14.00,-15.00,-15.00,-15.00,-15.00,-15.00,-15.00,-16.00,-16.00,-16.00,-16.00,-16.00,-16.00,-17.00,-17.00,-17.00,-17.00,-17.00,-17.00,-18.00,-18.00,-18.00,-18.00,-18.00,-18.00,-19.00,-19.00,-19.00,-19.00,-19.00,-19.00,-19.00,-20.00,-20.00,-20.00,-20.00,-20.00,-20.00,-20.00,-21.00,-21.00,-21.00,-21.00,-21.00,-21.00,-21.00,-22.00,-22.00,-22.00,-22.00,-22.00,-22.00,-22.00,-23.00,-23.00,-23.00,-23.00,-23.00,-23.00,-23.00,-23.00,-24.00,-24.00,-24.00,-24.00,-24.00,-24.00,-24.00,-24.00,-25.00,-25.00,-25.00,-25.00,-25.00,-25.00,-25.00,-25.00,-26.00,-26.00,-26.00,-26.00,-26.00,-26.00,-26.00,-26.00,-26.00,-26.00,-27.00,-27.00,-27.00,-27.00,-27.00,-27.00,-27.00,-27.00,-27.00,-27.00,-28.00,-28.00,-28.00,-28.00,-28.00,-28.00,-28.00,-28.00,-28.00,-28.00,-28.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-28.00,-28.00,-28.00,-28.00,-28.00,-28.00,-28.00,-28.00,-28.00,-28.00,-28.00,-27.00,-27.00,-27.00,-27.00,-27.00,-27.00,-27.00,-27.00,-27.00,-27.00,-26.00,-26.00,-26.00,-26.00,-26.00,-26.00,-26.00,-26.00,-26.00,-26.00,-25.00,-25.00,-25.00,-25.00,-25.00,-25.00,-25.00,-25.00,-24.00,-24.00,-24.00,-24.00,-24.00,-24.00,-24.00,-24.00,-23.00,-23.00,-23.00,-23.00,-23.00,-23.00,-23.00,-23.00,-22.00,-22.00,-22.00,-22.00,-22.00,-22.00,-22.00,-21.00,-21.00,-21.00,-21.00,-21.00,-21.00,-21.00,-20.00,-20.00,-20.00,-20.00,-20.00,-20.00,-20.00,-19.00,-19.00,-19.00,-19.00,-19.00,-19.00,-19.00,-18.00,-18.00,-18.00,-18.00,-18.00,-18.00,-17.00,-17.00,-17.00,-17.00,-17.00,-17.00,-16.00,-16.00,-16.00,-16.00,-16.00,-16.00,-15.00,-15.00,-15.00,-15.00,-15.00,-15.00,-14.00,-14.00,-14.00,-14.00,-14.00,-14.00,-13.00,-13.00,-13.00,-13.00,-13.00,-13.00,-12.00,-12.00,-12.00,-12.00,-12.00,-11.00,-11.00,-11.00,-11.00,-11.00,-11.00,-10.00,-10.00,-10.00,-10.00,-10.00,-9.00,-9.00,-9.00,-9.00,-9.00,-9.00,-8.00,-8.00,-8.00,-8.00,-8.00,-7.00,-7.00,-7.00,-7.00,-7.00,-7.00,-6.00,-6.00,-6.00,-6.00,-6.00,-5.00,-5.00,-5.00,-5.00,-5.00,-4.00,-4.00,-4.00,-4.00,-4.00,-4.00,-3.00,-3.00,-3.00,-3.00,-3.00,-2.00,-2.00,-2.00,-2.00,-2.00,-1.00,-1.00,-1.00,-1.00,-1.00,0.00,0.00,0.00,1.00,1.00,1.00,1.00,1.00,2.00,2.00,2.00,2.00,2.00,2.00,3.00,3.00,3.00,3.00,3.00,4.00,4.00,4.00,4.00,4.00,5.00,5.00,5.00,5.00,5.00,5.00,6.00,6.00,6.00,6.00,6.00,7.00,7.00,7.00,7.00,7.00,8.00,8.00,8.00,8.00,8.00,8.00,9.00,9.00,9.00,9.00,9.00,10.00,10.00,10.00,10.00,10.00,10.00,11.00,11.00,11.00,11.00,11.00,12.00,12.00,12.00,12.00,12.00,12.00,13.00,13.00,13.00,13.00,13.00,13.00,14.00,14.00,14.00,14.00,14.00,14.00,15.00,15.00,15.00,15.00,15.00,15.00,16.00,16.00,16.00,16.00,16.00,16.00,17.00,17.00,17.00,17.00,17.00,17.00,18.00,18.00,18.00,18.00,18.00,18.00,18.00,19.00,19.00,19.00,19.00,19.00,19.00,20.00,20.00,20.00,20.00,20.00,20.00,20.00,21.00,21.00,21.00,21.00,21.00,21.00,21.00,22.00,22.00,22.00,22.00,22.00,22.00,22.00,22.00,23.00,23.00,23.00,23.00,23.00,23.00,23.00,23.00,24.00,24.00,24.00,24.00,24.00,24.00,24.00,24.00,25.00,25.00,25.00,25.00,25.00,25.00,25.00,25.00,25.00,26.00,26.00,26.00,26.00,26.00,26.00,26.00,26.00,26.00,26.00,27.00,27.00,27.00,27.00,27.00,27.00,27.00,27.00,27.00,27.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,27.00,27.00,27.00,27.00,27.00,27.00,27.00,27.00,27.00,27.00,26.00,26.00,26.00,26.00,26.00,26.00,26.00,26.00,26.00,26.00,25.00,25.00,25.00,25.00,25.00,25.00,25.00,25.00,25.00,24.00,24.00,24.00,24.00,24.00,24.00,24.00,24.00,23.00,23.00,23.00,23.00,23.00,23.00,23.00,23.00,22.00,22.00,22.00,22.00,22.00,22.00,22.00,22.00,21.00,21.00,21.00,21.00,21.00,21.00,21.00,20.00,20.00,20.00,20.00,20.00,20.00,20.00,19.00,19.00,19.00,19.00,19.00,19.00,18.00,18.00,18.00,18.00,18.00,18.00,18.00,17.00,17.00,17.00,17.00,17.00,17.00,16.00,16.00,16.00,16.00,16.00,16.00,15.00,15.00,15.00,15.00,15.00,15.00,14.00,14.00,14.00,14.00,14.00,14.00,13.00,13.00,13.00,13.00,13.00,13.00,12.00,12.00,12.00,12.00,12.00,12.00,11.00,11.00,11.00,11.00,11.00,10.00,10.00,10.00,10.00,10.00,10.00,9.00,9.00,9.00,9.00,9.00,8.00,8.00,8.00,8.00,8.00,8.00,7.00,7.00,7.00,7.00,7.00,6.00,6.00,6.00,6.00,6.00,5.00,5.00,5.00,5.00,5.00,5.00,4.00,4.00,4.00,4.00,4.00,3.00,3.00,3.00,3.00,3.00,2.00,2.00,2.00,2.00,2.00,2.00,1.00,1.00,1.00,1.00,1.00,0.00,0.00};
s16 cnmsin[1280]={0.00,0.00,0.00,1.00,1.00,1.00,1.00,1.00,2.00,2.00,2.00,2.00,2.00,2.00,3.00,3.00,3.00,3.00,3.00,4.00,4.00,4.00,4.00,4.00,5.00,5.00,5.00,5.00,5.00,5.00,6.00,6.00,6.00,6.00,6.00,7.00,7.00,7.00,7.00,7.00,8.00,8.00,8.00,8.00,8.00,8.00,9.00,9.00,9.00,9.00,9.00,10.00,10.00,10.00,10.00,10.00,10.00,11.00,11.00,11.00,11.00,11.00,12.00,12.00,12.00,12.00,12.00,12.00,13.00,13.00,13.00,13.00,13.00,13.00,14.00,14.00,14.00,14.00,14.00,14.00,15.00,15.00,15.00,15.00,15.00,15.00,16.00,16.00,16.00,16.00,16.00,16.00,17.00,17.00,17.00,17.00,17.00,17.00,18.00,18.00,18.00,18.00,18.00,18.00,18.00,19.00,19.00,19.00,19.00,19.00,19.00,20.00,20.00,20.00,20.00,20.00,20.00,20.00,21.00,21.00,21.00,21.00,21.00,21.00,21.00,22.00,22.00,22.00,22.00,22.00,22.00,22.00,22.00,23.00,23.00,23.00,23.00,23.00,23.00,23.00,23.00,24.00,24.00,24.00,24.00,24.00,24.00,24.00,24.00,25.00,25.00,25.00,25.00,25.00,25.00,25.00,25.00,25.00,26.00,26.00,26.00,26.00,26.00,26.00,26.00,26.00,26.00,26.00,27.00,27.00,27.00,27.00,27.00,27.00,27.00,27.00,27.00,27.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,27.00,27.00,27.00,27.00,27.00,27.00,27.00,27.00,27.00,27.00,26.00,26.00,26.00,26.00,26.00,26.00,26.00,26.00,26.00,26.00,25.00,25.00,25.00,25.00,25.00,25.00,25.00,25.00,25.00,24.00,24.00,24.00,24.00,24.00,24.00,24.00,24.00,23.00,23.00,23.00,23.00,23.00,23.00,23.00,23.00,22.00,22.00,22.00,22.00,22.00,22.00,22.00,22.00,21.00,21.00,21.00,21.00,21.00,21.00,21.00,20.00,20.00,20.00,20.00,20.00,20.00,20.00,19.00,19.00,19.00,19.00,19.00,19.00,18.00,18.00,18.00,18.00,18.00,18.00,18.00,17.00,17.00,17.00,17.00,17.00,17.00,16.00,16.00,16.00,16.00,16.00,16.00,15.00,15.00,15.00,15.00,15.00,15.00,14.00,14.00,14.00,14.00,14.00,14.00,13.00,13.00,13.00,13.00,13.00,13.00,12.00,12.00,12.00,12.00,12.00,12.00,11.00,11.00,11.00,11.00,11.00,10.00,10.00,10.00,10.00,10.00,10.00,9.00,9.00,9.00,9.00,9.00,8.00,8.00,8.00,8.00,8.00,8.00,7.00,7.00,7.00,7.00,7.00,6.00,6.00,6.00,6.00,6.00,5.00,5.00,5.00,5.00,5.00,5.00,4.00,4.00,4.00,4.00,4.00,3.00,3.00,3.00,3.00,3.00,2.00,2.00,2.00,2.00,2.00,2.00,1.00,1.00,1.00,1.00,1.00,0.00,0.00,0.00,-1.00,-1.00,-1.00,-1.00,-1.00,-2.00,-2.00,-2.00,-2.00,-2.00,-3.00,-3.00,-3.00,-3.00,-3.00,-4.00,-4.00,-4.00,-4.00,-4.00,-4.00,-5.00,-5.00,-5.00,-5.00,-5.00,-6.00,-6.00,-6.00,-6.00,-6.00,-7.00,-7.00,-7.00,-7.00,-7.00,-7.00,-8.00,-8.00,-8.00,-8.00,-8.00,-9.00,-9.00,-9.00,-9.00,-9.00,-9.00,-10.00,-10.00,-10.00,-10.00,-10.00,-11.00,-11.00,-11.00,-11.00,-11.00,-11.00,-12.00,-12.00,-12.00,-12.00,-12.00,-13.00,-13.00,-13.00,-13.00,-13.00,-13.00,-14.00,-14.00,-14.00,-14.00,-14.00,-14.00,-15.00,-15.00,-15.00,-15.00,-15.00,-15.00,-16.00,-16.00,-16.00,-16.00,-16.00,-16.00,-17.00,-17.00,-17.00,-17.00,-17.00,-17.00,-18.00,-18.00,-18.00,-18.00,-18.00,-18.00,-19.00,-19.00,-19.00,-19.00,-19.00,-19.00,-19.00,-20.00,-20.00,-20.00,-20.00,-20.00,-20.00,-20.00,-21.00,-21.00,-21.00,-21.00,-21.00,-21.00,-21.00,-22.00,-22.00,-22.00,-22.00,-22.00,-22.00,-22.00,-23.00,-23.00,-23.00,-23.00,-23.00,-23.00,-23.00,-23.00,-24.00,-24.00,-24.00,-24.00,-24.00,-24.00,-24.00,-24.00,-25.00,-25.00,-25.00,-25.00,-25.00,-25.00,-25.00,-25.00,-26.00,-26.00,-26.00,-26.00,-26.00,-26.00,-26.00,-26.00,-26.00,-26.00,-27.00,-27.00,-27.00,-27.00,-27.00,-27.00,-27.00,-27.00,-27.00,-27.00,-28.00,-28.00,-28.00,-28.00,-28.00,-28.00,-28.00,-28.00,-28.00,-28.00,-28.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-31.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-30.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-29.00,-28.00,-28.00,-28.00,-28.00,-28.00,-28.00,-28.00,-28.00,-28.00,-28.00,-28.00,-27.00,-27.00,-27.00,-27.00,-27.00,-27.00,-27.00,-27.00,-27.00,-27.00,-26.00,-26.00,-26.00,-26.00,-26.00,-26.00,-26.00,-26.00,-26.00,-26.00,-25.00,-25.00,-25.00,-25.00,-25.00,-25.00,-25.00,-25.00,-24.00,-24.00,-24.00,-24.00,-24.00,-24.00,-24.00,-24.00,-23.00,-23.00,-23.00,-23.00,-23.00,-23.00,-23.00,-23.00,-22.00,-22.00,-22.00,-22.00,-22.00,-22.00,-22.00,-21.00,-21.00,-21.00,-21.00,-21.00,-21.00,-21.00,-20.00,-20.00,-20.00,-20.00,-20.00,-20.00,-20.00,-19.00,-19.00,-19.00,-19.00,-19.00,-19.00,-19.00,-18.00,-18.00,-18.00,-18.00,-18.00,-18.00,-17.00,-17.00,-17.00,-17.00,-17.00,-17.00,-16.00,-16.00,-16.00,-16.00,-16.00,-16.00,-15.00,-15.00,-15.00,-15.00,-15.00,-15.00,-14.00,-14.00,-14.00,-14.00,-14.00,-14.00,-13.00,-13.00,-13.00,-13.00,-13.00,-13.00,-12.00,-12.00,-12.00,-12.00,-12.00,-11.00,-11.00,-11.00,-11.00,-11.00,-11.00,-10.00,-10.00,-10.00,-10.00,-10.00,-9.00,-9.00,-9.00,-9.00,-9.00,-9.00,-8.00,-8.00,-8.00,-8.00,-8.00,-7.00,-7.00,-7.00,-7.00,-7.00,-7.00,-6.00,-6.00,-6.00,-6.00,-6.00,-5.00,-5.00,-5.00,-5.00,-5.00,-4.00,-4.00,-4.00,-4.00,-4.00,-4.00,-3.00,-3.00,-3.00,-3.00,-3.00,-2.00,-2.00,-2.00,-2.00,-2.00,-1.00,-1.00,-1.00,-1.00,-1.00,-1.00,0.00,0.00,1.00,1.00,1.00,1.00,1.00,2.00,2.00,2.00,2.00,2.00,2.00,3.00,3.00,3.00,3.00,3.00,4.00,4.00,4.00,4.00,4.00,5.00,5.00,5.00,5.00,5.00,5.00,6.00,6.00,6.00,6.00,6.00,7.00,7.00,7.00,7.00,7.00,8.00,8.00,8.00,8.00,8.00,8.00,9.00,9.00,9.00,9.00,9.00,10.00,10.00,10.00,10.00,10.00,10.00,11.00,11.00,11.00,11.00,11.00,12.00,12.00,12.00,12.00,12.00,12.00,13.00,13.00,13.00,13.00,13.00,13.00,14.00,14.00,14.00,14.00,14.00,14.00,15.00,15.00,15.00,15.00,15.00,15.00,16.00,16.00,16.00,16.00,16.00,16.00,17.00,17.00,17.00,17.00,17.00,17.00,18.00,18.00,18.00,18.00,18.00,18.00,18.00,19.00,19.00,19.00,19.00,19.00,19.00,20.00,20.00,20.00,20.00,20.00,20.00,20.00,21.00,21.00,21.00,21.00,21.00,21.00,21.00,22.00,22.00,22.00,22.00,22.00,22.00,22.00,22.00,23.00,23.00,23.00,23.00,23.00,23.00,23.00,23.00,24.00,24.00,24.00,24.00,24.00,24.00,24.00,24.00,25.00,25.00,25.00,25.00,25.00,25.00,25.00,25.00,25.00,26.00,26.00,26.00,26.00,26.00,26.00,26.00,26.00,26.00,26.00,27.00,27.00,27.00,27.00,27.00,27.00,27.00,27.00,27.00,27.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,28.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,29.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,30.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00,31.00};
void UserTask_OneKeyCmd(void)//һ������
{
    static u8 one_key_takeoff_f = 1, one_key_land_f = 1, one_key_mission_f = 0;
    static u8 mission_step,eme_stop=1,pi_start_f=0,now_task_mode=0;
	mission_flag=one_key_mission_f;
	//////////////////////////////////////////////////////////////////////////////////
    //һ����� �����ж���ң���źŲ�ִ��
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
	//һ������
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
	///////////////////////////////////////////////////////////////////////
	//��������
	if((rc_in.rc_ch.st_data.ch_[ch_7_aux3]>1700 && rc_in.rc_ch.st_data.ch_[ch_7_aux3]<2200)||(received_data.next_task_sign==1&&mission_done_flag==0))
		{
			//��û��ִ��
			if(one_key_mission_f ==0)
			{
				//����Ѿ�ִ��
				one_key_mission_f = 1;
				//��ʼ����
				mission_step = 0;
				now_task_mode=received_data.task_sta;
			}
		}
		else
		{
			//��λ��ǣ��Ա��ٴ�ִ��1
			if(received_data.next_task_sign==0) mission_done_flag=0;
			one_key_mission_f = 0;	
			if(one_key_mission_f==1)OneKey_Land();			
		}
		
	//////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////
	//�����б�
	if(one_key_mission_f==1)
		{
			static u16 time_dly_cnt_ms;
			static s16 integ_x,integ_y;
			static s16 integ_x_base,integ_y_base;
			static s32 pos_x_base,pos_y_base;
			static u16 icount=0;
			//
			switch(now_task_mode)
			{
				case 0x00://������������
				{
					switch(mission_step)
					{
						case 0:
						{
							//reset
							time_dly_cnt_ms = 0;
							integ_x=0;
							integ_y=0;
							integ_x_base=0;
							integ_y_base=0;
							mission_step +=1;
						}
						break;
						case 1://����
						{
							if(time_dly_cnt_ms<10000)//������ʱ
							{
								time_dly_cnt_ms+=20;//ms
							}
							else
							{
								time_dly_cnt_ms = 0;
								PID_height_init();
								mission_step +=FC_Unlock();
							}
						}
						break;
						case 2://���60cm
						{
							if(time_dly_cnt_ms<3500)//ת����ʱ
							{
								time_dly_cnt_ms+=20;//ms
							}
							else
							{
								time_dly_cnt_ms = 0;
								mission_step += OneKey_Takeoff(60);
								one_key_takeoff_f=1;
							}
						}
						break;
						case 3://��10��
						{
							if(time_dly_cnt_ms<12000)//������ʱ
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
						case 4://����׼��
						{
							pid_speed=0;
							PID_height_init();
							time_dly_cnt_ms = 0;
							mission_step += 1;
						}	
						break;
						case 5://5S����120cm  ����׼��
						{
							if(time_dly_cnt_ms<11000)
							{
								tar_setdata(0,0,height_set(ano_of.of_alt_cm,120),0);
								time_dly_cnt_ms+=20;//ms
							}
							else
							{ 
								tar_setdata(0,0,0,0);
								time_dly_cnt_ms = 0;
								mission_step += 1;
								integ_x_base=ano_of.intergral_x;
							}
						}
						break;
						case 6://����120cm��ǰ��200cm
						{
							if(ano_of.intergral_x-integ_x_base<440)
							{
								tar_setdata(speed_x,0,height_set(ano_of.of_alt_cm,120),0);
							}
							else
							{
								integ_x_base=0;
								pid_speed=0;
								tar_setdata(0,0,0,0);
								time_dly_cnt_ms = 0;
								mission_step += 1;
							}
						}
						break;
						default:
						{
							mission_done_flag=1;
							OneKey_Land();
						}
						break;
					}
					
				}
				break;
				case 0x01://����
				{
					switch(mission_step)
					{
						case 0:
						{
							//reset
							time_dly_cnt_ms = 0;
							integ_x=0;
							integ_y=0;
							integ_x_base=0;
							integ_y_base=0;
							mission_step +=1;
						}
						break;
						case 1://����
							if(time_dly_cnt_ms<10000)//������ʱ
							{
								time_dly_cnt_ms+=20;//ms
							}
							else
							{
								time_dly_cnt_ms = 0;
								PID_height_init();
								mission_step +=FC_Unlock();
							}
						break;
						case 2://���60cm
						{
							if(time_dly_cnt_ms<3500)//ת����ʱ
							{
								time_dly_cnt_ms+=20;//ms
							}
							else
							{
								time_dly_cnt_ms = 0;
								mission_step += OneKey_Takeoff(100);
								one_key_takeoff_f=1;
							}
						}
						break;
						case 3://��1��
						{
							if(time_dly_cnt_ms<1000)//������ʱ
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
						case 4://����ǰ��׼��
						{
							pid_speed=0;
							PID_height_init();
							time_dly_cnt_ms = 0;
							mission_step += 1;
						}	
						break;
						case 5://����3s
						{
							if(time_dly_cnt_ms<3000)//ת����ʱ
							{
								time_dly_cnt_ms+=20;//ms
								tar_setdata(0,0,height_set(ano_of.of_alt_cm,100),0);
							}
							else
							{
								tar_setdata(0,0,0,0);
								time_dly_cnt_ms = 0;
								mission_step += 1;
								pos_x_base=fc_pos.st_data.vel_x;
							}
						}
						break;
						case 6://����100cm  GPS����ǰ��2m
						{
							if(fc_pos.st_data.vel_x-pos_x_base<200)
							{
								tar_setdata(speed_x,0,height_set(ano_of.of_alt_cm,100),0);
								time_dly_cnt_ms+=20;//ms
							}
							else
							{ 
								tar_setdata(0,0,0,0);
								time_dly_cnt_ms = 0;
								mission_step += 1;
								pos_y_base=fc_pos.st_data.vel_y;
							}
						}
						break;
						case 7://����100cm GPS��������2m
						{
							if(fc_pos.st_data.vel_y-pos_y_base<200)
							{
								tar_setdata(0,speed_y,height_set(ano_of.of_alt_cm,100),0);
							}
							else
							{
								pid_speed=0;
								tar_setdata(0,0,0,0);
								time_dly_cnt_ms = 0;
								pos_x_base=fc_pos.st_data.vel_x;
								mission_step += 1;
							}
						}
						break;
						case 8://����100cm  GPS���ֺ���2m
						{
							if(fc_pos.st_data.vel_x-pos_x_base>-200)
							{
								tar_setdata(-speed_x,0,height_set(ano_of.of_alt_cm,100),0);
								time_dly_cnt_ms+=20;//ms
							}
							else
							{ 
								tar_setdata(0,0,0,0);
								time_dly_cnt_ms = 0;
								pos_y_base=fc_pos.st_data.vel_y;
								mission_step += 1;
							}
						}
						break;
						case 9://����100cm  GPS��������2m
						{
							if(fc_pos.st_data.vel_y-pos_y_base>-200)
							{
								tar_setdata(0,-speed_y,height_set(ano_of.of_alt_cm,100),0);
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
						default:
						{
							mission_done_flag=1;
							OneKey_Land();
						}
						break;
					}
				}
				break;
				case 0x02://Բ��
				{
					switch(mission_step)
					{
						case 0:
						{
							//reset
							time_dly_cnt_ms = 0;
							integ_x=0;
							integ_y=0;
							integ_x_base=0;
							integ_y_base=0;
							mission_step +=1;
						}
						break;
						case 1://����
							if(time_dly_cnt_ms<10000)//������ʱ
							{
								time_dly_cnt_ms+=20;//ms
							}
							else
							{
								time_dly_cnt_ms = 0;
								PID_height_init();
								mission_step +=FC_Unlock();
							}
						break;
						case 2://���100cm
						{
							if(time_dly_cnt_ms<3500)//ת����ʱ
							{
								time_dly_cnt_ms+=20;//ms
							}
							else
							{
								time_dly_cnt_ms = 0;
								mission_step += OneKey_Takeoff(100);
								one_key_takeoff_f=1;
							}
						}
						break;
						case 3://��1��
						{
							if(time_dly_cnt_ms<1000)//������ʱ
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
						case 4://����ǰ��׼��
						{
							pid_speed=0;
							PID_height_init();
							time_dly_cnt_ms = 0;
							mission_step += 1;
							integ_x_base=ano_of.intergral_x;
						}	
						break;
					
						case 5://����5s ��Բ׼��
						{
							if(time_dly_cnt_ms<4000)//ת����ʱ
							{
								tar_setdata(0,0,height_set(ano_of.of_alt_cm,100),0);
								time_dly_cnt_ms+=20;//ms
							}
							else
							{
								time_dly_cnt_ms = 0;
								mission_step += 1;
								pos_x_base=fc_pos.st_data.vel_x;
								pos_y_base=fc_pos.st_data.vel_y;
								icount=0;
							}
						}
						break;
						case 6://����120cm��ǰ��200cm
						{
							if(ano_of.intergral_x-integ_x_base<120)
							{
								tar_setdata(speed_x,0,height_set(ano_of.of_alt_cm,100),0);
							}
							else
							{
								pid_speed=0;
								tar_setdata(0,0,0,0);
								time_dly_cnt_ms = 0;
								mission_step += 1;
							}
						}
						break;
						case 7://����100cm  ��Բ
						{
							if(time_dly_cnt_ms<20500)
							{
								tar_setdata(cnmsin[icount],cnmcos[icount],height_set(ano_of.of_alt_cm,100),0);
								time_dly_cnt_ms+=20;
								icount++;
							}
							else
							{ 
								tar_setdata(0,0,0,0);
								time_dly_cnt_ms = 0;
								mission_step += 1;
							}
						}
						break;
						default:
						{
							mission_done_flag=1;
							OneKey_Land();
						}
						break;
					}
				}
				break;
				case 0x03://��������
				{
					switch(mission_step)
					{
						case 0:
						{
							//reset
							time_dly_cnt_ms = 0;
							integ_x=0;
							integ_y=0;
							integ_x_base=0;
							integ_y_base=0;
							mission_step +=1;
						}
						break;
						case 1://����
						{
							if(time_dly_cnt_ms<10000)//������ʱ
							{
								time_dly_cnt_ms+=20;//ms
							}
							else
							{
								time_dly_cnt_ms = 0;
								PID_height_init();
								mission_step +=FC_Unlock();
							}
						}
						break;
						case 2://���60cm
						{
							if(time_dly_cnt_ms<3500)//ת����ʱ
							{
								time_dly_cnt_ms+=20;//ms
							}
							else
							{
								time_dly_cnt_ms = 0;
								mission_step += OneKey_Takeoff(100);
								one_key_takeoff_f=1;
							}
						}
						break;
						case 3://��1��
						{
							if(time_dly_cnt_ms<1000)//������ʱ
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
						case 4://����׼��
						{
							pid_speed=0;
							PID_height_init();
							time_dly_cnt_ms = 0;
							mission_step += 1;
						}	
						break;
						case 5://5S����120cm  ����׼��
						{
							if(time_dly_cnt_ms<5000)
							{
								tar_setdata(0,0,height_set(ano_of.of_alt_cm,100),0);
								time_dly_cnt_ms+=20;//ms
							}
							else
							{ 
								tar_setdata(0,0,0,0);
								time_dly_cnt_ms = 0;
								mission_step += 1;
								integ_x_base=ano_of.intergral_x;
							}
						}
						break;
						case 6://����120cm��ǰ��200cm
						{
							if(ano_of.intergral_x-integ_x_base<120)
							{
								tar_setdata(speed_x,0,height_set(ano_of.of_alt_cm,100),0);
							}
							else
							{
								integ_y_base=ano_of.intergral_y;
								pid_speed=0;
								tar_setdata(0,0,0,0);
								time_dly_cnt_ms = 0;
								mission_step += 1;
							}
						}
						break;
						case 7://����120cm��ǰ��200cm
						{
							if(ano_of.intergral_y-integ_y_base>-100)
							{
								tar_setdata(0,-speed_y,height_set(ano_of.of_alt_cm,100),0);
							}
							else
							{
								integ_x_base=ano_of.intergral_x;
								pid_speed=0;
								tar_setdata(0,0,0,0);
								time_dly_cnt_ms = 0;
								mission_step += 1;
							}
						}
						break;
						case 8://����120cm��ǰ��200cm
						{
							if(ano_of.intergral_x-integ_x_base<200)
							{
								tar_setdata(speed_x,0,height_set(ano_of.of_alt_cm,100),0);
							}
							else
							{
								integ_y_base=ano_of.intergral_y;
								pid_speed=0;
								tar_setdata(0,0,0,0);
								time_dly_cnt_ms = 0;
								mission_step += 1;
							}
						}
						break;
						case 9://����120cm��ǰ��200cm
						{
							if(ano_of.intergral_y-integ_y_base<190)
							{
								tar_setdata(0,speed_y,height_set(ano_of.of_alt_cm,100),0);
							}
							else
							{
								pid_speed=0;
								tar_setdata(0,0,0,0);
								time_dly_cnt_ms = 0;
								mission_step += 1;
							}
						}
						break;
						case 10://����120cm��ǰ��200cm
						{
							if(ano_of.intergral_x-integ_x_base>0)
							{
								tar_setdata(-speed_x,0,height_set(ano_of.of_alt_cm,100),0);
							}
							else
							{
								pid_speed=0;
								tar_setdata(0,0,0,0);
								time_dly_cnt_ms = 0;
								mission_step += 1;
							}
						}
						break;
						case 11://����120cm��ǰ��200cm
						{
							if(ano_of.intergral_y-integ_y_base>0)
							{
								tar_setdata(0,-speed_y,height_set(ano_of.of_alt_cm,100),0);
							}
							else
							{
								pid_speed=0;
								tar_setdata(0,0,0,0);
								time_dly_cnt_ms = 0;
								mission_step += 1;
							}
						}
						break;
						default:
						{
							mission_done_flag=1;
							OneKey_Land();
						}
						break;
					}
				break;			
				}
			}
			
		}
		else
		{
			mission_step = 0;
		}
}
*/
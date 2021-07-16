#ifndef __MOVE_H__
#define __MOVE_H__

#include "robo_base.h"
#include "usart_analysis.h"

typedef struct DirectSystem
{
    IMU_Info* IMU;
    PID Dir_Pos_Pid;
    double Direction_Tar;
    uint8_t Tar_Update;
}DirectSystem;

void Move_Analysis(float Vel_X, float Vel_Y, float Vel_W);
void Motor_Angle(MotorGroup* P_Motor,float dA_Tar);
void DirectSystem_Init(void);
#endif



#ifndef __MOVE_H__
#define __MOVE_H__

#include "main.h"
#include "robo_base.h"
#include "usart_analysis.h"

typedef struct DirectSystem
{
    IMU_Info* IMU;
    PID Dir_Pos_Pid;
    int16_t Direction_Tar;
    uint8_t Tar_Update;
}DirectSystem;

void Move_Analysis(double Vel_X, double Vel_Y, double Vel_W);
void Limit_Angle(double Angle);
void Motor_Angle(MotorGroup* P_Motor,double dA_Tar);
void Motor_Rotate(MotorGroup* P_Motor, double Rotation_Angle);
void DirectSystem_Init(void);
#endif



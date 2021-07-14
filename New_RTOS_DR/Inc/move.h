#ifndef __MOVE_H__
#define __MOVE_H__

#include "main.h"
#include "robo_base.h"
#include "usart_analysis.h"

void Move_Analysis(double Vel_X, double Vel_Y, double Vel_W);
void Pos_Motor_Turn(MotorGroup* P_Motor, double Angle_Tar , double Vel_Tar, double Length_X_Rotate, double Length_Y_Rotate, double Distance_Rotate);
void Motor_Rotate(MotorGroup* P_Motor, double Rotation_Angle);
#endif



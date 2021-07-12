#ifndef __MOVE_H__
#define __MOVE_H__

#include "main.h"
#include "robo_base.h"
#include "usart_analysis.h"

void Move_Analysis(void);
void Motor_Angle(MotorGroup* P_Motor,double dA_Tar);
void Motor_Rotate(MotorGroup* P_Motor, double Rotation_Angle);
#endif



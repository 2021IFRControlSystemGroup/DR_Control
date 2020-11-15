#ifndef __MOVE_H__
#define __MOVE_H__

#include "main.h"
#include "robo_base.h"

void Move_Analysis(ROBO_BASE* Robo);
void MotorGroup_Tar_Analysis(Pos_System* P_Pos,Speed_System* P_Speed,float E_Angle,float Speed);

#endif



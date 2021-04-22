#ifndef __MOVE_H__
#define __MOVE_H__

#include "main.h"
#include "math.h"
#include "odrive_can.h"
#include "robo_base.h"
#include "analysis.h"


void Motor_Speed(void);
void Motor_Angle(Pos_System* P_Pos,float* ODrive_Tar,double dA_Tar);

#endif



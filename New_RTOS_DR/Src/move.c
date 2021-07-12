#include "move.h"

#define TWO_PI (2*PI)
#define MAX_SPEED 10.0

double dA_Tar = 0;
double dS_Tar = 0;
double dR_Tar = 0;
void Move_Analysis(void)
{   
	dS_Tar = sqrt((Robo_Base.Speed_X * Robo_Base.Speed_X) + (Robo_Base.Speed_Y * Robo_Base.Speed_Y)) * MAX_SPEED;
    dR_Tar = Robo_Base.Speed_Rotate * 10;
    dA_Tar = Robo_Base.Angle / TWO_PI * ROTOR_ANGLE * GEAR_RATIO;
    if(Robo_Base.Angle < 0) dA_Tar += ROTOR_ANGLE * GEAR_RATIO;
	
	Robo_Base.LF._Axis->Input_Vel = -dS_Tar - dR_Tar;
	Robo_Base.LB._Axis->Input_Vel = -dS_Tar - dR_Tar;
	Robo_Base.RF._Axis->Input_Vel = dS_Tar + dR_Tar;
	Robo_Base.RB._Axis->Input_Vel = dS_Tar + dR_Tar;
	Motor_Angle(&Robo_Base.LF,dA_Tar);
	Motor_Angle(&Robo_Base.LB,dA_Tar);
	Motor_Angle(&Robo_Base.RF,dA_Tar);
	Motor_Angle(&Robo_Base.RB,dA_Tar);
}
int32_t Error_Angle_Test = 0;
void Motor_Angle(MotorGroup* P_Motor,double dA_Tar)
{
	MotorSystem* P_Pos = &P_Motor->_Pos;
	float* ODrive_Tar = &P_Motor->_Axis->Input_Vel;
    double Angle_Temp = abs(P_Pos->Info.Relative_Angle);
    double Relative_Angle_temp = P_Pos->Info.Relative_Angle;
    if(P_Pos->Info.Relative_Angle < 0) P_Pos->Info.Relative_Angle += (2 * PI_ANGLE);
        Error_Angle_Test = dA_Tar - P_Pos->Info.Relative_Angle;
    if(P_Pos->Info.Relative_Angle >= HALF_PI_ANGLE && P_Pos->Info.Relative_Angle <= HALF_PI_ANGLE + PI_ANGLE)
      if((dA_Tar >= P_Pos->Info.Relative_Angle - HALF_PI_ANGLE) && (dA_Tar <= P_Pos->Info.Relative_Angle + HALF_PI_ANGLE)){
        P_Pos->Tar_Pos = dA_Tar - P_Pos->Info.Relative_Angle + P_Pos->Info.Abs_Angle;
      }else if(dA_Tar < P_Pos->Info.Relative_Angle){
         P_Pos->Tar_Pos = dA_Tar - P_Pos->Info.Relative_Angle + P_Pos->Info.Abs_Angle + PI_ANGLE, *ODrive_Tar *= -1;
      }else P_Pos->Tar_Pos = dA_Tar - P_Pos->Info.Relative_Angle + P_Pos->Info.Abs_Angle - PI_ANGLE, *ODrive_Tar *= -1;
    else if(P_Pos->Info.Relative_Angle <= HALF_PI_ANGLE){
      if((dA_Tar > P_Pos->Info.Relative_Angle + HALF_PI_ANGLE) && (dA_Tar < P_Pos->Info.Relative_Angle + HALF_PI_ANGLE + PI_ANGLE)){
        P_Pos->Tar_Pos = dA_Tar - P_Pos->Info.Relative_Angle + P_Pos->Info.Abs_Angle - PI_ANGLE, *ODrive_Tar *= -1;
      }else if(dA_Tar < P_Pos->Info.Relative_Angle + HALF_PI_ANGLE){
        P_Pos->Tar_Pos = dA_Tar - P_Pos->Info.Relative_Angle + P_Pos->Info.Abs_Angle;
      }else P_Pos->Tar_Pos = dA_Tar - 2 * PI_ANGLE - P_Pos->Info.Relative_Angle + P_Pos->Info.Abs_Angle;
    }else{
      if((dA_Tar <= P_Pos->Info.Relative_Angle - HALF_PI_ANGLE) && (dA_Tar >= P_Pos->Info.Relative_Angle - HALF_PI_ANGLE - PI_ANGLE)){
        P_Pos->Tar_Pos = dA_Tar - P_Pos->Info.Relative_Angle + P_Pos->Info.Abs_Angle + PI_ANGLE, *ODrive_Tar *= -1;
      }else if(dA_Tar > P_Pos->Info.Relative_Angle - HALF_PI_ANGLE){
        P_Pos->Tar_Pos = dA_Tar - P_Pos->Info.Relative_Angle + P_Pos->Info.Abs_Angle;
      }else P_Pos->Tar_Pos = 2 * PI_ANGLE + dA_Tar - P_Pos->Info.Relative_Angle + P_Pos->Info.Abs_Angle;
    }P_Pos->Info.Relative_Angle = Relative_Angle_temp;
}

#include "move.h"

#define TWO_PI (2*PI)
#define MAX_SPEED 35.0
#define MAX_ROTATE 15.0

double Angle_Tar[4] = {0};
double Speed_Tar[4] = {0};
double Angle_XY = 0;
double Speed_XY = 0;
double dR_Tar = 0;
double Distance = 0;
double Radius = 0;
void Move_Analysis(double Vel_X, double Vel_Y, double Vel_W)
{
    Speed_XY = sqrt((Vel_X * Vel_X) + (Vel_Y * Vel_Y));
    Angle_XY = Robo_Base.Angle;
    if(Vel_W == 0 && (Vel_X != 0 || Vel_Y != 0)){
        //Radius = INFINITY;
        Angle_Tar[Robo_Base.LF._Pos.Motor_Num] = Angle_XY / TWO_PI * ONE_CIRCLE;
        Angle_Tar[Robo_Base.LB._Pos.Motor_Num] = Angle_XY / TWO_PI * ONE_CIRCLE;
        Angle_Tar[Robo_Base.RF._Pos.Motor_Num] = Angle_XY / TWO_PI * ONE_CIRCLE;
        Angle_Tar[Robo_Base.RB._Pos.Motor_Num] = Angle_XY / TWO_PI * ONE_CIRCLE;
        Speed_Tar[Robo_Base.LF._Pos.Motor_Num] = Speed_XY * MAX_SPEED / (0.205 * TWO_PI);
        Speed_Tar[Robo_Base.LB._Pos.Motor_Num] = Speed_XY * MAX_SPEED / (0.205 * TWO_PI);
        Speed_Tar[Robo_Base.RF._Pos.Motor_Num] = Speed_XY * MAX_SPEED / (0.205 * TWO_PI);
        Speed_Tar[Robo_Base.RB._Pos.Motor_Num] = Speed_XY * MAX_SPEED / (0.205 * TWO_PI);
    }else if((Vel_X == 0 && Vel_Y == 0) && Vel_W != 0){
        Radius = 0;
        Angle_Tar[Robo_Base.LF._Pos.Motor_Num] = (TWO_PI - HALF_PI / 2) / TWO_PI * ONE_CIRCLE;
        Angle_Tar[Robo_Base.LB._Pos.Motor_Num] = (HALF_PI / 2) / TWO_PI * ONE_CIRCLE;
        Angle_Tar[Robo_Base.RF._Pos.Motor_Num] = (PI + HALF_PI / 2) / TWO_PI * ONE_CIRCLE;
        Angle_Tar[Robo_Base.RB._Pos.Motor_Num] = (PI - HALF_PI / 2) / TWO_PI * ONE_CIRCLE;
        Speed_Tar[Robo_Base.LF._Pos.Motor_Num] = -Vel_W * MAX_SPEED * 1.414 / 2 * 0.205 * MAX_ROTATE ;
        Speed_Tar[Robo_Base.LB._Pos.Motor_Num] = -Vel_W * MAX_SPEED * 1.414 / 2 * 0.205 * MAX_ROTATE ;
        Speed_Tar[Robo_Base.RF._Pos.Motor_Num] = -Vel_W * MAX_SPEED * 1.414 / 2 * 0.205 * MAX_ROTATE ;
        Speed_Tar[Robo_Base.RB._Pos.Motor_Num] = -Vel_W * MAX_SPEED * 1.414 / 2 * 0.205 * MAX_ROTATE ;
    }else if((Vel_X != 0 || Vel_Y != 0) && Vel_W != 0){
        Radius = Speed_XY / Vel_W;
        Angle_Tar[Robo_Base.RF._Pos.Motor_Num] = (HALF_PI - atan2((Radius * cos(Angle_XY) + 0.205) , (Radius * sin(Angle_XY) + 0.205))) / TWO_PI * ONE_CIRCLE;
        Angle_Tar[Robo_Base.RB._Pos.Motor_Num] = (HALF_PI + atan2((Radius * cos(Angle_XY) + 0.205) , (-Radius * sin(Angle_XY) + 0.205))) / TWO_PI * ONE_CIRCLE;
        Angle_Tar[Robo_Base.LF._Pos.Motor_Num] = (HALF_PI - atan2((Radius * cos(Angle_XY) - 0.205) , (Radius * sin(Angle_XY) + 0.205))) / TWO_PI * ONE_CIRCLE;
        Angle_Tar[Robo_Base.LB._Pos.Motor_Num] = (HALF_PI + atan2((Radius * cos(Angle_XY) - 0.205) , (-Radius * sin(Angle_XY) + 0.205))) / TWO_PI * ONE_CIRCLE;
        Speed_Tar[Robo_Base.LF._Pos.Motor_Num] = Vel_W * 
        MAX_SPEED * sqrt((0.205 + Radius * sin(Angle_XY)) * (0.205 + Radius * sin(Angle_XY))+(0.205 + Radius * cos(Angle_XY))*(0.205 + Radius * cos(Angle_XY)));
        Speed_Tar[Robo_Base.LB._Pos.Motor_Num] = Vel_W * 
        MAX_SPEED * sqrt((0.205 - Radius * sin(Angle_XY)) * (0.205 - Radius * sin(Angle_XY))+(0.205 + Radius * cos(Angle_XY))*(0.205 + Radius * cos(Angle_XY)));
        Speed_Tar[Robo_Base.RF._Pos.Motor_Num] = Vel_W * 
        MAX_SPEED * sqrt((0.205 - Radius * sin(Angle_XY)) * (0.205 - Radius * sin(Angle_XY))+(-0.205 + Radius * cos(Angle_XY))*(-0.205 + Radius * cos(Angle_XY)));
        Speed_Tar[Robo_Base.RB._Pos.Motor_Num] = Vel_W * 
        MAX_SPEED * sqrt((0.205 + Radius * sin(Angle_XY)) * (0.205 + Radius * sin(Angle_XY))+(-0.205 + Radius * cos(Angle_XY))*(-0.205 + Radius * cos(Angle_XY)));
    }else{
        Speed_Tar[Robo_Base.LF._Pos.Motor_Num] = 0;
        Speed_Tar[Robo_Base.LB._Pos.Motor_Num] = 0;
        Speed_Tar[Robo_Base.RF._Pos.Motor_Num] = 0;
        Speed_Tar[Robo_Base.RB._Pos.Motor_Num] = 0;
    }
    
    if(Angle_Tar[Robo_Base.LF._Pos.Motor_Num] < 0) Angle_Tar[Robo_Base.LF._Pos.Motor_Num] += ONE_CIRCLE;
    if(Angle_Tar[Robo_Base.LB._Pos.Motor_Num] < 0) Angle_Tar[Robo_Base.LB._Pos.Motor_Num] += ONE_CIRCLE;
    if(Angle_Tar[Robo_Base.RF._Pos.Motor_Num] < 0) Angle_Tar[Robo_Base.RF._Pos.Motor_Num] += ONE_CIRCLE;
    if(Angle_Tar[Robo_Base.RB._Pos.Motor_Num] < 0) Angle_Tar[Robo_Base.RB._Pos.Motor_Num] += ONE_CIRCLE;
    
    Robo_Base.LF._Axis->Input_Vel = -Speed_Tar[Robo_Base.LF._Pos.Motor_Num];
	Robo_Base.LB._Axis->Input_Vel = -Speed_Tar[Robo_Base.LB._Pos.Motor_Num];
	Robo_Base.RF._Axis->Input_Vel = Speed_Tar[Robo_Base.RF._Pos.Motor_Num];
	Robo_Base.RB._Axis->Input_Vel = Speed_Tar[Robo_Base.RB._Pos.Motor_Num];
	Motor_Angle(&Robo_Base.LF,Angle_Tar[Robo_Base.LF._Pos.Motor_Num]);
	Motor_Angle(&Robo_Base.LB,Angle_Tar[Robo_Base.LB._Pos.Motor_Num]);
	Motor_Angle(&Robo_Base.RF,Angle_Tar[Robo_Base.RF._Pos.Motor_Num]);
	Motor_Angle(&Robo_Base.RB,Angle_Tar[Robo_Base.RB._Pos.Motor_Num]);
}

void Motor_Angle(MotorGroup* P_Motor,double dA_Tar)
{
	MotorSystem* P_Pos = &P_Motor->_Pos;
	float* ODrive_Tar = &P_Motor->_Axis->Input_Vel;
    double Angle_Temp = abs(P_Pos->Info.Relative_Angle);
    double Relative_Angle_temp = P_Pos->Info.Relative_Angle;
    if(P_Pos->Info.Relative_Angle < 0) P_Pos->Info.Relative_Angle += (2 * PI_ANGLE);
    if(P_Pos->Info.Relative_Angle >= HALF_PI_ANGLE && P_Pos->Info.Relative_Angle <= HALF_PI_ANGLE + PI_ANGLE)
      if((dA_Tar >= P_Pos->Info.Relative_Angle - HALF_PI_ANGLE) && (dA_Tar <= P_Pos->Info.Relative_Angle + HALF_PI_ANGLE)){
        P_Pos->Tar_Pos = dA_Tar - P_Pos->Info.Relative_Angle + P_Pos->Info.Abs_Angle;
      }else if(dA_Tar <= P_Pos->Info.Relative_Angle){
         P_Pos->Tar_Pos = dA_Tar - P_Pos->Info.Relative_Angle + P_Pos->Info.Abs_Angle + PI_ANGLE, *ODrive_Tar *= -1;
      }else P_Pos->Tar_Pos = dA_Tar - P_Pos->Info.Relative_Angle + P_Pos->Info.Abs_Angle - PI_ANGLE, *ODrive_Tar *= -1;
    else if(P_Pos->Info.Relative_Angle <= HALF_PI_ANGLE){
      if((dA_Tar >= P_Pos->Info.Relative_Angle + HALF_PI_ANGLE) && (dA_Tar <= P_Pos->Info.Relative_Angle + HALF_PI_ANGLE + PI_ANGLE)){
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



#include "move.h"
#include "robo_base.h"

void Move_Analysis(ROBO_BASE* Robo)
{
  float Speed=sqrt(pow((double)Robo->Speed_X,2)+pow((double)Robo->Speed_Y,2));
  float E_Angle=0;

  Robo->Angle=ToDegree(atan2((double)Robo->Speed_X,(double)Robo->Speed_Y));
	
  if(Robo->Angle<0) Robo->Angle+=360;

  E_Angle=Robo->Angle-Robo->Pos_MotorLF.Info.Relative_Angle;
  if(E_Angle<-180) E_Angle+=360; else if(E_Angle>180) E_Angle-=360;

  MotorGroup_Tar_Analysis(&Robo->Pos_MotorLF,&Robo->Speed_MotorLF,E_Angle,Speed);
  MotorGroup_Tar_Analysis(&Robo->Pos_MotorRF,&Robo->Speed_MotorRF,E_Angle,Speed);
  MotorGroup_Tar_Analysis(&Robo->Pos_MotorRB,&Robo->Speed_MotorRB,E_Angle,Speed);
  MotorGroup_Tar_Analysis(&Robo->Pos_MotorLB,&Robo->Speed_MotorLB,E_Angle,Speed);

}

void MotorGroup_Tar_Analysis(Pos_System* P_Pos,Speed_System* P_Speed,float E_Angle,float Speed)
{
  float Tar_Angle=0;
  
  if(P_Pos->Info.Abs_Angle>500000||P_Pos->Info.Abs_Angle<-500000) P_Pos->Info.Abs_Angle=0;
  if(E_Angle<90&&E_Angle>-90)
  {
	Tar_Angle=E_Angle/360*ONE_CIRCLE;
    P_Pos->Tar_Pos=Tar_Angle+P_Pos->Info.Abs_Angle;
	P_Speed->Tar_Speed=Speed;
  }
  else if(E_Angle>=90)
  {
    Tar_Angle=(E_Angle-180)/360*ONE_CIRCLE;
    P_Pos->Tar_Pos=Tar_Angle+P_Pos->Info.Abs_Angle;
	P_Speed->Tar_Speed=-Speed;
  }else if(E_Angle<=-90)
  {
    Tar_Angle=(180+E_Angle)/360*ONE_CIRCLE;
    P_Pos->Tar_Pos=Tar_Angle+P_Pos->Info.Abs_Angle;
	P_Speed->Tar_Speed=-Speed;
  }
}


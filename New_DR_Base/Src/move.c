#include "move.h"

#define TWO_PI (2*PI)
#define MAX_SPEED 10.0

extern ROBO_BASE Robo_Base;
extern RC_Ctl_t RC_Ctl;

double dA_Tar=0;
void Move_Analysis(void)
{
	static float Xbox_Angle=0;
	static float Xbox_Speed=0;

	float speed_x=(RC_Ctl.rc.ch0-1024)*1.0/660;
	float speed_y=(RC_Ctl.rc.ch1-1024)*1.0/660;
	if(speed_x!=0||speed_y!=0) Xbox_Angle=atan2(speed_x,speed_y);
	Xbox_Speed=sqrt((speed_x*speed_x)+(speed_y*speed_y))*MAX_SPEED;
  if(Xbox_Angle>=0) dA_Tar=Xbox_Angle/TWO_PI*ROTOR_ANGLE*GEAR_RATIO;
  else dA_Tar=(Xbox_Angle+TWO_PI)/TWO_PI*ROTOR_ANGLE*GEAR_RATIO;
	
	Robo_Base.LF._Axis->Input_Vel=Xbox_Speed;
	Robo_Base.LB._Axis->Input_Vel=Xbox_Speed;
	Robo_Base.RF._Axis->Input_Vel=Xbox_Speed;
	Robo_Base.RB._Axis->Input_Vel=Xbox_Speed;
	Motor_Angle(&Robo_Base.LF,dA_Tar);
	Motor_Angle(&Robo_Base.LB,dA_Tar);
	Motor_Angle(&Robo_Base.RF,dA_Tar);
	Motor_Angle(&Robo_Base.RB,dA_Tar);
}

void Motor_Angle(Motor_Group* P_Motor,double dA_Tar)
{
	Pos_System* P_Pos=&P_Motor->_Pos;
	float* ODrive_Tar=&P_Motor->_Axis->Input_Vel;
  double Angle_Temp=abs(P_Pos->Info.Relative_Angle);
    if(P_Pos->Info.Relative_Angle<0) P_Pos->Info.Relative_Angle+=2*PI_ANGLE;
		if(P_Pos->Info.Relative_Angle>HALF_PI_ANGLE&&P_Pos->Info.Relative_Angle<HALF_PI_ANGLE+PI_ANGLE) //?????????????????
      if((dA_Tar>P_Pos->Info.Relative_Angle-HALF_PI_ANGLE)&&(dA_Tar<P_Pos->Info.Abs_Angle+HALF_PI_ANGLE)){
        P_Pos->Tar_Pos=dA_Tar-P_Pos->Info.Relative_Angle+P_Pos->Info.Abs_Angle;           //???????????
      }else if(dA_Tar<P_Pos->Info.Relative_Angle){
         P_Pos->Tar_Pos=dA_Tar-P_Pos->Info.Relative_Angle+P_Pos->Info.Abs_Angle+PI_ANGLE,*ODrive_Tar*=-1;      //??????180
      }else P_Pos->Tar_Pos=dA_Tar-P_Pos->Info.Relative_Angle+P_Pos->Info.Abs_Angle-PI_ANGLE,*ODrive_Tar*=-1;   //??????180
    else if(P_Pos->Info.Relative_Angle<HALF_PI_ANGLE){                         //??????????€???
      if((dA_Tar>P_Pos->Info.Relative_Angle+HALF_PI_ANGLE)&&(dA_Tar<P_Pos->Info.Relative_Angle+HALF_PI_ANGLE+PI_ANGLE)){
        P_Pos->Tar_Pos=dA_Tar-P_Pos->Info.Relative_Angle+P_Pos->Info.Abs_Angle-PI_ANGLE,*ODrive_Tar*=-1;       //??????180
      }else if(dA_Tar<P_Pos->Info.Relative_Angle+HALF_PI_ANGLE){
        P_Pos->Tar_Pos=dA_Tar-P_Pos->Info.Relative_Angle+P_Pos->Info.Abs_Angle;           //??????????? ????????
      }else P_Pos->Tar_Pos=dA_Tar-2*PI_ANGLE-P_Pos->Info.Relative_Angle+P_Pos->Info.Abs_Angle;//??????????? ?€?????
    }else{                                                      //??????????????
      if((dA_Tar<P_Pos->Info.Relative_Angle-HALF_PI_ANGLE)&&(dA_Tar>P_Pos->Info.Relative_Angle-HALF_PI_ANGLE-PI_ANGLE)){
        P_Pos->Tar_Pos=dA_Tar-P_Pos->Info.Relative_Angle+P_Pos->Info.Abs_Angle+PI_ANGLE,*ODrive_Tar*=-1;       //??????180
      }else if(dA_Tar>P_Pos->Info.Relative_Angle-HALF_PI_ANGLE){
        P_Pos->Tar_Pos=dA_Tar-P_Pos->Info.Relative_Angle+P_Pos->Info.Abs_Angle;           //??????????? ????????
      }else P_Pos->Tar_Pos=2*PI_ANGLE+dA_Tar-P_Pos->Info.Relative_Angle+P_Pos->Info.Abs_Angle;//??????????? ?€?????
		}
}


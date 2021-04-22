#include "move.h"

#define TWO_PI (2*PI)
#define MAX_SPEED 10.0

extern ROBO_BASE Robo_Base;
extern ODrive ODrive0;
extern ODrive ODrive1;
extern RC_Ctl_t RC_Ctl;

double dA_Tar=0;
void Motor_Speed(void)
{
	double Xbox_Angle=0;
	double Xbox_Speed=0;

	
	float speed_x=(RC_Ctl.rc.ch0-1024)*1.0/660;
	float speed_y=(RC_Ctl.rc.ch1-1024)*1.0/660;
  Xbox_Angle=atan2(speed_x,speed_y);
  Xbox_Speed=sqrt((speed_x*speed_x)+(speed_y*speed_y))*MAX_SPEED;
  if(Xbox_Angle>=0) dA_Tar=Xbox_Angle/TWO_PI*ROTOR_ANGLE*GEAR_RATIO;
  else dA_Tar=(Xbox_Angle+TWO_PI)/TWO_PI*ROTOR_ANGLE*GEAR_RATIO;
	
	ODrive0.Axis0.Input_Vel=Xbox_Speed;
	ODrive0.Axis1.Input_Vel=Xbox_Speed;
	ODrive1.Axis0.Input_Vel=Xbox_Speed;
	ODrive1.Axis1.Input_Vel=Xbox_Speed;
	Motor_Angle(&Robo_Base.Pos_MotorLF,&ODrive0.Axis0.Input_Vel,dA_Tar);
}

void Motor_Angle(Pos_System* P_Pos,float* ODrive_Tar,double dA_Tar)
{
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


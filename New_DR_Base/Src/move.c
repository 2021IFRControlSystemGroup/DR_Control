#include "move.h"

#define TWO_PI (2*PI)
#define MAX_SPEED 10.0

extern ROBO_BASE Robo_Base;
extern ODrive ODrive0;
extern ODrive ODrive1;
extern RC_Ctl_t RC_Ctl;

void Motor_Speed(void)
{
	double Xbox_Angle=0;
	double Xbox_Speed=0;
  double dA_Tar=0;
	
	float speed_x=(RC_Ctl.rc.ch0-1024)*1.0/660;
	float speed_y=(RC_Ctl.rc.ch1-1024)*1.0/660;
  Xbox_Angle=atan2(speed_x,speed_y);
  Xbox_Speed=sqrt((speed_x*speed_x)+(speed_y*speed_y))*MAX_SPEED;
  if(Xbox_Angle>0) dA_Tar=Xbox_Angle;
  else dA_Tar=Xbox_Angle+TWO_PI;
	
	ODrive0.Axis0.Input_Vel=Xbox_Speed;
	ODrive0.Axis1.Input_Vel=Xbox_Speed;
	ODrive1.Axis0.Input_Vel=Xbox_Speed;
	ODrive1.Axis1.Input_Vel=Xbox_Speed;
	//Motor_Angle(&Robo_Base.Pos_MotorLB,&ODrive0.Axis0.Input_Vel,dA_Tar);
}

void Motor_Angle(Pos_System* P_Pos,int32_t* ODrive_Tar,uint16_t dA_Tar)
{
  double Angle_Temp=abs(P_Pos->Position);
    if(P_Pos->Position>=0){
      if(P_Pos->Position>HALF_PI&&P_Pos->Position<HALF_PI+PI) //?????????????????
        if((dA_Tar>P_Pos->Position-HALF_PI)&&(dA_Tar<P_Pos->Angle+HALF_PI)){
          P_Pos->Tar_Pos=dA_Tar-P_Pos->Position+P_Pos->Angle;           //???????????
        }else if(dA_Tar<P_Pos->Position){
           P_Pos->Tar_Pos=dA_Tar-P_Pos->Position+P_Pos->Angle+PI,*ODrive_Tar*=-1;      //??????180
        }else P_Pos->Tar_Pos=dA_Tar-P_Pos->Position+P_Pos->Angle-PI,*ODrive_Tar*=-1;   //??????180
      else if(P_Pos->Position<HALF_PI){                         //??????????€???
        if((dA_Tar>P_Pos->Position+HALF_PI)&&(dA_Tar<P_Pos->Position+HALF_PI+PI)){
          P_Pos->Tar_Pos=dA_Tar-P_Pos->Position+P_Pos->Angle-PI,*ODrive_Tar*=-1;       //??????180
        }else if(dA_Tar<P_Pos->Position+HALF_PI){
          P_Pos->Tar_Pos=dA_Tar-P_Pos->Position+P_Pos->Angle;           //??????????? ????????
        }else P_Pos->Tar_Pos=dA_Tar-TWO_PI-P_Pos->Position+P_Pos->Angle;//??????????? ?€?????
      }else{                                                      //??????????????
        if((dA_Tar<P_Pos->Position-HALF_PI)&&(dA_Tar>P_Pos->Position-HALF_PI-PI)){
          P_Pos->Tar_Pos=dA_Tar-P_Pos->Position+P_Pos->Angle+PI,*ODrive_Tar*=-1;       //??????180
        }else if(dA_Tar>P_Pos->Position-HALF_PI){
          P_Pos->Tar_Pos=dA_Tar-P_Pos->Position+P_Pos->Angle;           //??????????? ????????
        }else P_Pos->Tar_Pos=TWO_PI+dA_Tar-P_Pos->Position+P_Pos->Angle;//??????????? ?€?????
      }
    }else{
			P_Pos->Position+=TWO_PI;
			if(P_Pos->Position>HALF_PI&&P_Pos->Position<HALF_PI+PI) //?????????????????
        if((dA_Tar>P_Pos->Position-HALF_PI)&&(dA_Tar<P_Pos->Angle+HALF_PI)){
          P_Pos->Tar_Pos=dA_Tar-P_Pos->Position+P_Pos->Angle;           //???????????
        }else if(dA_Tar<P_Pos->Position){
           P_Pos->Tar_Pos=dA_Tar-P_Pos->Position+P_Pos->Angle+PI,*ODrive_Tar*=-1;      //??????180
        }else P_Pos->Tar_Pos=dA_Tar-P_Pos->Position+P_Pos->Angle-PI,*ODrive_Tar*=-1;   //??????180
      else if(P_Pos->Position<HALF_PI){                         //??????????€???
        if((dA_Tar>P_Pos->Position+HALF_PI)&&(dA_Tar<P_Pos->Position+HALF_PI+PI)){
          P_Pos->Tar_Pos=dA_Tar-P_Pos->Position+P_Pos->Angle-PI,*ODrive_Tar*=-1;       //??????180
        }else if(dA_Tar<P_Pos->Position+HALF_PI){
          P_Pos->Tar_Pos=dA_Tar-P_Pos->Position+P_Pos->Angle;           //??????????? ????????
        }else P_Pos->Tar_Pos=dA_Tar-TWO_PI-P_Pos->Position+P_Pos->Angle;//??????????? ?€?????
      }else{                                                      //??????????????
        if((dA_Tar<P_Pos->Position-HALF_PI)&&(dA_Tar>P_Pos->Position-HALF_PI-PI)){
          P_Pos->Tar_Pos=dA_Tar-P_Pos->Position+P_Pos->Angle+PI,*ODrive_Tar*=-1;       //??????180
        }else if(dA_Tar>P_Pos->Position-HALF_PI){
          P_Pos->Tar_Pos=dA_Tar-P_Pos->Position+P_Pos->Angle;           //??????????? ????????
        }else P_Pos->Tar_Pos=TWO_PI+dA_Tar-P_Pos->Position+P_Pos->Angle;//??????????? ?€?????
      }
		}
}


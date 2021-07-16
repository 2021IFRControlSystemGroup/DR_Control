#include "move.h"
#include "iwdg.h"
#include "arm_math.h"

#define TWO_PI (2 * PI)
#define MAX_SPEED 20.0f
#define MAX_ROTATE 20.0f

DirectSystem Direct_System;
float Angle_Tar[4] = {0};
float Speed_Tar[4] = {0};
float Angle_XY = 0;
float Speed_XY = 0;
float Radius = 0;

void DirectSystem_Init(void)
{
    Direct_System.IMU = &IMU;
    PID_Init(&Direct_System.Dir_Pos_Pid, 0.2, 0, 0 , PI, 0, 0, 0.8);
    Direct_System.Direction_Tar = 0;
    Direct_System.Tar_Update = SET;
}
float COS_Angle = 0;
float SIN_Angle = 0;
void Move_Analysis(float Vel_X, float Vel_Y, float Vel_W)
{
    arm_sqrt_f32((Vel_X * Vel_X) + (Vel_Y * Vel_Y), &Speed_XY);
    Angle_XY = Robo_Base.Angle;
    float Sqrt_Res = 0;
    COS_Angle = arm_cos_f32(Angle_XY);
    SIN_Angle = arm_sin_f32(Angle_XY);
    
    if(Direct_System.IMU->Yaw < 0) IMU.Yaw += TWO_PI;
    if(Vel_W == 0 && Direct_System.Tar_Update == SET){
        Direct_System.Direction_Tar =  Direct_System.IMU->Yaw;
        Direct_System.Tar_Update = RESET;
    }else if(Vel_W == 0 && Direct_System.Tar_Update == RESET) 
        if(Direct_System.IMU->Yaw < HALF_PI && Direct_System.Direction_Tar > TWO_PI - HALF_PI)
            PID_General_Cal(&Direct_System.Dir_Pos_Pid,Direct_System.IMU->Yaw,Direct_System.Direction_Tar - TWO_PI);
        else if(Direct_System.Direction_Tar < HALF_PI && Direct_System.IMU->Yaw > TWO_PI - HALF_PI)
            PID_General_Cal(&Direct_System.Dir_Pos_Pid,Direct_System.IMU->Yaw - TWO_PI,Direct_System.Direction_Tar);
        else
            PID_General_Cal(&Direct_System.Dir_Pos_Pid,Direct_System.IMU->Yaw, Direct_System.Direction_Tar);
    else if(Vel_W != 0) Direct_System.Tar_Update = SET;
    
    if(Vel_W == 0 && (Vel_X != 0 || Vel_Y != 0)) Vel_W += Direct_System.Dir_Pos_Pid.output;
    if(Vel_W == 0 && (Vel_X != 0 || Vel_Y != 0)){
        //Radius = INFINITY;
        HAL_IWDG_Refresh(&hiwdg);
        Angle_Tar[Robo_Base.LF._Pos.Motor_Num] = Angle_XY / TWO_PI * ONE_CIRCLE;
        Angle_Tar[Robo_Base.LB._Pos.Motor_Num] = Angle_XY / TWO_PI * ONE_CIRCLE;
        Angle_Tar[Robo_Base.RF._Pos.Motor_Num] = Angle_XY / TWO_PI * ONE_CIRCLE;
        Angle_Tar[Robo_Base.RB._Pos.Motor_Num] = Angle_XY / TWO_PI * ONE_CIRCLE;
        Speed_Tar[Robo_Base.LF._Pos.Motor_Num] = Speed_XY * MAX_SPEED / (0.205f * TWO_PI);
        Speed_Tar[Robo_Base.LB._Pos.Motor_Num] = Speed_XY * MAX_SPEED / (0.205f * TWO_PI);
        Speed_Tar[Robo_Base.RF._Pos.Motor_Num] = Speed_XY * MAX_SPEED / (0.205f * TWO_PI);
        Speed_Tar[Robo_Base.RB._Pos.Motor_Num] = Speed_XY * MAX_SPEED / (0.205f * TWO_PI);
    }else if((Vel_X == 0 && Vel_Y == 0) && Vel_W != 0){
        Radius = 0;
        HAL_IWDG_Refresh(&hiwdg);
        Angle_Tar[Robo_Base.LF._Pos.Motor_Num] = (TWO_PI - HALF_PI / 2) / TWO_PI * ONE_CIRCLE;
        Angle_Tar[Robo_Base.LB._Pos.Motor_Num] = (HALF_PI / 2) / TWO_PI * ONE_CIRCLE;
        Angle_Tar[Robo_Base.RF._Pos.Motor_Num] = (PI + HALF_PI / 2) / TWO_PI * ONE_CIRCLE;
        Angle_Tar[Robo_Base.RB._Pos.Motor_Num] = (PI - HALF_PI / 2) / TWO_PI * ONE_CIRCLE;
        Speed_Tar[Robo_Base.LF._Pos.Motor_Num] = -Vel_W * 1.414f / 2 * 0.205f * MAX_ROTATE / (0.205f * TWO_PI);
        Speed_Tar[Robo_Base.LB._Pos.Motor_Num] = -Vel_W * 1.414f / 2 * 0.205f * MAX_ROTATE / (0.205f * TWO_PI);
        Speed_Tar[Robo_Base.RF._Pos.Motor_Num] = -Vel_W * 1.414f / 2 * 0.205f * MAX_ROTATE / (0.205f * TWO_PI);
        Speed_Tar[Robo_Base.RB._Pos.Motor_Num] = -Vel_W * 1.414f / 2 * 0.205f * MAX_ROTATE / (0.205f * TWO_PI);
    }else if((Vel_X != 0 || Vel_Y != 0) && Vel_W != 0){
        HAL_IWDG_Refresh(&hiwdg);
        if(Vel_W != 0) Radius = Speed_XY / Vel_W;
        if((Radius * COS_Angle + 0.205f) != 0 && (Radius * SIN_Angle + 0.205f) != 0) 
            Angle_Tar[Robo_Base.RF._Pos.Motor_Num] = (HALF_PI - atan2((Radius * COS_Angle + 0.205f) , (Radius * SIN_Angle + 0.205f))) / TWO_PI * ONE_CIRCLE;
        if((Radius * COS_Angle + 0.205f) != 0 && (-Radius * SIN_Angle + 0.205f) != 0) 
            Angle_Tar[Robo_Base.RB._Pos.Motor_Num] = (HALF_PI + atan2((Radius * COS_Angle + 0.205f) , ((-Radius) * SIN_Angle + 0.205f))) / TWO_PI * ONE_CIRCLE;
        if((Radius * COS_Angle - 0.205f) != 0 && (Radius * SIN_Angle + 0.205f) != 0) 
            Angle_Tar[Robo_Base.LF._Pos.Motor_Num] = (HALF_PI - atan2((Radius * COS_Angle - 0.205f) , (Radius * SIN_Angle + 0.205f))) / TWO_PI * ONE_CIRCLE;
        if((Radius * COS_Angle - 0.205f) != 0 && (-Radius * SIN_Angle + 0.205f) != 0) 
            Angle_Tar[Robo_Base.LB._Pos.Motor_Num] = (HALF_PI + atan2((Radius * COS_Angle - 0.205f) , (-Radius * SIN_Angle + 0.205f))) / TWO_PI * ONE_CIRCLE;
        arm_sqrt_f32((0.205f + Radius * SIN_Angle) * (0.205f + Radius * SIN_Angle) + (0.205f + Radius * COS_Angle) * (0.205f + Radius * COS_Angle), &Sqrt_Res);
        Speed_Tar[Robo_Base.LF._Pos.Motor_Num] = Vel_W * MAX_SPEED * Sqrt_Res;
        arm_sqrt_f32((0.205f - Radius * SIN_Angle) * (0.205f - Radius * SIN_Angle) + (0.205f + Radius * COS_Angle) * (0.205f + Radius * COS_Angle), &Sqrt_Res);
        Speed_Tar[Robo_Base.LB._Pos.Motor_Num] = -Vel_W * MAX_SPEED * Sqrt_Res;
        arm_sqrt_f32((0.205f - Radius * SIN_Angle) * (0.205f - Radius * SIN_Angle) + (-0.205f + Radius * COS_Angle) * (-0.205f + Radius * COS_Angle), &Sqrt_Res);
        Speed_Tar[Robo_Base.RF._Pos.Motor_Num] = Vel_W * MAX_SPEED * Sqrt_Res;
        arm_sqrt_f32((0.205f + Radius * SIN_Angle) * (0.205f + Radius * SIN_Angle) + (-0.205f + Radius * COS_Angle) * (-0.205f + Radius * COS_Angle), &Sqrt_Res);
        Speed_Tar[Robo_Base.RB._Pos.Motor_Num] = -Vel_W * MAX_SPEED * Sqrt_Res;
    }else{
        HAL_IWDG_Refresh(&hiwdg);
        Angle_Tar[Robo_Base.LF._Pos.Motor_Num] = 0;
        Angle_Tar[Robo_Base.LB._Pos.Motor_Num] = 0;
        Angle_Tar[Robo_Base.RF._Pos.Motor_Num] = 0;
        Angle_Tar[Robo_Base.RB._Pos.Motor_Num] = 0;
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

void Motor_Angle(MotorGroup* P_Group,float Angle_Tar)
{
	MotorSystem* P_Pos = &P_Group->_Pos;
	float* ODrive_Tar = &P_Group->_Axis->Input_Vel;
    float Angle_Temp = abs(P_Pos->Info.Relative_Angle);
    float Relative_Angle_temp = P_Pos->Info.Relative_Angle;
    
    if(P_Pos->Info.Relative_Angle < 0) P_Pos->Info.Relative_Angle += (2 * PI_ANGLE);
    if(P_Pos->Info.Relative_Angle >= HALF_PI_ANGLE && P_Pos->Info.Relative_Angle <= HALF_PI_ANGLE + PI_ANGLE)
      if((Angle_Tar >= P_Pos->Info.Relative_Angle - HALF_PI_ANGLE) && (Angle_Tar <= P_Pos->Info.Relative_Angle + HALF_PI_ANGLE)){
        P_Pos->Tar_Pos = Angle_Tar - P_Pos->Info.Relative_Angle + P_Pos->Info.Abs_Angle;
      }else if(Angle_Tar <= P_Pos->Info.Relative_Angle){
         P_Pos->Tar_Pos = Angle_Tar - P_Pos->Info.Relative_Angle + P_Pos->Info.Abs_Angle + PI_ANGLE, *ODrive_Tar *= -1;
      }else P_Pos->Tar_Pos = Angle_Tar - P_Pos->Info.Relative_Angle + P_Pos->Info.Abs_Angle - PI_ANGLE, *ODrive_Tar *= -1;
    else if(P_Pos->Info.Relative_Angle <= HALF_PI_ANGLE){
      if((Angle_Tar >= P_Pos->Info.Relative_Angle + HALF_PI_ANGLE) && (Angle_Tar <= P_Pos->Info.Relative_Angle + HALF_PI_ANGLE + PI_ANGLE)){
        P_Pos->Tar_Pos = Angle_Tar - P_Pos->Info.Relative_Angle + P_Pos->Info.Abs_Angle - PI_ANGLE, *ODrive_Tar *= -1;
      }else if(Angle_Tar < P_Pos->Info.Relative_Angle + HALF_PI_ANGLE){
        P_Pos->Tar_Pos = Angle_Tar - P_Pos->Info.Relative_Angle + P_Pos->Info.Abs_Angle;
      }else P_Pos->Tar_Pos = Angle_Tar - 2 * PI_ANGLE - P_Pos->Info.Relative_Angle + P_Pos->Info.Abs_Angle;
    }else{
      if((Angle_Tar <= P_Pos->Info.Relative_Angle - HALF_PI_ANGLE) && (Angle_Tar >= P_Pos->Info.Relative_Angle - HALF_PI_ANGLE - PI_ANGLE)){
        P_Pos->Tar_Pos = Angle_Tar - P_Pos->Info.Relative_Angle + P_Pos->Info.Abs_Angle + PI_ANGLE, *ODrive_Tar *= -1;
      }else if(Angle_Tar > P_Pos->Info.Relative_Angle - HALF_PI_ANGLE){
        P_Pos->Tar_Pos = Angle_Tar - P_Pos->Info.Relative_Angle + P_Pos->Info.Abs_Angle;
      }else P_Pos->Tar_Pos = 2 * PI_ANGLE + Angle_Tar - P_Pos->Info.Relative_Angle + P_Pos->Info.Abs_Angle;
    }P_Pos->Info.Relative_Angle = Relative_Angle_temp;
}



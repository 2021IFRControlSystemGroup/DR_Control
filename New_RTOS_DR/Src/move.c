#include "move.h"

#define TWO_PI (2*PI)
#define MAX_SPEED 10.0
#define MAX_ROTATION 10.0

double Angle_XY = 0;
double Vel_XY = 0;
double Angle_Rotate = 0;
double Distance_Rotate = 0;
double Length_X_Rotate = 0;
double Length_Y_Rotate = 0;
double Vel_Rotate = 0;
void Move_Analysis(double Vel_X, double Vel_Y, double Vel_W)
{   
	Vel_XY = sqrt((Vel_X * Vel_X) + (Vel_Y * Vel_Y)) * MAX_SPEED;
    Vel_Rotate = Vel_W * MAX_ROTATION;
    Angle_XY = atan2(Vel_X, Vel_Y);
    if(Angle_XY < 0) Angle_XY += PI;
    
    if(Vel_W != 0){
        Distance_Rotate = Vel_XY/Vel_W;
        Length_X_Rotate = cos(Angle_XY) * Distance_Rotate;
        Length_Y_Rotate = sin(Angle_XY) * Distance_Rotate;
        if(Distance_Rotate == 0) Vel_XY = Vel_Rotate;
        if(Distance_Rotate < 0) Distance_Rotate = -Distance_Rotate;
    }else{
        Distance_Rotate = (double)INT_LEAST32_MAX;
        Length_X_Rotate = (double)INT_LEAST32_MAX;
        Length_Y_Rotate = (double)INT_LEAST32_MAX;;
    }
    
	Pos_Motor_Turn(&Robo_Base.LF, Angle_XY, Vel_XY, Length_X_Rotate, Length_Y_Rotate, Distance_Rotate);
	Pos_Motor_Turn(&Robo_Base.LB, Angle_XY, Vel_XY, Length_X_Rotate, Length_Y_Rotate, Distance_Rotate);
	Pos_Motor_Turn(&Robo_Base.RF, Angle_XY, Vel_XY, Length_X_Rotate, Length_Y_Rotate, Distance_Rotate);
	Pos_Motor_Turn(&Robo_Base.RB, Angle_XY, Vel_XY, Length_X_Rotate, Length_Y_Rotate, Distance_Rotate);
    
    Robo_Base.LF._Axis->Input_Vel *= -1;
	Robo_Base.LB._Axis->Input_Vel *= -1;
	Robo_Base.RF._Axis->Input_Vel *= 1;
	Robo_Base.RB._Axis->Input_Vel *= 1;
}

void Pos_Motor_Turn(MotorGroup* P_Motor, double Angle_Tar , double Vel_Tar, double Length_X_Rotate, double Length_Y_Rotate, double Distance_Rotate)
{
	MotorSystem* P_Pos = &P_Motor->_Pos;
    Axis* P_Axis = P_Motor->_Axis;
    
    //四轮舵轮运动学解算
    double Angle_Temp = abs(P_Pos->Info.Relative_Angle);
    double Relative_Angle_temp = P_Pos->Info.Relative_Angle;
    Angle_Rotate = atan2(P_Motor->_Pos.Length_X - Length_X_Rotate, P_Motor->_Pos.Length_Y - Length_Y_Rotate);
    Angle_Tar += Angle_Rotate; Angle_Tar /= TWO_PI; Angle_Tar *= ONE_CIRCLE;
    if(Distance_Rotate <= INT_LEAST32_MAX / 2 && Distance_Rotate != 0) 
        Vel_Tar *= sqrt((P_Pos->Length_X - Length_X_Rotate)*(P_Pos->Length_X - Length_X_Rotate) + (P_Pos->Length_Y - Length_Y_Rotate)*(P_Pos->Length_Y - Length_Y_Rotate)) / Distance_Rotate;
    P_Axis->Input_Vel = Vel_Tar;
    
    //保证角度旋转不会超过90°的算法
    if(P_Pos->Info.Relative_Angle < 0) P_Pos->Info.Relative_Angle += (2 * PI_ANGLE);
    if(P_Pos->Info.Relative_Angle >= HALF_PI_ANGLE && P_Pos->Info.Relative_Angle <= HALF_PI_ANGLE + PI_ANGLE)
      if((Angle_Tar >= P_Pos->Info.Relative_Angle - HALF_PI_ANGLE) && (Angle_Tar <= P_Pos->Info.Relative_Angle + HALF_PI_ANGLE)){
        P_Pos->Tar_Pos = Angle_Tar - P_Pos->Info.Relative_Angle + P_Pos->Info.Abs_Angle;
      }else if(Angle_Tar <= P_Pos->Info.Relative_Angle){
         P_Pos->Tar_Pos = Angle_Tar - P_Pos->Info.Relative_Angle + P_Pos->Info.Abs_Angle + PI_ANGLE, P_Axis->Input_Vel *= -1;
      }else P_Pos->Tar_Pos = Angle_Tar - P_Pos->Info.Relative_Angle + P_Pos->Info.Abs_Angle - PI_ANGLE, P_Axis->Input_Vel *= -1;
    else if(P_Pos->Info.Relative_Angle <= HALF_PI_ANGLE){
      if((Angle_Tar >= P_Pos->Info.Relative_Angle + HALF_PI_ANGLE) && (Angle_Tar <= P_Pos->Info.Relative_Angle + HALF_PI_ANGLE + PI_ANGLE)){
        P_Pos->Tar_Pos = Angle_Tar - P_Pos->Info.Relative_Angle + P_Pos->Info.Abs_Angle - PI_ANGLE, P_Axis->Input_Vel *= -1;
      }else if(Angle_Tar < P_Pos->Info.Relative_Angle + HALF_PI_ANGLE){
        P_Pos->Tar_Pos = Angle_Tar - P_Pos->Info.Relative_Angle + P_Pos->Info.Abs_Angle;
      }else P_Pos->Tar_Pos = Angle_Tar - 2 * PI_ANGLE - P_Pos->Info.Relative_Angle + P_Pos->Info.Abs_Angle;
    }else{
      if((Angle_Tar <= P_Pos->Info.Relative_Angle - HALF_PI_ANGLE) && (Angle_Tar >= P_Pos->Info.Relative_Angle - HALF_PI_ANGLE - PI_ANGLE)){
        P_Pos->Tar_Pos = Angle_Tar - P_Pos->Info.Relative_Angle + P_Pos->Info.Abs_Angle + PI_ANGLE, P_Axis->Input_Vel *= -1;
      }else if(Angle_Tar > P_Pos->Info.Relative_Angle - HALF_PI_ANGLE){
        P_Pos->Tar_Pos = Angle_Tar - P_Pos->Info.Relative_Angle + P_Pos->Info.Abs_Angle;
      }else P_Pos->Tar_Pos = 2 * PI_ANGLE + Angle_Tar - P_Pos->Info.Relative_Angle + P_Pos->Info.Abs_Angle;
    }P_Pos->Info.Relative_Angle = Relative_Angle_temp;
    
}

void W_Move(double Vel_XY, double Angle_XY, double Vel_W, double B_Length, double A_Length, double IMU_Yaw, double Tar_Angle)
{
    double Distance_Rotate = Vel_XY/Vel_W;
    double Angle_Rotate = asin((B_Length - sin(IMU_Yaw) * Distance_Rotate) / (A_Length - cos(IMU_Yaw)));
    Tar_Angle += Angle_Rotate;
}

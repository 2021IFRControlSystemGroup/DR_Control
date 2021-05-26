//文件名称:		motor_system.c
//对应头文件:	motor_system.h
//主要功能:
//		把电机数据/看门狗/PID控制抽象成一个系统, 并且提取出该系统的基本操作方法.
//		实现对电机的数据读取和分析, PID参数初始化, PID计算, 看门狗设置等方法
//
//时间:
//		2020/11/27
//
//版本:	1,0V
//
//状态: 待测试
//
//测试内容:
//		能够正常读取电机信息, 并且实现PID控制, 看门狗功能没有问题.

//---------头文件引用部分---------//
#include "motor_system.h"
//--------------------------------//

//---------变量声明部分-----------//
//--------------------------------//

//---------外部变量声明部分-------//
//--------------------------------//

void Motor_Info_Analysis(MotorInfo* P_Motor, uint8_t* RX_Data)
{
  //数据解析
  P_Motor->Angle = (uint16_t)RX_Data[0] << 8 | RX_Data[1];
  P_Motor->Speed = (uint16_t)RX_Data[2] << 8 | RX_Data[3];
  P_Motor->Electric = (uint16_t)RX_Data[4] << 8 | RX_Data[5];
  P_Motor->Temperature = RX_Data[6];

  //绝对角度计算
  if (P_Motor->Speed != 0){
        int16_t Error = P_Motor->Angle-P_Motor->Last_Angle;
        P_Motor->Abs_Angle += Error;
        if (Error < -4096)P_Motor->Abs_Angle += 8192;
        else if (Error > 4096)  P_Motor->Abs_Angle -= 8192;
    }P_Motor->Last_Angle = P_Motor->Angle;
    P_Motor->Circle_Num = P_Motor->Abs_Angle / (GEAR_RATIO * ROTOR_ANGLE);
    P_Motor->Relative_Angle = P_Motor->Abs_Angle - P_Motor->Circle_Num * GEAR_RATIO * ROTOR_ANGLE;
}

void PID_Init(PID *pid, float Kp, float Ki, float Kd, float error_max, float dead_line, float intergral_max, float output_max)
{
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->error_max = error_max;
	pid->output_max = output_max;
	pid->dead_line = dead_line;
	
	pid->intergral_max = intergral_max;
	
	pid->error = 0;
	pid->error_last = 0;
	pid->intergral = 0;
	pid->derivative = 0;
	pid->output = 0;
}

void PID_General_Cal(PID *pid, float fdbV, float tarV,uint8_t moto_num,uint8_t *Tx_msg)
{

	pid->error =  tarV - fdbV;
	if(pid->error > pid->error_max)
		pid->error = pid->error_max;
	if(pid->error < -pid->error_max)
		pid->error = -pid->error_max;
	if(pid->error > 0 && pid->error < pid->dead_line)
		pid->error = 0;
	if(pid->error < 0 && pid->error > pid->dead_line)
		pid->error = 0;
	
	pid->intergral = pid->intergral + pid->error;
	if(pid->intergral > pid->intergral_max)
		pid->intergral = pid->intergral_max;
	if(pid->intergral < -pid->intergral_max)
		pid->intergral = -pid->intergral_max;
	
	pid->derivative = pid->error - pid->error_last;
	pid->error_last = pid->error;
	
	pid->output = pid->Kp*pid->error + pid->Ki*pid->intergral + pid->Kd*pid->derivative;
	
	if(pid->output > pid->output_max)
		pid->output = pid->output_max;
	if(pid->output < -pid->output_max)
		pid->output = -pid->output_max;
	
	Tx_msg[moto_num * 2]=((int16_t)pid->output) >> 8;Tx_msg[moto_num * 2 + 1]=(int16_t)pid->output;
}

void PID_Pos_Cal(MotorSystem* P_System)
{
	P_System->Pos_PID.error =  P_System->Tar_Pos - P_System->Info.Abs_Angle;
	if(P_System->Pos_PID.error > P_System->Pos_PID.error_max)
		P_System->Pos_PID.error = P_System->Pos_PID.error_max;
	if(P_System->Pos_PID.error < -P_System->Pos_PID.error_max)
		P_System->Pos_PID.error = -P_System->Pos_PID.error_max;
	if(P_System->Pos_PID.error > 0 && P_System->Pos_PID.error < P_System->Pos_PID.dead_line)
		P_System->Pos_PID.error = 0;
	if(P_System->Pos_PID.error < 0 && P_System->Pos_PID.error > P_System->Pos_PID.dead_line)
		P_System->Pos_PID.error = 0;
	
	P_System->Pos_PID.intergral = P_System->Pos_PID.intergral + P_System->Pos_PID.error;
	if(P_System->Pos_PID.intergral > P_System->Pos_PID.intergral_max)
		P_System->Pos_PID.intergral = P_System->Pos_PID.intergral_max;
	if(P_System->Pos_PID.intergral < -P_System->Pos_PID.intergral_max)
		P_System->Pos_PID.intergral = -P_System->Pos_PID.intergral_max;
	
	P_System->Pos_PID.derivative = P_System->Pos_PID.error - P_System->Pos_PID.error_last;
	P_System->Pos_PID.error_last = P_System->Pos_PID.error;
	
	P_System->Pos_PID.output = P_System->Pos_PID.Kp*P_System->Pos_PID.error + P_System->Pos_PID.Ki*P_System->Pos_PID.intergral + P_System->Pos_PID.Kd*P_System->Pos_PID.derivative;
	
	if(P_System->Pos_PID.output > P_System->Pos_PID.output_max)
		P_System->Pos_PID.output = P_System->Pos_PID.output_max;
	if(P_System->Pos_PID.output < -P_System->Pos_PID.output_max)
		P_System->Pos_PID.output = -P_System->Pos_PID.output_max;
	
		P_System->Speed_PID.error =  P_System->Pos_PID.output - P_System->Info.Speed;
	if(P_System->Speed_PID.error > P_System->Speed_PID.error_max)
		P_System->Speed_PID.error = P_System->Speed_PID.error_max;
	if(P_System->Speed_PID.error < -P_System->Speed_PID.error_max)
		P_System->Speed_PID.error = -P_System->Speed_PID.error_max;
	if(P_System->Speed_PID.error > 0 && P_System->Speed_PID.error < P_System->Speed_PID.dead_line)
		P_System->Speed_PID.error = 0;
	if(P_System->Speed_PID.error < 0 && P_System->Speed_PID.error > P_System->Speed_PID.dead_line)
		P_System->Speed_PID.error = 0;
	
	P_System->Speed_PID.intergral = P_System->Speed_PID.intergral + P_System->Speed_PID.error;
	if(P_System->Speed_PID.intergral > P_System->Speed_PID.intergral_max)
		P_System->Speed_PID.intergral = P_System->Speed_PID.intergral_max;
	if(P_System->Speed_PID.intergral < -P_System->Speed_PID.intergral_max)
		P_System->Speed_PID.intergral = -P_System->Speed_PID.intergral_max;
	
	P_System->Speed_PID.derivative = P_System->Speed_PID.error - P_System->Speed_PID.error_last;
	P_System->Speed_PID.error_last = P_System->Speed_PID.error;
	
	P_System->Speed_PID.output = P_System->Speed_PID.Kp*P_System->Speed_PID.error + P_System->Speed_PID.Ki*P_System->Speed_PID.intergral + P_System->Speed_PID.Kd*P_System->Speed_PID.derivative;
	
	if(P_System->Speed_PID.output > P_System->Speed_PID.output_max)
		P_System->Speed_PID.output = P_System->Speed_PID.output_max;
	if(P_System->Speed_PID.output < -P_System->Speed_PID.output_max)
		P_System->Speed_PID.output = -P_System->Speed_PID.output_max;
}

void PID_Speed_Cal(MotorSystem* P_System)
{
	P_System->Speed_PID.error =  P_System->Tar_Speed - P_System->Info.Speed;
	if(P_System->Speed_PID.error > P_System->Speed_PID.error_max)
		P_System->Speed_PID.error = P_System->Speed_PID.error_max;
	if(P_System->Speed_PID.error < -P_System->Speed_PID.error_max)
		P_System->Speed_PID.error = -P_System->Speed_PID.error_max;
	if(P_System->Speed_PID.error > 0 && P_System->Speed_PID.error < P_System->Speed_PID.dead_line)
		P_System->Speed_PID.error = 0;
	if(P_System->Speed_PID.error < 0 && P_System->Speed_PID.error > P_System->Speed_PID.dead_line)
		P_System->Speed_PID.error = 0;
	
	P_System->Speed_PID.intergral = P_System->Speed_PID.intergral + P_System->Speed_PID.error;
	if(P_System->Speed_PID.intergral > P_System->Speed_PID.intergral_max)
		P_System->Speed_PID.intergral = P_System->Speed_PID.intergral_max;
	if(P_System->Speed_PID.intergral < -P_System->Speed_PID.intergral_max)
		P_System->Speed_PID.intergral = -P_System->Speed_PID.intergral_max;
	
	P_System->Speed_PID.derivative = P_System->Speed_PID.error - P_System->Speed_PID.error_last;
	P_System->Speed_PID.error_last = P_System->Speed_PID.error;
	
	P_System->Speed_PID.output = P_System->Speed_PID.Kp*P_System->Speed_PID.error + P_System->Speed_PID.Ki*P_System->Speed_PID.intergral + P_System->Speed_PID.Kd*P_System->Speed_PID.derivative;
	
	if(P_System->Speed_PID.output > P_System->Speed_PID.output_max)
		P_System->Speed_PID.output = P_System->Speed_PID.output_max;
	if(P_System->Speed_PID.output < -P_System->Speed_PID.output_max)
		P_System->Speed_PID.output = -P_System->Speed_PID.output_max;
}

void Motor_Init(MotorSystem* P_System,uint8_t ID)
{
    P_System->Motor_Num = ID;
	
    #if CAN_FUNCTION_ABLE == ENABLE
        P_System->TxMessage=&CanTxMessageList[0];
    #endif
  
    #if PROTECT_FUNCTION_ABLE == ENABLE
        P_System->Protect.Count_Time=0;
        WorkState_Set(&P_System->Protect,INITING);
        ErrorState_Set(&P_System->Protect,0);
    #endif
}

#if CAN_FUNCTION_ANBLE == ENABLE
    void Motor_Add_Can_TxMessageList(MotorSystem* Pos_Motor)
    {
      Pos_Motor->TxMessage->Data[Pos_Motor->Motor_Num * 2] = ((int16_t)Pos_Motor->Speed_PID.output) >> 8;
        Pos_Motor->TxMessage->Data[Pos_Motor->Motor_Num * 2 + 1] = ((int16_t)Pos_Motor->Speed_PID.output);
        Pos_Motor->TxMessage->Update = SET;
    }
#endif


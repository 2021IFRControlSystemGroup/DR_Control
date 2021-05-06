//文件名称:		robo_base.c
//对应头文件:	robo_base.h
//主要功能:
//		调用motor_system封装的速度环/位置环/看门狗函数, 实现底盘控制
//
//时间:
//		2020/11/27
//
//版本:	2.0V
//
//状态:
//		已初步测试
//
//测试内容:
//		无电机情况下测试成功, 看门狗功能实现, 串口通信正常

//---------头文件引用部分---------//
#include "robo_base.h"
//--------------------------------//

//---------变量声明部分-----------//
ROBO_BASE Robo_Base;
//--------------------------------//

//---------外部变量声明部分-------//
//--------------------------------//

//--------------------------------------------------------------------------------------------------//
//函数名称:
//		位置环电机数据分析的接口函数
//
//函数功能:
//		读取Robo_Base对应的CAN口储存的数据, 根据电机号码来分辨是哪一个轮子的信息, 然后储存电机数据.
//
//参数类型:
//		uint8_t* 电机信息的数组, 推荐使用Rx_CAN变量, 这样可以不需要自己去声明.
//		uint32_t 电机号码
//
//--------------------------------------------------------------------------------------------------//
void Motor_Pos_Analysis(uint8_t* RX_Data,uint32_t Motor_Num)
{
  Pos_System* P_Motor=NULL;
  switch(Motor_Num)
  {
    case 0x201:P_Motor=&Robo_Base.LF._Pos;break;
    case 0x202:P_Motor=&Robo_Base.LB._Pos;break;
    case 0x203:P_Motor=&Robo_Base.RF._Pos;break;
    case 0x204:P_Motor=&Robo_Base.RB._Pos;break;
	default:break;
  }if(!P_Motor) return ;
  Pos_Info_Analysis(&P_Motor->Info,RX_Data);
  Feed_WatchDog(&P_Motor->Protect);
}


//--------------------------------------------------------------------------------------------------//
//函数名称:
//		速度环电机数据分析的接口函数
//
//函数功能:
//		读取Robo_Base对应的CAN口储存的数据, 根据电机号码来分辨是哪一个轮子的信息, 然后储存电机数据.
//
//参数类型:
//		ROBO_BASE 指针, 底盘结构体的指针
//		uint8_t* 电机信息的数组, 推荐使用Rx_CAN变量, 这样可以不需要自己去声明.
//		uint32_t 电机号码
//
//移植建议:
//		直接对case的数据进行修改, 有几个速度环的轮子就加几个, 然后让指针指向对应的轮子就行.
//
//--------------------------------------------------------------------------------------------------//
void Motor_Speed_Analysis(uint8_t* RX_Data,uint32_t Motor_Num)
{
  Speed_System* S_Motor=NULL;
  switch(Motor_Num)
  {
		default:break;
  }if(!S_Motor) return ;
  Speed_Info_Analysis(&S_Motor->Info,RX_Data);
  Feed_WatchDog(&S_Motor->Protect);
}

//--------------------------------------------------------------------------------------------------//
//函数名称:
//		底盘参数初始化
//
//函数功能:
//		初始化底盘所有的信息,并且把odrive的axis挂载到结构体内
//
//参数类型:
//		无
//
//--------------------------------------------------------------------------------------------------//
void BASE_Init(void)
{
	extern ODrive ODrive0;
	extern ODrive ODrive1;
	
  Pos_System* P_Pos=NULL;																																								//转向电机初始化
	P_Pos=&Robo_Base.LF._Pos; PID_Init(&P_Pos->Pos_PID,			0.3,	0,	0,	10000,	0,	0,	10000);
	Motor_Init(P_Pos,0);			PID_Init(&P_Pos->Speed_PID,			5,	0,	0,	5000,	0,	0,	4000);
  P_Pos=&Robo_Base.LB._Pos; PID_Init(&P_Pos->Pos_PID,			0,	0,	0,	0,	0,	0,	0);
  Motor_Init(P_Pos,1);			PID_Init(&P_Pos->Speed_PID,			0,	0,	0,	0,	0,	0,	0); 
  P_Pos=&Robo_Base.RF._Pos; PID_Init(&P_Pos->Pos_PID,			0,	0,	0,	0,	0,	0,	0);
  Motor_Init(P_Pos,2);			PID_Init(&P_Pos->Speed_PID,			0,	0,	0,	0,	0,	0,	0); 
  P_Pos=&Robo_Base.RB._Pos; PID_Init(&P_Pos->Pos_PID,			0,	0,	0,	0,	0,	0,	0);
  Motor_Init(P_Pos,3);			PID_Init(&P_Pos->Speed_PID,			0,	0,	0,	0,	0,	0,	0); 

	Axis* P_Axis=NULL;																																										//驱动电机初始化
	P_Axis=Robo_Base.LF._Axis=&ODrive0.Axis0; Axis_Init(P_Axis,0);
	P_Axis=Robo_Base.LB._Axis=&ODrive0.Axis1; Axis_Init(P_Axis,1);
	P_Axis=Robo_Base.RF._Axis=&ODrive1.Axis0; Axis_Init(P_Axis,2);
	P_Axis=Robo_Base.RB._Axis=&ODrive1.Axis1; Axis_Init(P_Axis,3);
	Robo_Base.Working_State=1;
}

//--------------------------------------------------------------------------------------------------//
//函数名称:
//		PID发送函数
//
//函数功能:
//		发送电机PID
//
//参数类型:
//		ROBO_BASE* 底盘结构体指针
//
//移植建议:
//		有需要啥环的控制就让指针指向这个系统, 然后调用对应的PID计算函数进行处理
//
//--------------------------------------------------------------------------------------------------//
void Can_TxMessageCal()
{
	PID_Pos_Cal(&Robo_Base.LF._Pos);
	PID_Pos_Cal(&Robo_Base.LB._Pos);
	PID_Pos_Cal(&Robo_Base.RF._Pos);
	PID_Pos_Cal(&Robo_Base.RB._Pos);
	
	ODrive_Transmit(Robo_Base.LF._Axis,0x0D);
	ODrive_Transmit(Robo_Base.LB._Axis,0x0D);
	ODrive_Transmit(Robo_Base.RF._Axis,0x0D);
	ODrive_Transmit(Robo_Base.RB._Axis,0x0D);
}

//--------------------------------------------------------------------------------------------------//
//函数名称:
//		底盘看门狗检测函数
//
//函数功能:
//		定时检测底盘上各个电机状态, 出现问题后调整底盘工作模式
//
//参数类型:
//		ROBO_BASE* 底盘结构体指针
//
//移植建议:
//		需要添加或删除位置环系统就直接添加
//
//--------------------------------------------------------------------------------------------------//
void Base_WatchDog(void)
{
  uint32_t Error_State=0;
  Pos_System* P_Pos=NULL;
  P_Pos=&Robo_Base.LF._Pos;
	if(System_Check(&P_Pos->Protect)) Error_State|=(1<<(P_Pos->Motor_Num+1));
  P_Pos=&Robo_Base.LB._Pos;
	if(System_Check(&P_Pos->Protect)) Error_State|=(1<<(P_Pos->Motor_Num+1));
  P_Pos=&Robo_Base.RF._Pos;
	if(System_Check(&P_Pos->Protect)) Error_State|=(1<<(P_Pos->Motor_Num+1));
  P_Pos=&Robo_Base.RB._Pos;
	if(System_Check(&P_Pos->Protect)) Error_State|=(1<<(P_Pos->Motor_Num+1));
	
	Axis* P_Axis=NULL;
	P_Axis=Robo_Base.LF._Axis;
	if(P_Axis->Error!=0||System_Check(&P_Axis->Protect)) Error_State|=(1<<(P_Axis->Node_ID+5));
	P_Axis=Robo_Base.LB._Axis;
	if(P_Axis->Error!=0||System_Check(&P_Axis->Protect)) Error_State|=(1<<(P_Axis->Node_ID+5));
	P_Axis=Robo_Base.RF._Axis;
	if(P_Axis->Error!=0||System_Check(&P_Axis->Protect)) Error_State|=(1<<(P_Axis->Node_ID+5));
	P_Axis=Robo_Base.RB._Axis;
	if(P_Axis->Error!=0||System_Check(&P_Axis->Protect)) Error_State|=(1<<(P_Axis->Node_ID+5));
  
	if(Error_State) Robo_Base.Working_State=Error_State;
}

//--------------------------------------------------------------------------------------------------//
//函数名称:
//		底盘运行时间函数
//
//函数功能:
//		记录底盘运行时间
//
//参数类型:
//		无
//
//--------------------------------------------------------------------------------------------------//
void Counting_Time(void)
{
  Robo_Base.Running_Time++;
  if(Robo_Base.Running_Time>RUNNING_TIME_MAX) Robo_Base.Running_Time=0;
}

void Pos_CloseLoop_Init(Pos_System* P_Pos)
{
	static uint8_t num[4]={0};
	uint8_t* P_num=&num[P_Pos->Motor_Num];
	
	if(P_Pos->Info.Temperature==0) return ;
	if(*P_num<150){
		PID_Speed_Cal(P_Pos,300);
		if(P_Pos==&Robo_Base.LF._Pos) if(HAL_GPIO_ReadPin(TIM3_CH1_GPIO_Port,TIM3_CH1_Pin)==GPIO_PIN_RESET) (*P_num)++;
		if(P_Pos==&Robo_Base.LB._Pos) if(HAL_GPIO_ReadPin(TIM3_CH2_GPIO_Port,TIM3_CH2_Pin)==GPIO_PIN_RESET) (*P_num)++;
		if(P_Pos==&Robo_Base.RF._Pos) if(HAL_GPIO_ReadPin(TIM3_CH3_GPIO_Port,TIM3_CH3_Pin)==GPIO_PIN_RESET) (*P_num)++;
		if(P_Pos==&Robo_Base.RB._Pos) if(HAL_GPIO_ReadPin(TIM3_CH4_GPIO_Port,TIM3_CH4_Pin)==GPIO_PIN_RESET) (*P_num)++;
	}else if(*P_num==100) P_Pos->Info.Abs_Angle=0,PID_Speed_Cal(P_Pos,0);
}


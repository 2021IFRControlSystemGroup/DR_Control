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
//		ROBO_BASE 指针, 底盘结构体的指针
//		uint8_t* 电机信息的数组, 推荐使用Rx_CAN变量, 这样可以不需要自己去声明.
//		uint32_t 电机号码
//
//移植建议:
//		直接对case的数据进行修改, 有几个位置环的轮子就加几个, 然后让指针指向对应的轮子就行.
//
//--------------------------------------------------------------------------------------------------//
void Motor_Pos_Analysis(ROBO_BASE* Robo,uint8_t* RX_Data,uint32_t Motor_Num)
{
  Pos_System* P_Motor=NULL;
  switch(Motor_Num)
  {
    case 0x201:P_Motor=&Robo->Pos_MotorLF;break;
    case 0x202:P_Motor=&Robo->Pos_MotorRF;break;
    case 0x203:P_Motor=&Robo->Pos_MotorRB;break;
    case 0x204:P_Motor=&Robo->Pos_MotorLB;break;
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
void Motor_Speed_Analysis(ROBO_BASE* Robo,uint8_t* RX_Data,uint32_t Motor_Num)
{
  Speed_System* S_Motor=NULL;
  switch(Motor_Num)
  {
    case 0x201:S_Motor=&Robo->Speed_MotorLF;break;
    case 0x202:S_Motor=&Robo->Speed_MotorRF;break;
    case 0x203:S_Motor=&Robo->Speed_MotorRB;break;
    case 0x204:S_Motor=&Robo->Speed_MotorLB;break;
	default:break;
  }if(!S_Motor) return ;
  Speed_Info_Analysis(&S_Motor->Info,RX_Data);
  Feed_WatchDog(&S_Motor->Protect);
}

//--------------------------------------------------------------------------------------------------//
//函数名称:
//		系统输入输出值转化为字符串函数
//
//函数功能:
//		把速度环/位置环系统的输入输出值转化成字符串, 以便通过串口发送到上位机
//
//参数类型:
//		int32_t 系统输出值
//		int32_t 系统输入值
//
//移植建议:
//		如果担心数据特别大, 可以把temp的数据再调大一点, 但是需要保证缓冲区的长度够长
//--------------------------------------------------------------------------------------------------//
void SystemIO_Usart_ToString(ROBO_BASE* Robo,int32_t System_Out,int32_t System_In)
{
  uint8_t* Usart_Tx=Robo->Uart2.Tx_buffer;
  int32_t temp=1000000;
  uint8_t flag1=0;

  //转化系统输出值
  Robo->Uart2.Tx_length=0;
  if(System_Out<0) Usart_Tx[Robo->Uart2.Tx_length++]='-',System_Out=-System_Out;
  else if(System_Out==0) Usart_Tx[Robo->Uart2.Tx_length++]='0',temp=0;

  while(temp!=0)
  {
    if(System_Out/temp!=0)
    {
	  flag1=1;
	  Usart_Tx[Robo->Uart2.Tx_length++]='0'+System_Out/temp;
	  System_Out-=System_Out/temp*temp;
	  if(System_Out==0)
	  {
		temp/=10;
	    while(temp!=0)
		{
		  Usart_Tx[Robo->Uart2.Tx_length++]='0';
		  temp/=10;
		}break;
	  }
    }else if(flag1) Usart_Tx[Robo->Uart2.Tx_length++]='0';
	temp/=10;
  }Usart_Tx[Robo->Uart2.Tx_length++]=' ';
  temp=10000;
  flag1=0;

  //转化系统输入值
  if(System_In<0) Usart_Tx[Robo->Uart2.Tx_length++]='-',System_In=-System_In;
  else if(System_In==0) Usart_Tx[Robo->Uart2.Tx_length++]='0',temp=0;

  while(temp!=0)
  {
    if(System_In/temp!=0)
    {
	  flag1=1;
	  Usart_Tx[Robo->Uart2.Tx_length++]='0'+System_In/temp;
	  System_In-=System_In/temp*temp;
	  if(System_In==0)
	  {
		temp/=10;
	    while(temp!=0)
		{
		  Usart_Tx[Robo->Uart2.Tx_length++]='0';
		  temp/=10;
		}break;
	  }
    }else if(flag1)  Usart_Tx[Robo->Uart2.Tx_length++]='0';
	temp/=10;
  }
  Usart_Tx[Robo->Uart2.Tx_length++]='\r';
  Usart_Tx[Robo->Uart2.Tx_length]='\n';
}


//--------------------------------------------------------------------------------------------------//
//函数名称:
//		底盘参数初始化
//
//函数功能:
//		初始化底盘所有的信息
//
//参数类型:
//		ROBO_BASE 指针, 底盘结构体的指针
//
//移植建议:
//		有什么状态, 电机, 电机状态都先把数据封装进ROBO_BASE结构体里, 然后直接初始化就好了
//
//--------------------------------------------------------------------------------------------------//
void BASE_Init(void)
{
  Pos_System* P_Pos=NULL;
  P_Pos=&Robo_Base.Pos_MotorLF; PID_Init(&P_Pos->Pos_PID,			0.3,	0,	0,	5000,	0,	0,	7000);
  P_Pos->Motor_Num=0;		PID_Init(&P_Pos->Speed_PID,			5,	0,	0,	5000,	0,	0,	7000); 
  P_Pos->Protect.Count_Time=WATCHDOG_TIME_MAX;	SystemState_Set(&P_Pos->Protect,MISSING);
  P_Pos=&Robo_Base.Pos_MotorRF; PID_Init(&P_Pos->Pos_PID,			0,	0,	0,	0,	0,	0,	0);
  P_Pos->Motor_Num=1;		PID_Init(&P_Pos->Speed_PID,			0,	0,	0,	0,	0,	0,	0); 
  P_Pos->Protect.Count_Time=WATCHDOG_TIME_MAX;	SystemState_Set(&P_Pos->Protect,MISSING);
  P_Pos=&Robo_Base.Pos_MotorRB; PID_Init(&P_Pos->Pos_PID,			0,	0,	0,	0,	0,	0,	0);
  P_Pos->Motor_Num=2;		PID_Init(&P_Pos->Speed_PID,			0,	0,	0,	0,	0,	0,	0); 
  P_Pos->Protect.Count_Time=WATCHDOG_TIME_MAX;	SystemState_Set(&P_Pos->Protect,MISSING);
  P_Pos=&Robo_Base.Pos_MotorLB; PID_Init(&P_Pos->Pos_PID,			0,	0,	0,	0,	0,	0,	0);
  P_Pos->Motor_Num=3;		PID_Init(&P_Pos->Speed_PID,			0,	0,	0,	0,	0,	0,	0); 
  P_Pos->Protect.Count_Time=WATCHDOG_TIME_MAX;	SystemState_Set(&P_Pos->Protect,MISSING);
	
  Speed_System* P_Speed=NULL;
  P_Speed=&Robo_Base.Speed_MotorLF; PID_Init(&P_Speed->Speed_PID,	5,	0,	0,	5000,	0,	0,	7000); P_Speed->Motor_Num=0;
  P_Speed->Protect.Count_Time=WATCHDOG_TIME_MAX;	SystemState_Set(&P_Speed->Protect,MISSING);
  P_Speed=&Robo_Base.Speed_MotorRF; PID_Init(&P_Speed->Speed_PID,	0,	0,	0,	0,	0,	0,	0); P_Speed->Motor_Num=1;
  P_Speed->Protect.Count_Time=WATCHDOG_TIME_MAX;	SystemState_Set(&P_Speed->Protect,MISSING);
  P_Speed=&Robo_Base.Speed_MotorRB; PID_Init(&P_Speed->Speed_PID,	0,	0,	0,	0,	0,	0,	0); P_Speed->Motor_Num=2;
  P_Speed->Protect.Count_Time=WATCHDOG_TIME_MAX;	SystemState_Set(&P_Speed->Protect,MISSING);
  P_Speed=&Robo_Base.Speed_MotorLB; PID_Init(&P_Speed->Speed_PID,	0,	0,	0,	0,	0,	0,	0); P_Speed->Motor_Num=3;
  P_Speed->Protect.Count_Time=WATCHDOG_TIME_MAX;	SystemState_Set(&P_Speed->Protect,MISSING);
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
void PID_Send(ROBO_BASE* Robo)
{
  Base_WatchDog(Robo);

  Pos_System* P_Pos=NULL;
  P_Pos=&Robo->Pos_MotorLB; PID_Pos_Cal(P_Pos,Robo->Can2.Tx);
  P_Pos=&Robo->Pos_MotorRB; PID_Pos_Cal(P_Pos,Robo->Can2.Tx);
  P_Pos=&Robo->Pos_MotorLF; PID_Pos_Cal(P_Pos,Robo->Can2.Tx);
  P_Pos=&Robo->Pos_MotorRF; PID_Pos_Cal(P_Pos,Robo->Can2.Tx);
  Send_To_Motor(&hcan2,Robo->Can2.Tx);

  Speed_System* P_Speed=NULL;
  P_Speed=&Robo->Speed_MotorLB; PID_Speed_Cal(P_Speed,Robo->Can1.Tx);
  P_Speed=&Robo->Speed_MotorRB; PID_Speed_Cal(P_Speed,Robo->Can1.Tx);
  P_Speed=&Robo->Speed_MotorLF; PID_Speed_Cal(P_Speed,Robo->Can1.Tx);
  P_Speed=&Robo->Speed_MotorRF; PID_Speed_Cal(P_Speed,Robo->Can1.Tx);
  Send_To_Motor(&hcan1,Robo->Can2.Tx);
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
void Base_WatchDog(ROBO_BASE* Robo)
{
  RoboBaseState Error_State=SYSTEM_WORKING;
  Pos_System* P_Pos=NULL;
  P_Pos=&Robo->Pos_MotorLF; POS_SYSTEM_CHECK;
  P_Pos=&Robo->Pos_MotorRF; POS_SYSTEM_CHECK;
  P_Pos=&Robo->Pos_MotorRB; POS_SYSTEM_CHECK;
  P_Pos=&Robo->Pos_MotorLB; POS_SYSTEM_CHECK;
	
  Speed_System* P_Speed=NULL;
  P_Speed=&Robo->Speed_MotorLF; SPEED_SYSTEM_CHECK;
  P_Speed=&Robo->Speed_MotorRF; SPEED_SYSTEM_CHECK;
  P_Speed=&Robo->Speed_MotorRB; SPEED_SYSTEM_CHECK;
  P_Speed=&Robo->Speed_MotorLB; SPEED_SYSTEM_CHECK;

  Robo->State=Error_State;
  if(Error_State!=WORKING) LED_WARNING(Robo);
}

//--------------------------------------------------------------------------------------------------//
//函数名称:
//		底盘运行时间函数
//
//函数功能:
//		记录底盘运行时间
//
//参数类型:
//		ROBO_BASE* 底盘结构体指针
//
//--------------------------------------------------------------------------------------------------//
void Counting_Time(ROBO_BASE* Robo)
{
  Robo->Running_Time++;
  if(Robo->Running_Time>RUNNING_TIME_MAX) Robo->Running_Time=0;
}
	//--------------------------------------------------------------------------------------------------//
//函数名称:
//		底盘LED报警函数
//
//函数功能:
//		当底盘处于非正常工作状态时, 通过控制LED闪烁特定情况来提示错误
//
//参数类型:
//		ROBO_BASE* 底盘结构体指针
//
//--------------------------------------------------------------------------------------------------//
void LED_WARNING(ROBO_BASE* Robo)
{
  static uint8_t State=1;
  static uint8_t Flag=0;
  if(State==(uint8_t)Robo->State) State=0;
  if(Robo->Running_Time%3000-(State*300)<100)
  {
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
  }
  else
  {
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
	  if(!Flag) Flag=1;
  }
  if(Flag) State++,Flag=0;
}


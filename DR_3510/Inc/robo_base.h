#ifndef __ROBOBASE_H__
	#define __ROBOBASE_H__

	//-------------------------------------头文件引用----------------------------------------------//
	#include "motor_system.h"
	#include "uart_communicate.h"
	//---------------------------------------------------------------------------------------------//

	//---------------------------------------预编译------------------------------------------------//
	#ifndef ROBO_BASE_USER_ROOT
		#define ROBO_BASE_USER_ROOT 1
	#endif
	//---------------------------------------------------------------------------------------------//

	//------------------------------------基本宏定义-----------------------------------------------//
	#define RUNNING_TIME_MAX 5000000													//系统运行时间最大值
	#define COUNTING_TIME(_TIME) _TIME++;if(_TIME>=RUNNING_TIME_MAX) _TIME=0;		//宏定义函数, 记录程序运行时间
	#define LED_RED_OFF HAL_GPIO_WritePin(LED_RED_GPIO_Port,LED_RED_Pin,GPIO_PIN_RESET)
	#define LED_GRE_OFF HAL_GPIO_WritePin(LED_GRE_GPIO_Port,LED_GRE_Pin,GPIO_PIN_RESET)
	#define LED_RED_ON HAL_GPIO_WritePin(LED_RED_GPIO_Port,LED_RED_Pin,GPIO_PIN_SET)
	#define LED_GRE_ON HAL_GPIO_WritePin(LED_GRE_GPIO_Port,LED_GRE_Pin,GPIO_PIN_SET)

	//---------------------------------------------------------------------------------------------//
	
	//-----------------------------------底盘结构体部分--------------------------------------------//
	//	底盘状态
	//	SYSTEM_WORKING			=	(1<<0)|1,														//底盘正常工作
	//	ALL_MOTOR_ERROR			=	(1<<1),															//全系统系统错误
	//	POS_MOTOR_ERROR			=	(1<<2),															//位置环系统错误
	//	SPEED_MOTOR_ERROR		=	(1<<3),															//速度环系统错误

	typedef struct CAN_BUFFER										//CAN通信结构体
	{
		uint8_t Tx[8];														//发送数据帧
		uint8_t Rx[8];														//接收数据帧
	}CAN_BUFFER;

	typedef struct Robo_Base										//底盘结构体
	{

		Speed_System LF;									//速度环
		Speed_System LB;									//速度环
		Speed_System RF;									//速度环
		Speed_System RB;									//速度环

		int32_t Speed_X;													//底盘X方向上目标速度
		int32_t Speed_Y;													//底盘Y方向上目标速度
		float Angle;															//底盘运动的相对方向
		uint32_t Working_State;										//底盘状态
		CAN_BUFFER Can1;													//CAN1通信发送数据
		CAN_BUFFER Can2;													//CAN2通信发送数据
		uint32_t Running_Time;										//运行时间
	}ROBO_BASE;
	//---------------------------------------------------------------------------------------------//

	//--------------------------------------基本功能-----------------------------------------------//
	#if UART_COMMUNICATE_USER_ROOT
		weak void Motor_Analysis(uint8_t* RX_Data,uint32_t Motor_Num);
		weak void Motor_All_Analysis(uint8_t* RX_Data,uint32_t Motor_Num);
		weak void Motor_Pos_Analysis(uint8_t* RX_Data,uint32_t Motor_Num);			//位置环电机数据分析的接口函数
		weak void Motor_Speed_Analysis(uint8_t* RX_Data,uint32_t Motor_Num);		//速度环电机数据分析的接口函数
		weak void Base_Init(void);																							//底盘PID参数初始化的接口函数
		weak void Send_RoboBasePID(void);																				//PID发送函数
		weak void Base_WatchDog(void);
	#else
		void Motor_Analysis(uint8_t* RX_Data,uint32_t Motor_Num);
		void Motor_All_Analysis(uint8_t* RX_Data,uint32_t Motor_Num);
		void Motor_Pos_Analysis(uint8_t* RX_Data,uint32_t Motor_Num);						//位置环电机数据分析的接口函数
		void Motor_Speed_Analysis(uint8_t* RX_Data,uint32_t Motor_Num);					//速度环电机数据分析的接口函数
		void Base_Init(void);																										//底盘PID参数初始化的接口函数
		void Send_RoboBasePID(void);																						//PID发送函数
		void Base_WatchDog(void);
	#endif
	//---------------------------------------------------------------------------------------------//
		
	//---------------------------------------对外接口----------------------------------------------//
	extern ROBO_BASE Robo_Base;
	//---------------------------------------------------------------------------------------------//

	//---------------------------------------用户自定义功能区---------------------------------------//
	//E.G.
	//		预编译内容
	//					......
	//					......
	//
	//		结构体声明
	//					......
	//					......
	//
	//		宏定义
	//					......
	//					......
	//
	//		宏定义函数
	//					......
	//					......
	//
	//		函数声明
	//					......
	//					......
	//
	//----------------------------------------------------------------------------------------------//
#endif



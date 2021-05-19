#ifndef __MOTOR_SYSTEM_H__
	#define __MOTOR_SYSTEM_H__

	//-------------------------------------头文件引用----------------------------------------------//
	#include "main.h"
	#include "math.h"
	//----------------------------------------------------------------------------------------------//

	//---------------------------------------预编译-------------------------------------------------//
	#ifndef POS_SYSTEM_ENABLE													//针对位置环控制系统操作使能
		#define POS_SYSTEM_ENABLE 0
	#endif

	#ifndef SPEED_SYSTEM_ENABLE												//针对速度环控制系统操作使能
		#define SPEED_SYSTEM_ENABLE 1
	#endif

	#ifndef SYSTEM_ENABLE															//针对任意控制系统操作使能
		#define SYSTEM_ENABLE 0
	#endif
	
	#ifndef WATCHDOG_ENABLE														//针对看门狗保护操作使能
		#define WATCHDOG_ENABLE 1
	#endif

	#ifndef MOTOR_SYSTEM_USER_ROOT																	//修改函数权限
		#define MOTOR_SYSTEM_USER_ROOT 0
	#endif

	//----------------------------------------------------------------------------------------------//
	
	//-------------------------------------基本宏定义-----------------------------------------------//
	#define PI (2*acos(0))																	//PI圆周率的宏定义
	#define HALF_PI (PI/2)
	#define TWO_PI (2*PI)
	#define ToDegree(a) (a/PI*180)													//弧度转化成角度的宏定义
	#define ToRadian(a) (a/180.0*PI)													//角度转化成弧度的宏定义

	#define ROTOR_ANGLE 8192																//转子机械角度
	#define GEAR_RATIO 19																	//电机减速比(3508)
	//#define GEAR_RATIO (36*6)																//电机减速比(2006)
	#define ONE_CIRCLE (ROTOR_ANGLE*GEAR_RATIO)							//电机转动一圈的总机械角度

	#define CAN_TXMESSAGEINDEXMAX 2
	//----------------------------------------------------------------------------------------------//

	//--------------------------------------基本功能------------------------------------------------//
	typedef struct Can_TxMessageTypeDef					//CAN发送消息结构体
	{
		CAN_TxHeaderTypeDef Header;								//CAN帧头
		CAN_HandleTypeDef* Hcan;									//CAN号码
		uint8_t Data[8];													//CAN数据帧
		uint8_t Update;														//CAN更新标志位
	}Can_TxMessageTypeDef;

	typedef struct pid_init_val									//电机PID参数结构体
	{
		float Kp;																	//Kp
		float Ki;																	//Ki
		float Kd;																	//Kd
		float error;															//误差
		float error_last;													//上一次误差
		float error_max;													//最大误差
		float dead_line;													//死区
		float intergral;													//误差积分
		float intergral_max;											//误差积分最大值
		float derivative;													//误差微分
		float output;															//输出
		float output_max;													//输出最大值
		
	}PID;
																																								//宏定义函数, CAN发送消息帧头设置
	#define Can_TxMessageHeader_Set(_Header,_DLC,_IDE,_RTR,_StdId,_ExtId) \
				_Header.DLC=_DLC;_Header.IDE=_IDE;\
				_Header.RTR=_RTR;_Header.StdId=_StdId;\
				_Header.ExtId=_ExtId;_Header.TransmitGlobalTime=DISABLE;
	
	#if MOTOR_SYSTEM_USER_ROOT 
		weak void CAN_Start_IT(CAN_HandleTypeDef *hcan);																																											//CAN过滤器与中断配置
		weak void Can_TxMessage_Init(Can_TxMessageTypeDef* TxMessage,CAN_HandleTypeDef *hcan,uint32_t DLC,uint32_t IDE,uint32_t RTR,uint32_t StdId,uint32_t ExtId);	//CAN_TxMessage参数初始化函数
		weak void PID_Init(PID *pid, float Kp, float Ki, float Kd, float error_max, float dead_line, float intergral_max, float output_max);	//PID参数初始化函数
		weak void PID_General_Cal(PID *pid, float fdbV, float tarV);																																					//PID计算函数
		weak void Add_TxMessage(float Output,uint8_t* Tx_Data);																																								//CAN发送消息添加函数
		weak void CAN_Send(Can_TxMessageTypeDef* TxMessage);																																									//CAN发送消息函数			
	#else
		void CAN_Start_IT(CAN_HandleTypeDef *hcan);																																														//CAN过滤器与中断配置
		void Can_TxMessage_Init(Can_TxMessageTypeDef* TxMessage,CAN_HandleTypeDef *hcan,uint32_t DLC,uint32_t IDE,uint32_t RTR,uint32_t StdId,uint32_t ExtId);			//CAN_TxMessage参数初始化函数
		void PID_Init(PID *pid, float Kp, float Ki, float Kd, float error_max, float dead_line, float intergral_max, float output_max);				//PID参数初始化函数
		void PID_General_Cal(PID *pid, float fdbV, float tarV);																																								//PID计算函数
		void Add_TxMessage(float Output,uint8_t* Tx_Data);																																										//CAN发送消息添加函数
		void CAN_Send(Can_TxMessageTypeDef* TxMessage);																																												//CAN发送消息函数
	#endif
	//----------------------------------------------------------------------------------------------//
	
	//-------------------------------------看门狗功能-----------------------------------------------//
	#if WATCHDOG_ENABLE
		typedef enum SystemState										//系统状态
		{
			WORKING,																	//正常工作
			MISSING,																	//丢失
		}SystemState;

		typedef struct Protect_System								//系统看门狗结构体
		{
			SystemState State;												//系统当前状态
			int16_t Count_Time;												//看门狗时间
		}Protect_System;
		
		#define WATCHDOG_TIME_MAX 300																								//看门狗总时长
		#if MOTOR_SYSTEM_USER_ROOT
			weak void Feed_WatchDog(Protect_System* Dogs);														//看门狗喂狗函数
			weak void SystemState_Set(Protect_System* Dogs,SystemState State);				//系统状态切换函数
			weak uint8_t System_Check(Protect_System* Dogs);													//系统状态检测函数
		#else
			void Feed_WatchDog(Protect_System* Dogs);																	//看门狗喂狗函数
			void SystemState_Set(Protect_System* Dogs,SystemState State);							//系统状态切换函数
			uint8_t System_Check(Protect_System* Dogs);																//系统状态检测函数
		#endif
	#endif
	//----------------------------------------------------------------------------------------------//
	
	//--------------------------------------全系统功能----------------------------------------------//
	#if SYSTEM_ENABLE
		typedef struct Motor_Info										//进行全系统环控制的电机信息
		{
			int16_t Speed;														//电机速度				单位(rad/min 转/每分钟)
			uint16_t Angle;														//转子机械角度
			int32_t Abs_Angle;												//转子绝对机械角度
			float Relative_Angle;											//电机相对坐标角度		单位(° 度)
			uint8_t Temperature;											//电机温度				单位(℃ 摄氏度)
			int16_t Electric;													//电流					单位(mA 毫安)
			uint16_t Last_Angle;											//上一次的转子绝对角度
		}Motor_Info;
		
		typedef struct Motor_System									//全系统
		{
			Motor_Info Info;													//电机信息
			PID Pos_PID;															//位置环PID参数
			PID Speed_PID;														//速度环PID参数
			float Tar_Pos;														//目标位置
			float Tar_Speed;													//目标速度
			uint8_t Motor_Num;												//电机号码
			#if WATCHDOG_ENABLE
				Protect_System Protect; 								//看门狗保护
			#endif
			Can_TxMessageTypeDef* TxMessage;					//CAN发送消息指针
			void (*Info_Analysis) (Motor_Info* P_Motor,uint8_t* RX_Data);
		}Motor_System;
		
		#if MOTOR_SYSTEM_USER_ROOT
			weak void Motor_System_Init(Motor_System* P_Motor,uint8_t Motor_Num,Can_TxMessageTypeDef* TxMessage,void (*Info_Analysis)(Motor_Info* P_Motor,uint8_t* RX_Data));		//全系统控制初始化函数
			weak void Motor_System_Analysis(Motor_Info* P_Motor,uint8_t* RX_Data);																		//全系统控制电机数据分析函数
			weak void Motor_Extra_Analysis(Motor_Info* P_Motor);																										//全系统控制PID计算函数
		#else
			void Motor_System_Init(Motor_System* P_Motor,uint8_t Motor_Num,Can_TxMessageTypeDef* TxMessage,void (*Info_Analysis)(Motor_Info* P_Motor,uint8_t* RX_Data));				//全系统控制初始化函数
			void Motor_System_Analysis(Motor_Info* P_Motor,uint8_t* RX_Data);																					//全系统控制电机数据分析函数
			void Motor_Extra_Analysis(Motor_Info* P_Motor);																													//全系统控制PID计算函数
		#endif
	#endif
	//----------------------------------------------------------------------------------------------//

	//--------------------------------------位置环功能----------------------------------------------//
	#if POS_SYSTEM_ENABLE
		typedef struct Motor_Pos_Info								//进行位置环控制的电机信息
		{
			int16_t Speed;														//电机速度				单位(rad/min 转/每分钟)
			uint16_t Angle;														//转子机械角度
			int32_t Abs_Angle;												//转子绝对机械角度
			float Relative_Angle;											//电机相对坐标角度		单位(° 度)
			uint8_t Temperature;											//电机温度				单位(℃ 摄氏度)
			int16_t Electric;													//电流					单位(mA 毫安)
			uint16_t Last_Angle;											//上一次的转子绝对角度
		}Motor_Pos_Info;
						
		typedef struct Pos_System										//位置环系统
		{
			Motor_Pos_Info Info;											//位置环电机信息
			PID Pos_PID;															//位置环PID参数
			PID Speed_PID;														//速度环PID参数
			float Tar_Pos;														//目标位置
			uint8_t Motor_Num;												//电机号码
			#if WATCHDOG_ENABLE
				Protect_System Protect; 									//看门狗保护
			#endif
			Can_TxMessageTypeDef* TxMessage;					//CAN发送消息指针
			void (*Info_Analysis)(Motor_Pos_Info* P_Pos,uint8_t* RX_Data);
		}Pos_System;

		#if MOTOR_SYSTEM_USER_ROOT 
			weak void Pos_System_Init(Pos_System* P_Pos,uint8_t Motor_Num,Can_TxMessageTypeDef* TxMessage,void (*Info_Analysis)(Motor_Pos_Info* P_Pos,uint8_t* RX_Data););			//位置环控制系统初始化函数
			weak void Pos_System_Analysis(Motor_Pos_Info* P_Pos,uint8_t* RX_Data);															//位置环控制系统数据分析函数
			weak void PID_Pos_Cal(Pos_System* Pos_Motor);																												//位置环控制系统PID计算函数
		#else
			void Pos_System_Init(Pos_System* P_Pos,uint8_t Motor_Num,Can_TxMessageTypeDef* TxMessage,void (*Info_Analysis)(Motor_Pos_Info* P_Pos,uint8_t* RX_Data));					//位置环控制系统初始化函数
			void Pos_System_Analysis(Motor_Pos_Info* Motor,uint8_t* RX_Data);																		//位置环控制系统数据分析函数
			void PID_Pos_Cal(Pos_System* Pos_Motor);																														//位置环控制系统PID计算函数
		#endif
	#endif
	//----------------------------------------------------------------------------------------------//
	
	//--------------------------------------速度环功能----------------------------------------------//
	#if SPEED_SYSTEM_ENABLE
		typedef struct Motor_Speed_Info							//进行速度环控制的电机信息
		{
			int16_t Speed;														//电机速度				单位(rad/min 转/每分钟)
			uint8_t Temperature;											//电机温度				单位(℃ 摄氏度)
			int16_t Electric;													//电流					单位(mA 毫安)
		}Motor_Speed_Info;
		
		typedef struct Speed_System									//速度环系统
		{
			Motor_Speed_Info Info;										//速度环电机信息
			PID Speed_PID;														//速度环PID参数
			float Tar_Speed;													//目标速度
			uint8_t Motor_Num;												//电机号码
			#if WATCHDOG_ENABLE
				Protect_System Protect; 								//看门狗保护
			#endif
			Can_TxMessageTypeDef* TxMessage;					//CAN发送消息指针
			void (*Info_Analysis)(Motor_Speed_Info* Motor,uint8_t* RX_Data);	
		}Speed_System;
		
		#if MOTOR_SYSTEM_USER_ROOT 
			weak void Speed_System_Init(Speed_System* P_Speed,uint8_t Motor_Num,Can_TxMessageTypeDef* TxMessage,void (*Info_Analysis)(Motor_Speed_Info* Motor,uint8_t* RX_Data));	//速度环控制系统初始化函数
			weak void Speed_System_Analysis(Motor_Speed_Info* Motor,uint8_t* RX_Data);															//速度环电机数据分析的操作函数
			weak void PID_Speed_Cal(Speed_System* Speed_Motor);																										//速度环系统PID计算函数
		#else
			void Speed_System_Init(Speed_System* P_Speed,uint8_t Motor_Num,Can_TxMessageTypeDef* TxMessage,void (*Info_Analysis)(Motor_Speed_Info* Motor,uint8_t* RX_Data));			//速度环控制系统初始化函数
			void Speed_System_Analysis(Motor_Speed_Info* Motor,uint8_t* RX_Data);																		//速度环电机数据分析的操作函数
			void PID_Speed_Cal(Speed_System* Speed_Motor);																												//速度环系统PID计算函数
		#endif
	#endif
	//----------------------------------------------------------------------------------------------//
		
	//---------------------------------------对外接口----------------------------------------------//
	extern Can_TxMessageTypeDef Can_TxMessageList[CAN_TXMESSAGEINDEXMAX];
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


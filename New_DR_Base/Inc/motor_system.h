#ifndef __MOTOR_SYSTEM_H__
#define __MOTOR_SYSTEM_H__

//---------头文件包含部分----------//
#include "main.h"
#include "math.h"
#include "can.h"
//---------------------------------//

//---------#define部分-------------//
#define PI (2*acos(0))																	//PI圆周率的宏定义
#define HALF_PI (PI/2)
#define ToDegree(a) (a/PI*180)													//弧度转化成角度的宏定义
#define ToRadian(a) (a/180*PI)													//角度转化成弧度的宏定义

#define ROTOR_ANGLE 8192																//转子机械角度
//#define GEAR_RATIO 19																	//电机减速比(3508)
#define GEAR_RATIO (36*6)																		//电机减速比(2006)
#define ONE_CIRCLE (ROTOR_ANGLE*GEAR_RATIO)							//电机转动一圈的总机械角度

#define HALF_PI_ANGLE (GEAR_RATIO*ROTOR_ANGLE/4)
#define PI_ANGLE (GEAR_RATIO*ROTOR_ANGLE/2)

#define WATCHDOG_TIME_MAX 300														//看门狗总时长
#define RUNNING_TIME_MAX 500000													//系统运行时间最大值
#define POS_SYSTEM_CHECK if(System_Check(&P_Pos->Protect)) Error_State=(RoboBaseState)(P_Pos->Motor_Num+1);						//位置环看门狗检测
#define SPEED_SYSTEM_CHECK if(System_Check(&P_Speed->Protect)) Error_State=(RoboBaseState)(P_Speed->Motor_Num+5);			//速度环看门狗检测
//---------------------------------//

//---------底盘结构体部分----------//
typedef enum SystemState										//系统状态
{
	WORKING,																	//正常工作
	MISSING,																	//丢失
	SUSPENDING																//挂起
}SystemState;

typedef struct Protect_System								//系统看门狗结构体
{
  SystemState State;												//系统当前状态
  int16_t Count_Time;												//看门狗时间
}Protect_System;

typedef struct Motor_Pos_Info								//进行位置环控制的电机信息
{
  int16_t Speed;														//电机速度				单位(rad/min 转/每分钟)
  uint16_t Angle;														//转子机械角度
  int32_t Abs_Angle;												//转子绝对机械角度
  int32_t Relative_Angle;											//电机相对坐标角度		单位(° 度)
	int Circle_Num;
  uint8_t Temperature;											//电机温度				单位(℃ 摄氏度)
  int16_t Electric;													//电流					单位(mA 毫安)
  uint16_t Last_Angle;											//上一次的转子绝对角度
}Motor_Pos_Info;

typedef struct Motor_Speed_Info							//进行速度环控制的电机信息
{
  int16_t Speed;														//电机速度				单位(rad/min 转/每分钟)
  uint8_t Temperature;											//电机温度				单位(℃ 摄氏度)
  int16_t Electric;													//电流					单位(mA 毫安)
}Motor_Speed_Info;

typedef struct pid_init_val{								//电机PID参数结构体
	
	float Kp;
	float Ki;
	float Kd;
	
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

typedef struct Pos_System										//位置环系统
{
  Motor_Pos_Info Info;											//位置环电机信息
  PID Pos_PID;															//位置环PID参数
  PID Speed_PID;														//速度环PID参数
  float Tar_Pos;														//目标位置
  uint8_t Motor_Num;												//电机号码
  Protect_System Protect; 
}Pos_System;

typedef struct Speed_System									//速度环系统
{
  Motor_Speed_Info Info;										//速度环电机信息
  PID Speed_PID;														//速度环PID参数
  float Tar_Speed;													//目标速度
  uint8_t Motor_Num;												//电机号码
  Protect_System Protect; 
}Speed_System;
//---------------------------------//

//-------------函数声明------------//
void Speed_Info_Analysis(Motor_Speed_Info* Motor,uint8_t* RX_Data);									//速度环电机数据分析的操作函数
void Pos_Info_Analysis(Motor_Pos_Info* Motor,uint8_t* RX_Data);											//位置环电机数据分析的操作函数

void PID_Init(PID *pid, float Kp, float Ki, float Kd, float error_max, float dead_line, float intergral_max, float output_max);		//PID参数初始化函数
void PID_General_Cal(PID *pid, float fdbV, float tarV,uint8_t moto_num,uint8_t *Tx_msg);					//PID计算函数----为了向下兼容
void PID_Speed_Cal(Speed_System* Speed_Motor,uint8_t *Tx_msg);																		//速度环系统PID计算函数
void PID_Pos_Cal(Pos_System* Pos_Motor,uint8_t *Tx_msg);																					//位置环系统PID计算函数
void Send_To_Motor(CAN_HandleTypeDef *hcan,uint8_t* Tx_Data);								//CAN通信发送函数

void Feed_WatchDog(Protect_System* Dogs);																		//看门狗喂狗函数
void SystemState_Set(Protect_System* Dogs,SystemState State);								//系统状态切换函数
uint8_t System_Check(Protect_System* Dogs);																	//系统状态检测函数
//---------------------------------//
#endif


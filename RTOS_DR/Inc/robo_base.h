#ifndef __ROBOBASE_H__
#define __ROBOBASE_H__

//---------头文件包含部分----------//
#include "motor_system.h"
#include "odrive_can.h"
#include "uart_communicate.h"
//---------------------------------//

//---------#define部分-------------//
#define RUNNING_TIME_MAX 500000													//系统运行时间最大值
//---------------------------------//

//---------底盘结构体部分----------//
typedef enum RoboBaseState									//底盘状态
{
	SYSTEM_WORKING=0,														//正常工作
	LF_POS_ERROR=(1<<0),
	LB_POS_ERROR=(1<<1),
	RF_POS_ERROR=(1<<2),
	RB_POS_ERROR=(1<<3),
	LF_AXIS_ERROR=(1<<4),
	LB_AXIS_ERROR=(1<<5),
	RF_AXIS_ERROR=(1<<6),
	RB_AXIS_ERROR=(1<<7),
}RoboBaseState;

typedef struct CAN_BUFFER										//CAN通信结构体
{
	uint8_t Tx[8];														//发送数据帧
	uint8_t Rx[8];														//接收数据帧
}CAN_BUFFER;

typedef struct Motor_Group									//舵轮组结构体
{
	Pos_System _Pos;													//位置环系统结构体
	Axis* _Axis;															//ODrive的Axis结构体
}Motor_Group;

typedef struct Robo_Base										//底盘结构体
{
	Motor_Group LF;														//舵轮组--左前轮
	Motor_Group LB;														//舵轮组--左后轮
	Motor_Group RF;														//舵轮组--右前轮
	Motor_Group RB;														//舵轮组--右后轮

	int32_t Speed_X;													//底盘X方向上目标速度
	int32_t Speed_Y;													//底盘Y方向上目标速度
	float Angle;															//底盘运动的相对方向

	uint8_t Working_State;											//底盘状态
	
	CAN_BUFFER Can1;													//CAN1通信发送数据
	CAN_BUFFER Can2;													//CAN2通信发送数据
	
	uint32_t Running_Time;										//运行时间
}ROBO_BASE;

//---------------------------------//

//-------------函数声明------------//
void BASE_Init(void);																									//底盘结构体成员初始化的接口函数
void Motor_Pos_Analysis(uint8_t* RX_Data,uint32_t Motor_Num);					//位置环电机数据分析的接口函数
void Motor_Speed_Analysis(uint8_t* RX_Data,uint32_t Motor_Num);				//速度环电机数据分析的接口函数
void Can_TxMessageCal(void);																					//计算更新底盘的Can发送数据函数
void Counting_Time(void);																							//记录底盘运行时间函数
uint8_t Base_WatchDog(void);																					//底盘看门狗接口函数
//---------------------------------//
#endif



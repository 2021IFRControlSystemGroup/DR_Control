#ifndef __ROBOBASE_H__
#define __ROBOBASE_H__

//---------头文件包含部分----------//
#include "motor_system.h"
#include "odrive_can.h"
#include "uart_communicate.h"
//---------------------------------//

//---------#define部分-------------//
#define UART2_TX_LENGTH 20															//UART2发送通信字符总长度
#define UART2_RX_LENGTH 20															//UART2接收通信字符总长度
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

typedef struct UART_BUFFER									//UART通信结构体
{
	uint8_t Tx_buffer[UART2_TX_LENGTH];				//发送数据
	uint8_t Tx_length;												//发送数据长度
	uint8_t Rx_buffer[UART2_RX_LENGTH];				//发送数据
	uint8_t Rx_length;												//发送数据长度	
}UART_BUFFER;

typedef struct CAN_BUFFER										//CAN通信结构体
{
	uint8_t Tx[8];														//发送数据帧
	uint8_t Rx[8];														//接收数据帧
}CAN_BUFFER;

typedef struct Motor_Group
{
	Pos_System _Pos;
	Axis* _Axis;
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
void Motor_Pos_Analysis(uint8_t* RX_Data,uint32_t Motor_Num);				//位置环电机数据分析的接口函数
void Motor_Speed_Analysis(uint8_t* RX_Data,uint32_t Motor_Num);				//速度环电机数据分析的接口函数

void SystemIO_Usart_ToString(ROBO_BASE* Robo,int32_t System_Out,int32_t System_In);			//系统输入输入输出值转化成字符的函数

void BASE_Init(void);																									//底盘PID参数初始化的接口函数

void PID_Send(uint8_t ODrive_num);																							//PID发送函数

void Counting_Time(ROBO_BASE* Robo);
void LED_WARNING(ROBO_BASE* Robo);
uint8_t Base_WatchDog(void);														//底盘看门狗接口函数
//---------------------------------//
#endif



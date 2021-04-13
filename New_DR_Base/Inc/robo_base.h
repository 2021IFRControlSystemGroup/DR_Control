#ifndef __ROBOBASE_H__
#define __ROBOBASE_H__

//---------头文件包含部分----------//
#include "motor_system.h"
#include "uart_communicate.h"
//---------------------------------//

//---------#define部分-------------//
#define UART2_TX_LENGTH 20															//UART2发送通信字符总长度
#define UART2_RX_LENGTH 20															//UART2接收通信字符总长度
//---------------------------------//

//---------底盘结构体部分----------//
typedef enum RoboBaseState									//底盘状态
{
	SYSTEM_WORKING,														//正常工作
	Pos_MotorLF_ERROR,												//左前转向轮错误
	Pos_MotorLB_ERROR,												//左后转向轮错误
	Pos_MotorRF_ERROR,												//右前转向轮错误
	Pos_MotorRB_ERROR,												//右后转向轮错误
	Speed_MotorLF_ERROR,											//左前驱动轮错误
	Speed_MotorLB_ERROR,											//左后驱动轮错误
	Speed_MotorRF_ERROR,											//右前驱动轮错误
	Speed_MotorRB_ERROR,											//右后驱动轮错误
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

typedef struct Robo_Base										//底盘结构体
{
	Pos_System Pos_MotorLF;										//位置环--左前轮
	Pos_System Pos_MotorLB;										//位置环--左后轮
	Pos_System Pos_MotorRF;										//位置环--右前轮
	Pos_System Pos_MotorRB;										//位置环--右后轮
	
	Speed_System Speed_MotorLF;								//速度环--左前轮
	Speed_System Speed_MotorLB;								//速度环--左后轮
	Speed_System Speed_MotorRF;								//速度环--右前轮
	Speed_System Speed_MotorRB;								//速度环--右后轮

	int32_t Speed_X;													//底盘X方向上目标速度
	int32_t Speed_Y;													//底盘Y方向上目标速度
	float Angle;															//底盘运动的相对方向

	RoboBaseState State;											//底盘状态
	
	UART_BUFFER Uart2;												//Uart2通信发送数据
	CAN_BUFFER Can1;													//CAN1通信发送数据
	CAN_BUFFER Can2;													//CAN2通信发送数据
	
	uint32_t Running_Time;										//运行时间
}ROBO_BASE;

//---------------------------------//

//-------------函数声明------------//
void Motor_Pos_Analysis(ROBO_BASE* Robo,uint8_t* RX_Data,uint32_t Motor_Num);				//位置环电机数据分析的接口函数
void Motor_Speed_Analysis(ROBO_BASE* Robo,uint8_t* RX_Data,uint32_t Motor_Num);				//速度环电机数据分析的接口函数

void SystemIO_Usart_ToString(ROBO_BASE* Robo,int32_t System_Out,int32_t System_In);			//系统输入输入输出值转化成字符的函数

void BASE_Init(void);																									//底盘PID参数初始化的接口函数

void PID_Send(ROBO_BASE* Robo);																							//PID发送函数

void Counting_Time(ROBO_BASE* Robo);
void LED_WARNING(ROBO_BASE* Robo);
void Base_WatchDog(ROBO_BASE* Robo);														//底盘看门狗接口函数
//---------------------------------//
#endif



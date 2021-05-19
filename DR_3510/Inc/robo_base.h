#ifndef __ROBOBASE_H__
	#define __ROBOBASE_H__

	//-------------------------------------ͷ�ļ�����----------------------------------------------//
	#include "motor_system.h"
	#include "uart_communicate.h"
	//---------------------------------------------------------------------------------------------//

	//---------------------------------------Ԥ����------------------------------------------------//
	#ifndef ROBO_BASE_USER_ROOT
		#define ROBO_BASE_USER_ROOT 1
	#endif
	//---------------------------------------------------------------------------------------------//

	//------------------------------------�����궨��-----------------------------------------------//
	#define RUNNING_TIME_MAX 5000000													//ϵͳ����ʱ�����ֵ
	#define COUNTING_TIME(_TIME) _TIME++;if(_TIME>=RUNNING_TIME_MAX) _TIME=0;		//�궨�庯��, ��¼��������ʱ��
	#define LED_RED_OFF HAL_GPIO_WritePin(LED_RED_GPIO_Port,LED_RED_Pin,GPIO_PIN_RESET)
	#define LED_GRE_OFF HAL_GPIO_WritePin(LED_GRE_GPIO_Port,LED_GRE_Pin,GPIO_PIN_RESET)
	#define LED_RED_ON HAL_GPIO_WritePin(LED_RED_GPIO_Port,LED_RED_Pin,GPIO_PIN_SET)
	#define LED_GRE_ON HAL_GPIO_WritePin(LED_GRE_GPIO_Port,LED_GRE_Pin,GPIO_PIN_SET)

	//---------------------------------------------------------------------------------------------//
	
	//-----------------------------------���̽ṹ�岿��--------------------------------------------//
	//	����״̬
	//	SYSTEM_WORKING			=	(1<<0)|1,														//������������
	//	ALL_MOTOR_ERROR			=	(1<<1),															//ȫϵͳϵͳ����
	//	POS_MOTOR_ERROR			=	(1<<2),															//λ�û�ϵͳ����
	//	SPEED_MOTOR_ERROR		=	(1<<3),															//�ٶȻ�ϵͳ����

	typedef struct CAN_BUFFER										//CANͨ�Žṹ��
	{
		uint8_t Tx[8];														//��������֡
		uint8_t Rx[8];														//��������֡
	}CAN_BUFFER;

	typedef struct Robo_Base										//���̽ṹ��
	{

		Speed_System LF;									//�ٶȻ�
		Speed_System LB;									//�ٶȻ�
		Speed_System RF;									//�ٶȻ�
		Speed_System RB;									//�ٶȻ�

		int32_t Speed_X;													//����X������Ŀ���ٶ�
		int32_t Speed_Y;													//����Y������Ŀ���ٶ�
		float Angle;															//�����˶�����Է���
		uint32_t Working_State;										//����״̬
		CAN_BUFFER Can1;													//CAN1ͨ�ŷ�������
		CAN_BUFFER Can2;													//CAN2ͨ�ŷ�������
		uint32_t Running_Time;										//����ʱ��
	}ROBO_BASE;
	//---------------------------------------------------------------------------------------------//

	//--------------------------------------��������-----------------------------------------------//
	#if UART_COMMUNICATE_USER_ROOT
		weak void Motor_Analysis(uint8_t* RX_Data,uint32_t Motor_Num);
		weak void Motor_All_Analysis(uint8_t* RX_Data,uint32_t Motor_Num);
		weak void Motor_Pos_Analysis(uint8_t* RX_Data,uint32_t Motor_Num);			//λ�û�������ݷ����Ľӿں���
		weak void Motor_Speed_Analysis(uint8_t* RX_Data,uint32_t Motor_Num);		//�ٶȻ�������ݷ����Ľӿں���
		weak void Base_Init(void);																							//����PID������ʼ���Ľӿں���
		weak void Send_RoboBasePID(void);																				//PID���ͺ���
		weak void Base_WatchDog(void);
	#else
		void Motor_Analysis(uint8_t* RX_Data,uint32_t Motor_Num);
		void Motor_All_Analysis(uint8_t* RX_Data,uint32_t Motor_Num);
		void Motor_Pos_Analysis(uint8_t* RX_Data,uint32_t Motor_Num);						//λ�û�������ݷ����Ľӿں���
		void Motor_Speed_Analysis(uint8_t* RX_Data,uint32_t Motor_Num);					//�ٶȻ�������ݷ����Ľӿں���
		void Base_Init(void);																										//����PID������ʼ���Ľӿں���
		void Send_RoboBasePID(void);																						//PID���ͺ���
		void Base_WatchDog(void);
	#endif
	//---------------------------------------------------------------------------------------------//
		
	//---------------------------------------����ӿ�----------------------------------------------//
	extern ROBO_BASE Robo_Base;
	//---------------------------------------------------------------------------------------------//

	//---------------------------------------�û��Զ��幦����---------------------------------------//
	//E.G.
	//		Ԥ��������
	//					......
	//					......
	//
	//		�ṹ������
	//					......
	//					......
	//
	//		�궨��
	//					......
	//					......
	//
	//		�궨�庯��
	//					......
	//					......
	//
	//		��������
	//					......
	//					......
	//
	//----------------------------------------------------------------------------------------------//
#endif



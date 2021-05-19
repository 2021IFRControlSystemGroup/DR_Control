#ifndef __MOTOR_SYSTEM_H__
	#define __MOTOR_SYSTEM_H__

	//-------------------------------------ͷ�ļ�����----------------------------------------------//
	#include "main.h"
	#include "math.h"
	//----------------------------------------------------------------------------------------------//

	//---------------------------------------Ԥ����-------------------------------------------------//
	#ifndef POS_SYSTEM_ENABLE													//���λ�û�����ϵͳ����ʹ��
		#define POS_SYSTEM_ENABLE 0
	#endif

	#ifndef SPEED_SYSTEM_ENABLE												//����ٶȻ�����ϵͳ����ʹ��
		#define SPEED_SYSTEM_ENABLE 1
	#endif

	#ifndef SYSTEM_ENABLE															//����������ϵͳ����ʹ��
		#define SYSTEM_ENABLE 0
	#endif
	
	#ifndef WATCHDOG_ENABLE														//��Կ��Ź���������ʹ��
		#define WATCHDOG_ENABLE 1
	#endif

	#ifndef MOTOR_SYSTEM_USER_ROOT																	//�޸ĺ���Ȩ��
		#define MOTOR_SYSTEM_USER_ROOT 0
	#endif

	//----------------------------------------------------------------------------------------------//
	
	//-------------------------------------�����궨��-----------------------------------------------//
	#define PI (2*acos(0))																	//PIԲ���ʵĺ궨��
	#define HALF_PI (PI/2)
	#define TWO_PI (2*PI)
	#define ToDegree(a) (a/PI*180)													//����ת���ɽǶȵĺ궨��
	#define ToRadian(a) (a/180.0*PI)													//�Ƕ�ת���ɻ��ȵĺ궨��

	#define ROTOR_ANGLE 8192																//ת�ӻ�е�Ƕ�
	#define GEAR_RATIO 19																	//������ٱ�(3508)
	//#define GEAR_RATIO (36*6)																//������ٱ�(2006)
	#define ONE_CIRCLE (ROTOR_ANGLE*GEAR_RATIO)							//���ת��һȦ���ܻ�е�Ƕ�

	#define CAN_TXMESSAGEINDEXMAX 2
	//----------------------------------------------------------------------------------------------//

	//--------------------------------------��������------------------------------------------------//
	typedef struct Can_TxMessageTypeDef					//CAN������Ϣ�ṹ��
	{
		CAN_TxHeaderTypeDef Header;								//CAN֡ͷ
		CAN_HandleTypeDef* Hcan;									//CAN����
		uint8_t Data[8];													//CAN����֡
		uint8_t Update;														//CAN���±�־λ
	}Can_TxMessageTypeDef;

	typedef struct pid_init_val									//���PID�����ṹ��
	{
		float Kp;																	//Kp
		float Ki;																	//Ki
		float Kd;																	//Kd
		float error;															//���
		float error_last;													//��һ�����
		float error_max;													//������
		float dead_line;													//����
		float intergral;													//������
		float intergral_max;											//���������ֵ
		float derivative;													//���΢��
		float output;															//���
		float output_max;													//������ֵ
		
	}PID;
																																								//�궨�庯��, CAN������Ϣ֡ͷ����
	#define Can_TxMessageHeader_Set(_Header,_DLC,_IDE,_RTR,_StdId,_ExtId) \
				_Header.DLC=_DLC;_Header.IDE=_IDE;\
				_Header.RTR=_RTR;_Header.StdId=_StdId;\
				_Header.ExtId=_ExtId;_Header.TransmitGlobalTime=DISABLE;
	
	#if MOTOR_SYSTEM_USER_ROOT 
		weak void CAN_Start_IT(CAN_HandleTypeDef *hcan);																																											//CAN���������ж�����
		weak void Can_TxMessage_Init(Can_TxMessageTypeDef* TxMessage,CAN_HandleTypeDef *hcan,uint32_t DLC,uint32_t IDE,uint32_t RTR,uint32_t StdId,uint32_t ExtId);	//CAN_TxMessage������ʼ������
		weak void PID_Init(PID *pid, float Kp, float Ki, float Kd, float error_max, float dead_line, float intergral_max, float output_max);	//PID������ʼ������
		weak void PID_General_Cal(PID *pid, float fdbV, float tarV);																																					//PID���㺯��
		weak void Add_TxMessage(float Output,uint8_t* Tx_Data);																																								//CAN������Ϣ��Ӻ���
		weak void CAN_Send(Can_TxMessageTypeDef* TxMessage);																																									//CAN������Ϣ����			
	#else
		void CAN_Start_IT(CAN_HandleTypeDef *hcan);																																														//CAN���������ж�����
		void Can_TxMessage_Init(Can_TxMessageTypeDef* TxMessage,CAN_HandleTypeDef *hcan,uint32_t DLC,uint32_t IDE,uint32_t RTR,uint32_t StdId,uint32_t ExtId);			//CAN_TxMessage������ʼ������
		void PID_Init(PID *pid, float Kp, float Ki, float Kd, float error_max, float dead_line, float intergral_max, float output_max);				//PID������ʼ������
		void PID_General_Cal(PID *pid, float fdbV, float tarV);																																								//PID���㺯��
		void Add_TxMessage(float Output,uint8_t* Tx_Data);																																										//CAN������Ϣ��Ӻ���
		void CAN_Send(Can_TxMessageTypeDef* TxMessage);																																												//CAN������Ϣ����
	#endif
	//----------------------------------------------------------------------------------------------//
	
	//-------------------------------------���Ź�����-----------------------------------------------//
	#if WATCHDOG_ENABLE
		typedef enum SystemState										//ϵͳ״̬
		{
			WORKING,																	//��������
			MISSING,																	//��ʧ
		}SystemState;

		typedef struct Protect_System								//ϵͳ���Ź��ṹ��
		{
			SystemState State;												//ϵͳ��ǰ״̬
			int16_t Count_Time;												//���Ź�ʱ��
		}Protect_System;
		
		#define WATCHDOG_TIME_MAX 300																								//���Ź���ʱ��
		#if MOTOR_SYSTEM_USER_ROOT
			weak void Feed_WatchDog(Protect_System* Dogs);														//���Ź�ι������
			weak void SystemState_Set(Protect_System* Dogs,SystemState State);				//ϵͳ״̬�л�����
			weak uint8_t System_Check(Protect_System* Dogs);													//ϵͳ״̬��⺯��
		#else
			void Feed_WatchDog(Protect_System* Dogs);																	//���Ź�ι������
			void SystemState_Set(Protect_System* Dogs,SystemState State);							//ϵͳ״̬�л�����
			uint8_t System_Check(Protect_System* Dogs);																//ϵͳ״̬��⺯��
		#endif
	#endif
	//----------------------------------------------------------------------------------------------//
	
	//--------------------------------------ȫϵͳ����----------------------------------------------//
	#if SYSTEM_ENABLE
		typedef struct Motor_Info										//����ȫϵͳ�����Ƶĵ����Ϣ
		{
			int16_t Speed;														//����ٶ�				��λ(rad/min ת/ÿ����)
			uint16_t Angle;														//ת�ӻ�е�Ƕ�
			int32_t Abs_Angle;												//ת�Ӿ��Ի�е�Ƕ�
			float Relative_Angle;											//����������Ƕ�		��λ(�� ��)
			uint8_t Temperature;											//����¶�				��λ(�� ���϶�)
			int16_t Electric;													//����					��λ(mA ����)
			uint16_t Last_Angle;											//��һ�ε�ת�Ӿ��ԽǶ�
		}Motor_Info;
		
		typedef struct Motor_System									//ȫϵͳ
		{
			Motor_Info Info;													//�����Ϣ
			PID Pos_PID;															//λ�û�PID����
			PID Speed_PID;														//�ٶȻ�PID����
			float Tar_Pos;														//Ŀ��λ��
			float Tar_Speed;													//Ŀ���ٶ�
			uint8_t Motor_Num;												//�������
			#if WATCHDOG_ENABLE
				Protect_System Protect; 								//���Ź�����
			#endif
			Can_TxMessageTypeDef* TxMessage;					//CAN������Ϣָ��
			void (*Info_Analysis) (Motor_Info* P_Motor,uint8_t* RX_Data);
		}Motor_System;
		
		#if MOTOR_SYSTEM_USER_ROOT
			weak void Motor_System_Init(Motor_System* P_Motor,uint8_t Motor_Num,Can_TxMessageTypeDef* TxMessage,void (*Info_Analysis)(Motor_Info* P_Motor,uint8_t* RX_Data));		//ȫϵͳ���Ƴ�ʼ������
			weak void Motor_System_Analysis(Motor_Info* P_Motor,uint8_t* RX_Data);																		//ȫϵͳ���Ƶ�����ݷ�������
			weak void Motor_Extra_Analysis(Motor_Info* P_Motor);																										//ȫϵͳ����PID���㺯��
		#else
			void Motor_System_Init(Motor_System* P_Motor,uint8_t Motor_Num,Can_TxMessageTypeDef* TxMessage,void (*Info_Analysis)(Motor_Info* P_Motor,uint8_t* RX_Data));				//ȫϵͳ���Ƴ�ʼ������
			void Motor_System_Analysis(Motor_Info* P_Motor,uint8_t* RX_Data);																					//ȫϵͳ���Ƶ�����ݷ�������
			void Motor_Extra_Analysis(Motor_Info* P_Motor);																													//ȫϵͳ����PID���㺯��
		#endif
	#endif
	//----------------------------------------------------------------------------------------------//

	//--------------------------------------λ�û�����----------------------------------------------//
	#if POS_SYSTEM_ENABLE
		typedef struct Motor_Pos_Info								//����λ�û����Ƶĵ����Ϣ
		{
			int16_t Speed;														//����ٶ�				��λ(rad/min ת/ÿ����)
			uint16_t Angle;														//ת�ӻ�е�Ƕ�
			int32_t Abs_Angle;												//ת�Ӿ��Ի�е�Ƕ�
			float Relative_Angle;											//����������Ƕ�		��λ(�� ��)
			uint8_t Temperature;											//����¶�				��λ(�� ���϶�)
			int16_t Electric;													//����					��λ(mA ����)
			uint16_t Last_Angle;											//��һ�ε�ת�Ӿ��ԽǶ�
		}Motor_Pos_Info;
						
		typedef struct Pos_System										//λ�û�ϵͳ
		{
			Motor_Pos_Info Info;											//λ�û������Ϣ
			PID Pos_PID;															//λ�û�PID����
			PID Speed_PID;														//�ٶȻ�PID����
			float Tar_Pos;														//Ŀ��λ��
			uint8_t Motor_Num;												//�������
			#if WATCHDOG_ENABLE
				Protect_System Protect; 									//���Ź�����
			#endif
			Can_TxMessageTypeDef* TxMessage;					//CAN������Ϣָ��
			void (*Info_Analysis)(Motor_Pos_Info* P_Pos,uint8_t* RX_Data);
		}Pos_System;

		#if MOTOR_SYSTEM_USER_ROOT 
			weak void Pos_System_Init(Pos_System* P_Pos,uint8_t Motor_Num,Can_TxMessageTypeDef* TxMessage,void (*Info_Analysis)(Motor_Pos_Info* P_Pos,uint8_t* RX_Data););			//λ�û�����ϵͳ��ʼ������
			weak void Pos_System_Analysis(Motor_Pos_Info* P_Pos,uint8_t* RX_Data);															//λ�û�����ϵͳ���ݷ�������
			weak void PID_Pos_Cal(Pos_System* Pos_Motor);																												//λ�û�����ϵͳPID���㺯��
		#else
			void Pos_System_Init(Pos_System* P_Pos,uint8_t Motor_Num,Can_TxMessageTypeDef* TxMessage,void (*Info_Analysis)(Motor_Pos_Info* P_Pos,uint8_t* RX_Data));					//λ�û�����ϵͳ��ʼ������
			void Pos_System_Analysis(Motor_Pos_Info* Motor,uint8_t* RX_Data);																		//λ�û�����ϵͳ���ݷ�������
			void PID_Pos_Cal(Pos_System* Pos_Motor);																														//λ�û�����ϵͳPID���㺯��
		#endif
	#endif
	//----------------------------------------------------------------------------------------------//
	
	//--------------------------------------�ٶȻ�����----------------------------------------------//
	#if SPEED_SYSTEM_ENABLE
		typedef struct Motor_Speed_Info							//�����ٶȻ����Ƶĵ����Ϣ
		{
			int16_t Speed;														//����ٶ�				��λ(rad/min ת/ÿ����)
			uint8_t Temperature;											//����¶�				��λ(�� ���϶�)
			int16_t Electric;													//����					��λ(mA ����)
		}Motor_Speed_Info;
		
		typedef struct Speed_System									//�ٶȻ�ϵͳ
		{
			Motor_Speed_Info Info;										//�ٶȻ������Ϣ
			PID Speed_PID;														//�ٶȻ�PID����
			float Tar_Speed;													//Ŀ���ٶ�
			uint8_t Motor_Num;												//�������
			#if WATCHDOG_ENABLE
				Protect_System Protect; 								//���Ź�����
			#endif
			Can_TxMessageTypeDef* TxMessage;					//CAN������Ϣָ��
			void (*Info_Analysis)(Motor_Speed_Info* Motor,uint8_t* RX_Data);	
		}Speed_System;
		
		#if MOTOR_SYSTEM_USER_ROOT 
			weak void Speed_System_Init(Speed_System* P_Speed,uint8_t Motor_Num,Can_TxMessageTypeDef* TxMessage,void (*Info_Analysis)(Motor_Speed_Info* Motor,uint8_t* RX_Data));	//�ٶȻ�����ϵͳ��ʼ������
			weak void Speed_System_Analysis(Motor_Speed_Info* Motor,uint8_t* RX_Data);															//�ٶȻ�������ݷ����Ĳ�������
			weak void PID_Speed_Cal(Speed_System* Speed_Motor);																										//�ٶȻ�ϵͳPID���㺯��
		#else
			void Speed_System_Init(Speed_System* P_Speed,uint8_t Motor_Num,Can_TxMessageTypeDef* TxMessage,void (*Info_Analysis)(Motor_Speed_Info* Motor,uint8_t* RX_Data));			//�ٶȻ�����ϵͳ��ʼ������
			void Speed_System_Analysis(Motor_Speed_Info* Motor,uint8_t* RX_Data);																		//�ٶȻ�������ݷ����Ĳ�������
			void PID_Speed_Cal(Speed_System* Speed_Motor);																												//�ٶȻ�ϵͳPID���㺯��
		#endif
	#endif
	//----------------------------------------------------------------------------------------------//
		
	//---------------------------------------����ӿ�----------------------------------------------//
	extern Can_TxMessageTypeDef Can_TxMessageList[CAN_TXMESSAGEINDEXMAX];
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


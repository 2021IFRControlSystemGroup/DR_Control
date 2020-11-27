#ifndef __MOTOR_SYSTEM_H__
#define __MOTOR_SYSTEM_H__

//---------ͷ�ļ���������----------//
#include "main.h"
#include "math.h"
#include "can.h"
//---------------------------------//

//---------#define����-------------//
#define PI (2*acos(0))																	//PIԲ���ʵĺ궨��
#define ToDegree(a) (a/PI*180)													//����ת���ɽǶȵĺ궨��
#define ToRadian(a) (a/180*PI)													//�Ƕ�ת���ɻ��ȵĺ궨��

#define ROTOR_ANGLE 8192																//ת�ӻ�е�Ƕ�
#define GEAR_RATIO 36																		//������ٱ�
#define ONE_CIRCLE (ROTOR_ANGLE*GEAR_RATIO)							//���ת��һȦ���ܻ�е�Ƕ�

#define WATCHDOG_TIME_MAX 300														//���Ź���ʱ��
#define RUNNING_TIME_MAX 500000													//ϵͳ����ʱ�����ֵ
#define POS_SYSTEM_CHECK if(System_Check(&P_Pos->Protect)) Error_State=(RoboBaseState)(P_Pos->Motor_Num+1);						//λ�û����Ź����
#define SPEED_SYSTEM_CHECK if(System_Check(&P_Speed->Protect)) Error_State=(RoboBaseState)(P_Speed->Motor_Num+5);			//�ٶȻ����Ź����
//---------------------------------//

//---------���̽ṹ�岿��----------//
typedef enum SystemState										//ϵͳ״̬
{
	WORKING,																	//��������
	MISSING,																	//��ʧ
	SUSPENDING																//����
}SystemState;

typedef struct Protect_System								//ϵͳ���Ź��ṹ��
{
  SystemState State;												//ϵͳ��ǰ״̬
  int16_t Count_Time;												//���Ź�ʱ��
}Protect_System;

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

typedef struct Motor_Speed_Info							//�����ٶȻ����Ƶĵ����Ϣ
{
  int16_t Speed;														//����ٶ�				��λ(rad/min ת/ÿ����)
  uint8_t Temperature;											//����¶�				��λ(�� ���϶�)
  int16_t Electric;													//����					��λ(mA ����)
}Motor_Speed_Info;

typedef struct pid_init_val{								//���PID�����ṹ��
	
	float Kp;
	float Ki;
	float Kd;
	
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

typedef struct Pos_System										//λ�û�ϵͳ
{
  Motor_Pos_Info Info;											//λ�û������Ϣ
  PID Pos_PID;															//λ�û�PID����
  PID Speed_PID;														//�ٶȻ�PID����
  float Tar_Pos;														//Ŀ��λ��
  uint8_t Motor_Num;												//�������
  Protect_System Protect; 
}Pos_System;

typedef struct Speed_System									//�ٶȻ�ϵͳ
{
  Motor_Speed_Info Info;										//�ٶȻ������Ϣ
  PID Speed_PID;														//�ٶȻ�PID����
  float Tar_Speed;													//Ŀ���ٶ�
  uint8_t Motor_Num;												//�������
  Protect_System Protect; 
}Speed_System;
//---------------------------------//

//-------------��������------------//
void Speed_Info_Analysis(Motor_Speed_Info* Motor,uint8_t* RX_Data);									//�ٶȻ�������ݷ����Ĳ�������
void Pos_Info_Analysis(Motor_Pos_Info* Motor,uint8_t* RX_Data);											//λ�û�������ݷ����Ĳ�������

void PID_Init(PID *pid, float Kp, float Ki, float Kd, float error_max, float dead_line, float intergral_max, float output_max);		//PID������ʼ������
void PID_General_Cal(PID *pid, float fdbV, float tarV,uint8_t moto_num,uint8_t *Tx_msg);					//PID���㺯��----Ϊ�����¼���
void PID_Speed_Cal(Speed_System* Speed_Motor,uint8_t *Tx_msg);																		//�ٶȻ�ϵͳPID���㺯��
void PID_Pos_Cal(Pos_System* Pos_Motor,uint8_t *Tx_msg);																					//λ�û�ϵͳPID���㺯��
void Send_To_Motor(CAN_HandleTypeDef *hcan,uint8_t* Tx_Data);								//CANͨ�ŷ��ͺ���

void Feed_WatchDog(Protect_System* Dogs);																		//���Ź�ι������
void SystemState_Set(Protect_System* Dogs,SystemState State);								//ϵͳ״̬�л�����
uint8_t System_Check(Protect_System* Dogs);																	//ϵͳ״̬��⺯��
//---------------------------------//
#endif


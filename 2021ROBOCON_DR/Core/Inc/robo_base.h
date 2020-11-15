#ifndef __ROBOBASE_H__
#define __ROBOBASE_H__

//---------ͷ�ļ���������----------//
#include "main.h"
#include "math.h"
//---------------------------------//

//---------#define����-------------//
#define PI (2*acos(0))							//PIԲ���ʵĺ궨��
#define ToDegree(a) (a/PI*180)					//����ת���ɽǶȵĺ궨��
#define ToRadian(a) (a/180*PI)					//�Ƕ�ת���ɻ��ȵĺ궨��
#define TX_LENGTH 20							//����λ��ͨ���ַ��ܳ���
#define ROTOR_ANGLE 8192						//ת�ӻ�е�Ƕ�
#define GEAR_RATIO 19							//������ٱ�
#define ONE_CIRCLE (ROTOR_ANGLE*GEAR_RATIO)		//���ת��һȦ���ܻ�е�Ƕ�
//---------------------------------//

//---------���̽ṹ�岿��----------//
typedef struct Motor_Pos_Info		//����λ�û����Ƶĵ����Ϣ
{
  int16_t Speed;					//����ٶ�				��λ(rad/min ת/ÿ����)
  uint16_t Angle;					//ת�ӻ�е�Ƕ�
  int32_t Abs_Angle;				//ת�Ӿ��Ի�е�Ƕ�
  float Relative_Angle;				//����������Ƕ�		��λ(�� ��)
  uint8_t Temperature;				//����¶�				��λ(�� ���϶�)
  int16_t Electric;					//����					��λ(mA ����)
  uint16_t Last_Angle;				//��һ�ε�ת�Ӿ��ԽǶ�
}Motor_Pos_Info;

typedef struct Motor_Speed_Info		//�����ٶȻ����Ƶĵ����Ϣ
{
  int16_t Speed;					//����ٶ�				��λ(rad/min ת/ÿ����)
  uint8_t Temperature;				//����¶�				��λ(�� ���϶�)
  int16_t Electric;					//����					��λ(mA ����)
}Motor_Speed_Info;

typedef struct pid_init_val{		//���PID�����ṹ��
	
	float Kp;
	float Ki;
	float Kd;
	
	float error;					//���
	float error_last;				//��һ�����
	float error_max;				//������
	float dead_line;				//����
	
	float intergral;				//������
	float intergral_max;			//���������ֵ
	
	float derivative;				//���΢��
	
	float output;					//���
	float output_max;				//������ֵ
	
}PID;

typedef struct Pos_System			//λ�û�ϵͳ
{
  Motor_Pos_Info Info;				//λ�û������Ϣ
  PID Pos_PID;						//λ�û�PID����
  PID Speed_PID;					//�ٶȻ�PID����
  float Tar_Pos;					//Ŀ��λ��
  uint8_t Motor_Num;				//�������
}Pos_System;

typedef struct Speed_System			//�ٶȻ�ϵͳ
{
  Motor_Speed_Info Info;			//�ٶȻ������Ϣ
  PID Speed_PID;					//�ٶȻ�PID����
  float Tar_Speed;					//Ŀ���ٶ�
  uint8_t Motor_Num;				//�������
}Speed_System;

typedef struct Robo_Base			//���̽ṹ��
{
	Pos_System Pos_MotorLF;			//λ�û�--��ǰ��
	Pos_System Pos_MotorLB;			//λ�û�--�����
	Pos_System Pos_MotorRF;			//λ�û�--��ǰ��
	Pos_System Pos_MotorRB;			//λ�û�--�Һ���
	
	Speed_System Speed_MotorLF;		//�ٶȻ�--��ǰ��
	Speed_System Speed_MotorLB;		//�ٶȻ�--�����
	Speed_System Speed_MotorRF;		//�ٶȻ�--��ǰ��
	Speed_System Speed_MotorRB;		//�ٶȻ�--�Һ���

	int Speed_X;					//����X������Ŀ���ٶ�
	int Speed_Y;					//����Y������Ŀ���ٶ�
	float Angle;					//�����˶�����Է���
	
	uint8_t Tx_CAN2[8];				//CAN2ͨ�ŷ�������
	uint8_t Tx_CAN1[8];				//CAN1ͨ�ŷ�������
	uint8_t Rx_CAN2[8];				//CAN2ͨ�Ž�������
	uint8_t Rx_CAN1[8];				//CAN1ͨ�Ž�������
}ROBO_BASE;

typedef struct TX_BUFFER			//����λ��ͨ�Žṹ��
{
	uint8_t Tx_buffer[TX_LENGTH];	//��������
	uint8_t length;					//�������ݳ���
}TX_BUFFER;
//---------------------------------//

//-------------��������------------//
void Motor_Pos_Analysis(ROBO_BASE* Robo,uint8_t* RX_Data,uint32_t Motor_Num);				//λ�û�������ݷ����Ľӿں���
void Pos_Info_Analysis(Motor_Pos_Info* Motor,uint8_t* RX_Data);								//λ�û�������ݷ����Ĳ�������

void Motor_Speed_Analysis(ROBO_BASE* Robo,uint8_t* RX_Data,uint32_t Motor_Num);				//�ٶȻ�������ݷ����Ľӿں���
void Speed_Info_Analysis(Motor_Speed_Info* Motor,uint8_t* RX_Data);							//�ٶȻ�������ݷ����Ĳ�������

void SystemIO_Usart_ToString(int32_t System_Out,int32_t System_In);							//ϵͳ�����������ֵת�����ַ��ĺ���

void PID_Init(PID *pid, float Kp, float Ki, float Kd, float error_max, float dead_line, float intergral_max, float output_max);		//PID������ʼ������
void BASE_Init(ROBO_BASE *Robo);																									//����PID������ʼ���Ľӿں���

void PID_General_Cal(PID *pid, float fdbV, float tarV,uint8_t moto_num,uint8_t *Tx_msg);	//PID���㺯��----Ϊ�����¼���
void PID_Speed_Cal(Speed_System* Speed_Motor,uint8_t *Tx_msg);								//�ٶȻ�ϵͳPID���㺯��
void PID_Pos_Cal(Pos_System* Pos_Motor,uint8_t *Tx_msg);									//λ�û�ϵͳPID���㺯��

void PID_Send(ROBO_BASE* Robo);																//PID���ͺ���
void Send_To_Motor(CAN_HandleTypeDef *hcan,uint8_t* Tx_Data);								//CANͨ�ŷ��ͺ���
//---------------------------------//
#endif



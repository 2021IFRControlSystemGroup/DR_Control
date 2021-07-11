#ifndef __ROBOBASE_H__
#define __ROBOBASE_H__

//---------ͷ�ļ���������----------//
#include "motor_system.h"
#include "odrive_system.h"
#include "usart_analysis.h"
//---------------------------------//

//---------#define����-------------//
#define RUNNING_TIME_MAX 500000													//ϵͳ����ʱ�����ֵ
//---------------------------------//

//---------���̽ṹ�岿��----------//

//																����״̬
//	SYSTEM_WORKING	=	(1<<0)|1,														//��������
//	INIT_STATE			=	(1<<1)|1,														//��ʼ��ģʽ
//	MOVE_STATE			=	(1<<2)|1,														//�ƶ�ģʽ

//	LF_POS_ERROR		=	(1<<1),															//��ǰת��������
//	LB_POS_ERROR		=	(1<<2),															//���ת��������
//	RF_POS_ERROR		=	(1<<3),															//��ǰת��������
//	RB_POS_ERROR		=	(1<<4),															//�Һ�ת��������
//	LF_AXIS_ERROR		=	(1<<5),															//��ǰ�����������
//	LB_AXIS_ERROR		=	(1<<6),															//��������������
//	RF_AXIS_ERROR		=	(1<<7),															//��ǰ�����������
//	RB_AXIS_ERROR		=	(1<<8),															//�Һ������������

typedef struct CanBuffer										//CANͨ�Žṹ��
{
	uint8_t Tx[8];														//��������֡
	uint8_t Rx[8];														//��������֡
}CanBuffer;

typedef struct MotorGroup									//������ṹ��
{
	MotorSystem _Pos;													//λ�û�ϵͳ�ṹ��
	Axis* _Axis;															//ODrive��Axis�ṹ��
}MotorGroup;

typedef struct RoboBase										//���̽ṹ��
{
	MotorGroup LF;														//������--��ǰ��
	MotorGroup LB;														//������--�����
	MotorGroup RF;														//������--��ǰ��
	MotorGroup RB;														//������--�Һ���

	float Speed_X;													//����X������Ŀ���ٶ�
	float Speed_Y;													//����Y������Ŀ���ٶ�
	float Angle;															//�����˶�����Է���
    float Speed_Rotate;
    
    uint32_t Last_WorkingState;
	uint32_t Working_State;										//����״̬
	uint32_t Error_State;
	
	CanBuffer Can1;													//CAN1ͨ�ŷ�������
	CanBuffer Can2;													//CAN2ͨ�ŷ�������
	
	volatile uint32_t Running_Time;										//����ʱ��
}RoboBase;

//---------------------------------//

//-------------��������------------//
void Pos_CloseLoop_Init(MotorSystem* P_Motor);
void BASE_Init(void);																									//���̽ṹ���Ա��ʼ���Ľӿں���
void Motor_CAN_Recevice(uint32_t Motor_Num, uint8_t* RX_Data);
void Can_TxMessage_MoveMode(void);																					//������µ��̵�Can�������ݺ���
void Counting_Time(void);																							//��¼��������ʱ�亯��
void Base_WatchDog(void);																					//���̿��Ź��ӿں���
//---------------------------------//

extern RoboBase Robo_Base;
#endif



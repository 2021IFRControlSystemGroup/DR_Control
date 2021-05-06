#ifndef __ROBOBASE_H__
#define __ROBOBASE_H__

//---------ͷ�ļ���������----------//
#include "motor_system.h"
#include "odrive_can.h"
#include "uart_communicate.h"
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


typedef struct CAN_BUFFER										//CANͨ�Žṹ��
{
	uint8_t Tx[8];														//��������֡
	uint8_t Rx[8];														//��������֡
}CAN_BUFFER;

typedef struct Motor_Group									//������ṹ��
{
	Pos_System _Pos;													//λ�û�ϵͳ�ṹ��
	Axis* _Axis;															//ODrive��Axis�ṹ��
}Motor_Group;

typedef struct Robo_Base										//���̽ṹ��
{
	Motor_Group LF;														//������--��ǰ��
	Motor_Group LB;														//������--�����
	Motor_Group RF;														//������--��ǰ��
	Motor_Group RB;														//������--�Һ���

	float Speed_X;													//����X������Ŀ���ٶ�
	float Speed_Y;													//����Y������Ŀ���ٶ�
	float Angle;															//�����˶�����Է���

	uint32_t Working_State;										//����״̬
	
	CAN_BUFFER Can1;													//CAN1ͨ�ŷ�������
	CAN_BUFFER Can2;													//CAN2ͨ�ŷ�������
	
	uint32_t Running_Time;										//����ʱ��
}ROBO_BASE;

//---------------------------------//

//-------------��������------------//
void Pos_CloseLoop_Init(Pos_System* P_Pos);
void BASE_Init(void);																									//���̽ṹ���Ա��ʼ���Ľӿں���
void Motor_Pos_Analysis(uint8_t* RX_Data,uint32_t Motor_Num);					//λ�û�������ݷ����Ľӿں���
void Motor_Speed_Analysis(uint8_t* RX_Data,uint32_t Motor_Num);				//�ٶȻ�������ݷ����Ľӿں���
void Can_TxMessageCal(void);																					//������µ��̵�Can�������ݺ���
void Counting_Time(void);																							//��¼��������ʱ�亯��
void Base_WatchDog(void);																					//���̿��Ź��ӿں���
//---------------------------------//
#endif



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
typedef enum RoboBaseState									//����״̬
{
	SYSTEM_WORKING=0,														//��������
	LF_POS_ERROR=(1<<0),
	LB_POS_ERROR=(1<<1),
	RF_POS_ERROR=(1<<2),
	RB_POS_ERROR=(1<<3),
	LF_AXIS_ERROR=(1<<4),
	LB_AXIS_ERROR=(1<<5),
	RF_AXIS_ERROR=(1<<6),
	RB_AXIS_ERROR=(1<<7),
}RoboBaseState;

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

	int32_t Speed_X;													//����X������Ŀ���ٶ�
	int32_t Speed_Y;													//����Y������Ŀ���ٶ�
	float Angle;															//�����˶�����Է���

	uint8_t Working_State;											//����״̬
	
	CAN_BUFFER Can1;													//CAN1ͨ�ŷ�������
	CAN_BUFFER Can2;													//CAN2ͨ�ŷ�������
	
	uint32_t Running_Time;										//����ʱ��
}ROBO_BASE;

//---------------------------------//

//-------------��������------------//
void BASE_Init(void);																									//���̽ṹ���Ա��ʼ���Ľӿں���
void Motor_Pos_Analysis(uint8_t* RX_Data,uint32_t Motor_Num);					//λ�û�������ݷ����Ľӿں���
void Motor_Speed_Analysis(uint8_t* RX_Data,uint32_t Motor_Num);				//�ٶȻ�������ݷ����Ľӿں���
void Can_TxMessageCal(void);																					//������µ��̵�Can�������ݺ���
void Counting_Time(void);																							//��¼��������ʱ�亯��
uint8_t Base_WatchDog(void);																					//���̿��Ź��ӿں���
//---------------------------------//
#endif



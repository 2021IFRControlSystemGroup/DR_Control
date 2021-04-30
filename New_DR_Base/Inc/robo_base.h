#ifndef __ROBOBASE_H__
#define __ROBOBASE_H__

//---------ͷ�ļ���������----------//
#include "motor_system.h"
#include "odrive_can.h"
#include "uart_communicate.h"
//---------------------------------//

//---------#define����-------------//
#define UART2_TX_LENGTH 20															//UART2����ͨ���ַ��ܳ���
#define UART2_RX_LENGTH 20															//UART2����ͨ���ַ��ܳ���
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

typedef struct UART_BUFFER									//UARTͨ�Žṹ��
{
	uint8_t Tx_buffer[UART2_TX_LENGTH];				//��������
	uint8_t Tx_length;												//�������ݳ���
	uint8_t Rx_buffer[UART2_RX_LENGTH];				//��������
	uint8_t Rx_length;												//�������ݳ���	
}UART_BUFFER;

typedef struct CAN_BUFFER										//CANͨ�Žṹ��
{
	uint8_t Tx[8];														//��������֡
	uint8_t Rx[8];														//��������֡
}CAN_BUFFER;

typedef struct Motor_Group
{
	Pos_System _Pos;
	Axis* _Axis;
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
void Motor_Pos_Analysis(uint8_t* RX_Data,uint32_t Motor_Num);				//λ�û�������ݷ����Ľӿں���
void Motor_Speed_Analysis(uint8_t* RX_Data,uint32_t Motor_Num);				//�ٶȻ�������ݷ����Ľӿں���

void SystemIO_Usart_ToString(ROBO_BASE* Robo,int32_t System_Out,int32_t System_In);			//ϵͳ�����������ֵת�����ַ��ĺ���

void BASE_Init(void);																									//����PID������ʼ���Ľӿں���

void PID_Send(uint8_t ODrive_num);																							//PID���ͺ���

void Counting_Time(ROBO_BASE* Robo);
void LED_WARNING(ROBO_BASE* Robo);
uint8_t Base_WatchDog(void);														//���̿��Ź��ӿں���
//---------------------------------//
#endif



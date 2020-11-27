#ifndef __ROBOBASE_H__
#define __ROBOBASE_H__

//---------ͷ�ļ���������----------//
#include "motor_system.h"
#include "uart_communicate.h"
//---------------------------------//

//---------#define����-------------//
#define UART2_TX_LENGTH 20															//UART2����ͨ���ַ��ܳ���
#define UART2_RX_LENGTH 20															//UART2����ͨ���ַ��ܳ���
//---------------------------------//

//---------���̽ṹ�岿��----------//
typedef enum RoboBaseState									//����״̬
{
	SYSTEM_WORKING,														//��������
	Pos_MotorLF_ERROR,												//��ǰת���ִ���
	Pos_MotorLB_ERROR,												//���ת���ִ���
	Pos_MotorRF_ERROR,												//��ǰת���ִ���
	Pos_MotorRB_ERROR,												//�Һ�ת���ִ���
	Speed_MotorLF_ERROR,											//��ǰ�����ִ���
	Speed_MotorLB_ERROR,											//��������ִ���
	Speed_MotorRF_ERROR,											//��ǰ�����ִ���
	Speed_MotorRB_ERROR,											//�Һ������ִ���
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

typedef struct Robo_Base										//���̽ṹ��
{
	Pos_System Pos_MotorLF;										//λ�û�--��ǰ��
	Pos_System Pos_MotorLB;										//λ�û�--�����
	Pos_System Pos_MotorRF;										//λ�û�--��ǰ��
	Pos_System Pos_MotorRB;										//λ�û�--�Һ���
	
	Speed_System Speed_MotorLF;								//�ٶȻ�--��ǰ��
	Speed_System Speed_MotorLB;								//�ٶȻ�--�����
	Speed_System Speed_MotorRF;								//�ٶȻ�--��ǰ��
	Speed_System Speed_MotorRB;								//�ٶȻ�--�Һ���

	int32_t Speed_X;													//����X������Ŀ���ٶ�
	int32_t Speed_Y;													//����Y������Ŀ���ٶ�
	float Angle;															//�����˶�����Է���

	RoboBaseState State;											//����״̬
	
	UART_BUFFER Uart2;												//Uart2ͨ�ŷ�������
	CAN_BUFFER Can1;													//CAN1ͨ�ŷ�������
	CAN_BUFFER Can2;													//CAN2ͨ�ŷ�������
	
	uint32_t Running_Time;										//����ʱ��
}ROBO_BASE;

//---------------------------------//

//-------------��������------------//
void Motor_Pos_Analysis(ROBO_BASE* Robo,uint8_t* RX_Data,uint32_t Motor_Num);				//λ�û�������ݷ����Ľӿں���
void Motor_Speed_Analysis(ROBO_BASE* Robo,uint8_t* RX_Data,uint32_t Motor_Num);				//�ٶȻ�������ݷ����Ľӿں���

void SystemIO_Usart_ToString(ROBO_BASE* Robo,int32_t System_Out,int32_t System_In);			//ϵͳ�����������ֵת�����ַ��ĺ���

void BASE_Init(ROBO_BASE *Robo);																									//����PID������ʼ���Ľӿں���

void PID_Send(ROBO_BASE* Robo);																							//PID���ͺ���

void Counting_Time(ROBO_BASE* Robo);
void LED_WARNING(ROBO_BASE* Robo);
void Base_WatchDog(ROBO_BASE* Robo);														//���̿��Ź��ӿں���
//---------------------------------//
#endif



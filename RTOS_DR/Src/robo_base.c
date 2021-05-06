//�ļ�����:		robo_base.c
//��Ӧͷ�ļ�:	robo_base.h
//��Ҫ����:
//		����motor_system��װ���ٶȻ�/λ�û�/���Ź�����, ʵ�ֵ��̿���
//
//ʱ��:
//		2020/11/27
//
//�汾:	2.0V
//
//״̬:
//		�ѳ�������
//
//��������:
//		�޵������²��Գɹ�, ���Ź�����ʵ��, ����ͨ������

//---------ͷ�ļ����ò���---------//
#include "robo_base.h"
//--------------------------------//

//---------������������-----------//
ROBO_BASE Robo_Base;
//--------------------------------//

//---------�ⲿ������������-------//
//--------------------------------//

//--------------------------------------------------------------------------------------------------//
//��������:
//		λ�û�������ݷ����Ľӿں���
//
//��������:
//		��ȡRobo_Base��Ӧ��CAN�ڴ��������, ���ݵ���������ֱ�����һ�����ӵ���Ϣ, Ȼ�󴢴�������.
//
//��������:
//		uint8_t* �����Ϣ������, �Ƽ�ʹ��Rx_CAN����, �������Բ���Ҫ�Լ�ȥ����.
//		uint32_t �������
//
//--------------------------------------------------------------------------------------------------//
void Motor_Pos_Analysis(uint8_t* RX_Data,uint32_t Motor_Num)
{
  Pos_System* P_Motor=NULL;
  switch(Motor_Num)
  {
    case 0x201:P_Motor=&Robo_Base.LF._Pos;break;
    case 0x202:P_Motor=&Robo_Base.LB._Pos;break;
    case 0x203:P_Motor=&Robo_Base.RF._Pos;break;
    case 0x204:P_Motor=&Robo_Base.RB._Pos;break;
	default:break;
  }if(!P_Motor) return ;
  Pos_Info_Analysis(&P_Motor->Info,RX_Data);
  Feed_WatchDog(&P_Motor->Protect);
}


//--------------------------------------------------------------------------------------------------//
//��������:
//		�ٶȻ�������ݷ����Ľӿں���
//
//��������:
//		��ȡRobo_Base��Ӧ��CAN�ڴ��������, ���ݵ���������ֱ�����һ�����ӵ���Ϣ, Ȼ�󴢴�������.
//
//��������:
//		ROBO_BASE ָ��, ���̽ṹ���ָ��
//		uint8_t* �����Ϣ������, �Ƽ�ʹ��Rx_CAN����, �������Բ���Ҫ�Լ�ȥ����.
//		uint32_t �������
//
//��ֲ����:
//		ֱ�Ӷ�case�����ݽ����޸�, �м����ٶȻ������ӾͼӼ���, Ȼ����ָ��ָ���Ӧ�����Ӿ���.
//
//--------------------------------------------------------------------------------------------------//
void Motor_Speed_Analysis(uint8_t* RX_Data,uint32_t Motor_Num)
{
  Speed_System* S_Motor=NULL;
  switch(Motor_Num)
  {
		default:break;
  }if(!S_Motor) return ;
  Speed_Info_Analysis(&S_Motor->Info,RX_Data);
  Feed_WatchDog(&S_Motor->Protect);
}

//--------------------------------------------------------------------------------------------------//
//��������:
//		���̲�����ʼ��
//
//��������:
//		��ʼ���������е���Ϣ,���Ұ�odrive��axis���ص��ṹ����
//
//��������:
//		��
//
//--------------------------------------------------------------------------------------------------//
void BASE_Init(void)
{
	extern ODrive ODrive0;
	extern ODrive ODrive1;
	
  Pos_System* P_Pos=NULL;																																								//ת������ʼ��
	P_Pos=&Robo_Base.LF._Pos; PID_Init(&P_Pos->Pos_PID,			0.3,	0,	0,	10000,	0,	0,	10000);
	Motor_Init(P_Pos,0);			PID_Init(&P_Pos->Speed_PID,			5,	0,	0,	5000,	0,	0,	4000);
  P_Pos=&Robo_Base.LB._Pos; PID_Init(&P_Pos->Pos_PID,			0,	0,	0,	0,	0,	0,	0);
  Motor_Init(P_Pos,1);			PID_Init(&P_Pos->Speed_PID,			0,	0,	0,	0,	0,	0,	0); 
  P_Pos=&Robo_Base.RF._Pos; PID_Init(&P_Pos->Pos_PID,			0,	0,	0,	0,	0,	0,	0);
  Motor_Init(P_Pos,2);			PID_Init(&P_Pos->Speed_PID,			0,	0,	0,	0,	0,	0,	0); 
  P_Pos=&Robo_Base.RB._Pos; PID_Init(&P_Pos->Pos_PID,			0,	0,	0,	0,	0,	0,	0);
  Motor_Init(P_Pos,3);			PID_Init(&P_Pos->Speed_PID,			0,	0,	0,	0,	0,	0,	0); 

	Axis* P_Axis=NULL;																																										//���������ʼ��
	P_Axis=Robo_Base.LF._Axis=&ODrive0.Axis0; Axis_Init(P_Axis,0);
	P_Axis=Robo_Base.LB._Axis=&ODrive0.Axis1; Axis_Init(P_Axis,1);
	P_Axis=Robo_Base.RF._Axis=&ODrive1.Axis0; Axis_Init(P_Axis,2);
	P_Axis=Robo_Base.RB._Axis=&ODrive1.Axis1; Axis_Init(P_Axis,3);
	Robo_Base.Working_State=1;
}

//--------------------------------------------------------------------------------------------------//
//��������:
//		PID���ͺ���
//
//��������:
//		���͵��PID
//
//��������:
//		ROBO_BASE* ���̽ṹ��ָ��
//
//��ֲ����:
//		����Ҫɶ���Ŀ��ƾ���ָ��ָ�����ϵͳ, Ȼ����ö�Ӧ��PID���㺯�����д���
//
//--------------------------------------------------------------------------------------------------//
void Can_TxMessageCal()
{
	PID_Pos_Cal(&Robo_Base.LF._Pos);
	PID_Pos_Cal(&Robo_Base.LB._Pos);
	PID_Pos_Cal(&Robo_Base.RF._Pos);
	PID_Pos_Cal(&Robo_Base.RB._Pos);
	
	ODrive_Transmit(Robo_Base.LF._Axis,0x0D);
	ODrive_Transmit(Robo_Base.LB._Axis,0x0D);
	ODrive_Transmit(Robo_Base.RF._Axis,0x0D);
	ODrive_Transmit(Robo_Base.RB._Axis,0x0D);
}

//--------------------------------------------------------------------------------------------------//
//��������:
//		���̿��Ź���⺯��
//
//��������:
//		��ʱ�������ϸ������״̬, ���������������̹���ģʽ
//
//��������:
//		ROBO_BASE* ���̽ṹ��ָ��
//
//��ֲ����:
//		��Ҫ��ӻ�ɾ��λ�û�ϵͳ��ֱ�����
//
//--------------------------------------------------------------------------------------------------//
void Base_WatchDog(void)
{
  uint32_t Error_State=0;
  Pos_System* P_Pos=NULL;
  P_Pos=&Robo_Base.LF._Pos;
	if(System_Check(&P_Pos->Protect)) Error_State|=(1<<(P_Pos->Motor_Num+1));
  P_Pos=&Robo_Base.LB._Pos;
	if(System_Check(&P_Pos->Protect)) Error_State|=(1<<(P_Pos->Motor_Num+1));
  P_Pos=&Robo_Base.RF._Pos;
	if(System_Check(&P_Pos->Protect)) Error_State|=(1<<(P_Pos->Motor_Num+1));
  P_Pos=&Robo_Base.RB._Pos;
	if(System_Check(&P_Pos->Protect)) Error_State|=(1<<(P_Pos->Motor_Num+1));
	
	Axis* P_Axis=NULL;
	P_Axis=Robo_Base.LF._Axis;
	if(P_Axis->Error!=0||System_Check(&P_Axis->Protect)) Error_State|=(1<<(P_Axis->Node_ID+5));
	P_Axis=Robo_Base.LB._Axis;
	if(P_Axis->Error!=0||System_Check(&P_Axis->Protect)) Error_State|=(1<<(P_Axis->Node_ID+5));
	P_Axis=Robo_Base.RF._Axis;
	if(P_Axis->Error!=0||System_Check(&P_Axis->Protect)) Error_State|=(1<<(P_Axis->Node_ID+5));
	P_Axis=Robo_Base.RB._Axis;
	if(P_Axis->Error!=0||System_Check(&P_Axis->Protect)) Error_State|=(1<<(P_Axis->Node_ID+5));
  
	if(Error_State) Robo_Base.Working_State=Error_State;
}

//--------------------------------------------------------------------------------------------------//
//��������:
//		��������ʱ�亯��
//
//��������:
//		��¼��������ʱ��
//
//��������:
//		��
//
//--------------------------------------------------------------------------------------------------//
void Counting_Time(void)
{
  Robo_Base.Running_Time++;
  if(Robo_Base.Running_Time>RUNNING_TIME_MAX) Robo_Base.Running_Time=0;
}

void Pos_CloseLoop_Init(Pos_System* P_Pos)
{
	static uint8_t num[4]={0};
	uint8_t* P_num=&num[P_Pos->Motor_Num];
	
	if(P_Pos->Info.Temperature==0) return ;
	if(*P_num<150){
		PID_Speed_Cal(P_Pos,300);
		if(P_Pos==&Robo_Base.LF._Pos) if(HAL_GPIO_ReadPin(TIM3_CH1_GPIO_Port,TIM3_CH1_Pin)==GPIO_PIN_RESET) (*P_num)++;
		if(P_Pos==&Robo_Base.LB._Pos) if(HAL_GPIO_ReadPin(TIM3_CH2_GPIO_Port,TIM3_CH2_Pin)==GPIO_PIN_RESET) (*P_num)++;
		if(P_Pos==&Robo_Base.RF._Pos) if(HAL_GPIO_ReadPin(TIM3_CH3_GPIO_Port,TIM3_CH3_Pin)==GPIO_PIN_RESET) (*P_num)++;
		if(P_Pos==&Robo_Base.RB._Pos) if(HAL_GPIO_ReadPin(TIM3_CH4_GPIO_Port,TIM3_CH4_Pin)==GPIO_PIN_RESET) (*P_num)++;
	}else if(*P_num==100) P_Pos->Info.Abs_Angle=0,PID_Speed_Cal(P_Pos,0);
}


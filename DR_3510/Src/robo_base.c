//�ļ�����:		robo_base.c
//��Ӧͷ�ļ�:	robo_base.h
//��Ҫ����:
//		����motor_system��װ���ٶȻ�/λ�û�/���Ź�����, ʵ�ֵ��̿���
//
//ʱ��:
//		2021/5/8
//
//�汾:	3.0V
//
//---------ͷ�ļ����ò���---------//
#include "robo_base.h"
//--------------------------------//

//---------������������-----------//
ROBO_BASE Robo_Base;
//--------------------------------//


//--------------------------------------------------------------------------------------------------//
//��������:
//		���̲�����ʼ��
//
//��������:
//		��ʼ���������е���Ϣ
//
//��������:
//		ROBO_BASE ָ��, ���̽ṹ���ָ��
//
//��ֲ����:
//		��ʲô״̬, ���, ���״̬���Ȱ����ݷ�װ��ROBO_BASE�ṹ����, Ȼ��ֱ�ӳ�ʼ���ͺ���
//
//--------------------------------------------------------------------------------------------------//
#if UART_COMMUNICATE_USER_ROOT
	weak
#endif
void Base_Init(void)
{
  Speed_System* P_Speed=NULL; 
  P_Speed=&Robo_Base.LF; Speed_System_Init(P_Speed,0,&Can_TxMessageList[0],Speed_System_Analysis);
	PID_Init(&P_Speed->Speed_PID,			10,	0.05,	0,	5000,	0,	0,	7000);
	P_Speed=&Robo_Base.LB; Speed_System_Init(P_Speed,1,&Can_TxMessageList[0],Speed_System_Analysis);
	PID_Init(&P_Speed->Speed_PID,			5,	0,	0,	5000,	0,	0,	7000);
	P_Speed=&Robo_Base.RF; Speed_System_Init(P_Speed,2,&Can_TxMessageList[0],Speed_System_Analysis);
	PID_Init(&P_Speed->Speed_PID,			5,	0,	0,	5000,	0,	0,	7000);
	P_Speed=&Robo_Base.RB; Speed_System_Init(P_Speed,3,&Can_TxMessageList[0],Speed_System_Analysis);
	PID_Init(&P_Speed->Speed_PID,			5,	0,	0,	5000,	0,	0,	7000);
	

}

#if UART_COMMUNICATE_USER_ROOT
	weak
#endif
void Motor_Analysis(uint8_t* RX_Data,uint32_t Motor_Num)
{
	Speed_System* P_Speed=NULL;
	switch(Motor_Num)
	{
		case 0x201:P_Speed=&Robo_Base.LF;break;
		case 0x202:P_Speed=&Robo_Base.LB;break;
		case 0x203:P_Speed=&Robo_Base.RF;break;
		case 0x204:P_Speed=&Robo_Base.RB;break;
		default:break;
	}
	if(P_Speed) 	P_Speed->Info_Analysis(&P_Speed->Info,RX_Data);
	#if WATCHDOG_ENABLE
		Feed_WatchDog(&P_Speed->Protect);
	#endif
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
#if UART_COMMUNICATE_USER_ROOT
	weak
#endif
void Send_RoboBasePID(void)
{
  Speed_System* P_Speed=NULL;
  P_Speed=&Robo_Base.LF; PID_Speed_Cal(P_Speed);
  P_Speed=&Robo_Base.LB; PID_Speed_Cal(P_Speed);
  P_Speed=&Robo_Base.RF; PID_Speed_Cal(P_Speed);
  P_Speed=&Robo_Base.RB; PID_Speed_Cal(P_Speed);
	
	if(Robo_Base.Speed_X==0) Extended_Integral_PID();
	P_Speed=&Robo_Base.LF; Add_TxMessage(P_Speed->Speed_PID.output,P_Speed->TxMessage->Data);
	P_Speed=&Robo_Base.LB; Add_TxMessage(P_Speed->Speed_PID.output,P_Speed->TxMessage->Data+2);
	P_Speed=&Robo_Base.RF; Add_TxMessage(P_Speed->Speed_PID.output,P_Speed->TxMessage->Data+4);
	P_Speed=&Robo_Base.RB; Add_TxMessage(P_Speed->Speed_PID.output,P_Speed->TxMessage->Data+6);
	P_Speed->TxMessage->Update=1;
	CAN_Send(&Can_TxMessageList[0]);
}

#if SYSTEM_ENABLE
		//--------------------------------------------------------------------------------------------------//
	//��������:
	//		�ٶȻ�������ݷ����Ľӿں���
	//
	//��������:
	//		��ȡRobo_Base��Ӧ��CAN�ڴ��������, ���ݵ���������ֱ�����һ�����ӵ���Ϣ, Ȼ�󴢴�������.
	//
	//��������:
	//		uint8_t* �����Ϣ������, �Ƽ�ʹ��Rx_CAN����, �������Բ���Ҫ�Լ�ȥ����.
	//		uint32_t �������
	//
	//--------------------------------------------------------------------------------------------------//
	#if UART_COMMUNICATE_USER_ROOT
		weak
	#endif
	void Motor_All_Analysis(uint8_t* RX_Data,uint32_t Motor_Num)
	{
		Motor_System* P_Motor=NULL;
		switch(Motor_Num)
		{
			case 0x201:P_Motor=&Robo_Base.All_Motor;break;
			case 0x202:P_Motor=&Robo_Base.All_Motor;break;
			case 0x203:P_Motor=&Robo_Base.All_Motor;break;
			case 0x204:P_Motor=&Robo_Base.All_Motor;break;
			default:break;
		}if(!P_Motor) return ;
		P_Motor->Info_Analysis(&P_Motor->Info,RX_Data);
		#if WATCHDOG_ENABLE
			Feed_WatchDog(&P_Motor->Protect);
		#endif
	}
#endif

	
#if POS_SYSTEM_ENABLE
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
	#if UART_COMMUNICATE_USER_ROOT
		weak
	#endif
	void Motor_Pos_Analysis(uint8_t* RX_Data,uint32_t Motor_Num)
	{
		Pos_System* P_Pos=NULL;
		switch(Motor_Num)
		{
			case 0x201:P_Pos=&Robo_Base.Pos_Motor;break;
			case 0x202:P_Pos=&Robo_Base.Pos_Motor;break;
			case 0x203:P_Pos=&Robo_Base.Pos_Motor;break;
			case 0x204:P_Pos=&Robo_Base.Pos_Motor;break;
			default:break;
		}if(!P_Pos) return ;
		P_Pos->Info_Analysis(&P_Pos->Info,RX_Data);
		#if WATCHDOG_ENABLE
			Feed_WatchDog(&P_Pos->Protect);
		#endif
	}
#endif

#if SPEED_SYSTEM_ENABLE
	//--------------------------------------------------------------------------------------------------//
	//��������:
	//		�ٶȻ�������ݷ����Ľӿں���
	//
	//��������:
	//		��ȡRobo_Base��Ӧ��CAN�ڴ��������, ���ݵ���������ֱ�����һ�����ӵ���Ϣ, Ȼ�󴢴�������.
	//
	//��������:
	//		uint8_t* �����Ϣ������, �Ƽ�ʹ��Rx_CAN����, �������Բ���Ҫ�Լ�ȥ����.
	//		uint32_t �������
	//
	//--------------------------------------------------------------------------------------------------//
	#if UART_COMMUNICATE_USER_ROOT
		weak
	#endif
	void Motor_Speed_Analysis(uint8_t* RX_Data,uint32_t Motor_Num)
	{
		Speed_System* P_Speed=NULL;
		switch(Motor_Num)
		{
			case 0x201:P_Speed=&Robo_Base.LF;break;
			case 0x202:P_Speed=&Robo_Base.LB;break;
			case 0x203:P_Speed=&Robo_Base.RF;break;
			case 0x204:P_Speed=&Robo_Base.RB;break;
			default:break;
		}if(!P_Speed) return ;
		P_Speed->Info_Analysis(&P_Speed->Info,RX_Data);
		#if WATCHDOG_ENABLE
			Feed_WatchDog(&P_Speed->Protect);
		#endif
	}
#endif

#if WATCHDOG_ENABLE
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

		if(System_Check(&Robo_Base.LF.Protect)) Error_State|=(1<<(Robo_Base.LF.Motor_Num+1));
		if(System_Check(&Robo_Base.LB.Protect)) Error_State|=(1<<(Robo_Base.LB.Motor_Num+1));
		if(System_Check(&Robo_Base.RF.Protect)) Error_State|=(1<<(Robo_Base.RF.Motor_Num+1));
		if(System_Check(&Robo_Base.RB.Protect)) Error_State|=(1<<(Robo_Base.RB.Motor_Num+1));
		if(Error_State) Robo_Base.Working_State=Error_State;
	}
#endif

void LED_WARNING(void)
{
	static int8_t P_Error=8;
	static uint32_t P_Time=0;
	
	//HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
	if(P_Time+800<=Robo_Base.Running_Time) P_Time=Robo_Base.Running_Time,P_Error--;
	if(P_Error==-1){
		P_Error=8;
	}else if(P_Error==0){
		LED_RED_OFF;
		LED_GRE_OFF;
	}else if(Robo_Base.Working_State&(1<<P_Error)){
		if(P_Time+600<=Robo_Base.Running_Time){
			LED_RED_OFF;
			LED_GRE_OFF;
		}else{
			LED_RED_ON;
			LED_GRE_OFF;
		}
	}else{
		if(P_Time+600<=Robo_Base.Running_Time){
			LED_RED_OFF;
			LED_GRE_OFF;
		}else{
			LED_RED_OFF;
			LED_GRE_ON;
		}
	}
}




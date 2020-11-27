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
//		ROBO_BASE ָ��, ���̽ṹ���ָ��
//		uint8_t* �����Ϣ������, �Ƽ�ʹ��Rx_CAN����, �������Բ���Ҫ�Լ�ȥ����.
//		uint32_t �������
//
//��ֲ����:
//		ֱ�Ӷ�case�����ݽ����޸�, �м���λ�û������ӾͼӼ���, Ȼ����ָ��ָ���Ӧ�����Ӿ���.
//
//--------------------------------------------------------------------------------------------------//
void Motor_Pos_Analysis(ROBO_BASE* Robo,uint8_t* RX_Data,uint32_t Motor_Num)
{
  Pos_System* P_Motor=NULL;
  switch(Motor_Num)
  {
    case 0x201:P_Motor=&Robo->Pos_MotorLF;break;
    case 0x202:P_Motor=&Robo->Pos_MotorRF;break;
    case 0x203:P_Motor=&Robo->Pos_MotorRB;break;
    case 0x204:P_Motor=&Robo->Pos_MotorLB;break;
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
void Motor_Speed_Analysis(ROBO_BASE* Robo,uint8_t* RX_Data,uint32_t Motor_Num)
{
  Speed_System* S_Motor=NULL;
  switch(Motor_Num)
  {
    case 0x201:S_Motor=&Robo->Speed_MotorLF;break;
    case 0x202:S_Motor=&Robo->Speed_MotorRF;break;
    case 0x203:S_Motor=&Robo->Speed_MotorRB;break;
    case 0x204:S_Motor=&Robo->Speed_MotorLB;break;
	default:break;
  }if(!S_Motor) return ;
  Speed_Info_Analysis(&S_Motor->Info,RX_Data);
  Feed_WatchDog(&S_Motor->Protect);
}

//--------------------------------------------------------------------------------------------------//
//��������:
//		ϵͳ�������ֵת��Ϊ�ַ�������
//
//��������:
//		���ٶȻ�/λ�û�ϵͳ���������ֵת�����ַ���, �Ա�ͨ�����ڷ��͵���λ��
//
//��������:
//		int32_t ϵͳ���ֵ
//		int32_t ϵͳ����ֵ
//
//��ֲ����:
//		������������ر��, ���԰�temp�������ٵ���һ��, ������Ҫ��֤�������ĳ��ȹ���
//--------------------------------------------------------------------------------------------------//
void SystemIO_Usart_ToString(ROBO_BASE* Robo,int32_t System_Out,int32_t System_In)
{
  uint8_t* Usart_Tx=Robo->Uart2.Tx_buffer;
  int32_t temp=1000000;
  uint8_t flag1=0;

  //ת��ϵͳ���ֵ
  Robo->Uart2.Tx_length=0;
  if(System_Out<0) Usart_Tx[Robo->Uart2.Tx_length++]='-',System_Out=-System_Out;
  else if(System_Out==0) Usart_Tx[Robo->Uart2.Tx_length++]='0',temp=0;

  while(temp!=0)
  {
    if(System_Out/temp!=0)
    {
	  flag1=1;
	  Usart_Tx[Robo->Uart2.Tx_length++]='0'+System_Out/temp;
	  System_Out-=System_Out/temp*temp;
	  if(System_Out==0)
	  {
		temp/=10;
	    while(temp!=0)
		{
		  Usart_Tx[Robo->Uart2.Tx_length++]='0';
		  temp/=10;
		}break;
	  }
    }else if(flag1) Usart_Tx[Robo->Uart2.Tx_length++]='0';
	temp/=10;
  }Usart_Tx[Robo->Uart2.Tx_length++]=' ';
  temp=10000;
  flag1=0;

  //ת��ϵͳ����ֵ
  if(System_In<0) Usart_Tx[Robo->Uart2.Tx_length++]='-',System_In=-System_In;
  else if(System_In==0) Usart_Tx[Robo->Uart2.Tx_length++]='0',temp=0;

  while(temp!=0)
  {
    if(System_In/temp!=0)
    {
	  flag1=1;
	  Usart_Tx[Robo->Uart2.Tx_length++]='0'+System_In/temp;
	  System_In-=System_In/temp*temp;
	  if(System_In==0)
	  {
		temp/=10;
	    while(temp!=0)
		{
		  Usart_Tx[Robo->Uart2.Tx_length++]='0';
		  temp/=10;
		}break;
	  }
    }else if(flag1)  Usart_Tx[Robo->Uart2.Tx_length++]='0';
	temp/=10;
  }
  Usart_Tx[Robo->Uart2.Tx_length++]='\r';
  Usart_Tx[Robo->Uart2.Tx_length]='\n';
}


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
void BASE_Init(ROBO_BASE *Robo)
{
  Pos_System* P_Pos=NULL;
  P_Pos=&Robo->Pos_MotorLF; PID_Init(&P_Pos->Pos_PID,			0.3,	0,	0,	5000,	0,	0,	7000);
  P_Pos->Motor_Num=0;		PID_Init(&P_Pos->Speed_PID,			5,	0,	0,	5000,	0,	0,	7000); 
  P_Pos->Protect.Count_Time=WATCHDOG_TIME_MAX;	SystemState_Set(&P_Pos->Protect,MISSING);
  P_Pos=&Robo->Pos_MotorRF; PID_Init(&P_Pos->Pos_PID,			0,	0,	0,	0,	0,	0,	0);
  P_Pos->Motor_Num=1;		PID_Init(&P_Pos->Speed_PID,			0,	0,	0,	0,	0,	0,	0); 
  P_Pos->Protect.Count_Time=WATCHDOG_TIME_MAX;	SystemState_Set(&P_Pos->Protect,MISSING);
  P_Pos=&Robo->Pos_MotorRB; PID_Init(&P_Pos->Pos_PID,			0,	0,	0,	0,	0,	0,	0);
  P_Pos->Motor_Num=2;		PID_Init(&P_Pos->Speed_PID,			0,	0,	0,	0,	0,	0,	0); 
  P_Pos->Protect.Count_Time=WATCHDOG_TIME_MAX;	SystemState_Set(&P_Pos->Protect,MISSING);
  P_Pos=&Robo->Pos_MotorLB; PID_Init(&P_Pos->Pos_PID,			0,	0,	0,	0,	0,	0,	0);
  P_Pos->Motor_Num=3;		PID_Init(&P_Pos->Speed_PID,			0,	0,	0,	0,	0,	0,	0); 
  P_Pos->Protect.Count_Time=WATCHDOG_TIME_MAX;	SystemState_Set(&P_Pos->Protect,MISSING);
	
  Speed_System* P_Speed=NULL;
  P_Speed=&Robo->Speed_MotorLF; PID_Init(&P_Speed->Speed_PID,	5,	0,	0,	5000,	0,	0,	7000); P_Speed->Motor_Num=0;
  P_Speed->Protect.Count_Time=WATCHDOG_TIME_MAX;	SystemState_Set(&P_Speed->Protect,MISSING);
  P_Speed=&Robo->Speed_MotorRF; PID_Init(&P_Speed->Speed_PID,	0,	0,	0,	0,	0,	0,	0); P_Speed->Motor_Num=1;
  P_Speed->Protect.Count_Time=WATCHDOG_TIME_MAX;	SystemState_Set(&P_Speed->Protect,MISSING);
  P_Speed=&Robo->Speed_MotorRB; PID_Init(&P_Speed->Speed_PID,	0,	0,	0,	0,	0,	0,	0); P_Speed->Motor_Num=2;
  P_Speed->Protect.Count_Time=WATCHDOG_TIME_MAX;	SystemState_Set(&P_Speed->Protect,MISSING);
  P_Speed=&Robo->Speed_MotorLB; PID_Init(&P_Speed->Speed_PID,	0,	0,	0,	0,	0,	0,	0); P_Speed->Motor_Num=3;
  P_Speed->Protect.Count_Time=WATCHDOG_TIME_MAX;	SystemState_Set(&P_Speed->Protect,MISSING);
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
void PID_Send(ROBO_BASE* Robo)
{
  Base_WatchDog(Robo);

  Pos_System* P_Pos=NULL;
  P_Pos=&Robo->Pos_MotorLB; PID_Pos_Cal(P_Pos,Robo->Can2.Tx);
  P_Pos=&Robo->Pos_MotorRB; PID_Pos_Cal(P_Pos,Robo->Can2.Tx);
  P_Pos=&Robo->Pos_MotorLF; PID_Pos_Cal(P_Pos,Robo->Can2.Tx);
  P_Pos=&Robo->Pos_MotorRF; PID_Pos_Cal(P_Pos,Robo->Can2.Tx);
  Send_To_Motor(&hcan2,Robo->Can2.Tx);

  Speed_System* P_Speed=NULL;
  P_Speed=&Robo->Speed_MotorLB; PID_Speed_Cal(P_Speed,Robo->Can1.Tx);
  P_Speed=&Robo->Speed_MotorRB; PID_Speed_Cal(P_Speed,Robo->Can1.Tx);
  P_Speed=&Robo->Speed_MotorLF; PID_Speed_Cal(P_Speed,Robo->Can1.Tx);
  P_Speed=&Robo->Speed_MotorRF; PID_Speed_Cal(P_Speed,Robo->Can1.Tx);
  Send_To_Motor(&hcan1,Robo->Can2.Tx);
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
void Base_WatchDog(ROBO_BASE* Robo)
{
  RoboBaseState Error_State=SYSTEM_WORKING;
  Pos_System* P_Pos=NULL;
  P_Pos=&Robo->Pos_MotorLF; POS_SYSTEM_CHECK;
  P_Pos=&Robo->Pos_MotorRF; POS_SYSTEM_CHECK;
  P_Pos=&Robo->Pos_MotorRB; POS_SYSTEM_CHECK;
  P_Pos=&Robo->Pos_MotorLB; POS_SYSTEM_CHECK;
	
  Speed_System* P_Speed=NULL;
  P_Speed=&Robo->Speed_MotorLF; SPEED_SYSTEM_CHECK;
  P_Speed=&Robo->Speed_MotorRF; SPEED_SYSTEM_CHECK;
  P_Speed=&Robo->Speed_MotorRB; SPEED_SYSTEM_CHECK;
  P_Speed=&Robo->Speed_MotorLB; SPEED_SYSTEM_CHECK;

  Robo->State=Error_State;
  if(Error_State!=WORKING) LED_WARNING(Robo);
}

//--------------------------------------------------------------------------------------------------//
//��������:
//		��������ʱ�亯��
//
//��������:
//		��¼��������ʱ��
//
//��������:
//		ROBO_BASE* ���̽ṹ��ָ��
//
//--------------------------------------------------------------------------------------------------//
void Counting_Time(ROBO_BASE* Robo)
{
  Robo->Running_Time++;
  if(Robo->Running_Time>RUNNING_TIME_MAX) Robo->Running_Time=0;
}
	//--------------------------------------------------------------------------------------------------//
//��������:
//		����LED��������
//
//��������:
//		�����̴��ڷ���������״̬ʱ, ͨ������LED��˸�ض��������ʾ����
//
//��������:
//		ROBO_BASE* ���̽ṹ��ָ��
//
//--------------------------------------------------------------------------------------------------//
void LED_WARNING(ROBO_BASE* Robo)
{
  static uint8_t State=1;
  static uint8_t Flag=0;
  if(State==(uint8_t)Robo->State) State=0;
  if(Robo->Running_Time%3000-(State*300)<100)
  {
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
  }
  else
  {
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
	  if(!Flag) Flag=1;
  }
  if(Flag) State++,Flag=0;
}


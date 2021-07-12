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
RoboBase Robo_Base;
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
void Motor_CAN_Recevice(uint32_t Motor_Num, uint8_t* RX_Data)
{
    MotorSystem* P_Motor = NULL;
    switch(Motor_Num)
    {
        case 0x201:P_Motor = &Robo_Base.LF._Pos;break;
        case 0x202:P_Motor = &Robo_Base.LB._Pos;break;
        case 0x203:P_Motor = &Robo_Base.RF._Pos;break;
        case 0x204:P_Motor = &Robo_Base.RB._Pos;break;
        default:break;
    }if(!P_Motor) return ;
    Motor_Info_Analysis(&P_Motor->Info, RX_Data);
    Feed_WatchDog(&P_Motor->Protect);
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
    MotorSystem* P_Motor = NULL;
    P_Motor = &Robo_Base.LF._Pos; PID_Init(&P_Motor->Pos_PID,			0.8,	0,	0,	10000,	0,	0,	8000);
	Motor_Init(P_Motor, 0);			  PID_Init(&P_Motor->Speed_PID,			5,	0,	0,	5000,	0,	0,	7000);
    P_Motor = &Robo_Base.LB._Pos; PID_Init(&P_Motor->Pos_PID,			0.8,	0.02,	0,	10000,	0,	2000,	8000);
    Motor_Init(P_Motor, 1);			  PID_Init(&P_Motor->Speed_PID,			5,	0,	0,	5000,	0,	0,	7000); 
    P_Motor = &Robo_Base.RF._Pos; PID_Init(&P_Motor->Pos_PID,			0.8,	0,	0,	10000,	0,	0,	8000);
    Motor_Init(P_Motor, 2);			  PID_Init(&P_Motor->Speed_PID,			5,	0,	0,	5000,	0,	0,	7000); 
    P_Motor = &Robo_Base.RB._Pos; PID_Init(&P_Motor->Pos_PID,			0.8,	0,	0,	10000,	0,	0,	8000);
    Motor_Init(P_Motor, 3);			  PID_Init(&P_Motor->Speed_PID,			5,	0,	0,	5000,	0,	0,	7000); 

	Axis* P_Axis = NULL;																																										//���������ʼ��
	P_Axis = Robo_Base.RF._Axis = &ODrive0.Axis0; Axis_Init(P_Axis, 0);
	P_Axis = Robo_Base.RB._Axis = &ODrive0.Axis1; Axis_Init(P_Axis, 1);
	P_Axis = Robo_Base.LB._Axis = &ODrive1.Axis0; Axis_Init(P_Axis, 2);
	P_Axis = Robo_Base.LF._Axis = &ODrive1.Axis1; Axis_Init(P_Axis, 3);
	Robo_Base.Working_State = 1;
    Robo_Base.Error_State = 0;
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
void Can_TxMessage_MoveMode(void)
{
	PID_Pos_Cal(&Robo_Base.LF._Pos);
    Motor_Add_Can_TxMessageList(&Robo_Base.LF._Pos);
	PID_Pos_Cal(&Robo_Base.LB._Pos);
    Motor_Add_Can_TxMessageList(&Robo_Base.LB._Pos);
	PID_Pos_Cal(&Robo_Base.RF._Pos);
    Motor_Add_Can_TxMessageList(&Robo_Base.RF._Pos);
	PID_Pos_Cal(&Robo_Base.RB._Pos);
	Motor_Add_Can_TxMessageList(&Robo_Base.RB._Pos);
    
	ODrive_CAN_Transmit(Robo_Base.LF._Axis, 0x0D);
	ODrive_CAN_Transmit(Robo_Base.LB._Axis, 0x0D);
	ODrive_CAN_Transmit(Robo_Base.RF._Axis, 0x0D);
	ODrive_CAN_Transmit(Robo_Base.RB._Axis, 0x0D);
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
    uint32_t Error_State = 0;
    MotorSystem* P_Motor = NULL;
    P_Motor = &Robo_Base.LF._Pos;
	if(!System_Check(&P_Motor->Protect)) Error_State |= (1 << P_Motor->Motor_Num);
    P_Motor = &Robo_Base.LB._Pos;
	if(!System_Check(&P_Motor->Protect)) Error_State |= (1 << P_Motor->Motor_Num);
    P_Motor = &Robo_Base.RF._Pos;
	if(!System_Check(&P_Motor->Protect)) Error_State |= (1 << P_Motor->Motor_Num);
    P_Motor = &Robo_Base.RB._Pos;
	if(!System_Check(&P_Motor->Protect)) Error_State |= (1 << P_Motor->Motor_Num);
	
	Axis* P_Axis = NULL;
	P_Axis = Robo_Base.LF._Axis;
	if(P_Axis->Error != 0 || !System_Check(&P_Axis->Protect)) Error_State |= (1 << (P_Axis->Node_ID + 4));
	P_Axis = Robo_Base.LB._Axis;
	if(P_Axis->Error != 0 || !System_Check(&P_Axis->Protect)) Error_State |= (1 << (P_Axis->Node_ID + 4));
	P_Axis=Robo_Base.RF._Axis;
	if(P_Axis->Error != 0 || !System_Check(&P_Axis->Protect)) Error_State |= (1 << (P_Axis->Node_ID + 4));
	P_Axis=Robo_Base.RB._Axis;
	if(P_Axis->Error != 0 || !System_Check(&P_Axis->Protect)) Error_State |= (1 << (P_Axis->Node_ID + 4));
  
	if(Error_State) Robo_Base.Error_State = Error_State;
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
  if(Robo_Base.Running_Time > RUNNING_TIME_MAX) Robo_Base.Running_Time = 0;
}
int32_t Speed = 3000;
void Pos_CloseLoop_Init(MotorSystem* P_Motor)
{
	static uint8_t num[4] = {0};
	uint8_t* P_num = &num[P_Motor->Motor_Num];
	
	//if(P_Motor->Info.Electric==0) return ;
	if(*P_num < 20){
        P_Motor->Tar_Speed = Speed;
		PID_Speed_Cal(P_Motor);
		if(P_Motor == &Robo_Base.LF._Pos) if(HAL_GPIO_ReadPin(TIM3_CH1_GPIO_Port, TIM3_CH1_Pin) == GPIO_PIN_SET) (*P_num)++;
		if(P_Motor == &Robo_Base.LB._Pos) if(HAL_GPIO_ReadPin(TIM3_CH2_GPIO_Port, TIM3_CH2_Pin) == GPIO_PIN_SET) (*P_num)++;
		if(P_Motor == &Robo_Base.RF._Pos) if(HAL_GPIO_ReadPin(TIM3_CH3_GPIO_Port, TIM3_CH3_Pin) == GPIO_PIN_SET) (*P_num)++;
		if(P_Motor == &Robo_Base.RB._Pos) if(HAL_GPIO_ReadPin(TIM3_CH4_GPIO_Port, TIM3_CH4_Pin) == GPIO_PIN_SET) (*P_num)++;
	}if(*P_num == 20) P_Motor->Info.Abs_Angle = P_Motor->Tar_Speed = 0, (*P_num)++;
	if(*P_num == 21) P_Motor->Tar_Pos = 0, WorkState_Set(&P_Motor->Protect, WORKING), PID_Pos_Cal(P_Motor);
    Motor_Add_Can_TxMessageList(P_Motor);
}


//�ļ�����:		motor_system.c
//��Ӧͷ�ļ�:	motor_system.h
//��Ҫ����:
//		�ѵ������/���Ź�/PID���Ƴ����һ��ϵͳ, ������ȡ����ϵͳ�Ļ�����������.
//		ʵ�ֶԵ�������ݶ�ȡ�ͷ���, PID������ʼ��, PID����, ���Ź����õȷ���
//
//ʱ��:
//		2020/11/27
//
//�汾:	1,0V
//
//״̬: ������
//
//��������:
//		�ܹ�������ȡ�����Ϣ, ����ʵ��PID����, ���Ź�����û������.

//---------ͷ�ļ����ò���---------//
#include "motor_system.h"
//--------------------------------//

//---------������������-----------//
//--------------------------------//

//---------�ⲿ������������-------//
//--------------------------------//

//--------------------------------------------------------------------------------------------------//
//��������:
//		λ�û�������ݷ����Ĳ�������
//
//��������:
//		�������ݴ����������ݽ��н���, ���Ҽ�������ԽǶȺ���ԽǶ�.
//		Ĭ��ͨ��Э������:  	Data[0]----����ٶȸ�8λ
//							Data[1]----����ٶȵ�8λ
//							Data[2]----ת�ӽǶȸ�8λ
//							Data[3]----ת�ӽǶȵ�8λ
//							Data[4]----������С��8λ
//							Data[5]----������С��8λ
//							Data[6]----�¶�
//							Data[7]----NULL
//
//��������:
//		Motor_Pos_Info* λ�û������Ϣָ��
//		uint8_t* �����Ϣ������
//
//��ֲ����:
//		���ܲ���Ҫ��, Ҫ�ĵĻ�, ��Ϣ�����ĵط�����ͨ��Э�����ľ���.
//--------------------------------------------------------------------------------------------------//
void Pos_Info_Analysis(Motor_Pos_Info* Motor,uint8_t* RX_Data)
{
  //���ݽ���
  Motor->Angle=(uint16_t)RX_Data[0]<<8|RX_Data[1];
  Motor->Speed=(uint16_t)RX_Data[2]<<8|RX_Data[3];
  Motor->Electric=(uint16_t)RX_Data[4]<<8|RX_Data[5];
  Motor->Temperature=RX_Data[6];

  //���ԽǶȼ���
  if (Motor->Speed!=0)
  {
    int16_t Error=Motor->Angle-Motor->Last_Angle;
    Motor->Abs_Angle += Error;
    if (Error < -4096)Motor->Abs_Angle += 8192;
    else if (Error > 4096)  Motor->Abs_Angle -= 8192;
  }Motor->Last_Angle=Motor->Angle;

  //��ԽǶȼ���, Ĭ�Ϸ�Χ0-360
  if(Motor->Abs_Angle>=0) Motor->Relative_Angle=(Motor->Abs_Angle%ONE_CIRCLE)*360.0/ONE_CIRCLE;
  else Motor->Relative_Angle=360-((-Motor->Abs_Angle)%ONE_CIRCLE)*360.0/ONE_CIRCLE;
}
//--------------------------------------------------------------------------------------------------//
//��������:
//		�ٶȻ�������ݷ����Ĳ�������
//
//��������:
//		�������ݴ����������ݽ��н���.
//		Ĭ��ͨ��Э������:  	Data[0]----����ٶȸ�8λ
//							Data[1]----����ٶȵ�8λ
//							Data[2]----ת�ӽǶȸ�8λ
//							Data[3]----ת�ӽǶȵ�8λ
//							Data[4]----������С��8λ
//							Data[5]----������С��8λ
//							Data[6]----�¶�
//							Data[7]----NULL
//
//��������:
//		Motor_Speed_Info* �ٶȻ������Ϣָ��
//		uint8_t* �����Ϣ������
//
//��ֲ����:
//		���ܲ���Ҫ��, Ҫ�ĵĻ�, ��Ϣ�����ĵط�����ͨ��Э�����ľ���.
//--------------------------------------------------------------------------------------------------//
void Speed_Info_Analysis(Motor_Speed_Info* Motor,uint8_t* RX_Data)
{
  Motor->Speed=(uint16_t)RX_Data[2]<<8|RX_Data[3];
  Motor->Electric=(uint16_t)RX_Data[4]<<8|RX_Data[5];
  Motor->Temperature=RX_Data[6];
}
//--------------------------------------------------------------------------------------------------//
//��������:
//		PID������ʼ������
//
//��������:
//		��ʼ��ϵͳPID����
//
//��������:
//		PID* PIDָ��
//		float PID��Kp
//		float PID��Ki
//		float PID��Kd
//		float ������ֵ
//		float ����
//		float ����ۻ����ֵ
//		float ������ֵ
//
//--------------------------------------------------------------------------------------------------//
void PID_Init(PID *pid, float Kp, float Ki, float Kd, float error_max, float dead_line, float intergral_max, float output_max)
{
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->error_max = error_max;
	pid->output_max = output_max;
	pid->dead_line = dead_line;
	
	pid->intergral_max = intergral_max;
	
	pid->error = 0;
	pid->error_last = 0;
	pid->intergral = 0;
	pid->derivative = 0;
	pid->output = 0;
}

//--------------------------------------------------------------------------------------------------//
//��������:
//		PID���㺯��
//
//��������:
//		����PID����
//
//��������:
//		PID* PIDָ��
//		float ��ǰֵ
//		float Ŀ��ֵ
//		uint8_t �������
//		uint8_t �������ݵ�����
//
//��ֲ����:
//		���ܷ�װ����ô��, ���鱣���ú�����Ҫ�޸�, ��Ϊ���¼��ݻ��߼�����ĺ���
//
//--------------------------------------------------------------------------------------------------//
void PID_General_Cal(PID *pid, float fdbV, float tarV,uint8_t moto_num,uint8_t *Tx_msg)
{

	pid->error =  tarV - fdbV;
	if(pid->error > pid->error_max)
		pid->error = pid->error_max;
	if(pid->error < -pid->error_max)
		pid->error = -pid->error_max;
	if(pid->error > 0 && pid->error < pid->dead_line)
		pid->error = 0;
	if(pid->error < 0 && pid->error > pid->dead_line)
		pid->error = 0;
	
	pid->intergral = pid->intergral + pid->error;
	if(pid->intergral > pid->intergral_max)
		pid->intergral = pid->intergral_max;
	if(pid->intergral < -pid->intergral_max)
		pid->intergral = -pid->intergral_max;
	
	pid->derivative = pid->error - pid->error_last;
	pid->error_last = pid->error;
	
	pid->output = pid->Kp*pid->error + pid->Ki*pid->intergral + pid->Kd*pid->derivative;
	
	if(pid->output > pid->output_max)
		pid->output = pid->output_max;
	if(pid->output < -pid->output_max)
		pid->output = -pid->output_max;
	
	Tx_msg[moto_num*2]=((int16_t)pid->output)>>8;Tx_msg[moto_num*2+1]=(int16_t)pid->output;
}

//--------------------------------------------------------------------------------------------------//
//��������:
//		λ�û�PID���㺯��
//
//��������:
//		����λ�û�PID����
//
//��������:
//		Pos_System* λ�û�ϵͳָ��
//		uint8_t* �������ݵ�����
//
//--------------------------------------------------------------------------------------------------//
void PID_Pos_Cal(Pos_System* Pos_Motor,uint8_t *Tx_msg)
{
	if(Pos_Motor->Protect.State!=WORKING) return ;
	Pos_Motor->Pos_PID.error =  Pos_Motor->Tar_Pos - Pos_Motor->Info.Abs_Angle;
	if(Pos_Motor->Pos_PID.error > Pos_Motor->Pos_PID.error_max)
		Pos_Motor->Pos_PID.error = Pos_Motor->Pos_PID.error_max;
	if(Pos_Motor->Pos_PID.error < -Pos_Motor->Pos_PID.error_max)
		Pos_Motor->Pos_PID.error = -Pos_Motor->Pos_PID.error_max;
	if(Pos_Motor->Pos_PID.error > 0 && Pos_Motor->Pos_PID.error < Pos_Motor->Pos_PID.dead_line)
		Pos_Motor->Pos_PID.error = 0;
	if(Pos_Motor->Pos_PID.error < 0 && Pos_Motor->Pos_PID.error > Pos_Motor->Pos_PID.dead_line)
		Pos_Motor->Pos_PID.error = 0;
	
	Pos_Motor->Pos_PID.intergral = Pos_Motor->Pos_PID.intergral + Pos_Motor->Pos_PID.error;
	if(Pos_Motor->Pos_PID.intergral > Pos_Motor->Pos_PID.intergral_max)
		Pos_Motor->Pos_PID.intergral = Pos_Motor->Pos_PID.intergral_max;
	if(Pos_Motor->Pos_PID.intergral < -Pos_Motor->Pos_PID.intergral_max)
		Pos_Motor->Pos_PID.intergral = -Pos_Motor->Pos_PID.intergral_max;
	
	Pos_Motor->Pos_PID.derivative = Pos_Motor->Pos_PID.error - Pos_Motor->Pos_PID.error_last;
	Pos_Motor->Pos_PID.error_last = Pos_Motor->Pos_PID.error;
	
	Pos_Motor->Pos_PID.output = Pos_Motor->Pos_PID.Kp*Pos_Motor->Pos_PID.error + Pos_Motor->Pos_PID.Ki*Pos_Motor->Pos_PID.intergral + Pos_Motor->Pos_PID.Kd*Pos_Motor->Pos_PID.derivative;
	
	if(Pos_Motor->Pos_PID.output > Pos_Motor->Pos_PID.output_max)
		Pos_Motor->Pos_PID.output = Pos_Motor->Pos_PID.output_max;
	if(Pos_Motor->Pos_PID.output < -Pos_Motor->Pos_PID.output_max)
		Pos_Motor->Pos_PID.output = -Pos_Motor->Pos_PID.output_max;
	
		Pos_Motor->Speed_PID.error =  Pos_Motor->Pos_PID.output - Pos_Motor->Info.Speed;
	if(Pos_Motor->Speed_PID.error > Pos_Motor->Speed_PID.error_max)
		Pos_Motor->Speed_PID.error = Pos_Motor->Speed_PID.error_max;
	if(Pos_Motor->Speed_PID.error < -Pos_Motor->Speed_PID.error_max)
		Pos_Motor->Speed_PID.error = -Pos_Motor->Speed_PID.error_max;
	if(Pos_Motor->Speed_PID.error > 0 && Pos_Motor->Speed_PID.error < Pos_Motor->Speed_PID.dead_line)
		Pos_Motor->Speed_PID.error = 0;
	if(Pos_Motor->Speed_PID.error < 0 && Pos_Motor->Speed_PID.error > Pos_Motor->Speed_PID.dead_line)
		Pos_Motor->Speed_PID.error = 0;
	
	Pos_Motor->Speed_PID.intergral = Pos_Motor->Speed_PID.intergral + Pos_Motor->Speed_PID.error;
	if(Pos_Motor->Speed_PID.intergral > Pos_Motor->Speed_PID.intergral_max)
		Pos_Motor->Speed_PID.intergral = Pos_Motor->Speed_PID.intergral_max;
	if(Pos_Motor->Speed_PID.intergral < -Pos_Motor->Speed_PID.intergral_max)
		Pos_Motor->Speed_PID.intergral = -Pos_Motor->Speed_PID.intergral_max;
	
	Pos_Motor->Speed_PID.derivative = Pos_Motor->Speed_PID.error - Pos_Motor->Speed_PID.error_last;
	Pos_Motor->Speed_PID.error_last = Pos_Motor->Speed_PID.error;
	
	Pos_Motor->Speed_PID.output = Pos_Motor->Speed_PID.Kp*Pos_Motor->Speed_PID.error + Pos_Motor->Speed_PID.Ki*Pos_Motor->Speed_PID.intergral + Pos_Motor->Speed_PID.Kd*Pos_Motor->Speed_PID.derivative;
	
	if(Pos_Motor->Speed_PID.output > Pos_Motor->Speed_PID.output_max)
		Pos_Motor->Speed_PID.output = Pos_Motor->Speed_PID.output_max;
	if(Pos_Motor->Speed_PID.output < -Pos_Motor->Speed_PID.output_max)
		Pos_Motor->Speed_PID.output = -Pos_Motor->Speed_PID.output_max;
	
	
	Tx_msg[Pos_Motor->Motor_Num*2]=((int16_t)Pos_Motor->Speed_PID.output)>>8;Tx_msg[Pos_Motor->Motor_Num*2+1]=(int16_t)Pos_Motor->Speed_PID.output;
}

//--------------------------------------------------------------------------------------------------//
//��������:
//		�ٶȻ�PID���㺯��
//
//��������:
//		�����ٶȻ�PID����
//
//��������:
//		Speed_System* �ٶȻ�ϵͳָ��
//		uint8_t* �������ݵ�����
//
//--------------------------------------------------------------------------------------------------//
void PID_Speed_Cal(Speed_System* Speed_Motor,uint8_t *Tx_msg)
{
//`	if(Speed_Motor->Protect.State!=WORKING) return ;
	Speed_Motor->Speed_PID.error =  Speed_Motor->Tar_Speed - Speed_Motor->Info.Speed;
	if(Speed_Motor->Speed_PID.error > Speed_Motor->Speed_PID.error_max)
		Speed_Motor->Speed_PID.error = Speed_Motor->Speed_PID.error_max;
	if(Speed_Motor->Speed_PID.error < -Speed_Motor->Speed_PID.error_max)
		Speed_Motor->Speed_PID.error = -Speed_Motor->Speed_PID.error_max;
	if(Speed_Motor->Speed_PID.error > 0 && Speed_Motor->Speed_PID.error < Speed_Motor->Speed_PID.dead_line)
		Speed_Motor->Speed_PID.error = 0;
	if(Speed_Motor->Speed_PID.error < 0 && Speed_Motor->Speed_PID.error > Speed_Motor->Speed_PID.dead_line)
		Speed_Motor->Speed_PID.error = 0;
	
	Speed_Motor->Speed_PID.intergral = Speed_Motor->Speed_PID.intergral + Speed_Motor->Speed_PID.error;
	if(Speed_Motor->Speed_PID.intergral > Speed_Motor->Speed_PID.intergral_max)
		Speed_Motor->Speed_PID.intergral = Speed_Motor->Speed_PID.intergral_max;
	if(Speed_Motor->Speed_PID.intergral < -Speed_Motor->Speed_PID.intergral_max)
		Speed_Motor->Speed_PID.intergral = -Speed_Motor->Speed_PID.intergral_max;
	
	Speed_Motor->Speed_PID.derivative = Speed_Motor->Speed_PID.error - Speed_Motor->Speed_PID.error_last;
	Speed_Motor->Speed_PID.error_last = Speed_Motor->Speed_PID.error;
	
	Speed_Motor->Speed_PID.output = Speed_Motor->Speed_PID.Kp*Speed_Motor->Speed_PID.error + Speed_Motor->Speed_PID.Ki*Speed_Motor->Speed_PID.intergral + Speed_Motor->Speed_PID.Kd*Speed_Motor->Speed_PID.derivative;
	
	if(Speed_Motor->Speed_PID.output > Speed_Motor->Speed_PID.output_max)
		Speed_Motor->Speed_PID.output = Speed_Motor->Speed_PID.output_max;
	if(Speed_Motor->Speed_PID.output < -Speed_Motor->Speed_PID.output_max)
		Speed_Motor->Speed_PID.output = -Speed_Motor->Speed_PID.output_max;
	
	Tx_msg[Speed_Motor->Motor_Num*2]=((int16_t)Speed_Motor->Speed_PID.output)>>8;Tx_msg[Speed_Motor->Motor_Num*2+1]=(int16_t)Speed_Motor->Speed_PID.output;
}

//--------------------------------------------------------------------------------------------------//
//��������:
//		CANͨ�ŷ��ͺ���
//
//��������:
//		��������
//
//��������:
//		CAN_HandleTypeDef* CAN�ľ��
//		uint8_t* �������ݵ�����
//
//��ֲ����:
//		����Ҫ���޸ı�ʶ������

//--------------------------------------------------------------------------------------------------//
void Send_To_Motor(CAN_HandleTypeDef *hcan,uint8_t* Tx_Data)
{
  CAN_TxHeaderTypeDef TxHeader;
  uint32_t TxMailbox; 

  TxHeader.RTR = 0;
  TxHeader.IDE = 0;            
  TxHeader.StdId=0x200;
  TxHeader.TransmitGlobalTime = DISABLE;
  TxHeader.DLC = 8;
        
  if (HAL_CAN_AddTxMessage(hcan, &TxHeader, Tx_Data, &TxMailbox) != HAL_OK)
  {
   /* Transmission request Error */
     Error_Handler();
  }
}

//--------------------------------------------------------------------------------------------------//
//��������:
//		ϵͳ״̬�л�����
//
//��������:
//		����ϵͳ״̬
//
//��������:
//		Protect_System* ϵͳ�ڵĿ��Ź��ṹ��ָ��
//		SystemState ϵͳ״ֵ̬
//
//--------------------------------------------------------------------------------------------------//
void SystemState_Set(Protect_System* Dogs,SystemState State)
{
  Dogs->State=State;
}

//--------------------------------------------------------------------------------------------------//
//��������:
//		���Ź�ι������
//
//��������:
//		�����Ź��ļ�������
//
//��������:
//		Protect_System* ϵͳ�ڵĿ��Ź��ṹ��ָ��
//
//--------------------------------------------------------------------------------------------------//
void Feed_WatchDog(Protect_System* Dogs)
{
  Dogs->Count_Time=0;
}

//--------------------------------------------------------------------------------------------------//
//��������:
//		ϵͳ���Ź���⺯��
//
//��������:
//		�жϿ��Ź���ʱ�Ƿ񳬳�, ���ö�Ӧ��ϵͳ״̬
//
//��������:
//		Protect_System* ϵͳ�ڵĿ��Ź��ṹ��ָ��
//
//--------------------------------------------------------------------------------------------------//
uint8_t System_Check(Protect_System* Dogs)
{
  if(Dogs->Count_Time<WATCHDOG_TIME_MAX)
  {
    SystemState_Set(Dogs,WORKING);
		Dogs->Count_Time++;
		return 0;
  }else SystemState_Set(Dogs,MISSING);
  return 1;
}

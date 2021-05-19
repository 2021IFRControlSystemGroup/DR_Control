//�ļ�����:		motor_system.c
//��Ӧͷ�ļ�:	motor_system.h
//��Ҫ����:
//				���ļ����Ĵ󲿷ֹ���, �ֱ�Ϊ���Ź�����, ȫϵͳ���ƹ���, λ�û����ƹ���, �ٶȻ����ƹ���.
//				ͨ���ı�Ԥ�����Ӧ��ʹ�ܱ�ʶ��, ��ʹ�ܸ������ֵĹ���.
//
//				���Ź�����
//					--���ù���״̬
//					--��⹤��״̬
//
//				ȫϵͳ���ƹ���
//					--�����ʼ��
//					--������ݽ���
//
//				λ�û����ƹ���
//					--�����ʼ��
//					--������ݽ���
//					--λ�û�PID����
//
//				�ٶȻ����ƹ���
//					--�����ʼ��
//					--������ݽ���
//					--�ٶȻ�PID����
//
//				��������
//					--����CANͨ��
//					--CAN������Ϣ��ʼ��
//					--CAN������Ϣ
//					--PID����
//
//ʱ��:
//		2021/5/7
//
//�汾:	2.0V
//
//---------ͷ�ļ����ò���---------//
#include "motor_system.h"
//--------------------------------//

//---------������������-----------//
Can_TxMessageTypeDef Can_TxMessageList[CAN_TXMESSAGEINDEXMAX];
//--------------------------------//

#ifdef __MOTOR_SYSTEM_H__
		//--------------------------------------------------------------------------------------------------//
		//��������:
		//		CAN���������ж�����
		//
		//��������:
		//		����CAN�Ĺ�����, ���ý����������, CAN1Ϊ0~13, CAN2Ϊ14~28;
		//		��ʹ��can�Ľ����ж�.
		//
		//��������:
		//		CAN_HandleTypeDef* hcan 	ָ��CAN�ŵ�ָ��
		//
		//--------------------------------------------------------------------------------------------------//
		#if MOTOR_SYSTEM_USER_ROOT 
			weak
		#endif
		void CAN_Start_IT(CAN_HandleTypeDef *hcan)
		{
			CAN_FilterTypeDef CAN_Filter;
			extern CAN_HandleTypeDef hcan1;
			
			if(hcan==&hcan1)
				CAN_Filter.FilterBank=0,CAN_Filter.SlaveStartFilterBank=14;
			else 
			CAN_Filter.FilterBank=14,CAN_Filter.SlaveStartFilterBank=14;
			
			CAN_Filter.FilterActivation=CAN_FILTER_ENABLE;
			CAN_Filter.FilterIdHigh=0;
			CAN_Filter.FilterIdLow=0;
			CAN_Filter.FilterMaskIdHigh=0;
			CAN_Filter.FilterMaskIdLow=0;
			CAN_Filter.FilterFIFOAssignment=CAN_FILTER_FIFO0;
			CAN_Filter.FilterMode=CAN_FILTERMODE_IDMASK;
			CAN_Filter.FilterScale=CAN_FILTERSCALE_32BIT;
			
			HAL_CAN_ConfigFilter(hcan,&CAN_Filter);
			
			HAL_CAN_Start(hcan);
			HAL_CAN_ActivateNotification(hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
		}

		//--------------------------------------------------------------------------------------------------//
		//��������:
		//		PID������ʼ������
		//
		//��������:
		//		��ʼ��ϵͳPID����
		//
		//��������:
		//		PID* pid							PIDָ��
		//		float Kp							PID��Kp
		//		float Ki							PID��Ki
		//		float Kd							PID��Kd
		//		float error_max				������ֵ
		//		float dead_line				����
		//		float intergral_max		����ۻ����ֵ
		//		float output_max			������ֵ
		//
		//--------------------------------------------------------------------------------------------------//
		#if MOTOR_SYSTEM_USER_ROOT 
			weak
		#endif
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
		//		Can_TxMessage������ʼ������
		//
		//��������:
		//		��ʼ��ϵͳCan_TxMessage����
		//
		//��������:
		//		Can_TxMessageTypeDef* TxMessage		ָ��Can_TxMessage��ָ��
		//		CAN_HandleTypeDef *hcan 					ָ��CAN��ָ��
		//		uint32_t DLC											CAN������Ϣ������֡����
		//		uint32_t RTR											CANң�ر�ʶ֡λ
		//		uint32_t IDE											CAN��չ֡��ʶλ
		//		uint32_t StdId										CAN��׼��ʶ��λ
		//		uint32_t ExtId										CAN��չ��ʶ��λ
		//
		//--------------------------------------------------------------------------------------------------//
		#if MOTOR_SYSTEM_USER_ROOT 
			weak
		#endif
		void Can_TxMessage_Init(Can_TxMessageTypeDef* TxMessage,CAN_HandleTypeDef *hcan,uint32_t DLC,uint32_t IDE,uint32_t RTR,uint32_t StdId,uint32_t ExtId)
		{
			TxMessage->Hcan=hcan;
			TxMessage->Update=0;
			Can_TxMessageHeader_Set(TxMessage->Header,DLC,IDE,RTR,StdId,ExtId);
		}
		
		//--------------------------------------------------------------------------------------------------//
		//��������:
		//		PID���㺯��
		//
		//��������:
		//		����PID����
		//
		//��������:
		//		PID* pid 		PIDָ��
		//		float fdbV 	��ǰֵ
		//		float tarV 	Ŀ��ֵ
		//
		//--------------------------------------------------------------------------------------------------//
		#if MOTOR_SYSTEM_USER_ROOT 
			weak
		#endif
		void PID_General_Cal(PID *pid, float fdbV, float tarV)
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
		}

		//--------------------------------------------------------------------------------------------------//
		//��������:
		//		CAN������Ϣ��Ӻ���
		//
		//��������:
		//		��PID������������ֵ���Ϊ��λ���ݴ���ָ������
		//
		//��������:
		//		float Output 			Ŀ��ֵ
		//		uint8_t* Tx_Data	Ŀ������
		//
		//--------------------------------------------------------------------------------------------------//
		#if MOTOR_SYSTEM_USER_ROOT 
			weak
		#endif
		void Add_TxMessage(float Output,uint8_t* Tx_Data)
		{
			Tx_Data[0]=((int16_t)Output>>8)&0xff;
			Tx_Data[1]=(int16_t)Output&0xff;
		}

		//--------------------------------------------------------------------------------------------------//
		//��������:
		//		CANͨ�ŷ��ͺ���
		//
		//��������:
		//		����CAN����
		//
		//��������:
		//		Can_TxMessageTypeDef* TxMessage		ָ��CAN������Ϣ�ṹ��ָ��
		//
		//--------------------------------------------------------------------------------------------------//
		#if MOTOR_SYSTEM_USER_ROOT 
			weak
		#endif
		void CAN_Send(Can_TxMessageTypeDef* TxMessage)
		{
			uint32_t TxMailbox;
			
			if(TxMessage->Update!=1) return ;
			while(HAL_CAN_GetTxMailboxesFreeLevel(TxMessage->Hcan)==0) ;
				if (HAL_CAN_AddTxMessage(TxMessage->Hcan, &TxMessage->Header, TxMessage->Data, &TxMailbox) != HAL_OK) Error_Handler();
			TxMessage->Update=0;
		}
#endif

#if WATCHDOG_ENABLE
		//--------------------------------------------------------------------------------------------------//
		//��������:
		//		ϵͳ״̬�л�����
		//
		//��������:
		//		����ϵͳ״̬
		//
		//��������:
		//		Protect_System* Dogs	ϵͳ�ڵĿ��Ź��ṹ��ָ��
		//		SystemState State			ϵͳ״ֵ̬
		//
		//--------------------------------------------------------------------------------------------------//
		#if MOTOR_SYSTEM_USER_ROOT 
			weak
		#endif
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
		//		Protect_System* Dogs	ϵͳ�ڵĿ��Ź��ṹ��ָ��
		//
		//--------------------------------------------------------------------------------------------------//
		#if MOTOR_SYSTEM_USER_ROOT 
			weak
		#endif
		void Feed_WatchDog(Protect_System* Dogs)
		{
			Dogs->Count_Time=0;
			SystemState_Set(Dogs,WORKING);
		}

		//--------------------------------------------------------------------------------------------------//
		//��������:
		//		ϵͳ���Ź���⺯��
		//
		//��������:
		//		�жϿ��Ź���ʱ�Ƿ񳬳�, ���ö�Ӧ��ϵͳ״̬
		//
		//��������:
		//		Protect_System* Dogs	ϵͳ�ڵĿ��Ź��ṹ��ָ��
		//
		//--------------------------------------------------------------------------------------------------//
		#if MOTOR_SYSTEM_USER_ROOT 
			weak
		#endif
		uint8_t System_Check(Protect_System* Dogs)
		{
			if(Dogs->Count_Time<WATCHDOG_TIME_MAX)
			{
				Dogs->Count_Time++;
				return 0;
			}SystemState_Set(Dogs,MISSING);
			return 1;
		}
#endif

#if SYSTEM_ENABLE
		//--------------------------------------------------------------------------------------------------//
		//��������:
		//	ȫϵͳ���Ƴ�ʼ��
		//
		//��������:
		//
		//��������:
		//		Motor_System* P_Motor 						ָ��ȫϵͳ����ָ��
		//		uint8_t Motor_Num 								����������
		//		Can_TxMessageTypeDef* TxMessage 	ָ��CAN������Ϣ��ָ��
		//
		//--------------------------------------------------------------------------------------------------//
		#if MOTOR_SYSTEM_USER_ROOT 
			weak
		#endif
		void Motor_System_Init(Motor_System* P_Motor,uint8_t Motor_Num,Can_TxMessageTypeDef* TxMessage,void (*Info_Analysis)(Motor_Info* P_Motor,uint8_t* RX_Data))
		{
			P_Motor->TxMessage=TxMessage;
			P_Motor->Motor_Num=Motor_Num;
			P_Motor->Protect.Count_Time=0;	
			P_Motor->Info_Analysis=Info_Analysis;
			SystemState_Set(&P_Motor->Protect,WORKING);
		}
		
		//--------------------------------------------------------------------------------------------------//
		//��������:
		//		ȫϵͳ������ݷ����Ĳ�������
		//
		//��������:
		//		�������ݴ����������ݽ��н���, ���Ҽ�������ԽǶȺ���ԽǶ�.
		//		Ĭ��ͨ��Э������:
		//							Data[0]----����ٶȸ�8λ
		//							Data[1]----����ٶȵ�8λ
		//							Data[2]----ת�ӽǶȸ�8λ
		//							Data[3]----ת�ӽǶȵ�8λ
		//							Data[4]----������С��8λ
		//							Data[5]----������С��8λ
		//							Data[6]----�¶�
		//							Data[7]----NULL
		//		�����ڽ��������Motor_Extra_Analysis,�����������ݽ���.
		//
		//��������:
		//		Motor_Info* P_Motor 	ָ��ȫϵͳ�����Ϣָ��
		//		uint8_t* RX_Data 			��������Ϣ������
		//
		//--------------------------------------------------------------------------------------------------//
		#if MOTOR_SYSTEM_USER_ROOT 
			weak
		#endif
		void Motor_System_Analysis(Motor_Info* P_Motor,uint8_t* RX_Data)
		{
			//���ݽ���
			P_Motor->Angle=(uint16_t)RX_Data[0]<<8|RX_Data[1];
			P_Motor->Speed=(uint16_t)RX_Data[2]<<8|RX_Data[3];
			P_Motor->Electric=(uint16_t)RX_Data[4]<<8|RX_Data[5];
			P_Motor->Temperature=RX_Data[6];
			
			if(P_Motor->Abs_Angle<=(1<<30)) Motor_Extra_Analysis(P_Motor);
		}
		//--------------------------------------------------------------------------------------------------//
		//��������:
		//		ȫϵͳ����������ݷ�������
		//
		//��������:
		//		ͨ�����������ݽ�һ������������.
		//
		//��������:
		//		Motor_Info* P_Motor 	ָ��ȫϵͳ�����Ϣָ��
		//
		//--------------------------------------------------------------------------------------------------//
		#if MOTOR_SYSTEM_USER_ROOT 
			weak
		#endif
		void Motor_Extra_Analysis(Motor_Info* P_Motor)
		{
			 //���ԽǶȼ���
			if (P_Motor->Speed!=0)
			{
				int16_t Error=P_Motor->Angle-P_Motor->Last_Angle;
				P_Motor->Abs_Angle += Error;
				if (Error < -4096)P_Motor->Abs_Angle += 8192;
				else if (Error > 4096)  P_Motor->Abs_Angle -= 8192;
			}P_Motor->Last_Angle=P_Motor->Angle;

			//��ԽǶȼ���, Ĭ�Ϸ�Χ0-360
			if(P_Motor->Abs_Angle>=0) P_Motor->Relative_Angle=(P_Motor->Abs_Angle%ONE_CIRCLE)*360.0/ONE_CIRCLE;
			else P_Motor->Relative_Angle=360-((-P_Motor->Abs_Angle)%ONE_CIRCLE)*360.0/ONE_CIRCLE;
		}
#endif

#if POS_SYSTEM_ENABLE
		//--------------------------------------------------------------------------------------------------//
		//��������:
		//		λ�û�����ϵͳ��ʼ��
		//
		//��������:
		//		��ʼ��λ�û�����ϵͳ����
		//
		//��������:
		//		Pos_System* P_Pos 								ָ��λ�û������Ϣָ��
		//		uint8_t Motor_Num 								����������
		//		Can_TxMessageTypeDef* TxMessage 	ָ��CAN������Ϣ��ָ��
		//
		//--------------------------------------------------------------------------------------------------//
		#if MOTOR_SYSTEM_USER_ROOT 
			weak
		#endif
		void Pos_System_Init(Pos_System* P_Pos,uint8_t Motor_Num,Can_TxMessageTypeDef* TxMessage,void (*Info_Analysis)(Motor_Pos_Info* P_Pos,uint8_t* RX_Data))
		{
			P_Pos->TxMessage=TxMessage;
			P_Pos->Motor_Num=Motor_Num;
			P_Pos->Protect.Count_Time=0;
			P_Pos->Info_Analysis=Info_Analysis;
			SystemState_Set(&P_Pos->Protect,WORKING);
		}
		
		//--------------------------------------------------------------------------------------------------//
		//��������:
		//		λ�û�������ݷ����Ĳ�������
		//
		//��������:
		//		�������ݴ����������ݽ��н���, ���Ҽ�������ԽǶȺ���ԽǶ�.
		//		Ĭ��ͨ��Э������:
		//							Data[0]----����ٶȸ�8λ
		//							Data[1]----����ٶȵ�8λ
		//							Data[2]----ת�ӽǶȸ�8λ
		//							Data[3]----ת�ӽǶȵ�8λ
		//							Data[4]----������С��8λ
		//							Data[5]----������С��8λ
		//							Data[6]----�¶�
		//							Data[7]----NULL
		//
		//��������:
		//		Motor_Pos_Info* P_Pos 	ָ��λ�û������Ϣָ��
		//		uint8_t* RX_Data 				��������Ϣ������
		//
		//--------------------------------------------------------------------------------------------------//
		#if MOTOR_SYSTEM_USER_ROOT 
			weak
		#endif
		void Pos_System_Analysis(Motor_Pos_Info* P_Pos,uint8_t* RX_Data)
		{
			//���ݽ���
			P_Pos->Angle=(uint16_t)RX_Data[0]<<8|RX_Data[1];
			P_Pos->Speed=(uint16_t)RX_Data[2]<<8|RX_Data[3];
			P_Pos->Electric=(uint16_t)RX_Data[4]<<8|RX_Data[5];
			P_Pos->Temperature=RX_Data[6];

			//���ԽǶȼ���
			if (P_Pos->Speed!=0)
			{
				int16_t Error=P_Pos->Angle-P_Pos->Last_Angle;
				P_Pos->Abs_Angle += Error;
				if (Error < -4096)P_Pos->Abs_Angle += 8192;
				else if (Error > 4096)  P_Pos->Abs_Angle -= 8192;
			}P_Pos->Last_Angle=P_Pos->Angle;

			//��ԽǶȼ���, Ĭ�Ϸ�Χ0-360
			if(P_Pos->Abs_Angle>=0) P_Pos->Relative_Angle=(P_Pos->Abs_Angle%ONE_CIRCLE)*360.0/ONE_CIRCLE;
			else P_Pos->Relative_Angle=360-((-P_Pos->Abs_Angle)%ONE_CIRCLE)*360.0/ONE_CIRCLE;
		}

		//--------------------------------------------------------------------------------------------------//
		//��������:
		//		λ�û�PID���㺯��
		//
		//��������:
		//		����λ�û�PID����
		//
		//��������:
		//		Pos_System* P_Pos 	λ�û�ϵͳָ��
		//
		//--------------------------------------------------------------------------------------------------//
		#if MOTOR_SYSTEM_USER_ROOT 
			weak
		#endif
		void PID_Pos_Cal(Pos_System* P_Pos)
		{
			P_Pos->Pos_PID.error =  P_Pos->Tar_Pos - P_Pos->Info.Abs_Angle;
			if(P_Pos->Pos_PID.error > P_Pos->Pos_PID.error_max)
				P_Pos->Pos_PID.error = P_Pos->Pos_PID.error_max;
			if(P_Pos->Pos_PID.error < -P_Pos->Pos_PID.error_max)
				P_Pos->Pos_PID.error = -P_Pos->Pos_PID.error_max;
			if(P_Pos->Pos_PID.error > 0 && P_Pos->Pos_PID.error < P_Pos->Pos_PID.dead_line)
				P_Pos->Pos_PID.error = 0;
			if(P_Pos->Pos_PID.error < 0 && P_Pos->Pos_PID.error > P_Pos->Pos_PID.dead_line)
				P_Pos->Pos_PID.error = 0;
			
			P_Pos->Pos_PID.intergral = P_Pos->Pos_PID.intergral + P_Pos->Pos_PID.error;
			if(P_Pos->Pos_PID.intergral > P_Pos->Pos_PID.intergral_max)
				P_Pos->Pos_PID.intergral = P_Pos->Pos_PID.intergral_max;
			if(P_Pos->Pos_PID.intergral < -P_Pos->Pos_PID.intergral_max)
				P_Pos->Pos_PID.intergral = -P_Pos->Pos_PID.intergral_max;
			
			P_Pos->Pos_PID.derivative = P_Pos->Pos_PID.error - P_Pos->Pos_PID.error_last;
			P_Pos->Pos_PID.error_last = P_Pos->Pos_PID.error;
			
			P_Pos->Pos_PID.output = P_Pos->Pos_PID.Kp*P_Pos->Pos_PID.error + P_Pos->Pos_PID.Ki*P_Pos->Pos_PID.intergral + P_Pos->Pos_PID.Kd*P_Pos->Pos_PID.derivative;
			
			if(P_Pos->Pos_PID.output > P_Pos->Pos_PID.output_max)
				P_Pos->Pos_PID.output = P_Pos->Pos_PID.output_max;
			if(P_Pos->Pos_PID.output < -P_Pos->Pos_PID.output_max)
				P_Pos->Pos_PID.output = -P_Pos->Pos_PID.output_max;
			
				P_Pos->Speed_PID.error =  P_Pos->Pos_PID.output - P_Pos->Info.Speed;
			if(P_Pos->Speed_PID.error > P_Pos->Speed_PID.error_max)
				P_Pos->Speed_PID.error = P_Pos->Speed_PID.error_max;
			if(P_Pos->Speed_PID.error < -P_Pos->Speed_PID.error_max)
				P_Pos->Speed_PID.error = -P_Pos->Speed_PID.error_max;
			if(P_Pos->Speed_PID.error > 0 && P_Pos->Speed_PID.error < P_Pos->Speed_PID.dead_line)
				P_Pos->Speed_PID.error = 0;
			if(P_Pos->Speed_PID.error < 0 && P_Pos->Speed_PID.error > P_Pos->Speed_PID.dead_line)
				P_Pos->Speed_PID.error = 0;
			
			P_Pos->Speed_PID.intergral = P_Pos->Speed_PID.intergral + P_Pos->Speed_PID.error;
			if(P_Pos->Speed_PID.intergral > P_Pos->Speed_PID.intergral_max)
				P_Pos->Speed_PID.intergral = P_Pos->Speed_PID.intergral_max;
			if(P_Pos->Speed_PID.intergral < -P_Pos->Speed_PID.intergral_max)
				P_Pos->Speed_PID.intergral = -P_Pos->Speed_PID.intergral_max;
			
			P_Pos->Speed_PID.derivative = P_Pos->Speed_PID.error - P_Pos->Speed_PID.error_last;
			P_Pos->Speed_PID.error_last = P_Pos->Speed_PID.error;
			
			P_Pos->Speed_PID.output = P_Pos->Speed_PID.Kp*P_Pos->Speed_PID.error + P_Pos->Speed_PID.Ki*P_Pos->Speed_PID.intergral + P_Pos->Speed_PID.Kd*P_Pos->Speed_PID.derivative;
			
			if(P_Pos->Speed_PID.output > P_Pos->Speed_PID.output_max)
				P_Pos->Speed_PID.output = P_Pos->Speed_PID.output_max;
			if(P_Pos->Speed_PID.output < -P_Pos->Speed_PID.output_max)
				P_Pos->Speed_PID.output = -P_Pos->Speed_PID.output_max;
		}
#endif
	
#if SPEED_SYSTEM_ENABLE
		//--------------------------------------------------------------------------------------------------//
		//��������:
		//		�ٶȻ�����ϵͳ��ʼ��
		//
		//��������:
		//		��ʼ���ٶȻ�����ϵͳ����
		//
		//��������:
		//		Speed_System* P_Speed 						ָ���ٶȻ�ϵͳָ��
		//		uint8_t Motor_Num 								�������
		//		Can_TxMessageTypeDef* TxMessage 	ָ��CAN������Ϣָ��
		//
		//--------------------------------------------------------------------------------------------------//
		#if MOTOR_SYSTEM_USER_ROOT 
			weak
		#endif
		void Speed_System_Init(Speed_System* P_Speed,uint8_t Motor_Num,Can_TxMessageTypeDef* TxMessage,void (*Info_Analysis)(Motor_Speed_Info* Motor,uint8_t* RX_Data))
		{
			P_Speed->TxMessage=TxMessage;
			P_Speed->Motor_Num=Motor_Num;
			P_Speed->Protect.Count_Time=0;
			P_Speed->Info_Analysis=Info_Analysis;
			SystemState_Set(&P_Speed->Protect,WORKING);
		}
		
		//--------------------------------------------------------------------------------------------------//
		//��������:
		//		�ٶȻ�������ݷ����Ĳ�������
		//
		//��������:
		//		�������ݴ����������ݽ��н���.
		//		Ĭ��ͨ��Э������:
		//							Data[0]----����ٶȸ�8λ
		//							Data[1]----����ٶȵ�8λ
		//							Data[2]----ת�ӽǶȸ�8λ
		//							Data[3]----ת�ӽǶȵ�8λ
		//							Data[4]----������С��8λ
		//							Data[5]----������С��8λ
		//							Data[6]----�¶�
		//							Data[7]----NULL
		//
		//��������:
		//		Motor_Speed_Info* P_Speed 	ָ���ٶȻ������Ϣָ��
		//		uint8_t* RX_Data 						��������Ϣ������
		//
		//��ֲ����:
		//		���ܲ���Ҫ��, Ҫ�ĵĻ�, ��Ϣ�����ĵط�����ͨ��Э�����ľ���.
		//--------------------------------------------------------------------------------------------------//
		#if MOTOR_SYSTEM_USER_ROOT 
			weak
		#endif
		void Speed_System_Analysis(Motor_Speed_Info* P_Speed,uint8_t* RX_Data)
		{
			P_Speed->Speed=(uint16_t)RX_Data[2]<<8|RX_Data[3];
			P_Speed->Electric=(uint16_t)RX_Data[4]<<8|RX_Data[5];
			P_Speed->Temperature=RX_Data[6];
		}

		//--------------------------------------------------------------------------------------------------//
		//��������:
		//		�ٶȻ�PID���㺯��
		//
		//��������:
		//		�����ٶȻ�PID����
		//
		//��������:
		//		Speed_System* 	�ٶȻ�ϵͳָ��
		//
		//--------------------------------------------------------------------------------------------------//
		#if MOTOR_SYSTEM_USER_ROOT 
			weak
		#endif
		void PID_Speed_Cal(Speed_System* P_Speed)
		{
			P_Speed->Speed_PID.error =  P_Speed->Tar_Speed - P_Speed->Info.Speed;
			if(P_Speed->Speed_PID.error > P_Speed->Speed_PID.error_max)
				P_Speed->Speed_PID.error = P_Speed->Speed_PID.error_max;
			if(P_Speed->Speed_PID.error < -P_Speed->Speed_PID.error_max)
				P_Speed->Speed_PID.error = -P_Speed->Speed_PID.error_max;
			if(P_Speed->Speed_PID.error > 0 && P_Speed->Speed_PID.error < P_Speed->Speed_PID.dead_line)
				P_Speed->Speed_PID.error = 0;
			if(P_Speed->Speed_PID.error < 0 && P_Speed->Speed_PID.error > P_Speed->Speed_PID.dead_line)
				P_Speed->Speed_PID.error = 0;
			
			P_Speed->Speed_PID.intergral = P_Speed->Speed_PID.intergral + P_Speed->Speed_PID.error;
			if(P_Speed->Speed_PID.intergral > P_Speed->Speed_PID.intergral_max)
				P_Speed->Speed_PID.intergral = P_Speed->Speed_PID.intergral_max;
			if(P_Speed->Speed_PID.intergral < -P_Speed->Speed_PID.intergral_max)
				P_Speed->Speed_PID.intergral = -P_Speed->Speed_PID.intergral_max;
			
			P_Speed->Speed_PID.derivative = P_Speed->Speed_PID.error - P_Speed->Speed_PID.error_last;
			P_Speed->Speed_PID.error_last = P_Speed->Speed_PID.error;
			
			P_Speed->Speed_PID.output = P_Speed->Speed_PID.Kp*P_Speed->Speed_PID.error + P_Speed->Speed_PID.Ki*P_Speed->Speed_PID.intergral + P_Speed->Speed_PID.Kd*P_Speed->Speed_PID.derivative;
			
			if(P_Speed->Speed_PID.output > P_Speed->Speed_PID.output_max)
				P_Speed->Speed_PID.output = P_Speed->Speed_PID.output_max;
			if(P_Speed->Speed_PID.output < -P_Speed->Speed_PID.output_max)
				P_Speed->Speed_PID.output = -P_Speed->Speed_PID.output_max;
		}
#endif



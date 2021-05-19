//文件名称:		motor_system.c
//对应头文件:	motor_system.h
//主要功能:
//				该文件有四大部分功能, 分别为看门狗保护, 全系统控制功能, 位置环控制功能, 速度环控制功能.
//				通过改变预编译对应的使能标识符, 来使能各个部分的功能.
//
//				看门狗保护
//					--设置工作状态
//					--检测工作状态
//
//				全系统控制功能
//					--电机初始化
//					--电机数据解析
//
//				位置环控制功能
//					--电机初始化
//					--电机数据解析
//					--位置环PID计算
//
//				速度环控制功能
//					--电机初始化
//					--电机数据解析
//					--速度环PID计算
//
//				基本功能
//					--配置CAN通信
//					--CAN发送消息初始化
//					--CAN发送消息
//					--PID计算
//
//时间:
//		2021/5/7
//
//版本:	2.0V
//
//---------头文件引用部分---------//
#include "motor_system.h"
//--------------------------------//

//---------变量声明部分-----------//
Can_TxMessageTypeDef Can_TxMessageList[CAN_TXMESSAGEINDEXMAX];
//--------------------------------//

#ifdef __MOTOR_SYSTEM_H__
		//--------------------------------------------------------------------------------------------------//
		//函数名称:
		//		CAN过滤器与中断配置
		//
		//函数功能:
		//		配置CAN的过滤器, 设置接收邮箱号码, CAN1为0~13, CAN2为14~28;
		//		并使能can的接收中断.
		//
		//参数类型:
		//		CAN_HandleTypeDef* hcan 	指向CAN号的指针
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
		//函数名称:
		//		PID参数初始化函数
		//
		//函数功能:
		//		初始化系统PID参数
		//
		//参数类型:
		//		PID* pid							PID指针
		//		float Kp							PID的Kp
		//		float Ki							PID的Ki
		//		float Kd							PID的Kd
		//		float error_max				误差最大值
		//		float dead_line				死区
		//		float intergral_max		误差累积最大值
		//		float output_max			输出最大值
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
		//函数名称:
		//		Can_TxMessage参数初始化函数
		//
		//函数功能:
		//		初始化系统Can_TxMessage参数
		//
		//参数类型:
		//		Can_TxMessageTypeDef* TxMessage		指向Can_TxMessage的指针
		//		CAN_HandleTypeDef *hcan 					指向CAN的指针
		//		uint32_t DLC											CAN发送消息的数据帧长度
		//		uint32_t RTR											CAN遥控标识帧位
		//		uint32_t IDE											CAN拓展帧标识位
		//		uint32_t StdId										CAN标准标识符位
		//		uint32_t ExtId										CAN拓展标识符位
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
		//函数名称:
		//		PID计算函数
		//
		//函数功能:
		//		进行PID计算
		//
		//参数类型:
		//		PID* pid 		PID指针
		//		float fdbV 	当前值
		//		float tarV 	目标值
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
		//函数名称:
		//		CAN发送消息添加函数
		//
		//函数功能:
		//		将PID计算出来的输出值拆分为两位数据存入指定数组
		//
		//参数类型:
		//		float Output 			目标值
		//		uint8_t* Tx_Data	目标数组
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
		//函数名称:
		//		CAN通信发送函数
		//
		//函数功能:
		//		发送CAN数据
		//
		//参数类型:
		//		Can_TxMessageTypeDef* TxMessage		指向CAN发送消息结构体指针
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
		//函数名称:
		//		系统状态切换函数
		//
		//函数功能:
		//		设置系统状态
		//
		//参数类型:
		//		Protect_System* Dogs	系统内的看门狗结构体指针
		//		SystemState State			系统状态值
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
		//函数名称:
		//		看门狗喂狗函数
		//
		//函数功能:
		//		将看门狗的计数清零
		//
		//参数类型:
		//		Protect_System* Dogs	系统内的看门狗结构体指针
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
		//函数名称:
		//		系统看门狗检测函数
		//
		//函数功能:
		//		判断看门狗计时是否超出, 设置对应的系统状态
		//
		//参数类型:
		//		Protect_System* Dogs	系统内的看门狗结构体指针
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
		//函数名称:
		//	全系统控制初始化
		//
		//函数功能:
		//
		//参数类型:
		//		Motor_System* P_Motor 						指向全系统控制指针
		//		uint8_t Motor_Num 								保存电机号码
		//		Can_TxMessageTypeDef* TxMessage 	指向CAN发送消息的指针
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
		//函数名称:
		//		全系统电机数据分析的操作函数
		//
		//函数功能:
		//		根据数据传进来的数据进行解析, 并且计算出绝对角度和相对角度.
		//		默认通信协议内容:
		//							Data[0]----电机速度高8位
		//							Data[1]----电机速度低8位
		//							Data[2]----转子角度高8位
		//							Data[3]----转子角度低8位
		//							Data[4]----电流大小高8位
		//							Data[5]----电流大小低8位
		//							Data[6]----温度
		//							Data[7]----NULL
		//		并且在解包完后调用Motor_Extra_Analysis,进行其他数据解析.
		//
		//参数类型:
		//		Motor_Info* P_Motor 	指向全系统电机信息指针
		//		uint8_t* RX_Data 			保存电机信息的数组
		//
		//--------------------------------------------------------------------------------------------------//
		#if MOTOR_SYSTEM_USER_ROOT 
			weak
		#endif
		void Motor_System_Analysis(Motor_Info* P_Motor,uint8_t* RX_Data)
		{
			//数据解析
			P_Motor->Angle=(uint16_t)RX_Data[0]<<8|RX_Data[1];
			P_Motor->Speed=(uint16_t)RX_Data[2]<<8|RX_Data[3];
			P_Motor->Electric=(uint16_t)RX_Data[4]<<8|RX_Data[5];
			P_Motor->Temperature=RX_Data[6];
			
			if(P_Motor->Abs_Angle<=(1<<30)) Motor_Extra_Analysis(P_Motor);
		}
		//--------------------------------------------------------------------------------------------------//
		//函数名称:
		//		全系统电机额外数据分析函数
		//
		//函数功能:
		//		通过解包后的数据进一步计算电机数据.
		//
		//参数类型:
		//		Motor_Info* P_Motor 	指向全系统电机信息指针
		//
		//--------------------------------------------------------------------------------------------------//
		#if MOTOR_SYSTEM_USER_ROOT 
			weak
		#endif
		void Motor_Extra_Analysis(Motor_Info* P_Motor)
		{
			 //绝对角度计算
			if (P_Motor->Speed!=0)
			{
				int16_t Error=P_Motor->Angle-P_Motor->Last_Angle;
				P_Motor->Abs_Angle += Error;
				if (Error < -4096)P_Motor->Abs_Angle += 8192;
				else if (Error > 4096)  P_Motor->Abs_Angle -= 8192;
			}P_Motor->Last_Angle=P_Motor->Angle;

			//相对角度计算, 默认范围0-360
			if(P_Motor->Abs_Angle>=0) P_Motor->Relative_Angle=(P_Motor->Abs_Angle%ONE_CIRCLE)*360.0/ONE_CIRCLE;
			else P_Motor->Relative_Angle=360-((-P_Motor->Abs_Angle)%ONE_CIRCLE)*360.0/ONE_CIRCLE;
		}
#endif

#if POS_SYSTEM_ENABLE
		//--------------------------------------------------------------------------------------------------//
		//函数名称:
		//		位置环控制系统初始化
		//
		//函数功能:
		//		初始化位置环控制系统参数
		//
		//参数类型:
		//		Pos_System* P_Pos 								指向位置环电机信息指针
		//		uint8_t Motor_Num 								保存电机号码
		//		Can_TxMessageTypeDef* TxMessage 	指向CAN发送消息的指针
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
		//函数名称:
		//		位置环电机数据分析的操作函数
		//
		//函数功能:
		//		根据数据传进来的数据进行解析, 并且计算出绝对角度和相对角度.
		//		默认通信协议内容:
		//							Data[0]----电机速度高8位
		//							Data[1]----电机速度低8位
		//							Data[2]----转子角度高8位
		//							Data[3]----转子角度低8位
		//							Data[4]----电流大小高8位
		//							Data[5]----电流大小低8位
		//							Data[6]----温度
		//							Data[7]----NULL
		//
		//参数类型:
		//		Motor_Pos_Info* P_Pos 	指向位置环电机信息指针
		//		uint8_t* RX_Data 				保存电机信息的数组
		//
		//--------------------------------------------------------------------------------------------------//
		#if MOTOR_SYSTEM_USER_ROOT 
			weak
		#endif
		void Pos_System_Analysis(Motor_Pos_Info* P_Pos,uint8_t* RX_Data)
		{
			//数据解析
			P_Pos->Angle=(uint16_t)RX_Data[0]<<8|RX_Data[1];
			P_Pos->Speed=(uint16_t)RX_Data[2]<<8|RX_Data[3];
			P_Pos->Electric=(uint16_t)RX_Data[4]<<8|RX_Data[5];
			P_Pos->Temperature=RX_Data[6];

			//绝对角度计算
			if (P_Pos->Speed!=0)
			{
				int16_t Error=P_Pos->Angle-P_Pos->Last_Angle;
				P_Pos->Abs_Angle += Error;
				if (Error < -4096)P_Pos->Abs_Angle += 8192;
				else if (Error > 4096)  P_Pos->Abs_Angle -= 8192;
			}P_Pos->Last_Angle=P_Pos->Angle;

			//相对角度计算, 默认范围0-360
			if(P_Pos->Abs_Angle>=0) P_Pos->Relative_Angle=(P_Pos->Abs_Angle%ONE_CIRCLE)*360.0/ONE_CIRCLE;
			else P_Pos->Relative_Angle=360-((-P_Pos->Abs_Angle)%ONE_CIRCLE)*360.0/ONE_CIRCLE;
		}

		//--------------------------------------------------------------------------------------------------//
		//函数名称:
		//		位置环PID计算函数
		//
		//函数功能:
		//		进行位置环PID计算
		//
		//参数类型:
		//		Pos_System* P_Pos 	位置环系统指针
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
		//函数名称:
		//		速度环控制系统初始化
		//
		//函数功能:
		//		初始化速度环控制系统参数
		//
		//参数类型:
		//		Speed_System* P_Speed 						指向速度环系统指针
		//		uint8_t Motor_Num 								电机号码
		//		Can_TxMessageTypeDef* TxMessage 	指向CAN发送消息指针
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
		//函数名称:
		//		速度环电机数据分析的操作函数
		//
		//函数功能:
		//		根据数据传进来的数据进行解析.
		//		默认通信协议内容:
		//							Data[0]----电机速度高8位
		//							Data[1]----电机速度低8位
		//							Data[2]----转子角度高8位
		//							Data[3]----转子角度低8位
		//							Data[4]----电流大小高8位
		//							Data[5]----电流大小低8位
		//							Data[6]----温度
		//							Data[7]----NULL
		//
		//参数类型:
		//		Motor_Speed_Info* P_Speed 	指向速度环电机信息指针
		//		uint8_t* RX_Data 						保存电机信息的数组
		//
		//移植建议:
		//		大框架不需要改, 要改的话, 信息解析的地方根据通信协议来改就行.
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
		//函数名称:
		//		速度环PID计算函数
		//
		//函数功能:
		//		进行速度环PID计算
		//
		//参数类型:
		//		Speed_System* 	速度环系统指针
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



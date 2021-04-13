#include "odrive_can.h"


ODrive ODrive0;
ODrive ODrive1;
extern CAN_HandleTypeDef hcan1;

void ODrive_Recevice(uint16_t StdID,uint8_t* Data)
{
	Axis* P_Axis=NULL;
	uint16_t RxNode_ID=(StdID&(0x3f<<5))>>5;
	uint8_t CMD=StdID&0x1f;
	
			 if(RxNode_ID==ODrive0.Axis0.Node_ID) P_Axis=&ODrive0.Axis0;
	else if(RxNode_ID==ODrive0.Axis1.Node_ID) P_Axis=&ODrive0.Axis1;
	else if(RxNode_ID==ODrive1.Axis0.Node_ID) P_Axis=&ODrive1.Axis0;
	else if(RxNode_ID==ODrive1.Axis1.Node_ID) P_Axis=&ODrive1.Axis1;
	else return ;

	switch(CMD)
	{
		case 0x01:{
			P_Axis->Error=*((uint32_t*)Data);
			P_Axis->Current_State=*((uint32_t*)(Data+4));
			break;
		}case 0x03:{
			P_Axis->Motor_Error=*((uint32_t*)Data);
			break;
		}case 0x04:{
			P_Axis->Encoder_Error=*((uint32_t*)Data);
			break;
		}case 0x05:{
			P_Axis->Sensorless_Error=*((uint32_t*)Data);
			break;
		}case 0x09:{
			P_Axis->Encoder_Pos_Estimate=*((float*)Data);
			P_Axis->Encoder_Vel_Estimate=*((float*)(Data+4));
			break;
		}case 0x0A:{
			P_Axis->Encoder_Shadow_Count=*((int32_t*)Data);
			P_Axis->Encoder_Count_in_CPR=*((int32_t*)(Data+4));
			break;
		}default:break;
	}if(P_Axis->Error!=0) Error_Handler();
}

void ODrive_Transmit(Axis* _Axis,uint16_t CMD)
{
	if(_Axis==NULL) return ;
	_Axis->StdID|=(CMD&0x1f);
	
	switch(CMD)
	{
		case 0x3:Get_Motor_Error(_Axis);break;
		case 0x4:Get_Encoder_Error(_Axis);break;
		case 0x7:Set_Axis_Requested_State(_Axis);break;
		case 0x9:Get_Encoder_Estimates(_Axis);break;
		case 0xA:Get_Encoder_Count(_Axis);break;
		case 0xD:Set_Input_Vel(_Axis);break;
		case 0x16:Reboot_ODrive(_Axis);break;
		default:break;
	}	_Axis->StdID=_Axis->Node_ID<<5;
	
}

void Get_Motor_Error(Axis* _Axis)									//0x3
{
	Send_To_ODrive(&hcan1,_Axis->StdID,NULL,0,1);
}

void Get_Encoder_Error(Axis* _Axis)								//0x4
{
	Send_To_ODrive(&hcan1,_Axis->StdID,NULL,0,1);
}

	uint8_t data[4]={0};
void Set_Axis_Requested_State(Axis* _Axis)				//0x7
{
	data[0]=*((uint8_t*)&_Axis->Requested_State);
	data[1]=*((uint8_t*)&_Axis->Requested_State+1);
	Send_To_ODrive(&hcan1,_Axis->StdID,data,4,0);
}

void Get_Encoder_Estimates(Axis* _Axis)						//0x9
{
	Send_To_ODrive(&hcan1,_Axis->StdID,NULL,0,1);
}

void Get_Encoder_Count(Axis* _Axis)								//0xA
{
	Send_To_ODrive(&hcan1,_Axis->StdID,NULL,0,1);
}

void Set_Input_Vel(Axis* _Axis)										//0xD
{
	uint8_t data[8]={0};
	int32_t Tar_Vel=_Axis->Input_Vel*100;
	data[0]=*((uint8_t*)&Tar_Vel);
	data[1]=*((uint8_t*)&Tar_Vel+1);
	data[2]=*((uint8_t*)&Tar_Vel+2);
	data[3]=*((uint8_t*)&Tar_Vel+3);
	Send_To_ODrive(&hcan1,_Axis->StdID,data,8,0);
}

void Reboot_ODrive(Axis* _Axis)
{
	Send_To_ODrive(&hcan1,_Axis->StdID,NULL,0,0);
}
void Send_To_ODrive(CAN_HandleTypeDef *hcan,uint16_t StdID,uint8_t* Data,uint8_t len,uint8_t RTR)
{
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;
	
  TxHeader.RTR = RTR;
  TxHeader.IDE = 0;            
  TxHeader.StdId=StdID;
  TxHeader.TransmitGlobalTime = DISABLE;
  TxHeader.DLC = len;
	
	if (HAL_CAN_AddTxMessage(hcan, &TxHeader, Data, &TxMailbox) != HAL_OK)
  {
   /* Transmission request Error */
     Error_Handler();
	}
}

void ODrive_Init(void)
{
	Axis_Init(&ODrive0.Axis0,0);
	Axis_Init(&ODrive0.Axis1,1);
	
	Axis_Init(&ODrive1.Axis0,2);
	Axis_Init(&ODrive1.Axis1,3);
	
	//Motor_Init(&ODrive0.Axis0);
}

void Motor_Init(Axis* _Axis)
{
	_Axis->Requested_State=3;
	ODrive_Transmit(_Axis,0x7);
	while(_Axis->Current_State!=0x4);
	while(_Axis->Current_State!=0x7);
	while(_Axis->Current_State!=0x1);
	if(_Axis->Error==0) _Axis->Requested_State=8,ODrive_Transmit(_Axis,0x7);
}

void Axis_Init(Axis* _Axis,uint8_t NodeID)
{
	_Axis->CMD=0;
	_Axis->Current_State=0;
	_Axis->Encoder_Count_in_CPR=0;
	_Axis->Encoder_Error=0;
	_Axis->Encoder_Pos_Estimate=0;
	_Axis->Encoder_Shadow_Count=0;
	_Axis->Encoder_Vel_Estimate=0;
	_Axis->Error=0;
	_Axis->Input_Vel=0;
	_Axis->Motor_Error=0;
	_Axis->Node_ID=NodeID;
	_Axis->Requested_State=0;
	_Axis->Sensorless_Error=0;
	_Axis->StdID=NodeID<<5;
	_Axis->Vbus_Voltage=0;
}
	

#include "odrive_can.h"

ODrive ODrive0;
ODrive ODrive1;
extern CAN_HandleTypeDef hcan1;

uint8_t data[4]={0};
uint8_t test_data=0;

void ODrive_Recevice(uint16_t StdID,uint8_t* Data)
{
	Axis* P_Axis=NULL;
	uint32_t temp=0;
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
		}case 0x09:{
			temp|=Data[4];
			temp|=Data[5]<<8;
			temp|=Data[6]<<16;
			temp|=Data[7]<<24;
			P_Axis->Encoder_Vel_Estimate=(float)temp;
			break;
		}default:break;
	}if(P_Axis->Error!=0) Error_Handler();
	Feed_WatchDog(&P_Axis->Protect);
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
	Send_To_ODrive(&hcan1,_Axis->StdID,NULL,4,2);
}

void Get_Encoder_Error(Axis* _Axis)								//0x4
{
	Send_To_ODrive(&hcan1,_Axis->StdID,&test_data,4,2);
}


void Set_Axis_Requested_State(Axis* _Axis)				//0x7
{
	data[0]=*((uint8_t*)&_Axis->Requested_State);
	data[1]=*((uint8_t*)&_Axis->Requested_State+1);
	Send_To_ODrive(&hcan1,_Axis->StdID,data,4,0);
}

void Get_Encoder_Estimates(Axis* _Axis)						//0x9
{
	Send_To_ODrive(&hcan1,_Axis->StdID,NULL,8,2);
}

void Get_Encoder_Count(Axis* _Axis)								//0xA
{
	Send_To_ODrive(&hcan1,_Axis->StdID,&test_data,8,2);
}

void Set_Input_Vel(Axis* _Axis)										//0xD
{
	if(_Axis->Error!=0) return ;
	uint8_t data[8]={0};
	float Tar_Vel=_Axis->Input_Vel;
	data[0]=*((uint8_t*)&Tar_Vel);
	data[1]=*((uint8_t*)&Tar_Vel+1);
	data[2]=*((uint8_t*)&Tar_Vel+2);
	data[3]=*((uint8_t*)&Tar_Vel+3);
	Send_To_ODrive(&hcan1,_Axis->StdID,data,8,0);
}

void Reboot_ODrive(Axis* _Axis)
{
	Send_To_ODrive(&hcan1,_Axis->StdID,&test_data,0,0);
}

uint8_t flag=0;

void Send_To_ODrive(CAN_HandleTypeDef *hcan,uint16_t StdID,uint8_t* Data,uint8_t len,uint8_t RTR)
{
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;
	
  TxHeader.RTR = RTR;
  TxHeader.IDE = 0;            
  TxHeader.StdId=StdID;
  TxHeader.TransmitGlobalTime = DISABLE;
  TxHeader.DLC = len;

//	if (HAL_CAN_AddTxMessage(hcan, &TxHeader, Data, &TxMailbox) != HAL_OK)
	{
	/* Transmission request Error */
		//Error_Handler();
	}

}

void ODrive_Init(void)
{
	Axis_Init(&ODrive0.Axis0,0);
	Axis_Init(&ODrive0.Axis1,1);
	
	Axis_Init(&ODrive1.Axis0,2);
	Axis_Init(&ODrive1.Axis1,3);
}

void Motor_Init(Axis* _Axis)
{
	static uint8_t num[4]={0};
	uint8_t* P_num=&num[_Axis->Node_ID];
	
	if(_Axis->Current_State==0) return ;
	if(*P_num==0){
		_Axis->Requested_State=3;
		ODrive_Transmit(_Axis,0x7);
		(*P_num)++;
	}if(*P_num==1&&_Axis->Current_State==4) *P_num=2;
	if(*P_num==2&&_Axis->Current_State==7) *P_num=3;
	if(*P_num==3&&_Axis->Current_State==1) *P_num=4;
	if(*P_num==4){
		_Axis->Requested_State=8;
		ODrive_Transmit(_Axis,0x7);
		_Axis->Requested_State=0;
		*P_num++;
	}
}

void Axis_Init(Axis* _Axis,uint8_t NodeID)
{
	_Axis->CMD=0;
	_Axis->Current_State=0;
	_Axis->Encoder_Error=0;
	_Axis->Encoder_Vel_Estimate=0;
	_Axis->Error=0;
	_Axis->Input_Vel=0;
	_Axis->Motor_Error=0;
	_Axis->Node_ID=NodeID;
	_Axis->Requested_State=0;
	_Axis->StdID=NodeID<<5;
	_Axis->Vbus_Voltage=0;
}
	


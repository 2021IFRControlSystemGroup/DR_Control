#include "odrive_can.h"

ODrive ODrive0;
ODrive ODrive1;
extern CAN_HandleTypeDef hcan1;
extern Can_TxMessageTypeDef CanTxMessageList[8];

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
	}_Axis->TxMessage->Update=1;
	_Axis->StdID=_Axis->Node_ID<<5;
}

void Get_Motor_Error(Axis* _Axis)									//0x3
{
	Send_To_ODrive(_Axis->TxMessage,_Axis->StdID,NULL,4,2);
}

void Get_Encoder_Error(Axis* _Axis)								//0x4
{
	Send_To_ODrive(_Axis->TxMessage,_Axis->StdID,&test_data,4,2);
}


void Set_Axis_Requested_State(Axis* _Axis)				//0x7
{
	_Axis->TxMessage->Data[0]=*((uint8_t*)&_Axis->Requested_State);
	_Axis->TxMessage->Data[1]=*((uint8_t*)&_Axis->Requested_State+1);
	_Axis->TxMessage->Header.StdId=_Axis->StdID;
	_Axis->TxMessage->Update=1;
	Can_DataTypeSet(&_Axis->TxMessage->Header);
	
	Send_To_ODrive(_Axis->TxMessage,_Axis->StdID,data,4,0);
}

void Get_Encoder_Estimates(Axis* _Axis)						//0x9
{
	Send_To_ODrive(_Axis->TxMessage,_Axis->StdID,NULL,8,2);
}

void Get_Encoder_Count(Axis* _Axis)								//0xA
{
	Send_To_ODrive(_Axis->TxMessage,_Axis->StdID,&test_data,8,2);
}

void Set_Input_Vel(Axis* _Axis)										//0xD
{
	if(_Axis->Error!=0) return ;
	float Tar_Vel=_Axis->Input_Vel;
	_Axis->TxMessage->Data[0]=*((uint8_t*)&Tar_Vel);
	_Axis->TxMessage->Data[1]=*((uint8_t*)&Tar_Vel+1);
	_Axis->TxMessage->Data[2]=*((uint8_t*)&Tar_Vel+2);
	_Axis->TxMessage->Data[3]=*((uint8_t*)&Tar_Vel+3);
	_Axis->TxMessage->Header.StdId=_Axis->StdID;
	_Axis->TxMessage->Update=1;
	Can_DataTypeSet(&_Axis->TxMessage->Header);

}

void Can_DataTypeSet(CAN_TxHeaderTypeDef* TxHeader)
{
	TxHeader->RTR = 0;
  TxHeader->IDE = 0;            
	TxHeader->TransmitGlobalTime = DISABLE;
  TxHeader->DLC = 8;
}

void Reboot_ODrive(Axis* _Axis)
{
	Send_To_ODrive(_Axis->TxMessage,_Axis->StdID,&test_data,0,0);
}

uint8_t flag=0;

void Send_To_ODrive(Can_TxMessageTypeDef* TxMessage,uint16_t StdID,uint8_t* Data,uint8_t len,uint8_t RTR)
{
  TxMessage->Header.RTR = RTR;
  TxMessage->Header.IDE = 0;            
  TxMessage->Header.StdId=StdID;
  TxMessage->Header.TransmitGlobalTime = DISABLE;
  TxMessage->Header.DLC = len;
}

void ODrive_Init(ODrive* _ODrive)
{
	Axis_Init(&_ODrive->Axis0,0);
	Axis_Init(&_ODrive->Axis1,1);
}

void Axis_CloseLoop_Init(Axis* _Axis)
{
	static uint8_t num[4]={0};
	uint8_t* P_num=&num[_Axis->Node_ID];
	
	if(_Axis->Error!=0||_Axis->Current_State==0) return ;
	if(_Axis->Current_State==8) SystemState_Set(&_Axis->Protect,WORKING);
    if(*P_num==0&&_Axis->Current_State==1){
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
		*P_num=5;
	}if(*P_num==5&&_Axis->Current_State==8) SystemState_Set(&_Axis->Protect,WORKING);
}

void Axis_Init(Axis* _Axis,uint8_t NodeID)
{
	_Axis->CMD=0;
	_Axis->Current_State=0;																//调试的时候为1, 实际情况下为0
	_Axis->Encoder_Error=0;
	_Axis->Encoder_Vel_Estimate=0;
	_Axis->Error=0;
	_Axis->Input_Vel=0;
	_Axis->Motor_Error=0;
	_Axis->Node_ID=NodeID;
	_Axis->Requested_State=0;
	_Axis->StdID=NodeID<<5;
	_Axis->Vbus_Voltage=0;
	
	_Axis->TxMessage=&CanTxMessageList[NodeID+1];
	_Axis->Instruction=&CanTxMessageList[NodeID+5];
	_Axis->Protect.Count_Time=0;	SystemState_Set(&_Axis->Protect,INITING);
}
	


#include "odrive_can.h"


extern CAN_HandleTypeDef hcan1;


ODrive ODrive0;
ODrive ODrive1;

uint16_t ID=0;
uint32_t State=3;
int32_t Mode=0x1;

void ODrive_Init(ODrive* _ODrive)
{
	if(_ODrive==&ODrive1)
	{
		_ODrive->Axis0.Node_ID=0x1<<5;
		_ODrive->Axis1.Node_ID=0x2<<5;
	}
	if(_ODrive==&ODrive0)
	{
		;
	}
}

void ODrive_Recevice(uint16_t StdID,uint8_t* Data)
{
	Axis* P_Axis=NULL;
	if(RX_NODE_ID==ODRIVE1_AXIS0) P_Axis=&ODrive1.Axis0;
	if(RX_NODE_ID==ODRIVE1_AXIS1) P_Axis=&ODrive1.Axis1;
	if(!P_Axis) return ;
	
	switch(StdID&0X03)
	{
		case 0x01:HEARTBEAT_ANALYSIS;break;
		case 0x03:MOTOR_ERROR_ANALYSIS;break;
		case 0x04:ENCODER_ERROR_ANALYSIS;break;
		case 0x05:SENSORLES_ERROR_ANALYSIS;break;
		case 0x09:ENCODER_ESTIMATES_ANALYSIS;break;
		case 0x0A:ENCODER_COUNT_ANALYSIS;break;
		case 0x14:GET_IQ_ANALYSIS;break;
		case 0x15:SENSORLESS_ESTIMATES_ANALYSIS;break;
	}
}


void ODrive_Send(Axis* _Axis,uint16_t CMD)
{
	uint16_t StdID=0;
	if(_Axis==&ODrive1.Axis0) StdID|=ODRIVE1_AXIS0;
	if(_Axis==&ODrive1.Axis1) StdID|=ODRIVE1_AXIS1;
	if(!_Axis) return ;
	StdID|=CMD;
	
	switch(CMD)
	{
//		case 0x3:Get_Motor_Error();break;
		case 0x6:Set_Axis_Node_ID(_Axis,StdID,ID);break;
		case 0x7:Set_Axis_Requested_State(_Axis,StdID,State);break;
		case 0xB:Set_Controller_Modes(_Axis,StdID,Mode);break;
		case 0xC:Set_Input_Pos(_Axis,StdID);break;
		case 0xD:Set_Input_Vel(_Axis,StdID);break;
		case 0x16:Reboot_ODrive(StdID);
		default:break;
	}
}

void Get_Motor_Error(Axis* _Axis,uint16_t StdID,uint16_t ID)
{
	
	//Send_To_ODrive();

}

uint8_t Data123[8]={0};
void Set_Axis_Node_ID(Axis* _Axis,uint16_t StdID,uint16_t ID)
{
	uint8_t Data[8]={0};
	_Axis->Node_ID=ID;
	Data[0]=*((uint8_t*)&ID);
	Data[1]=*((uint8_t*)&ID+1);
	Send_To_ODrive(&hcan1,StdID,Data,4,0);
}

void Set_Axis_Requested_State(Axis* _Axis,uint16_t StdID,uint16_t State)
{
	uint8_t Data[4]={0};
	_Axis->Requested_State=State;
	Data[0]=*((uint8_t*)&State);
	Data[1]=*((uint8_t*)&State+1);
	Send_To_ODrive(&hcan1,StdID,Data,4,0);
}

void Set_Controller_Modes(Axis* _Axis,uint16_t StdID,int32_t Mode)
{
	uint8_t Data[8]={0};
	Data[0]=*((uint8_t*)&Mode);
	Data[1]=*((uint8_t*)&Mode+1);
	Data[2]=*((uint8_t*)&Mode+2);
	Data[3]=*((uint8_t*)&Mode+3);
	Send_To_ODrive(&hcan1,StdID,Data,8,0);
}

void Set_Input_Vel(Axis* _Axis,uint16_t StdID)
{
	if(_Axis->Current_State!=8) return ;
	uint8_t Data[8]={0};
	int32_t Vel=(int32_t)_Axis->Input_Vel*100;
	Data[0]=*((uint8_t*)&Vel);
	Data[1]=*((uint8_t*)&Vel+1);
	Data[2]=*((uint8_t*)&Vel+2);
	Data[3]=*((uint8_t*)&Vel+3);
	Data[4]=0;
	Data[5]=0;
	Data[6]=0;
	Data[7]=0;
	Send_To_ODrive(&hcan1,StdID,Data,8,0);
}

void Set_Input_Pos(Axis* _Axis,uint16_t StdID)
{
	if(_Axis->Current_State!=8) return ;
	uint8_t Data[8]={0};
	int32_t Pos=(int32_t)_Axis->Input_Pos;
	Data[0]=*((uint8_t*)&Pos);
	Data[1]=*((uint8_t*)&Pos+1);
	Data[2]=*((uint8_t*)&Pos+2);
	Data[3]=*((uint8_t*)&Pos+3);
	Send_To_ODrive(&hcan1,StdID,Data,8,0);
}

void Reboot_ODrive(uint16_t StdID)
{
	Send_To_ODrive(&hcan1,StdID,NULL,0,1);
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


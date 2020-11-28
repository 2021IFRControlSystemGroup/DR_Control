#include "odrive_can.h"

ODrive ODrive0;
ODrive ODrive1;
uint8_t OK=0;
void ODrive_Recevice(ODrive* _ODrive,uint16_t StdID,uint8_t* Data)
{
	Axis* P_Axis=NULL;
	if(StdID&ODRIVE1_AXIS0)
	{
		P_Axis=&_ODrive->Axis0;
	}
	if(StdID&ODRIVE1_AXIS1)
	{
		P_Axis=&_ODrive->Axis1;
	}
	if(StdID==TEST)
	{
		OK=1;
	}
	if(P_Axis) Axis_CMD(P_Axis,StdID&0X03,Data);
}

void Axis_CMD(Axis* _Axis,uint16_t CMD,uint8_t* Data)
{
	switch(CMD)
	{
		case 0x01:HeartBeat_Analysis(_Axis,Data);break;
		case 0x03:Motor_Error_Analysis(_Axis,Data);break;
		case 0x17:Get_Vbus_Voltage_Analysis(_Axis,Data);break;
	}
}

void HeartBeat_Analysis(Axis* _Axis,uint8_t* Data)
{
	_Axis->Error=Data[3];_Axis->Error<<=8;
	_Axis->Error|=Data[2];_Axis->Error<<=8;
	_Axis->Error|=Data[1];_Axis->Error<<=8;
	_Axis->Error|=Data[0];
	
	_Axis->Current_State=Data[7];_Axis->Current_State<<=8;
	_Axis->Current_State|=Data[6];_Axis->Current_State<<=8;
	_Axis->Current_State|=Data[5];_Axis->Current_State<<=8;
	_Axis->Current_State|=Data[4];
}

void Get_Vbus_Voltage_Analysis(Axis* _Axis,uint8_t* Data)
{
	uint32_t Temp=0;
	Temp=Data[3];Temp<<=8;
	Temp|=Data[2];Temp<<=8;
	Temp|=Data[1];Temp<<=8;
	Temp|=Data[0];
	_Axis->Vbus_Voltage=(float)Temp;
}
uint16_t StdID=0;

void Motor_Error_Analysis(Axis* _Axis,uint8_t* Data)
{
	_Axis->Motor_Error=Data[7];_Axis->Motor_Error<<=8;
	_Axis->Motor_Error|=Data[6];_Axis->Motor_Error<<=8;
	_Axis->Motor_Error|=Data[5];_Axis->Motor_Error<<=8;
	_Axis->Motor_Error|=Data[4];
}

void ODrive_Send(CAN_HandleTypeDef *hcan,Axis* _Axis,uint16_t CMD)
{

	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;
	uint8_t Tx_Data[4]={3,0,0,0};
	
	if(_Axis==&ODrive1.Axis0) StdID|=ODRIVE1_AXIS0;
	if(_Axis==&ODrive1.Axis1) StdID|=ODRIVE1_AXIS1;
	
	StdID|=(CMD&0x1f);
	
  TxHeader.RTR = 0;
  TxHeader.IDE = 0;            
  TxHeader.StdId=StdID;
  TxHeader.TransmitGlobalTime = DISABLE;
  TxHeader.DLC = 4;
	
	
	if (HAL_CAN_AddTxMessage(hcan, &TxHeader, Tx_Data, &TxMailbox) != HAL_OK)
  {
   /* Transmission request Error */
     Error_Handler();
	}
}


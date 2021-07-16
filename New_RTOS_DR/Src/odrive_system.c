#include "odrive_system.h"

ODrive ODrive0;
ODrive ODrive1;
uint8_t Blank_Data[8] = {0};
uint8_t test_data = 0;

void ODrive_CAN_Recevice(uint16_t StdID,uint8_t* Data)
{
	Axis* P_Axis = NULL;
	uint16_t RxNode_ID = (StdID & (0x3f << 5)) >> 5;
	uint8_t CMD = StdID & 0x1f;
	
		 if(RxNode_ID == ODrive0.Axis0.Node_ID) P_Axis = &ODrive0.Axis0;
	else if(RxNode_ID == ODrive0.Axis1.Node_ID) P_Axis = &ODrive0.Axis1;
	else if(RxNode_ID == ODrive1.Axis0.Node_ID) P_Axis = &ODrive1.Axis0;
	else if(RxNode_ID == ODrive1.Axis1.Node_ID) P_Axis = &ODrive1.Axis1;

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
            P_Axis->Encoder_Pos_Estimate=*((float*)(&Data[0]));
			P_Axis->Encoder_Vel_Estimate=*((float*)(&Data[4]));
			break;
        }case 0x017:
            P_Axis->Vbus_Voltage=*((float*)(&Data[0]));
        default:break;
	}Feed_WatchDog(&P_Axis->Protect);
}

void ODrive_CAN_Transmit(Axis* _Axis,uint16_t CMD)
{
	if(_Axis == NULL) return ;
	_Axis->StdID |= (CMD & 0x1f);
	
	switch(CMD)
	{
		case 0x3:Get_Motor_Error(_Axis);break;
		case 0x4:Get_Encoder_Error(_Axis);break;
		case 0x7:Set_Axis_Requested_State(_Axis);break;
		case 0x9:Get_Encoder_Estimates(_Axis);break;
		case 0xD:Set_Input_Vel(_Axis);break;
		case 0x16:Reboot_ODrive(_Axis);break;
        case 0x17:Get_Vbus_Voltage(_Axis);break;
		default:break;
	}_Axis->StdID = _Axis->Node_ID << 5;
}

void Get_Motor_Error(Axis* _Axis)									//0x3
{
  TxMessageData_Add(_Axis->Instruction, Blank_Data, 0, 1);
  TxMessageHeader_Set(_Axis->Instruction,8,0,0,2,_Axis->StdID);
}

void Get_Encoder_Error(Axis* _Axis)								//0x4
{
  TxMessageData_Add(_Axis->Instruction, Blank_Data, 0, 1);
  TxMessageHeader_Set(_Axis->Instruction,8,0,0,2,_Axis->StdID);
}

void Set_Axis_Requested_State(Axis* _Axis)				//0x7
{
  TxMessageData_Add(_Axis->TxMessage, (uint8_t*)&_Axis->Requested_State, 0, 1);
  TxMessageHeader_Set(_Axis->TxMessage,4,0,0,0,_Axis->StdID);
}

void Get_Encoder_Estimates(Axis* _Axis)						//0x9
{
  TxMessageData_Add(_Axis->Instruction, Blank_Data, 0, 1);
  TxMessageHeader_Set(_Axis->Instruction,8,0,0,2,_Axis->StdID);
}

void Set_Input_Vel(Axis* _Axis)										//0xD
{
  TxMessageData_Add(_Axis->TxMessage, (uint8_t*)&_Axis->Input_Vel, 0, 3);
  TxMessageHeader_Set(_Axis->TxMessage, 8, 0, 0, 0, _Axis->StdID);
}

void Reboot_ODrive(Axis* _Axis)
{
  TxMessageData_Add(_Axis->Instruction, Blank_Data, 0, 1);
  TxMessageHeader_Set(_Axis->Instruction,8,0,0,2,_Axis->StdID);
}

void Get_Vbus_Voltage(Axis* _Axis)
{
  TxMessageData_Add(_Axis->Instruction, Blank_Data, 0, 1);
  TxMessageHeader_Set(_Axis->Instruction,4,0,0,2,_Axis->StdID);
}

void ODrive_Init(ODrive* _ODrive)
{
	Axis_Init(&_ODrive->Axis0,0);
	Axis_Init(&_ODrive->Axis1,1);
}

void Axis_CloseLoop_Init(Axis* _Axis)
{
	static uint8_t Axis_Init_State[4] = {0};
  
	if(_Axis->Error != 0 || _Axis->Current_State == 0) return ;
    if(_Axis->Current_State == 8){
        _Axis->Input_Vel = 0;
        ODrive_CAN_Transmit(_Axis, 0x0D);
        WorkState_Set(&_Axis->Protect, WORKING);
        return ;
    }switch(Axis_Init_State[_Axis->Node_ID]){
        case 0:
          if(_Axis->Current_State == 1){
            _Axis->Requested_State = 3;
            ODrive_CAN_Transmit(_Axis, 0x7);
            Axis_Init_State[_Axis->Node_ID]++;
          }break;
        case 1:
          if(_Axis->Current_State == 7) Axis_Init_State[_Axis->Node_ID]++;
          break;
        case 2:
          if(_Axis->Current_State == 1){
            _Axis->Requested_State=8;
            ODrive_CAN_Transmit(_Axis,0x7);
            Axis_Init_State[_Axis->Node_ID]++;
          }break;
        case 3: if(_Axis->Current_State == 8) _Axis->Requested_State = 0;
        default:break;
   }
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
	
	_Axis->TxMessage=&Can_TxMessageList[NodeID+1];
	_Axis->Instruction=&Can_TxMessageList[NodeID+5];
	_Axis->Protect.Count_Time=0;	WorkState_Set(&_Axis->Protect,INITING);
}
	
void Reboot_ALL_ODrives(uint32_t Time)
{
    static uint32_t Reboot_ODrive0Time = 0;
	static uint32_t Reboot_ODrive1Time = 0;
	static uint8_t Reboot_ODrive0Flag = 0;
	static uint8_t Reboot_ODrive1Flag = 0;
    
	if(HAL_GPIO_ReadPin(REBOOT0_GPIO_Port, REBOOT0_Pin) == GPIO_PIN_RESET){
		if(Reboot_ODrive0Time == 0) Reboot_ODrive0Time = Time;
		if(Time - Reboot_ODrive0Time > 200){
			if(Reboot_ODrive0Flag == 0){
				ODrive_CAN_Transmit(&ODrive0.Axis0, 0x16);
				Reboot_ODrive0Flag = 1;
				WorkState_Set(&ODrive0.Axis0.Protect, REBOOT);
				WorkState_Set(&ODrive0.Axis1.Protect, REBOOT);
				Axis_Init(&ODrive0.Axis0, 0);
				Axis_Init(&ODrive0.Axis1, 1);
			}if(Time - Reboot_ODrive0Time < 230) HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
			else HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
		}
	}else Reboot_ODrive0Time = Reboot_ODrive0Flag = 0, HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);;
	
	if(HAL_GPIO_ReadPin(REBOOT1_GPIO_Port,REBOOT1_Pin) == GPIO_PIN_RESET){
		if(Reboot_ODrive1Time == 0) Reboot_ODrive1Time = Time;
		if(Time - Reboot_ODrive1Time > 200){
			if(Reboot_ODrive1Flag == 0){
				ODrive_CAN_Transmit(&ODrive1.Axis0, 0x16);
				Reboot_ODrive1Flag = 1;
				WorkState_Set(&ODrive1.Axis0.Protect, REBOOT);
				WorkState_Set(&ODrive1.Axis1.Protect, REBOOT);
				Axis_Init(&ODrive1.Axis0, 2);
				Axis_Init(&ODrive1.Axis1, 3);
			}if(Time - Reboot_ODrive1Time < 230) HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
			else HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
		}
	}else Reboot_ODrive1Time = Reboot_ODrive1Flag = 0;
}

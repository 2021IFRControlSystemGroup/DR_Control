#ifndef __ODRIVE_CAN_H__
#define __ODRIVE_CAN_H__

#include "stm32f4xx_hal.h"
#include "motor_system.h"

typedef struct Axis
{
	uint32_t Error;
	volatile	uint32_t Current_State;
	uint32_t Requested_State;
	uint32_t Motor_Error;
	uint32_t Encoder_Error;
	uint32_t Node_ID;

	float Encoder_Pos_Estimate;
	float Encoder_Vel_Estimate;
	float Input_Vel;
	float Vbus_Voltage;
	
	uint8_t CMD;
	uint16_t StdID;
	
	Protect_System Protect;
	Can_TxMessageTypeDef* TxMessage;
}Axis;

typedef struct ODrive
{
	Axis Axis0;
	Axis Axis1;
}ODrive;

void Axis_Init(Axis* _Axis,uint8_t NodeID);
void Axis_CloseLoop_Init(Axis* _Axis);
void ODrive_Init(ODrive* _ODrive);


void ODrive_Transmit(Axis* _Axis,uint16_t CMD);
void ODrive_Recevice(uint16_t StdID,uint8_t* Data);
void Send_To_ODrive(Can_TxMessageTypeDef* TxMessage,uint16_t StdID,uint8_t* Data,uint8_t len,uint8_t RTR);
void Can_DataTypeSet(CAN_TxHeaderTypeDef* TxHeader);

void Get_Motor_Error(Axis* _Axis);
void Get_Encoder_Error(Axis* _Axis);
void Get_Sensorless_Error(Axis* _Axis);
void Set_Axis_Requested_State(Axis* _Axis);
void Get_Encoder_Estimates(Axis* _Axis);
void Get_Encoder_Count(Axis* _Axis);
void Get_IQ(Axis* _Axis);
void Get_Sensorless_Estimates(Axis* _Axis);
void Set_Input_Vel(Axis* _Axis);
void Reboot_ODrive(Axis* _Axis);
#endif



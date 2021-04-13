#ifndef __ODRIVE_CAN_H__
#define __ODRIVE_CAN_H__

#include "main.h"

#define RX_NODE_ID (StdID&(0x3f<<5))
#define ODRIVE1_AXIS0 ODrive1.Axis0.Node_ID
#define ODRIVE1_AXIS1 ODrive1.Axis1.Node_ID

#define HEARTBEAT_ANALYSIS 	P_Axis->Error=*((uint32_t*)Data); P_Axis->Current_State=*((uint32_t*)(Data+4));
#define MOTOR_ERROR_ANALYSIS 	P_Axis->Motor_Error=*((uint32_t*)Data);
#define ENCODER_ERROR_ANALYSIS P_Axis->Encoder_Error=*((uint32_t*)Data);
#define SENSORLES_ERROR_ANALYSIS P_Axis->Sensorless_Error=*((uint32_t*)Data);
#define ENCODER_ESTIMATES_ANALYSIS P_Axis->Encoder_Pos_Estimate=*((float*)Data);P_Axis->Encoder_Vel_Estimate=*((float*)(Data+4));
#define ENCODER_COUNT_ANALYSIS P_Axis->Encoder_Shadow_Count=*((int32_t*)Data);P_Axis->Encoder_Count_in_CPR=*((int32_t*)(Data+4));
#define GET_IQ_ANALYSIS P_Axis->Iq_Setpoint=*((float*)Data);P_Axis->Iq_Measured=*((float*)(Data+4));
#define SENSORLESS_ESTIMATES_ANALYSIS P_Axis->Sensorless_Pos_Estimate=*((float*)Data);P_Axis->Sensorless_Vel_Estimate=*((float*)(Data+4));

typedef struct Axis
{
	uint32_t Error;
	uint32_t Current_State;
	uint32_t Requested_State;
	uint32_t Motor_Error;
	uint32_t Encoder_Error;
	uint32_t Sensorless_Error;
	uint32_t Node_ID;
	int32_t Encoder_Shadow_Count;
	int32_t Encoder_Count_in_CPR;
	float Encoder_Pos_Estimate;
	float Encoder_Vel_Estimate;
	float Input_Vel;
	float Input_Pos;
	float Iq_Setpoint;
	float Iq_Measured;
	float Sensorless_Pos_Estimate;
	float Sensorless_Vel_Estimate;
	float Vbus_Voltage;
}Axis;

typedef struct ODrive
{
	Axis Axis0;
	Axis Axis1;
}ODrive;

void ODrive_Init(ODrive* _ODrive);
void ODrive_Recevice(uint16_t StdID,uint8_t* Data);
void ODrive_Send(Axis* _Axis,uint16_t CMD);
void Send_To_ODrive(CAN_HandleTypeDef *hcan,uint16_t StdID,uint8_t* Data,uint8_t len,uint8_t RTR);


void Get_Motor_Error(Axis* _Axis,uint16_t StdID,uint16_t ID);				//0x3√¸¡Ó

void Set_Axis_Node_ID(Axis* _Axis,uint16_t StdID,uint16_t ID);
void Set_Axis_Requested_State(Axis* _Axis,uint16_t StdID,uint16_t State);
void Set_Input_Vel(Axis* _Axis,uint16_t StdID);
void Set_Input_Pos(Axis* _Axis,uint16_t StdID);
void Reboot_ODrive(uint16_t StdID);
void Set_Controller_Modes(Axis* _Axis,uint16_t StdID,int32_t Mode);
#endif


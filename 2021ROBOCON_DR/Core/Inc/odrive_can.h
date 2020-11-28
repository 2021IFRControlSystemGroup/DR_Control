#ifndef __ODRIVE_CAN_H__
#define __ODRIVE_CAN_H__

#include "main.h"

#define ODRIVE1_AXIS0 0x20
#define ODRIVE1_AXIS1 0x40
#define TEST 0X61

typedef struct Axis
{
	uint32_t Error;
	uint32_t Current_State;
	uint32_t Motor_Error;
	float Vbus_Voltage;
}Axis;

typedef struct ODrive
{
	Axis Axis0;
	Axis Axis1;
	uint8_t To_Master[8];
	uint8_t From_Master[8];
}ODrive;

void ODrive_Recevice(ODrive* _ODrive,uint16_t StdID,uint8_t* Data);
void Axis_CMD(Axis* _Axis,uint16_t CMD,uint8_t* Data);
void HeartBeat_Analysis(Axis* _Axis,uint8_t* Data);
void Get_Vbus_Voltage_Analysis(Axis* _Axis,uint8_t* Data);
void ODrive_Send(CAN_HandleTypeDef *hcan,Axis* _Axis,uint16_t CMD);
void Motor_Error_Analysis(Axis* _Axis,uint8_t* Data);
#endif


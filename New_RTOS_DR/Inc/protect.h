#ifndef __PROTECT_H__
#define __PROTECT_H__

#include "main.h"

#define PROTECT_FUNCTION_ABLE ENABLE
#define WATCHDOG_TIME_MAX 500														//ߴąٷ؜ʱӤ

typedef enum WorkState										//ϵͳ״̬
{
	WORKING,																	//ֽӣ٤ط
	INITING,
	REBOOT
}WorkState;

typedef struct ProtectSystem								//ϵͳߴąٷޡٹͥ
{
  WorkState Work_State;												//ϵͳձǰ״̬
  uint16_t Error_State; 
  int16_t Count_Time;												//ߴąٷʱݤ
}ProtectSystem;

void Feed_WatchDog(ProtectSystem* Dogs);																		//ߴąٷιٷگ˽
void WorkState_Set(ProtectSystem* Dogs, WorkState State);								//ϵͳ״̬Ȑۻگ˽
void ErrorState_Set(ProtectSystem* Dogs, uint16_t State);								//ϵͳ״̬Ȑۻگ˽
uint8_t System_Check(ProtectSystem* Dogs);

#endif



#include "protect.h"

void WorkState_Set(ProtectSystem* Dogs,WorkState State)
{
  Dogs->Work_State = State;
}

void ErrorState_Set(ProtectSystem* Dogs,uint16_t Error)
{
  Dogs->Error_State = Error;
}

void Feed_WatchDog(ProtectSystem* Dogs)
{
  Dogs->Count_Time = 0;
}

uint8_t System_Check(ProtectSystem* Dogs)
{
  if(Dogs->Count_Time < WATCHDOG_TIME_MAX)
  {
		Dogs->Count_Time++;
        ErrorState_Set(Dogs, RESET);
		return SET;
  }ErrorState_Set(Dogs, SET);
  return RESET;
}



#include "led.h"

extern ROBO_BASE Robo_Base;

void Led_Task(void)
{
	if((Robo_Base.Working_State&1)==0) LED_WARNING();
	else {
		switch(Robo_Base.Working_State){
			case 1:ALL_Always();break;
			case 3:Green_Quick();break;
			case 5:Green_Always();break;
		}
	}
}

void LED_WARNING(void)
{
	static int8_t P_Error=8;
	static uint32_t P_Time=0;
	
	//HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
	if(P_Time+800<=Robo_Base.Running_Time) P_Time=Robo_Base.Running_Time,P_Error--;
	if(P_Error==-1){
		P_Error=8;
	}else if(P_Error==0){
		LED_RED_OFF;
		LED_GRE_OFF;
	}else if(Robo_Base.Error_State&(1<<P_Error)){
		if(P_Time+600<=Robo_Base.Running_Time){
			LED_RED_OFF;
			LED_GRE_OFF;
		}else{
			LED_RED_ON;
			LED_GRE_OFF;
		}
	}else{
		if(P_Time+600<=Robo_Base.Running_Time){
			LED_RED_OFF;
			LED_GRE_OFF;
		}else{
			LED_RED_OFF;
			LED_GRE_ON;
		}
	}
}

void Green_Quick(void)
{
	if(Robo_Base.Running_Time%500>400){
		LED_RED_OFF;
		LED_GRE_OFF;
	}else{
		LED_RED_OFF;
		LED_GRE_ON;
	}
}

void Green_Always(void)
{
	LED_RED_OFF;
	LED_GRE_ON;
}

void ALL_Always(void)
{
	LED_RED_ON;
	LED_GRE_ON;
}


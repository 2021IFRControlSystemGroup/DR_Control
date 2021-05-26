#include "led.h"

uint32_t LED_Time = 0;
void Led_Task(uint32_t Working_State, uint32_t Error_State, uint32_t Time)
{
    LED_Time = Time;
	if(Error_State != 0) LED_WARNING(Error_State);
	else switch(Working_State){
		case 1:ALL_Always();break;
		case 3:Green_Quick();break;
		case 5:Green_Always();break;
	}
}

void LED_WARNING(uint16_t Error_State)
{
    const static uint8_t Warning_Max = 9;
	static int8_t Warning_Num = 0;
	static uint32_t P_Time = 0;
	
	HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
	if(LED_Time >= P_Time + 800) P_Time = LED_Time, Warning_Num++;
    
    if(Warning_Num < Warning_Max){
        if((Error_State & (1 << Warning_Num)) && (LED_Time < P_Time + 600)) LED_RED_ON, LED_GRE_OFF;
        else if(LED_Time < P_Time + 600) LED_RED_OFF, LED_GRE_ON;
        else LED_RED_OFF, LED_GRE_OFF;
    }else if(Warning_Num == Warning_Max) LED_RED_OFF, LED_GRE_OFF;
    else Warning_Num = 0;
}

void Green_Quick(void)
{
	if(LED_Time%500>400){
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


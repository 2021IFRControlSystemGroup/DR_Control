#include "led.h"

void LED_WARNING(ROBO_BASE* Robo)
{
	static int8_t P_Error=7;
	static uint32_t P_Time=0;
	
	if(P_Time+800<=Robo->Running_Time) P_Time=Robo->Running_Time,P_Error--;
	if(P_Error==-2){
		P_Error=7;
	}else if(P_Error==-1){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
	}else if(Robo->Working_State&(1<<P_Error)){
		if(P_Time+600<=Robo->Running_Time){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
		}else{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
		}
	}else{
		if(P_Time+600<=Robo->Running_Time){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
		}else{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
		}
	}
}

void Green_Quick(uint32_t Running_Time)
{
	if(Running_Time%500>400){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
	}else{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
	}
}

void Green_Always(void)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
}



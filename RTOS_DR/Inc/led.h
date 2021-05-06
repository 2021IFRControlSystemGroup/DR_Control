#ifndef __LED_H__
#define __LED_H__

#include "main.h"
#include "robo_base.h"

#define LED_RED_OFF HAL_GPIO_WritePin(LED_RED_GPIO_Port,LED_RED_Pin,GPIO_PIN_RESET)
#define LED_GRE_OFF HAL_GPIO_WritePin(LED_GRE_GPIO_Port,LED_GRE_Pin,GPIO_PIN_RESET)
#define LED_RED_ON HAL_GPIO_WritePin(LED_RED_GPIO_Port,LED_RED_Pin,GPIO_PIN_SET)
#define LED_GRE_ON HAL_GPIO_WritePin(LED_GRE_GPIO_Port,LED_GRE_Pin,GPIO_PIN_SET)

void Led_Task(void);

void LED_WARNING(void);
void Green_Always(void);
void Green_Quick(void);
void ALL_Always(void);
#endif



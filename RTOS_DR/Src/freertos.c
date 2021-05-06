/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "move.h"
#include "led.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_TXMESSAGEINDEXMAX 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
Can_TxMessageTypeDef CanTxMessageList[CAN_TXMESSAGEINDEXMAX+1]={{0x200,0,0,0,8,DISABLE}};
extern ROBO_BASE Robo_Base;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId moveTaskHandle;
osThreadId canSendTaskHandle;
osThreadId errorCheckTaskHandle;
osThreadId initTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void MoveTask(void const * argument);
void CanSendTask(void const * argument);
void ErrorCheckTask(void const * argument);
void InitTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of moveTask */
  osThreadDef(moveTask, MoveTask, osPriorityNormal, 0, 128);
  moveTaskHandle = osThreadCreate(osThread(moveTask), NULL);

  /* definition and creation of canSendTask */
  osThreadDef(canSendTask, CanSendTask, osPriorityAboveNormal, 0, 128);
  canSendTaskHandle = osThreadCreate(osThread(canSendTask), NULL);

  /* definition and creation of errorCheckTask */
  osThreadDef(errorCheckTask, ErrorCheckTask, osPriorityHigh, 0, 128);
  errorCheckTaskHandle = osThreadCreate(osThread(errorCheckTask), NULL);

  /* definition and creation of initTask */
  osThreadDef(initTask, InitTask, osPriorityNormal, 0, 128);
  initTaskHandle = osThreadCreate(osThread(initTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	vTaskSuspend(moveTaskHandle);
	vTaskSuspend(errorCheckTaskHandle);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	static uint8_t State=1;
	
  /* Infinite loop */
  for(;;)
  {
		//Base_WatchDog();
		if((Robo_Base.Working_State&1)==0){
			if(State){
				vTaskSuspend(canSendTaskHandle);
				vTaskSuspend(moveTaskHandle);
				vTaskSuspend(initTaskHandle);
				State=0;
			}LED_WARNING(&Robo_Base);
		}else{
			if(State==0){
				vTaskResume(canSendTaskHandle);
				vTaskResume(moveTaskHandle);
				vTaskResume(initTaskHandle);
				State=1;
			}else{
				switch(Robo_Base.Working_State){
					case 1:break;
					case 3:Green_Quick(Robo_Base.Running_Time);break;
					case 5:Green_Always();break;
				}
			}
		}osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_MoveTask */
/**
* @brief Function implementing the moveTask thread.
* @param argument: Not used
* @retval None
*/

/* USER CODE END Header_MoveTask */
void MoveTask(void const * argument)
{
  /* USER CODE BEGIN MoveTask */
	Robo_Base.Working_State=0x5;
  /* Infinite loop */
  for(;;)
  {
		Move_Analysis();
		Can_TxMessageCal();
    osDelay(1);
  }
  /* USER CODE END MoveTask */
}

/* USER CODE BEGIN Header_CanSendTask */
/**
* @brief Function implementing the canSendTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CanSendTask */
void CanSendTask(void const * argument)
{
  /* USER CODE BEGIN CanSendTask */
	int i=0;
  /* Infinite loop */
  for(;;)
  {
		for(i=0;i<CAN_TXMESSAGEINDEXMAX;i++) if(CanTxMessageList[i].Update!=0) Can_Send(&hcan1,&CanTxMessageList[i]);
		osDelay(1);
  }
  /* USER CODE END CanSendTask */
}

/* USER CODE BEGIN Header_ErrorCheckTask */
/**
* @brief Function implementing the errorCheckTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ErrorCheckTask */
void ErrorCheckTask(void const * argument)
{
  /* USER CODE BEGIN ErrorCheckTask */
  /* Infinite loop */
  for(;;)
  {
		osDelay(1);
  }
  /* USER CODE END ErrorCheckTask */
}

/* USER CODE BEGIN Header_InitTask */
/**
* @brief Function implementing the iniTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_InitTask */
void InitTask(void const * argument)
{
  /* USER CODE BEGIN InitTask */
	Robo_Base.Working_State=0x3;
  /* Infinite loop */
  for(;;)
  {
		if(
			Robo_Base.LF._Axis->Current_State==8&&Robo_Base.LF._Axis->Error==0&&Robo_Base.LF._Pos.Info.Abs_Angle==0
//			Robo_Base.LB._Axis->Current_State==8&&Robo_Base.LB._Axis->Error==0
//			Robo_Base.RF._Axis->Current_State==8&&Robo_Base.RF._Axis->Error==0&&
//			Robo_Base.RB._Axis->Current_State==8&&Robo_Base.RB._Axis->Error==0	

		){
			vTaskResume(moveTaskHandle);
			vTaskSuspend(initTaskHandle);
		}else{
			Pos_CloseLoop_Init(&Robo_Base.LF._Pos);
			Axis_CloseLoop_Init(Robo_Base.LF._Axis);
//			Axis_CloseLoop_Init(Robo_Base.LB._Axis);
//			Axis_CloseLoop_Init(Robo_Base.RF._Axis);
//			Axis_CloseLoop_Init(Robo_Base.RB._Axis);
		}osDelay(1);
  }
  /* USER CODE END InitTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

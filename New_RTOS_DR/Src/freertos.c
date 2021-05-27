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
#include "iwdg.h"
#include "move.h"
#include "led.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId moveTaskHandle;
osThreadId canSendTaskHandle;
osThreadId initTaskHandle;
osMessageQId CAN_TxMessageQueueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void MoveTask(void const * argument);
void CanSendTask(void const * argument);
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

  /* Create the queue(s) */
  /* definition and creation of CAN_TxMessageQueue */
  osMessageQDef(CAN_TxMessageQueue, 90, uint8_t);
  CAN_TxMessageQueueHandle = osMessageCreate(osMessageQ(CAN_TxMessageQueue), NULL);

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

  /* definition and creation of initTask */
  osThreadDef(initTask, InitTask, osPriorityNormal, 0, 128);
  initTaskHandle = osThreadCreate(osThread(initTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  vTaskSuspend(moveTaskHandle);
  /* add threads, ... */
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
    static uint8_t Suspend_Flag=1;
  /* Infinite loop */
  
    for(;;)
    {
        HAL_IWDG_Refresh(&hiwdg);
        //Base_WatchDog();
        Led_Task(Robo_Base.Working_State, Robo_Base.Error_State, Robo_Base.Running_Time);
        if((Robo_Base.Error_State != 0) && Suspend_Flag){
			vTaskSuspend(canSendTaskHandle);
			if(Robo_Base.Working_State == 2) vTaskSuspend(moveTaskHandle);
			else if(Robo_Base.Working_State == 1) vTaskSuspend(initTaskHandle);
			Suspend_Flag = RESET;
		}if((Robo_Base.Error_State == 0) && !Suspend_Flag){
			vTaskResume(canSendTaskHandle);
			if(Robo_Base.Working_State == 2) vTaskResume(moveTaskHandle);
			else if(Robo_Base.Working_State == 1) vTaskResume(initTaskHandle);
			Suspend_Flag = SET;
		}
        osDelay(1);
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
  Robo_Base.Working_State = 2;
  /* Infinite loop */
  for(;;)
  {
    //Move_Analysis();
	Can_TxMessage_MoveMode();
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
/* USER CODE END Header_CanSendTask */    uint8_t num = 0 ;    uint8_t i = 0;
void CanSendTask(void const * argument)
{
  /* USER CODE BEGIN CanSendTask */


  /* Infinite loop */
    for(;;)
    {
        for(i = 0;i<CANTXMESSAGELISTMAX;i++){
            //if(Can_TxMessageList[i].Update == SET) xQueueSend(CAN_TxMessageQueueHandle, &i, 1);
            if(Can_TxMessageList[i].Update == SET) Can_Send(&hcan1,&Can_TxMessageList[i], 10);
        }osDelay(1);
//        i = 0;
//        if(xQueueIsQueueEmptyFromISR(CAN_TxMessageQueueHandle) != pdTRUE){
//            while(xQueueReceive(CAN_TxMessageQueueHandle, &num, 10) == pdPASS && i < 3)
//                Can_Send(&hcan1,&Can_TxMessageList[num], 10),i++;
//        }
    }
  /* USER CODE END CanSendTask */
}

/* USER CODE BEGIN Header_InitTask */
/**
* @brief Function implementing the initTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_InitTask */
void InitTask(void const * argument)
{
  /* USER CODE BEGIN InitTask */
    Robo_Base.Working_State = 1;
  /* Infinite loop */
    for(;;)
    {
        if( //Robo_Base.LF._Axis->Protect.Work_State == WORKING && Robo_Base.LF._Pos.Protect.Work_State == WORKING &&
            Robo_Base.LB._Axis->Protect.Work_State == WORKING //&& Robo_Base.LB._Pos.Protect.Work_State == WORKING &&
            //Robo_Base.RF._Axis->Protect.Work_State == WORKING && Robo_Base.RF._Pos.Protect.Work_State == WORKING &&
            //Robo_Base.RB._Axis->Protect.Work_State == WORKING && Robo_Base.RB._Pos.Protect.Work_State == WORKING
		){
            vTaskResume(moveTaskHandle); vTaskSuspend(initTaskHandle);
        }else {
			//Pos_CloseLoop_Init(&Robo_Base.LF._Pos); Axis_CloseLoop_Init(Robo_Base.LF._Axis);
			Pos_CloseLoop_Init(&Robo_Base.LB._Pos); Axis_CloseLoop_Init(Robo_Base.LB._Axis);
			//Pos_CloseLoop_Init(&Robo_Base.RF._Pos); Axis_CloseLoop_Init(Robo_Base.RF._Axis);
			//Pos_CloseLoop_Init(&Robo_Base.RB._Pos); Axis_CloseLoop_Init(Robo_Base.RB._Axis);
        }osDelay(1);
    }
  /* USER CODE END InitTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

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
#define CLOSE_MOVE vTaskSuspend(moveTaskHandle)
#define START_MOVE vTaskResume(moveTaskHandle)
#define CLOSE_INIT vTaskSuspend(initTaskHandle)
#define START_INIT vTaskResume(initTaskHandle)
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
void Control_Task(void);
void Stop_Move(void);
void Task_Swtich(void);
void Remote_Control(void);
void Bucket_Turning(void);
void Arrow_PickUp(void);
void Arrow_HandOver(void);

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
  CLOSE_MOVE;
  CLOSE_INIT;
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
        Control_Task();
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

  /* Infinite loop */
  for(;;)
  {
    Move_Analysis();
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
/* USER CODE END Header_CanSendTask */
void CanSendTask(void const * argument)
{
  /* USER CODE BEGIN CanSendTask */
    int i = 0;
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
  /* Infinite loop */
    for(;;)
    {
        if(
            //Robo_Base.LF._Axis->Protect.Work_State == WORKING &&
        Robo_Base.LF._Pos.Protect.Work_State == WORKING &&
            //Robo_Base.LB._Axis->Protect.Work_State == WORKING &&
        Robo_Base.LB._Pos.Protect.Work_State == WORKING &&
            //Robo_Base.RF._Axis->Protect.Work_State == WORKING &&
        Robo_Base.RF._Pos.Protect.Work_State == WORKING &&
            //Robo_Base.RB._Axis->Protect.Work_State == WORKING &&
        Robo_Base.RB._Pos.Protect.Work_State == WORKING
		){
            START_MOVE; CLOSE_INIT;
            Robo_Base.Working_State = 2;
        }else {
			Pos_CloseLoop_Init(&Robo_Base.LF._Pos); Axis_CloseLoop_Init(Robo_Base.LF._Axis);
			Pos_CloseLoop_Init(&Robo_Base.LB._Pos); Axis_CloseLoop_Init(Robo_Base.LB._Axis);
			Pos_CloseLoop_Init(&Robo_Base.RF._Pos); Axis_CloseLoop_Init(Robo_Base.RF._Axis);
			Pos_CloseLoop_Init(&Robo_Base.RB._Pos); Axis_CloseLoop_Init(Robo_Base.RB._Axis);
        }osDelay(1);
    }
  /* USER CODE END InitTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void Task_Swtich(void)
{
    TxMessageHeader_Set(&Can_TxMessageList[10],8,0,0,0,0x10);
    TxMessageData_Add(&Can_TxMessageList[10],(uint8_t*)&Robo_Base.Working_State,0,1);
//    switch(RC_Ctl.rc.switch_left){
//        case 0:Robo_Base.Working_State = 1;break;
//        case 1:Robo_Base.Working_State = 1;break;
//        case 2:Robo_Base.Working_State = 1;break;
//        case 3:Robo_Base.Working_State = 1;break;
//    }
}

void Control_Task(void)
{
//    if(RC_Ctl.State_Update == SET) Task_Swtich(),RC_Ctl.State_Update = RESET;
//    
    switch(Robo_Base.Working_State){
        case 0:Stop_Move();break;
        case 1:START_INIT;break;
        case 2:Remote_Control();break;
        case 3:Bucket_Turning();break;
        case 4:Arrow_PickUp();break;
        case 5:Arrow_HandOver();break;
    }
}

void Stop_Move(void)
{
    Robo_Base.LF._Axis->Input_Vel = 0;
    Robo_Base.LB._Axis->Input_Vel = 0;
    Robo_Base.RF._Axis->Input_Vel = 0;
    Robo_Base.RB._Axis->Input_Vel = 0;
}
void Remote_Control(void)
{
    Robo_Base.Speed_X=(RC_Ctl.rc.ch0-1024)*1.0/660;
	Robo_Base.Speed_Y=(RC_Ctl.rc.ch1-1024)*1.0/660;
	if(sqrt((RC_Ctl.rc.ch0 - 1024) * (RC_Ctl.rc.ch0 - 1024) + (RC_Ctl.rc.ch1 - 1024) * (RC_Ctl.rc.ch1 - 1024) > 10))
    Robo_Base.Angle = atan2(Robo_Base.Speed_X, Robo_Base.Speed_Y);
    //Robo_Base.Speed_Rotate = (RC_Ctl.rc.ch0-1024)*1.0/660;
}

void Bucket_Turning(void)
{
    static uint8_t Bucket_Turning_State = 0;
    switch(Bucket_Turning_State){
        case 0:{
            if(Task_End_Flag == 0xA/*是否到位*/) Bucket_Turning_State = 1,Task_End_Flag = 0;
            break;
        }case 1:{
            if(Task_End_Flag == 0xB/*是否抓取结束*/) Bucket_Turning_State = 2,Task_End_Flag = 0;
            break;
        }case 2:{
            if(Task_End_Flag == 0xC/*是否到位*/) Bucket_Turning_State = 3,Task_End_Flag = 0;
            break;
        }case 3:{
            Bucket_Turning_State = 0;
            break;
        }
    }
}

void Arrow_PickUp(void)
{
    static uint8_t Arrow_PickUp_State = 0;
    switch(Arrow_PickUp_State){
        case 0:{
            if(1/*是否到位*/) Arrow_PickUp_State = 1;
            break;
        }case 1:{
            if(1/*是否抓取结束*/) Arrow_PickUp_State = 2;
            break;
        }case 2:{
            Arrow_PickUp_State = 0;
            Robo_Base.Working_State = 0;
            break;
        }
    }
}

void Arrow_HandOver(void)
{
    static uint8_t Arrow_HandOver_State = 0;
    switch(Arrow_HandOver_State){
        case 0:{
            if(1/*是否到位*/) Arrow_HandOver_State = 1;
            break;
        }case 1:{
            if(1/*是否交接结束*/) Arrow_HandOver_State = 2;
            break;
        }case 2:{
            if(1/*是否分离*/) Arrow_HandOver_State = 3;
            break;
        }case 3:{
            Arrow_HandOver_State = 0;
            Robo_Base.Working_State = 0;
            break;
        }
    }
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

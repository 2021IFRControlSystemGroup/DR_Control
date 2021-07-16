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
#define X_OFFSET 40
#define Y_OFFSET 40
#define Z_OFFSET 40
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
//uint8_t Queue_List[16]={0};
//uint16_t PQueue = 0;
//    uint8_t num = 0;
//uint8_t Num_List[CANTXMESSAGELISTMAX] = {0, 1, 2, 3, 4, 5, 6};
uint8_t Flag_12_34 = 0;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId moveTaskHandle;
osThreadId canSendTaskHandle;
osThreadId initTaskHandle;
osThreadId usartTaskHandle;
osMessageQId CAN_TxMessageQueueHandle;
osSemaphoreId Usart_SemHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void Remote_Control(void);
void Stop_Move(void);
void Bucket_Turning(void);
void Arrow_PickUp(void);
void Arrow_HandOver(void);
void Control_Task(void);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void MoveTask(void const * argument);
void CanSendTask(void const * argument);
void InitTask(void const * argument);
void Usart_Task(void const * argument);

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

  /* Create the semaphores(s) */
  /* definition and creation of Usart_Sem */
  osSemaphoreDef(Usart_Sem);
  Usart_SemHandle = osSemaphoreCreate(osSemaphore(Usart_Sem), 1);

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

  /* definition and creation of usartTask */
  osThreadDef(usartTask, Usart_Task, osPriorityAboveNormal, 0, 128);
  usartTaskHandle = osThreadCreate(osThread(usartTask), NULL);

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

  /* Infinite loop */
    for(;;)
    {
        Move_Analysis(Robo_Base.Speed_X, Robo_Base.Speed_Y, Robo_Base.Speed_Rotate);
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
//    int i = 0;

  /* Infinite loop */
    for(;;)
    {
        HAL_IWDG_Refresh(&hiwdg);
        Can_Send(&hcan1,&Can_TxMessageList[0], 0);
        if(Flag_12_34 == 0){
            Can_Send(&hcan1,&Can_TxMessageList[1], 0);
            Can_Send(&hcan1,&Can_TxMessageList[2], 0);
            Flag_12_34 = 1;
        }else{
            Can_Send(&hcan1,&Can_TxMessageList[3], 0);
            Can_Send(&hcan1,&Can_TxMessageList[4], 0);
            Flag_12_34 = 0;
        }osDelay(1);
//        for(i = 0;i<CANTXMESSAGELISTMAX;i++){
//            if(Can_TxMessageList[i].Update == SET) xQueueSendToBack(CAN_TxMessageQueueHandle, &Num_List[i], 0);
//        }
//        i = 0;
//        if(xQueueIsQueueEmptyFromISR(CAN_TxMessageQueueHandle) != pdTRUE){
//            while(i < 3 && xQueueReceive(CAN_TxMessageQueueHandle, &num, 0) == pdPASS){
//                Can_Send(&hcan1,&Can_TxMessageList[num], 0);
//                i++;
//                Queue_List[PQueue] = num;
//                PQueue++;
//                PQueue%=15;
//            }
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
        HAL_IWDG_Refresh(&hiwdg);
        if(
            Robo_Base.LF._Axis->Protect.Work_State == WORKING &&
            Robo_Base.LB._Axis->Protect.Work_State == WORKING &&
            Robo_Base.RF._Axis->Protect.Work_State == WORKING &&
            Robo_Base.RB._Axis->Protect.Work_State == WORKING &&
            Robo_Base.LF._Pos.Protect.Work_State == WORKING &&
            Robo_Base.LB._Pos.Protect.Work_State == WORKING &&
            Robo_Base.RF._Pos.Protect.Work_State == WORKING &&
            Robo_Base.RB._Pos.Protect.Work_State == WORKING  
		){
            START_MOVE; CLOSE_INIT;
            Robo_Base.Working_State = 2;
        }else {
            Axis_CloseLoop_Init(Robo_Base.LF._Axis);
            Axis_CloseLoop_Init(Robo_Base.LB._Axis);
            Axis_CloseLoop_Init(Robo_Base.RF._Axis);
            Axis_CloseLoop_Init(Robo_Base.RB._Axis);
			Pos_CloseLoop_Init(&Robo_Base.LF._Pos); 
			Pos_CloseLoop_Init(&Robo_Base.LB._Pos); 
			Pos_CloseLoop_Init(&Robo_Base.RF._Pos); 
			Pos_CloseLoop_Init(&Robo_Base.RB._Pos); 
        }osDelay(1);
    }
  /* USER CODE END InitTask */
}

/* USER CODE BEGIN Header_Usart_Task */
/**
* @brief Function implementing the usartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Usart_Task */
void Usart_Task(void const * argument)
{
  /* USER CODE BEGIN Usart_Task */
    osEvent Event;
  /* Infinite loop */
  for(;;)
  {
    Event = osSignalWait(0x03, osWaitForever);
    if(Event.status == osEventSignal){
        if(Event.value.signals & 0x01){
            if(Uart1_Rx.Frame_Length == Uart1_Rx.Length_Max){
                if(Uart1_Rx.Buffer_Num == 1){
                    RemoteData_analysis(Uart1_Rx.Buffer[1]);
                    Uart1_Rx.Buffer_Num = 0;
                }else{
                    RemoteData_analysis(Uart1_Rx.Buffer[0]);
                    Uart1_Rx.Buffer_Num = 1;
                }
            }
        }if(Event.value.signals & 0x02){
            if(Uart3_Rx.Frame_Length == Uart3_Rx.Length_Max){
                if(Uart3_Rx.Buffer_Num == 1){
                    IMU_analysis(Uart3_Rx.Buffer[1]);
                    Uart3_Rx.Buffer_Num = 0;
                }else{
                    IMU_analysis(Uart3_Rx.Buffer[0]);
                    Uart3_Rx.Buffer_Num = 1;
                }
            }
        }
    }

                
  }
  /* USER CODE END Usart_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void Task_Swtich(void)
{
//    TxMessageHeader_Set(&Can_TxMessageList[10],8,0,0,0,0x10);
//    TxMessageData_Add(&Can_TxMessageList[10],(uint8_t*)&Robo_Base.Working_State,0,1);
    switch(RC_Ctl.rc.switch_left){
        case 1:Robo_Base.Working_State = 0;break;
        case 2:Robo_Base.Working_State = 1;break;
        case 3:break;
    }
}

void Control_Task(void)
{
    if(RC_Ctl.State_Update == SET) Task_Swtich(), RC_Ctl.State_Update = RESET;
    
    switch(Robo_Base.Working_State){
        case 0:Stop_Move();break;
        case 1:START_INIT;break;
        case 2:Remote_Control();break;
        //case 3:MiniPC_Control();break;
        case 4:Bucket_Turning();break;
        case 5:Arrow_PickUp();break;
        case 6:Arrow_HandOver();break;
    }
}

void Stop_Move(void)
{
    Robo_Base.LF._Pos.Tar_Pos = 0;
    Robo_Base.LB._Pos.Tar_Pos = 0;
    Robo_Base.RF._Pos.Tar_Pos = 0;
    Robo_Base.RB._Pos.Tar_Pos = 0;
    
    Robo_Base.LF._Axis->Input_Vel = 0;
    Robo_Base.LB._Axis->Input_Vel = 0;
    Robo_Base.RF._Axis->Input_Vel = 0;
    Robo_Base.RB._Axis->Input_Vel = 0;
}

void Remote_Control(void)
{
         if(RC_Ctl.rc.ch0 >= 1024 + X_OFFSET) Robo_Base.Speed_X = -(RC_Ctl.rc.ch0 - 1024 - X_OFFSET) * 1.0 / (660 - X_OFFSET);
    else if(RC_Ctl.rc.ch0 <= 1024 - X_OFFSET) Robo_Base.Speed_X = -(RC_Ctl.rc.ch0 - 1024 + X_OFFSET) * 1.0 / (660 - X_OFFSET);
    else Robo_Base.Speed_X = 0;
         if(RC_Ctl.rc.ch1 >= 1024 + Y_OFFSET) Robo_Base.Speed_Y = (RC_Ctl.rc.ch1 - 1024 - Y_OFFSET) * 1.0 / (660 - Y_OFFSET);
    else if(RC_Ctl.rc.ch1 <= 1024 - Y_OFFSET) Robo_Base.Speed_Y = (RC_Ctl.rc.ch1 - 1024 + Y_OFFSET) * 1.0 / (660 - Y_OFFSET);
    else Robo_Base.Speed_Y = 0;
        if(RC_Ctl.rc.ch2 >= 1024 + Z_OFFSET) Robo_Base.Speed_Rotate = -(RC_Ctl.rc.ch2 - 1024 - Z_OFFSET) * 2.0 / (660 - Z_OFFSET);
    else if(RC_Ctl.rc.ch2 <= 1024 - Z_OFFSET) Robo_Base.Speed_Rotate = -(RC_Ctl.rc.ch2 - 1024 + Z_OFFSET) * 2.0 / (660 - Z_OFFSET);
	else Robo_Base.Speed_Rotate = 0;
    
    if(Robo_Base.Speed_X != 0 || Robo_Base.Speed_Y != 0) Robo_Base.Angle = atan2(Robo_Base.Speed_X, Robo_Base.Speed_Y);
}

//void MiniPC_Control(void)
//{
//         if(RC_Ctl.rc.ch0 >= 1024 + X_OFFSET) Robo_Base.Speed_X = -(RC_Ctl.rc.ch0 - 1024 - X_OFFSET) * 1.0 / (660 - X_OFFSET);
//    else if(RC_Ctl.rc.ch0 <= 1024 - X_OFFSET) Robo_Base.Speed_X = -(RC_Ctl.rc.ch0 - 1024 + X_OFFSET) * 1.0 / (660 - X_OFFSET);
//    else Robo_Base.Speed_X = 0;
//         if(RC_Ctl.rc.ch1 >= 1024 + Y_OFFSET) Robo_Base.Speed_Y = (RC_Ctl.rc.ch1 - 1024 - Y_OFFSET) * 1.0 / (660 - Y_OFFSET);
//    else if(RC_Ctl.rc.ch1 <= 1024 - Y_OFFSET) Robo_Base.Speed_Y = (RC_Ctl.rc.ch1 - 1024 + Y_OFFSET) * 1.0 / (660 - Y_OFFSET);
//    else Robo_Base.Speed_Y = 0;
//        if(RC_Ctl.rc.ch2 >= 1024 + Z_OFFSET) Robo_Base.Speed_Rotate = -(RC_Ctl.rc.ch2 - 1024 - Z_OFFSET) * 2.0 / (660 - Z_OFFSET);
//    else if(RC_Ctl.rc.ch2 <= 1024 - Z_OFFSET) Robo_Base.Speed_Rotate = -(RC_Ctl.rc.ch2 - 1024 + Z_OFFSET) * 2.0 / (660 - Z_OFFSET);
//	else Robo_Base.Speed_Rotate = 0;
//    
//    if(Robo_Base.Speed_X != 0 || Robo_Base.Speed_Y != 0) Robo_Base.Angle = atan2(Robo_Base.Speed_X, Robo_Base.Speed_Y);
//}
    
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

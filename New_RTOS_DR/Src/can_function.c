#include "can_function.h"
#include "robo_base.h"

CanTxMessageTypeDef Can_TxMessageList[CANTXMESSAGELISTMAX]={{0x200,0,0,0,8,DISABLE}};

void CAN_Start_IT(CAN_HandleTypeDef *hcan)
{
	CAN_FilterTypeDef CAN_Filter;
	extern CAN_HandleTypeDef hcan1;
	
	if(hcan==&hcan1)
		CAN_Filter.FilterBank=0,CAN_Filter.SlaveStartFilterBank=14;
	else 
	CAN_Filter.FilterBank=14,CAN_Filter.SlaveStartFilterBank=14;
	
	CAN_Filter.FilterActivation=CAN_FILTER_ENABLE;
	CAN_Filter.FilterIdHigh=0;
	CAN_Filter.FilterIdLow=0;
	CAN_Filter.FilterMaskIdHigh=0;
	CAN_Filter.FilterMaskIdLow=0;
	CAN_Filter.FilterFIFOAssignment=CAN_FILTER_FIFO0;
	CAN_Filter.FilterMode=CAN_FILTERMODE_IDMASK;
	CAN_Filter.FilterScale=CAN_FILTERSCALE_32BIT;
	
	HAL_CAN_ConfigFilter(hcan,&CAN_Filter);
	
	HAL_CAN_Start(hcan);
	HAL_CAN_ActivateNotification(hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
}

void TxMessageData_Add(CanTxMessageTypeDef* TxMessage, uint8_t* Data, uint8_t Start_byte, uint8_t End_Byte)
{
  if(Start_byte>6||End_Byte>7||End_Byte<=Start_byte) return ;
  
  int i;
  for(i = Start_byte;i <= End_Byte;i++){
    TxMessage->Data[i] = *(Data + i);
  }TxMessage->Update = SET;
}

void TxMessageHeader_Set(CanTxMessageTypeDef* TxMessage, uint8_t DLC, uint8_t ExtId, uint8_t IDE, uint8_t RTR, uint8_t StdId)
{
  TxMessage->Header.DLC=DLC;
  TxMessage->Header.ExtId=ExtId;
  TxMessage->Header.IDE=IDE;
  TxMessage->Header.RTR=RTR;
  TxMessage->Header.StdId=StdId;
  TxMessage->Header.TransmitGlobalTime=DISABLE;
  TxMessage->Update = SET;
}
void Can_Send(CAN_HandleTypeDef *hcan,CanTxMessageTypeDef* TxMessage, uint32_t Limit_Time)
{
	uint32_t TxMailbox;
    uint32_t Time = Robo_Base.Running_Time;
	while(HAL_CAN_GetTxMailboxesFreeLevel(hcan) == RESET) if(Robo_Base.Running_Time - Time >= Limit_Time) break;
    if (HAL_CAN_AddTxMessage(hcan, &TxMessage->Header, TxMessage->Data, &TxMailbox) != HAL_OK) CAN_Error_Handler(hcan);
	TxMessage->Update = RESET;
}
uint16_t Error_Times = 0;
void CAN_Error_Handler(CAN_HandleTypeDef *hcan)
{
    if(hcan->ErrorCode != HAL_CAN_ERROR_NONE){
        HAL_CAN_AbortTxRequest(hcan,CAN_TX_MAILBOX0|CAN_TX_MAILBOX1|CAN_TX_MAILBOX2);
        HAL_CAN_ResetError(hcan);
        HAL_CAN_Stop(hcan);
        CAN_Start_IT(hcan);
        Error_Times++;
    }
}

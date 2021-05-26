#ifndef __CAN_FUNCTION_H__
#define __CAN_FUNCTION_H__

#include "main.h"
#include "can.h"

#define CAN_FUNCTION_ABLE ENABLE
#define CanTXMESSAGELISTMAX 9
typedef struct CanTxMessageTypeDef
{
	CAN_TxHeaderTypeDef Header;
	uint8_t Data[8];
	uint8_t Update;
}CanTxMessageTypeDef;

void Can_Send(CAN_HandleTypeDef *hcan,CanTxMessageTypeDef* TxMessage);
void TxMessageHeader_Set(CanTxMessageTypeDef* TxMessage, uint8_t DLC, uint8_t ExtId, uint8_t IDE, uint8_t RTR, uint8_t StdId);
void TxMessageData_Add(CanTxMessageTypeDef* TxMessage, uint64_t Data, uint8_t Start_byte, uint8_t End_Byte);
void CAN_Start_IT(CAN_HandleTypeDef *hcan);

extern CanTxMessageTypeDef CanTxMessageList[CanTXMESSAGELISTMAX];
#endif


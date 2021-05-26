#ifndef __UART_ANALYSIS_H__
#define __UART_ANALYSIS_H__

//---------ͷτݾѼڬҿؖ----------//
#include "main.h"
#include "usart.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
//---------------------------------//

//---------#defineҿؖ-------------//
#define BUFFER_LEN_MAX 100
#define USART2_RX_LEN_MAX 20
#define USART2_TX_LEN_MAX 50

#define USART1_RX_LEN_MAX 18
#define USART3_RX_LEN_MAX 10
#define USART6_RX_LEN_MAX 10
//---------------------------------//

//-------Ԯࠚͨхޡٹͥҿؖ--------//
typedef struct UsartRxBuffer
{
	uint8_t* Buffer[2];
	uint8_t Buffer_Num;
	uint16_t Length_Max;
}UsartRxBuffer;
//---------------------------------//

//-------------گ˽ʹķ------------//
void Usart_All_Init(void);						//ԮࠚͨхԵʼۯگ˽ 
void usart_sendData_DMA(UART_HandleTypeDef *huart, uint8_t *Data, uint8_t len);			//ԮࠚDMAע̍گ˽ 
void Usart_DMA_Process(UART_HandleTypeDef *huart,DMA_HandleTypeDef* hdma_usart_rx, 
					UsartRxBuffer* Uart_Rx,void(*DataProcessFunc)(uint8_t *pData));	//ԮࠚDMAޓ˕Ԧmگ˽ 
//---------------------------------//
#endif


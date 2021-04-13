#ifndef __UART_COMMUNICATE_H__
#define __UART_COMMUNICATE_H__

//---------ͷ�ļ���������----------//
#include "main.h"
#include "usart.h"
#include "analysis.h"
//---------------------------------//

//---------#define����-------------//
#define BUFFER_LEN_MAX 100
#define USART2_RX_LEN_MAX 20
#define USART2_TX_LEN_MAX 50

#define USART1_RX_LEN_MAX 18
#define USART3_RX_LEN_MAX 10
#define USART6_RX_LEN_MAX 10
//---------------------------------//

//-------����ͨ�Žṹ�岿��--------//
typedef struct UART_RX_BUFFER
{
	uint8_t* Buffer[2];
	uint8_t Buffer_Num;
	uint16_t Length_Max;
}UART_RX_BUFFER;
//---------------------------------//

//-------------��������------------//
void Usart_All_Init(void);						//����ͨ�ų�ʼ������ 
void uart_sendData_DMA(UART_HandleTypeDef *huart, uint8_t *Data, uint8_t len);			//����DMA���ͺ��� 
void Uart_DMA_Process(UART_HandleTypeDef *huart,DMA_HandleTypeDef* hdma_usart_rx, 
					UART_RX_BUFFER* Uart_Rx,void(*DataProcessFunc)(uint8_t *pData));	//����DMA���մ����� 
//---------------------------------//
#endif


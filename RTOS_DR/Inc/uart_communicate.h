#ifndef __UART_COMMUNICATE_H__
#define __UART_COMMUNICATE_H__

//---------头文件包含部分----------//
#include "main.h"
#include "usart.h"
#include "analysis.h"
//---------------------------------//

//---------#define部分-------------//
#define BUFFER_LEN_MAX 100
#define USART2_RX_LEN_MAX 20
#define USART2_TX_LEN_MAX 50

#define USART1_RX_LEN_MAX 18
#define USART3_RX_LEN_MAX 10
#define USART6_RX_LEN_MAX 10
//---------------------------------//

//-------串口通信结构体部分--------//
typedef struct UART_RX_BUFFER
{
	uint8_t* Buffer[2];
	uint8_t Buffer_Num;
	uint16_t Length_Max;
}UART_RX_BUFFER;
//---------------------------------//

//-------------函数声明------------//
void Usart_All_Init(void);						//串口通信初始化函数 
void uart_sendData_DMA(UART_HandleTypeDef *huart, uint8_t *Data, uint8_t len);			//串口DMA发送函数 
void Uart_DMA_Process(UART_HandleTypeDef *huart,DMA_HandleTypeDef* hdma_usart_rx, 
					UART_RX_BUFFER* Uart_Rx,void(*DataProcessFunc)(uint8_t *pData));	//串口DMA接收处理函数 
//---------------------------------//
#endif


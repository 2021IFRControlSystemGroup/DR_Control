#ifndef __UART_COMMUNICATE_H__
	#define __UART_COMMUNICATE_H__

	//-------------------------------------头文件引用----------------------------------------------//
	#include "main.h"
	#include "usart.h"
	#include <stdio.h>
	#include "analysis.h"
	//---------------------------------------------------------------------------------------------//
	
	//---------------------------------------预编译------------------------------------------------//
	#ifndef UART_COMMUNICATE_USER_ROOT
		#define UART_COMMUNICATE_USER_ROOT 0
	#endif
	//---------------------------------------------------------------------------------------------//


	//------------------------------------基本宏定义-----------------------------------------------//
	#define BUFFER_LEN_MAX 100
	
	#define USART2_RX_LEN_MAX 20
	#define USART2_TX_LEN_MAX 50

	#define USART1_RX_LEN_MAX 18
	//---------------------------------------------------------------------------------------------//
	
	//-----------------------------------串口通信结构体--------------------------------------------//
	typedef struct UART_RX_BUFFER
	{
		UART_HandleTypeDef *huart;
		DMA_HandleTypeDef* hdma_usart_rx;
		void(*DataProcessFunc)(uint8_t *pData);
		uint8_t Buffer[2][BUFFER_LEN_MAX];
		uint8_t Buffer_Num;
		uint16_t Length_Max;
	}UART_RX_BUFFER;
	//---------------------------------------------------------------------------------------------//
	
	//--------------------------------------基本功能-----------------------------------------------//
	#if UART_COMMUNICATE_USER_ROOT
		weak void Usart_ALL_Init(void);
		weak void Usart_Init(UART_RX_BUFFER* Uart_Rx,UART_HandleTypeDef *huart,DMA_HandleTypeDef* hdma_usart_rx,
							uint16_t Length_Max,void(*DataProcessFunc)(uint8_t *pData));																		//串口通信初始化函数 
		weak void uart_sendData_DMA(UART_HandleTypeDef *huart, uint8_t *Data, uint8_t len);												//串口DMA发送函数 
		weak void Uart_DMA_Process(UART_RX_BUFFER* Uart_Rx);																											//串口DMA接收处理函数
	#else
		void Usart_ALL_Init(void);
		void Usart_Init(UART_RX_BUFFER* Uart_Rx,UART_HandleTypeDef *huart,DMA_HandleTypeDef* hdma_usart_rx,
							uint16_t Length_Max,void(*DataProcessFunc)(uint8_t *pData));																		//串口通信初始化函数
		void uart_sendData_DMA(UART_HandleTypeDef *huart, uint8_t *Data, uint8_t len);														//串口DMA发送函数
		void Uart_DMA_Process(UART_RX_BUFFER* Uart_Rx);																														//串口DMA接收处理函数
	#endif
	//---------------------------------------------------------------------------------------------//
		
	//---------------------------------------对外接口----------------------------------------------//
	extern UART_RX_BUFFER Uart2_Rx;
	extern uint8_t Uart2_Tx[USART2_TX_LEN_MAX];

	extern UART_RX_BUFFER Uart1_Rx;
	//---------------------------------------------------------------------------------------------//

	//---------------------------------------用户自定义功能区---------------------------------------//
	//E.G.
	//		预编译内容
	//					......
	//					......
	//
	//		结构体声明
	//					......
	//					......
	//
	//		宏定义
	//					......
	//					......
	//
	//		宏定义函数
	//					......
	//					......
	//
	//		函数声明
	//					......
	//					......
	//
	//----------------------------------------------------------------------------------------------//
#endif


#ifndef __UART_COMMUNICATE_H__
	#define __UART_COMMUNICATE_H__

	//-------------------------------------ͷ�ļ�����----------------------------------------------//
	#include "main.h"
	#include "usart.h"
	#include <stdio.h>
	#include "analysis.h"
	//---------------------------------------------------------------------------------------------//
	
	//---------------------------------------Ԥ����------------------------------------------------//
	#ifndef UART_COMMUNICATE_USER_ROOT
		#define UART_COMMUNICATE_USER_ROOT 0
	#endif
	//---------------------------------------------------------------------------------------------//


	//------------------------------------�����궨��-----------------------------------------------//
	#define BUFFER_LEN_MAX 100
	
	#define USART2_RX_LEN_MAX 20
	#define USART2_TX_LEN_MAX 50

	#define USART1_RX_LEN_MAX 18
	//---------------------------------------------------------------------------------------------//
	
	//-----------------------------------����ͨ�Žṹ��--------------------------------------------//
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
	
	//--------------------------------------��������-----------------------------------------------//
	#if UART_COMMUNICATE_USER_ROOT
		weak void Usart_ALL_Init(void);
		weak void Usart_Init(UART_RX_BUFFER* Uart_Rx,UART_HandleTypeDef *huart,DMA_HandleTypeDef* hdma_usart_rx,
							uint16_t Length_Max,void(*DataProcessFunc)(uint8_t *pData));																		//����ͨ�ų�ʼ������ 
		weak void uart_sendData_DMA(UART_HandleTypeDef *huart, uint8_t *Data, uint8_t len);												//����DMA���ͺ��� 
		weak void Uart_DMA_Process(UART_RX_BUFFER* Uart_Rx);																											//����DMA���մ�����
	#else
		void Usart_ALL_Init(void);
		void Usart_Init(UART_RX_BUFFER* Uart_Rx,UART_HandleTypeDef *huart,DMA_HandleTypeDef* hdma_usart_rx,
							uint16_t Length_Max,void(*DataProcessFunc)(uint8_t *pData));																		//����ͨ�ų�ʼ������
		void uart_sendData_DMA(UART_HandleTypeDef *huart, uint8_t *Data, uint8_t len);														//����DMA���ͺ���
		void Uart_DMA_Process(UART_RX_BUFFER* Uart_Rx);																														//����DMA���մ�����
	#endif
	//---------------------------------------------------------------------------------------------//
		
	//---------------------------------------����ӿ�----------------------------------------------//
	extern UART_RX_BUFFER Uart2_Rx;
	extern uint8_t Uart2_Tx[USART2_TX_LEN_MAX];

	extern UART_RX_BUFFER Uart1_Rx;
	//---------------------------------------------------------------------------------------------//

	//---------------------------------------�û��Զ��幦����---------------------------------------//
	//E.G.
	//		Ԥ��������
	//					......
	//					......
	//
	//		�ṹ������
	//					......
	//					......
	//
	//		�궨��
	//					......
	//					......
	//
	//		�궨�庯��
	//					......
	//					......
	//
	//		��������
	//					......
	//					......
	//
	//----------------------------------------------------------------------------------------------//
#endif


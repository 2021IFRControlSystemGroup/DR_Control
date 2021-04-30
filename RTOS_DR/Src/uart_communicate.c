//文件名称:		uart_communicate.c
//对应头文件:	uart_communicate.h
//主要功能:
//		实现通过DMA传输的串口通信接收和发送功能, 其中接收采用DMA双缓冲方式.
//
//时间:
//		2020/11/27
//
//版本:	1.0V
//
//状态: 已测试
//
//测试内容:
//		使用大疆的遥控器, 配合405的芯片成读取到遥控器信息

//---------头文件引用部分---------//
#include "uart_communicate.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
//--------------------------------//

//---------变量声明部分-----------//
UART_RX_BUFFER Uart2_Rx;
uint8_t Uart2_Tx[USART2_TX_LEN_MAX]={0};

UART_RX_BUFFER Uart1_Rx;
UART_RX_BUFFER Uart3_Rx;
UART_RX_BUFFER Uart6_Rx;
//--------------------------------//

//---------外部变量声明部分-------//
//--------------------------------//

//--------------------------------------------------------------------------------------------------//
//函数名称:
//		printf打印函数
//
//函数功能:
//		通过选定的串口, 把printf中的参数数据传输出去.
//
//参数类型:
//		和正常printf一样.
//
//移植建议:
//		需要用到哪个串口, 就修改HAL_UART_Transmit(&huartx, (uint8_t *)&ch, 1, 0xFFFF);中的huartx到对应串口就行.
//
//--------------------------------------------------------------------------------------------------//
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* ?????????????? */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

//--------------------------------------------------------------------------------------------------//
//函数名称:
//		printf
//
//函数功能:
//		通过选定的串口, 把printf中的参数数据传输出去.
//
//参数类型:
//		和正常printf一样.
//
//移植建议:
//		需要用到哪个串口, 就修改HAL_UART_Transmit(&huartx, (uint8_t *)&ch, 1, 0xFFFF);中的huartx到对应串口就行.
//
//--------------------------------------------------------------------------------------------------//
void Usart_All_Init(void)
{
	Uart6_Rx.Buffer[0]=(uint8_t*)malloc(sizeof(uint8_t)*USART6_RX_LEN_MAX);
	Uart6_Rx.Buffer[1]=(uint8_t*)malloc(sizeof(uint8_t)*USART6_RX_LEN_MAX);
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
	Uart6_Rx.Buffer_Num = 0;
	Uart6_Rx.Length_Max=USART6_RX_LEN_MAX;
	HAL_UART_Receive_DMA(&huart6, Uart6_Rx.Buffer[0], USART6_RX_LEN_MAX);
	
	Uart3_Rx.Buffer[0]=(uint8_t*)malloc(sizeof(uint8_t)*USART3_RX_LEN_MAX);
	Uart3_Rx.Buffer[1]=(uint8_t*)malloc(sizeof(uint8_t)*USART3_RX_LEN_MAX);
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
	Uart3_Rx.Buffer_Num = 0;
	Uart3_Rx.Length_Max=USART3_RX_LEN_MAX;
	HAL_UART_Receive_DMA(&huart3, Uart3_Rx.Buffer[0], USART3_RX_LEN_MAX);
	
	Uart2_Rx.Buffer[0]=(uint8_t*)malloc(sizeof(uint8_t)*USART2_RX_LEN_MAX);
	Uart2_Rx.Buffer[1]=(uint8_t*)malloc(sizeof(uint8_t)*USART2_RX_LEN_MAX);
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	Uart2_Rx.Buffer_Num = 0;
	Uart2_Rx.Length_Max=USART2_RX_LEN_MAX;
	HAL_UART_Receive_DMA(&huart2, Uart2_Rx.Buffer[0], USART2_RX_LEN_MAX);
	
	Uart1_Rx.Buffer[0]=(uint8_t*)malloc(sizeof(uint8_t)*USART1_RX_LEN_MAX);
	Uart1_Rx.Buffer[1]=(uint8_t*)malloc(sizeof(uint8_t)*USART1_RX_LEN_MAX);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	Uart1_Rx.Buffer_Num = 0;
	Uart1_Rx.Length_Max=USART1_RX_LEN_MAX;
	HAL_UART_Receive_DMA(&huart1, Uart1_Rx.Buffer[0], USART1_RX_LEN_MAX);
}

//--------------------------------------------------------------------------------------------------//
//函数名称:
//		串口DMA数据发送函数
//
//函数功能:
//		通过按照规定的串口DMA发送端口, 把固定长度的数据发送出去, 
//
//参数类型:
//		UART_HandleTypeDef 串口句柄
//		uint8_t* 发送数组
//		uint8_t 数组长度
//
//移植建议:
//		需要用到哪个串口, 就添加一个判断,然后把数组数据通过memcpy复制到对应的缓冲区中.
//
//--------------------------------------------------------------------------------------------------//
void uart_sendData_DMA(UART_HandleTypeDef *huart, uint8_t *Data, uint8_t len)
{
	if(huart->Instance == USART2)
	{
		memcpy(Uart2_Tx, Data, len);
		HAL_UART_Transmit_DMA(&huart2, Uart2_Tx, len);
	}
}

//--------------------------------------------------------------------------------------------------//
//函数名称:
//		串口DMA接收处理函数
//
//函数功能:
//		通过DMA双缓存, 把串口接收端数据读入并且调用函数进行处理
//
//参数类型:
//		UART_HandleTypeDef* 串口句柄
//		DMA_HandleTypeDef* DMA句柄
//		UART_RX_BUFFER* 接收数组结构体
//		void(*)(uint8_t *pData) 数据处理函数指针
//
//--------------------------------------------------------------------------------------------------//
void Uart_DMA_Process(UART_HandleTypeDef *huart,DMA_HandleTypeDef* hdma_usart_rx,UART_RX_BUFFER* Uart_Rx,void(*DataProcessFunc)(uint8_t *pData))
{
	uint8_t this_frame_len = 0;
	
	if((__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) != RESET))  
		{   
			__HAL_DMA_DISABLE(hdma_usart_rx);
			__HAL_UART_CLEAR_IDLEFLAG(huart);  
			
			this_frame_len = Uart_Rx->Length_Max - __HAL_DMA_GET_COUNTER(hdma_usart_rx);
			if(Uart_Rx->Buffer_Num)
			{
				Uart_Rx->Buffer_Num = 0;
				HAL_UART_Receive_DMA(huart, Uart_Rx->Buffer[0], Uart_Rx->Length_Max);
				if(this_frame_len == Uart_Rx->Length_Max)
					if(DataProcessFunc) DataProcessFunc(Uart_Rx->Buffer[1]);
			}
			else
			{
				Uart_Rx->Buffer_Num = 1;
				HAL_UART_Receive_DMA(huart, Uart_Rx->Buffer[1], Uart_Rx->Length_Max);
				if(this_frame_len == Uart_Rx->Length_Max)
					if(DataProcessFunc) DataProcessFunc(Uart_Rx->Buffer[0]);
			}
		}
}



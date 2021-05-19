//文件名称:		uart_communicate.c
//对应头文件:	uart_communicate.h
//主要功能:
//		实现通过DMA传输的串口通信接收和发送功能, 其中接收采用DMA双缓冲方式.
//
//时间:
//		2021/5/8
//
//版本:	2.0V
//

//---------头文件引用部分---------//
#include "uart_communicate.h"
//--------------------------------//

//---------变量声明部分-----------//
UART_RX_BUFFER Uart2_Rx;
uint8_t Uart2_Tx[USART2_TX_LEN_MAX]={0};

UART_RX_BUFFER Uart1_Rx;
//--------------------------------//

//-------------------------------------printf移植----------------------------------------------//
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
//---------------------------------------------------------------------------------------------//

#if UART_COMMUNICATE_USER_ROOT
	weak
#endif
void Usart_ALL_Init(void)
{
	extern DMA_HandleTypeDef hdma_usart1_rx;
	extern DMA_HandleTypeDef hdma_usart2_rx;
	
	Usart_Init(&Uart2_Rx,&huart2,&hdma_usart2_rx,USART2_RX_LEN_MAX,NULL);
	Usart_Init(&Uart1_Rx,&huart1,&hdma_usart1_rx,USART1_RX_LEN_MAX,RemoteData_analysis);
}

//--------------------------------------------------------------------------------------------------//
//函数名称:
//		串口接收结构体参数初始化
//
//函数功能:
//		初始化串口接收结构体
//
//参数类型:
//		UART_RX_BUFFER* Uart_Rx										指向UART_RX_BUFFER的指针
//		UART_HandleTypeDef *huart									指向uart的指针
//		DMA_HandleTypeDef* hdma_usart_rx					指向dma_rx的指针
//		uint16_t Length_Max												接收数组的最大值
//		void(*DataProcessFunc)(uint8_t *pData)		指向处理函数的指针
//
//--------------------------------------------------------------------------------------------------//
#if UART_COMMUNICATE_USER_ROOT
	weak
#endif
void Usart_Init(UART_RX_BUFFER* Uart_Rx,UART_HandleTypeDef *huart,DMA_HandleTypeDef* hdma_usart_rx,uint16_t Length_Max,void(*DataProcessFunc)(uint8_t *pData))
{
	Uart_Rx->huart=huart;
	Uart_Rx->hdma_usart_rx=hdma_usart_rx;
	Uart_Rx->DataProcessFunc=DataProcessFunc;
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	Uart_Rx->Buffer_Num = 0;
	Uart_Rx->Length_Max=Length_Max;
	HAL_UART_Receive_DMA(huart, Uart_Rx->Buffer[0], Length_Max);
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
//--------------------------------------------------------------------------------------------------//
#if UART_COMMUNICATE_USER_ROOT
	weak
#endif
void uart_sendData_DMA(UART_HandleTypeDef *huart, uint8_t *Data, uint8_t len)
{
	if(huart->Instance == USART2)
	{
		HAL_UART_Transmit_DMA(&huart2, Data, len);
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
//		UART_RX_BUFFER* 接收数组结构体
//
//--------------------------------------------------------------------------------------------------//
#if UART_COMMUNICATE_USER_ROOT
	weak
#endif
void Uart_DMA_Process(UART_RX_BUFFER* Uart_Rx)
{
	uint8_t this_frame_len = 0;
	
	if((__HAL_UART_GET_FLAG(Uart_Rx->huart,UART_FLAG_IDLE) != RESET))  
		{   
			__HAL_DMA_DISABLE(Uart_Rx->hdma_usart_rx);
			__HAL_UART_CLEAR_IDLEFLAG(Uart_Rx->huart);  
			
			this_frame_len = Uart_Rx->Length_Max - __HAL_DMA_GET_COUNTER(Uart_Rx->hdma_usart_rx);
			if(Uart_Rx->Buffer_Num)
			{
				Uart_Rx->Buffer_Num = 0;
				HAL_UART_Receive_DMA(Uart_Rx->huart, Uart_Rx->Buffer[0], Uart_Rx->Length_Max);
				if(this_frame_len == Uart_Rx->Length_Max)
					if(Uart_Rx->DataProcessFunc) Uart_Rx->DataProcessFunc(Uart_Rx->Buffer[1]);
			}
			else
			{
				Uart_Rx->Buffer_Num = 1;
				HAL_UART_Receive_DMA(Uart_Rx->huart, Uart_Rx->Buffer[1], Uart_Rx->Length_Max);
				if(this_frame_len == Uart_Rx->Length_Max)
					if(Uart_Rx->DataProcessFunc) Uart_Rx->DataProcessFunc(Uart_Rx->Buffer[0]);
			}
		}
}



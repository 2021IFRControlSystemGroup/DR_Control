//�ļ�����:		uart_communicate.c
//��Ӧͷ�ļ�:	uart_communicate.h
//��Ҫ����:
//		ʵ��ͨ��DMA����Ĵ���ͨ�Ž��պͷ��͹���, ���н��ղ���DMA˫���巽ʽ.
//
//ʱ��:
//		2021/5/8
//
//�汾:	2.0V
//

//---------ͷ�ļ����ò���---------//
#include "uart_communicate.h"
//--------------------------------//

//---------������������-----------//
UART_RX_BUFFER Uart2_Rx;
uint8_t Uart2_Tx[USART2_TX_LEN_MAX]={0};

UART_RX_BUFFER Uart1_Rx;
//--------------------------------//

//-------------------------------------printf��ֲ----------------------------------------------//
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
//��������:
//		���ڽ��սṹ�������ʼ��
//
//��������:
//		��ʼ�����ڽ��սṹ��
//
//��������:
//		UART_RX_BUFFER* Uart_Rx										ָ��UART_RX_BUFFER��ָ��
//		UART_HandleTypeDef *huart									ָ��uart��ָ��
//		DMA_HandleTypeDef* hdma_usart_rx					ָ��dma_rx��ָ��
//		uint16_t Length_Max												������������ֵ
//		void(*DataProcessFunc)(uint8_t *pData)		ָ��������ָ��
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
//��������:
//		����DMA���ݷ��ͺ���
//
//��������:
//		ͨ�����չ涨�Ĵ���DMA���Ͷ˿�, �ѹ̶����ȵ����ݷ��ͳ�ȥ, 
//
//��������:
//		UART_HandleTypeDef ���ھ��
//		uint8_t* ��������
//		uint8_t ���鳤��
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
//��������:
//		����DMA���մ�����
//
//��������:
//		ͨ��DMA˫����, �Ѵ��ڽ��ն����ݶ��벢�ҵ��ú������д���
//
//��������:
//		UART_RX_BUFFER* ��������ṹ��
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



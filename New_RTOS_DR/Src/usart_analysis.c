//τݾĻԆ:		uart_analysis.c
//הӦͷτݾ:	uart_analysis.h
//׷Ҫ٦Ŝ:
//		ʵЖͨڽDMAԫˤքԮࠚͨхޓ˕ۍע̍٦Ŝ, Ǥאޓ˕ӉԃDMA˫ۺԥ׽ʽ.
//
//ʱݤ:
//		2020/11/27
//
//ѦѾ:	1.0V
//
//״̬: ӑӢ˔
//
//Ӣ˔Śɝ:
//		ʹԃճݮքң࠘Ƿ, Ƥۏ405քоƬԉׁȡսң࠘ǷхϢ

//---------ͷτݾӽԃҿؖ---------//
#include "usart_analysis.h"
//--------------------------------//

//---------Ҥʹķҿؖ-----------//
uint8_t Task_End_Flag;
uint8_t Uart2_Tx[USART2_TX_LEN_MAX]={0};

UsartRxBuffer Uart2_Rx;
UsartRxBuffer Uart1_Rx;
UsartRxBuffer Uart3_Rx;
UsartRxBuffer Uart6_Rx;
IMU_Info IMU;

VISION_DATA Vision_Data = VISION_DATA_DEFAULT;
RC_Ctl_t RC_Ctl = RC_DATA_DEFAULT;
//--------------------------------//

//---------΢ҿҤʹķҿؖ-------//
//--------------------------------//

//--------------------------------------------------------------------------------------------------//
//گ˽ĻԆ:
//		printfղӡگ˽
//
//گ˽٦Ŝ:
//		ͨڽѡ֨քԮࠚ, ёprintfאքӎ˽˽ߝԫˤԶȥ.
//
//ӎ˽`э:
//		ۍֽӣprintfһҹ.
//
//ӆֲݨө:
//		ѨҪԃսńٶԮࠚ, ߍўلHAL_UART_Transmit(&huartx, (uint8_t *)&ch, 1, 0xFFFF);אքhuartxսהӦԮࠚߍѐ.
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

void Usart_All_Init(void)
{
	Uart6_Rx.Buffer[0] = (uint8_t*)malloc(sizeof(uint8_t) * USART6_RX_LEN_MAX);
	Uart6_Rx.Buffer[1] = (uint8_t*)malloc(sizeof(uint8_t) * USART6_RX_LEN_MAX);
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
	Uart6_Rx.Buffer_Num = 0;
	Uart6_Rx.Length_Max = USART6_RX_LEN_MAX;
	HAL_UART_Receive_DMA(&huart6, Uart6_Rx.Buffer[0], USART6_RX_LEN_MAX);
	
	Uart2_Rx.Buffer[0] = (uint8_t*)malloc(sizeof(uint8_t) * USART2_RX_LEN_MAX);
	Uart2_Rx.Buffer[1] = (uint8_t*)malloc(sizeof(uint8_t) * USART2_RX_LEN_MAX);
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	Uart2_Rx.Buffer_Num = 0;
	Uart2_Rx.Length_Max = USART2_RX_LEN_MAX;
	HAL_UART_Receive_DMA(&huart2, Uart2_Rx.Buffer[0], USART2_RX_LEN_MAX);
	
    Uart3_Rx.Buffer[0] = (uint8_t*)malloc(sizeof(uint8_t) * USART3_RX_LEN_MAX);
	Uart3_Rx.Buffer[1] = (uint8_t*)malloc(sizeof(uint8_t) * USART3_RX_LEN_MAX);
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
	Uart3_Rx.Buffer_Num = 0;
	Uart3_Rx.Length_Max = USART3_RX_LEN_MAX;
	HAL_UART_Receive_DMA(&huart3, Uart3_Rx.Buffer[0], USART3_RX_LEN_MAX);
	
	Uart1_Rx.Buffer[0] = (uint8_t*)malloc(sizeof(uint8_t) * USART1_RX_LEN_MAX);
	Uart1_Rx.Buffer[1] = (uint8_t*)malloc(sizeof(uint8_t) * USART1_RX_LEN_MAX);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	Uart1_Rx.Buffer_Num = 0;
	Uart1_Rx.Length_Max = USART1_RX_LEN_MAX;
	HAL_UART_Receive_DMA(&huart1, Uart1_Rx.Buffer[0], USART1_RX_LEN_MAX);
}

void usart_sendData_DMA(UART_HandleTypeDef *huart, uint8_t *Data, uint8_t len)
{
	if(huart->Instance == USART2)
	{
		memcpy(Uart2_Tx, Data, len);
		HAL_UART_Transmit_DMA(&huart2, Uart2_Tx, len);
	}
}

void Usart_DMA_Process(UART_HandleTypeDef *huart,DMA_HandleTypeDef* hdma_usart_rx,UsartRxBuffer* Uart_Rx,void(*DataProcessFunc)(uint8_t *pData))
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

void IMU_Analysis(uint8_t *pData)
{
	if(pData[6] == 0xD0){
        IMU.Row = (int16_t)(((pData[8] << 8) | pData[7]) & 0xffff) / 100.0;
        IMU.Pitch = (int16_t)(((pData[10] << 8) | pData[9]) & 0xffff) / 100.0;
        IMU.Yaw = (int16_t)(((pData[12] << 8) | pData[11]) & 0xffff) / 10.0;
    };
}

void RemoteData_analysis(uint8_t *sbus_rx_buffer)
{
	if(sbus_rx_buffer == 0)
	{
			return;
	}

	RC_Ctl.rc.ch2 = (sbus_rx_buffer[0]| (sbus_rx_buffer[1] << 8)) & 0x07ff; //!< Channel 0
	RC_Ctl.rc.ch3 = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff; //!< Channel 1
	RC_Ctl.rc.ch0 = ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) | (sbus_rx_buffer[4] << 10)) & 0x07ff;//!< Channel 2	
	RC_Ctl.rc.ch1 = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff; //!< Channel 3
	
    if(RC_Ctl.rc.switch_left != (((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2)) RC_Ctl.State_Update = SET;
    if(RC_Ctl.rc.switch_right != ((sbus_rx_buffer[5] >> 4)& 0x0003)) RC_Ctl.State_Update = SET;
	RC_Ctl.rc.switch_left = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2; //!< Switch left
	RC_Ctl.rc.switch_right = ((sbus_rx_buffer[5] >> 4)& 0x0003); //!< Switch right
	RC_Ctl.mouse.x = sbus_rx_buffer[6] | (sbus_rx_buffer[7] << 8); //!< Mouse X axis
	RC_Ctl.mouse.y = sbus_rx_buffer[8] | (sbus_rx_buffer[9] << 8); //!< Mouse Y axis
	RC_Ctl.mouse.z = sbus_rx_buffer[10] | (sbus_rx_buffer[11] << 8); //!< Mouse Z axis
	RC_Ctl.mouse.press_l = sbus_rx_buffer[12]; //!< Mouse Left Is Press ?
	RC_Ctl.mouse.press_r = sbus_rx_buffer[13]; //!< Mouse Right Is Press ?
	RC_Ctl.key.v_l = sbus_rx_buffer[14]; //!< KeyBoard value
	RC_Ctl.key.v_h = sbus_rx_buffer[15];

}

void Control_CAN_Recevice(uint32_t StdID, uint8_t *pData)
{
    if(StdID == 0x20 || StdID == 0x21 || StdID == 0x22 || StdID == 0x30) Task_End_Flag = pData[0];
}


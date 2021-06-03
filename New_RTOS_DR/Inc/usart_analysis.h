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


#define DMA_VISION_RX_BUF_LENGTH    100u //???
#define VISION_DATA_DEFAULT  {0,0,0,0,0,0,0,0,0,0,0,0};
#define RC_DATA_DEFAULT {\
	{1024,1024,1024,1024,3,3},\
	{0},\
	{0},0, \
}

typedef  struct
{
	uint8_t armor_sign;	//???????
	uint8_t buff_sign;   
	uint8_t armor_type;	//????
	uint16_t armor_dis_or_buff_cy;	//????
	float tar_x;	//x??
	float tar_y;	//y??
	int16_t Velocity_x_or_buff_cx; //???????
	float angel_x_v;	//??????????	//???0.05???
	float angle_x_v_filter;
	uint8_t control_state;
	uint8_t Num;
	uint8_t runtime;
	
}VISION_DATA;

typedef struct
{
	struct
	{
		uint16_t ch0;
		uint16_t ch1;
		uint16_t ch2;
		uint16_t ch3;
		uint8_t switch_left;
		uint8_t switch_right;
	}rc;
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_r;
	}mouse;
	struct
	{
		uint8_t v_l;
		uint8_t v_h;
	}key;
    
    uint8_t State_Update;
}RC_Ctl_t;

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
void VisionData_analysis(uint8_t *pData);
void RemoteData_analysis(uint8_t *sbus_rx_buffer);

extern UsartRxBuffer Uart2_Rx;
extern UsartRxBuffer Uart1_Rx;
extern UsartRxBuffer Uart3_Rx;
extern UsartRxBuffer Uart6_Rx;

extern VISION_DATA Vision_Data;
extern RC_Ctl_t RC_Ctl;
//---------------------------------//
#endif


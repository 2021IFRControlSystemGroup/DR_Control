#ifndef __ANALYSIS_H__
#define __ANALYSIS_H__

#include "main.h"


#define DMA_VISION_RX_BUF_LENGTH    100u //???
#define VISION_DATA_DEFAULT  {0,0,0,0,0,0,0,0,0,0,0,0};
#define RC_DATA_DEFAULT {\
	{1024,1024,1024,1024,3,3},\
	{0},\
	{0},\
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
}RC_Ctl_t;

void VisionData_analysis(uint8_t *pData);
void RemoteData_analysis(uint8_t *sbus_rx_buffer);
#endif



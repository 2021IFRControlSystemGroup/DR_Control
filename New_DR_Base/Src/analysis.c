#include "analysis.h"

uint8_t Vision_frame_rx_len = 0;
uint32_t cs_time = 0;
uint32_t last_time = 0;
VISION_DATA Vision_Data = VISION_DATA_DEFAULT;
uint32_t time_tick_1ms=1;


RC_Ctl_t RC_Ctl=RC_DATA_DEFAULT;
//KeyBoardTypeDef KeyBoardData[KEY_NUMS]={0};

void VisionData_analysis(uint8_t *pData)
{
	int i = 0;
//	u16 CRC16 = 0;
	int tar_x_raw = 0;
	int tar_y_raw = 0;	
	
	if(pData == 0) return;
  
	while(i<Vision_frame_rx_len)
	{
		while(pData[i] != 0x5a)
		{
			i++;
			if(i>=Vision_frame_rx_len) break;
		}

//		CRC16 = (pData[i+8]<<8)|pData[i+9];
//		if(Get_CRC16_Check_Sum(&pData[i],8) == CRC16)
//		{
			//DeviceFpsFeed(LOST_VISION);
			cs_time = time_tick_1ms - last_time;
			last_time = time_tick_1ms;									//4.29		
		    Vision_Data.buff_sign = pData[i+1]>>4;
//			Vision_Data.armor_dis = ((pData[i+2]<<8)|pData[i+3]) & 0xffff;
//			tar_x_raw = ((pData[i+4]<<8)|pData[i+5]) & 0xffff;
//			tar_y_raw = ((pData[i+6]<<8)|pData[i+7]) & 0xffff;			//5.4
			Vision_Data.armor_sign = pData[i+1]>>7;
			Vision_Data.armor_dis_or_buff_cy = ((pData[i+2]<<8)|pData[i+3]) & 0xffff;
			tar_x_raw = ((pData[i+4]<<8)|pData[i+5]) & 0xffff;
			tar_y_raw = ((pData[i+6]<<8)|pData[i+7]) & 0xffff;	
			Vision_Data.Velocity_x_or_buff_cx = ((pData[i+8]<<8)|pData[i+9]) & 0xffff;
			Vision_Data.tar_x = tar_x_raw/10.0f;
			Vision_Data.tar_y = tar_y_raw/10.0f;	
//			Vision_Data.armor_dis_or_buff_cy -= 30.0f; 
			Vision_Data.runtime = pData[i+10];
			Vision_Data.Num++;
			if(Vision_Data.Num > 200) Vision_Data.Num = 0;
			i = i + 11;
			/*??????*/
			//LostCountFeed(&(Error_Check.count[LOST_VISION]));
//		}
//		else  i = i + 10;
	}
}


extern int8_t Chassis_Control_Heading;
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
	
	RC_Ctl.rc.switch_left = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2; //!< Switch left
	RC_Ctl.rc.switch_right = ((sbus_rx_buffer[5] >> 4)& 0x0003); //!< Switch right
	RC_Ctl.mouse.x = sbus_rx_buffer[6] | (sbus_rx_buffer[7] << 8); //!< Mouse X axis
	RC_Ctl.mouse.y = sbus_rx_buffer[8] | (sbus_rx_buffer[9] << 8); //!< Mouse Y axis
	RC_Ctl.mouse.z = sbus_rx_buffer[10] | (sbus_rx_buffer[11] << 8); //!< Mouse Z axis
	RC_Ctl.mouse.press_l = sbus_rx_buffer[12]; //!< Mouse Left Is Press ?
	RC_Ctl.mouse.press_r = sbus_rx_buffer[13]; //!< Mouse Right Is Press ?
	RC_Ctl.key.v_l = sbus_rx_buffer[14]; //!< KeyBoard value
	RC_Ctl.key.v_h = sbus_rx_buffer[15];
	

	//Key_Analysis();
}


//void Key_Analysis(void)
//{
//	
//	

//	KeyBoardData[KEY_W].value=RC_Ctl.key.v_l&0x01;
//	KeyBoardData[KEY_S].value=(RC_Ctl.key.v_l&0x02)>>1;
//	KeyBoardData[KEY_A].value=(RC_Ctl.key.v_l&0x04)>>2;
//	KeyBoardData[KEY_D].value=(RC_Ctl.key.v_l&0x08)>>3;

//	KeyBoardData[KEY_SHIFT].value=(RC_Ctl.key.v_l&0x10)>>4;
//	KeyBoardData[KEY_CTRL].value=(RC_Ctl.key.v_l&0x20)>>5;
//	KeyBoardData[KEY_Q].value=(RC_Ctl.key.v_l&0x40)>>6;
//	KeyBoardData[KEY_E].value=(RC_Ctl.key.v_l&0x80)>>7;

//	KeyBoardData[KEY_R].value=RC_Ctl.key.v_h&0x01;
//	KeyBoardData[KEY_F].value=(RC_Ctl.key.v_h&0x02)>>1;
//	KeyBoardData[KEY_G].value=(RC_Ctl.key.v_h&0x04)>>2;
//	KeyBoardData[KEY_Z].value=(RC_Ctl.key.v_h&0x08)>>3;
//	KeyBoardData[KEY_X].value=(RC_Ctl.key.v_h&0x10)>>4;
//	KeyBoardData[KEY_C].value=(RC_Ctl.key.v_h&0x20)>>5;
//	KeyBoardData[KEY_V].value=(RC_Ctl.key.v_h&0x40)>>6;
//	KeyBoardData[KEY_B].value=(RC_Ctl.key.v_h&0x80)>>7;
//	
////	for(int keyid=0;keyid<KEY_NUMS;keyid++)	//??????
////	{
////		ButtonStatu_Verdict(&KeyBoardData[keyid]);
////	}
//}

//u8 ButtonStatu_Verdict(KeyBoardTypeDef * Key)	
//{																		
//	if(Key->last==1)
//	{
//		Key->count++;
//	}
//	else
//	{
//		Key->count=0;
//	}
//	
//	if(Key->count>10)	//????? 10ms
//	{
//		if(Key->count<500)	//1s
//		{
//			if(Key->last==1&&Key->value==0)
//			{
//				Key->statu=1;
//			}
//		}
//		else
//		{			
//			if(Key->last==1&&Key->value==0)
//			{
//				Key->statu=2;
//			}
//		}
//	}
//	else
//	{
//		Key->statu=0;
//	}
//	Key->last=Key->value;
//	
//	return Key->statu;
//}


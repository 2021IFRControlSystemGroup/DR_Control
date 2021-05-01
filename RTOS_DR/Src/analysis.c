#include "analysis.h"
#include "robo_base.h"

extern ROBO_BASE Robo_Base;
VISION_DATA Vision_Data = VISION_DATA_DEFAULT;
RC_Ctl_t RC_Ctl=RC_DATA_DEFAULT;

void VisionData_analysis(uint8_t *pData)
{
	
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

void MiniPCData_Analysis(uint8_t *pData)
{
	if(pData==NULL) return ;
	
	Robo_Base.Speed_X=(pData[0]|(pData[1]<<8))&0x07ff;
	Robo_Base.Speed_Y=(pData[2]|(pData[3]<<8))&0x07ff;
	
	
}



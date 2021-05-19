#include "analysis.h"
#include "robo_base.h"
#include <stdlib.h>

#define CHASSIS_INTEGRAL_PID_KI 0.01f
#define CHASSIS_INTEGRAL_PID_I_SUM_LIM 			1000
#define CHASSIS_INTEGRAL_PID_KP 						2.5f    //拓展性PID用于走直线

VISION_DATA Vision_Data = VISION_DATA_DEFAULT;
RC_Ctl_t RC_Ctl=RC_DATA_DEFAULT;

void VisionData_analysis(uint8_t *pData)
{
	;
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
	
	Robo_Base.Speed_X=(RC_Ctl.rc.ch0-1024)*1.0/660*2000;
	Robo_Base.Speed_Y=(RC_Ctl.rc.ch1-1024)*1.0/660*2000;
	if(Robo_Base.Speed_X!=0||Robo_Base.Speed_Y!=0) Robo_Base.Angle=atan2(Robo_Base.Speed_X,Robo_Base.Speed_Y);
	Move_Analysis();
}

void MiniPCData_Analysis(uint8_t *pData)
{
	if(!pData||pData[0]!=0xa5) return ;
	Robo_Base.Speed_X=(pData[0]|(pData[1]<<8))&0x07ff;
	Robo_Base.Speed_Y=(pData[2]|(pData[3]<<8))&0x07ff;
}

void Move_Analysis(void)
{
	Robo_Base.LF.Tar_Speed=Robo_Base.Speed_X+Robo_Base.Speed_Y;
	Robo_Base.RF.Tar_Speed=Robo_Base.Speed_X-Robo_Base.Speed_Y;
	
	Robo_Base.LB.Tar_Speed=-Robo_Base.Speed_X+Robo_Base.Speed_Y;
	Robo_Base.RB.Tar_Speed=-Robo_Base.Speed_X-Robo_Base.Speed_Y;
	
	if(RC_Ctl.rc.ch2-1024!=0)
	{
		Robo_Base.LF.Tar_Speed+=(RC_Ctl.rc.ch2-1024)*1.0/660*2000;
		Robo_Base.RF.Tar_Speed+=(RC_Ctl.rc.ch2-1024)*1.0/660*2000;
	
		Robo_Base.LB.Tar_Speed+=(RC_Ctl.rc.ch2-1024)*1.0/660*2000;
		Robo_Base.RB.Tar_Speed+=(RC_Ctl.rc.ch2-1024)*1.0/660*2000;
	}
	if(RC_Ctl.rc.ch3-1024!=0)
	{
		Robo_Base.LF.Tar_Speed+=(RC_Ctl.rc.ch3-1024)*1.0/660*2000;
		Robo_Base.RF.Tar_Speed+=(RC_Ctl.rc.ch3-1024)*1.0/660*2000;
	
		Robo_Base.LB.Tar_Speed+=(RC_Ctl.rc.ch3-1024)*1.0/660*2000;
		Robo_Base.RB.Tar_Speed+=(RC_Ctl.rc.ch3-1024)*1.0/660*2000;
	}
}


void Extended_Integral_PID(void)
{
	float tarv_sum=abs(Robo_Base.LF.Tar_Speed)+abs(Robo_Base.LB.Tar_Speed)+abs(Robo_Base.RF.Tar_Speed)+abs(Robo_Base.RB.Tar_Speed);
	float fdbv_sum=abs(Robo_Base.LF.Info.Speed)+abs(Robo_Base.LB.Info.Speed)+abs(Robo_Base.RF.Info.Speed)+abs(Robo_Base.RB.Info.Speed);
	float expect[4]={0};
	float error[4]={0};
	static float inte[4];
	int32_t Com[4]={0};
	
	if(abs(tarv_sum)<0.1f)	//相当于被除数为0
	{
		expect[0]=0;
		expect[1]=0;
		expect[2]=0;
		expect[3]=0;
	}
	else
	{
		expect[0]=fdbv_sum*Robo_Base.LF.Tar_Speed/tarv_sum;
		expect[1]=fdbv_sum*Robo_Base.LB.Tar_Speed/tarv_sum;
		expect[2]=fdbv_sum*Robo_Base.RF.Tar_Speed/tarv_sum;
		expect[3]=fdbv_sum*Robo_Base.RB.Tar_Speed/tarv_sum;
	}
	
	error[0]=expect[0]-Robo_Base.LF.Info.Speed;
	error[1]=expect[1]-Robo_Base.LB.Info.Speed;
	error[2]=expect[2]-Robo_Base.RF.Info.Speed;
	error[3]=expect[3]-Robo_Base.RB.Info.Speed;
	
	
	inte[0]+=error[0]*CHASSIS_INTEGRAL_PID_KI;
	inte[1]+=error[1]*CHASSIS_INTEGRAL_PID_KI;
	inte[2]+=error[2]*CHASSIS_INTEGRAL_PID_KI;
	inte[3]+=error[3]*CHASSIS_INTEGRAL_PID_KI;
	
	for(int id=0;id<4;id++)
	{
		inte[id]=inte[id]>CHASSIS_INTEGRAL_PID_I_SUM_LIM?CHASSIS_INTEGRAL_PID_I_SUM_LIM:inte[id];
		inte[id]=inte[id]<-CHASSIS_INTEGRAL_PID_I_SUM_LIM?-CHASSIS_INTEGRAL_PID_I_SUM_LIM:inte[id];
	}
	
	Com[0]=(uint32_t)(error[0]*CHASSIS_INTEGRAL_PID_KP+inte[0]);
	Com[1]=(uint32_t)(error[1]*CHASSIS_INTEGRAL_PID_KP+inte[1]);
	Com[2]=(uint32_t)(error[2]*CHASSIS_INTEGRAL_PID_KP+inte[2]);
	Com[3]=(uint32_t)(error[3]*CHASSIS_INTEGRAL_PID_KP+inte[3]);
	
	for(int id=0;id<4;id++)
	{
		Com[id]=Com[id]>4000?4000:Com[id];
		Com[id]=Com[id]<-4000?-4000:Com[id];
	}
	
	Robo_Base.LF.Speed_PID.output+=Com[0];
	Robo_Base.LB.Speed_PID.output+=Com[1];
	Robo_Base.RF.Speed_PID.output+=Com[2];
	Robo_Base.RB.Speed_PID.output+=Com[3];
}


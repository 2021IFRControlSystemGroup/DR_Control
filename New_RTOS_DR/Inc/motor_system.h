#ifndef __MOTOR_SYSTEM_H__
#define __MOTOR_SYSTEM_H__

//---------头文件包含部分----------//
#include "main.h"
#include "math.h"
#include "can_function.h"
#include "protect.h"
//---------------------------------//

//---------#define部分-------------//
#define PI (2 * acos(0))																	//PI圆周率的宏定义
#define HALF_PI (PI / 2)
#define ToDegree(a) (a / PI * 180)													//弧度转化成角度的宏定义
#define ToRadian(a) (a / 180 * PI)													//角度转化成弧度的宏定义

#define ROTOR_ANGLE 8192																//转子机械角度
//#define GEAR_RATIO 19																	//电机减速比(3508)
#define GEAR_RATIO (36 * 6.5)															//电机减速比(2006)
#define ONE_CIRCLE (ROTOR_ANGLE * GEAR_RATIO)							//电机转动一圈的总机械角度

#define HALF_PI_ANGLE (GEAR_RATIO * ROTOR_ANGLE / 4)
#define PI_ANGLE (GEAR_RATIO * ROTOR_ANGLE / 2)
//---------------------------------//

//---------底盘结构体部分----------//

typedef struct MotorInfo								//进行位置环控制的电机信息
{
  int16_t Speed;														//电机速度				单位(rad/min 转/每分钟)
  uint16_t Angle;														//转子机械角度
  int32_t Abs_Angle;												//转子绝对机械角度
  int32_t Relative_Angle;										//电机相对坐标角度		单位(° 度)
	int Circle_Num;
  uint8_t Temperature;											//电机温度				单位(℃ 摄氏度)
  int16_t Electric;													//电流					单位(mA 毫安)
  uint16_t Last_Angle;											//上一次的转子绝对角度
}MotorInfo;

typedef struct PID{								//电机PID参数结构体
	
	float Kp;
	float Ki;
	float Kd;
	
	float error;															//误差
	float error_last;													//上一次误差
	float error_max;													//最大误差
	float dead_line;													//死区
	
	float intergral;													//误差积分
	float intergral_max;											//误差积分最大值
	
	float derivative;													//误差微分

	float output;															//输出
	float output_max;													//输出最大值
}PID;

typedef struct MotorSystem										//位置环系统
{
    MotorInfo Info;											//位置环电机信息
    PID Pos_PID;															//位置环PID参数
    PID Speed_PID;														//速度环PID参数
    float Tar_Pos;														//目标位置
    float Tar_Speed;
    uint8_t Motor_Num;												//电机号码
    #if PROTECT_FUNCTION_ABLE == ENABLE
        ProtectSystem Protect; 
	#endif
  
    #if CAN_FUNCTION_ABLE == ENABLE
        CanTxMessageTypeDef* TxMessage;
    #endif
}MotorSystem;


//---------------------------------//

//-------------函数声明------------//
void PID_Init(PID *pid, float Kp, float Ki, float Kd, float error_max, float dead_line, float intergral_max, float output_max);		//PID訋私缘始郫诏私
void Motor_Init(MotorSystem* P_System, uint8_t ID);

void Motor_Info_Analysis(MotorInfo* P_Motor, uint8_t* Rx_Data);											//位置环电机数据分析的操作函数

void PID_General_Cal(PID *pid, float fdbV, float tarV, uint8_t moto_num, uint8_t *Tx_msg);					//PID计算函数----为了向下兼容
void PID_Pos_Cal(MotorSystem* P_System);																					//位置环系统PID计算函数
void PID_Speed_Cal(MotorSystem* P_System);
void Motor_Add_Can_TxMessageList(MotorSystem* Pos_Motor);
//---------------------------------//
#endif


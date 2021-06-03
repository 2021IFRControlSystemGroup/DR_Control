#ifndef __MOTOR_SYSTEM_H__
#define __MOTOR_SYSTEM_H__

//---------ͷ�ļ���������----------//
#include "main.h"
#include "math.h"
#include "can_function.h"
#include "protect.h"
//---------------------------------//

//---------#define����-------------//
#define PI (2 * acos(0))																	//PIԲ���ʵĺ궨��
#define HALF_PI (PI / 2)
#define ToDegree(a) (a / PI * 180)													//����ת���ɽǶȵĺ궨��
#define ToRadian(a) (a / 180 * PI)													//�Ƕ�ת���ɻ��ȵĺ궨��

#define ROTOR_ANGLE 8192																//ת�ӻ�е�Ƕ�
//#define GEAR_RATIO 19																	//������ٱ�(3508)
#define GEAR_RATIO (36 * 6.5)															//������ٱ�(2006)
#define ONE_CIRCLE (ROTOR_ANGLE * GEAR_RATIO)							//���ת��һȦ���ܻ�е�Ƕ�

#define HALF_PI_ANGLE (GEAR_RATIO * ROTOR_ANGLE / 4)
#define PI_ANGLE (GEAR_RATIO * ROTOR_ANGLE / 2)
//---------------------------------//

//---------���̽ṹ�岿��----------//

typedef struct MotorInfo								//����λ�û����Ƶĵ����Ϣ
{
  int16_t Speed;														//����ٶ�				��λ(rad/min ת/ÿ����)
  uint16_t Angle;														//ת�ӻ�е�Ƕ�
  int32_t Abs_Angle;												//ת�Ӿ��Ի�е�Ƕ�
  int32_t Relative_Angle;										//����������Ƕ�		��λ(�� ��)
	int Circle_Num;
  uint8_t Temperature;											//����¶�				��λ(�� ���϶�)
  int16_t Electric;													//����					��λ(mA ����)
  uint16_t Last_Angle;											//��һ�ε�ת�Ӿ��ԽǶ�
}MotorInfo;

typedef struct PID{								//���PID�����ṹ��
	
	float Kp;
	float Ki;
	float Kd;
	
	float error;															//���
	float error_last;													//��һ�����
	float error_max;													//������
	float dead_line;													//����
	
	float intergral;													//������
	float intergral_max;											//���������ֵ
	
	float derivative;													//���΢��

	float output;															//���
	float output_max;													//������ֵ
}PID;

typedef struct MotorSystem										//λ�û�ϵͳ
{
    MotorInfo Info;											//λ�û������Ϣ
    PID Pos_PID;															//λ�û�PID����
    PID Speed_PID;														//�ٶȻ�PID����
    float Tar_Pos;														//Ŀ��λ��
    float Tar_Speed;
    uint8_t Motor_Num;												//�������
    #if PROTECT_FUNCTION_ABLE == ENABLE
        ProtectSystem Protect; 
	#endif
  
    #if CAN_FUNCTION_ABLE == ENABLE
        CanTxMessageTypeDef* TxMessage;
    #endif
}MotorSystem;


//---------------------------------//

//-------------��������------------//
void PID_Init(PID *pid, float Kp, float Ki, float Kd, float error_max, float dead_line, float intergral_max, float output_max);		//PIDӎ˽Եʼۯگ˽
void Motor_Init(MotorSystem* P_System, uint8_t ID);

void Motor_Info_Analysis(MotorInfo* P_Motor, uint8_t* Rx_Data);											//λ�û�������ݷ����Ĳ�������

void PID_General_Cal(PID *pid, float fdbV, float tarV, uint8_t moto_num, uint8_t *Tx_msg);					//PID���㺯��----Ϊ�����¼���
void PID_Pos_Cal(MotorSystem* P_System);																					//λ�û�ϵͳPID���㺯��
void PID_Speed_Cal(MotorSystem* P_System);
void Motor_Add_Can_TxMessageList(MotorSystem* Pos_Motor);
//---------------------------------//
#endif


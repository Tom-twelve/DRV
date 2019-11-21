/**
 ******************************************************************************
 * @file		control.h
 * @author		WrathWings
 * @version 	V1.0
 * @date		2019.1.17
 * @brief		The header file of control.c
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONTROL_H
#define __CONTROL_H

/* Private includes ----------------------------------------------------------*/
/* CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "MotorConfig.h"
#include "math.h"
#include "foc.h"
/* CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */
struct CurrLoop_t
{
	float LimitCurrQ;
	float ExptCurrD;
	float ExptCurrQ;
	float CtrlVolD;
	float CtrlVolQ;
	float ErrD;
	float ErrQ;
	float IntegralErrD;
	float IntegralErrQ;
	float Kp_D;
	float Ki_D;
	float Kp_Q;
	float Ki_Q;
};

struct SpdLoop_t
{
	float ExptMecAngularSpeed_rad;		//������е���ٶ�(rad/s), ������
	float MaxExptMecAngularSpeed_rad;	//���������е���ٶ�(rad/s), ������
	float Acceleration;		//���ٶ�(rad/s2), ������
	float Deceleration;		//���ٶ�(rad/s2), ������
	float Err;
	float IntegralErr;
	float Kp;
	float Ki;
};

struct PosLoop_t
{
	float ExptMecAngle_rad;	//Ŀ��Ƕ�, ������
	float MecAngleUpperLimit_rad;		//λ�û�λ������(rad), ������
	float MecAngleLowerLimit_rad;		//λ�û�λ������(rad), ������
	float RefMecAngle_rad;		//�ο���е�Ƕ�(rad), �ϵ�ʱ����, ������
	float Err;
	float LastErr;
	float DiffErr;
	float Kp;
	float Kd;
};

struct TorqueCtrl_t
{
	float ExptTorque;
	float EleTorque;
	float MaxMecSpd;
};

struct Regulator_t
{
	float Kp;
	float Ki;
	float Kd;
	float ActualPeriod_s;
	int16_t TargetFSYNC;
};

struct MainCtrl_t
{
	int32_t ExptMecAngularSpeed_pulse;		//Ŀ���е���ٶ�, ����
	volatile uint32_t MaxMecAngularSpeed_pulse;		//�ٶȻ�����ٶ�, ����
	uint32_t Acceleration_pulse;	//���ٶ�, ����
	uint32_t Deceleration_pulse;	//���ٶ�, ����
	int32_t ExptMecAngle_pulse;				//Ŀ��λ��, ����
	int32_t MecAngleUpperLimit_pulse;		//λ�û�λ������, ����
	int32_t MecAngleLowerLimit_pulse;		//λ�û�λ������, ����
	int32_t RefMecAngle_pulse;				//�ο���е�Ƕ�, �ϵ�ʱ����, ����
	float MaxTorque_Nm;						//���Ť��, ţ��
	uint16_t PresentMecAngle_pulse;
	uint16_t LastMecAngle_pulse;
};

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PERIOD_MULTIPLE					10	//(�ٶȻ�, λ�û����� / ����������)
#define OUTER_LOOP_PERIOD				(Regulator.ActualPeriod_s * PERIOD_MULTIPLE)	//�⻷��������

#define CURR_INTEGRAL_ERR_LIM_D 		(1.0f / CurrLoop.Ki_D)	//Id�����޷�
#define CURR_INTEGRAL_ERR_LIM_Q 		(1.0f / CurrLoop.Ki_Q)	//Iq�����޷�

#define SPD_INTEGRAL_ERR_LIM			(5.0 * 2.f * PI)		//(rad/s)

#define CURRENT_CONTROL_KP_D			(INDUCTANCE_D * 1500.f)	//d���� * ����������
#define CURRENT_CONTROL_KI_D			(PHASE_RES * 1500.f)	//����� * ����������

#define CURRENT_CONTROL_KP_Q			(INDUCTANCE_Q * 1500.f)	//q���� * ����������
#define CURRENT_CONTROL_KI_Q			(PHASE_RES * 1500.f)	//����� * ����������

#define SPEED_CONTROL_KP				1.0f
#define SPEED_CONTROL_KI				1.0f

#define POSITION_CONTROL_KP				60.0f
#define POSITION_CONTROL_KD				0.1f

#define PERIOD_REGULATOR_KP 			(0.3 * 0.00596f)
#define PERIOD_REGULATOR_KI 			(0.005952380952381 * DEFAULT_CARRIER_PERIOD_s)
#define PERIOD_REGULATOR_KD 			(0.005952380952381) // ���������������ֵ��ͨ���򵥵ļ���һ�£�����ƫ��Ĳ�ֳ�����1�ı䶯���ı�������ڴ��ԵĹ��Ƶģ�����ʹ��KD = Kd / dt)
#define PERIOD_REGULATOR_LIM			5	//�ز����ڵ������޷�ֵ	

#define MC_CTRL_RESOLUTION				4096

/*����ģʽ, ����driver.cͬ��*/
#define SPD_CURR_CTRL_MODE 				1
#define POS_SPD_CURR_CTRL_MODE 			2
#define POS_CURR_CTRL_MODE 				3
#define TORQUE_CTRL_MODE 				4
#define	SPD_VOL_CTRL_MODE 				5
#define	POS_SPD_VOL_CTRL_MODE 			6

#define WORK_MODE						1
#define MEASURE_ANGLE_TABLE_MODE		2
#define MEASURE_PARAM_MODE				3

/* USER CODE END PD */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

void DriverInit(void);
void CurrentLoopInit(void);
void SpeedLoopInit(void);
void PositionLoopInit(void);
void TorqueCtrlInit(void);
void RefAngleInit(void);
void ZeroPosSet(uint16_t posOffset);
void CurrentLoop(float exptCurrD, float exptCurrQ, float realCurrD, float realCurrQ, float *ctrlVolD, float *ctrlVolQ);
void SpeedLoop(float expectedMecAngularSpeed, float realMecAngularSpeed, float *ctrlCurrQ);
void PositionLoop(float exptMecAngle, float realMecAngle, float *ctrlAngularSpeed);
void SpdCurrController(void);
void PosSpdCurrController(void);
void PosCurrController(void);
void TorqueController(void);
float VelSlopeGenerator(float exptVelocity);
void DriverCtrlModeInit(void);
void PeriodRegulator(void);
/* USER CODE END PFP */


#endif

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

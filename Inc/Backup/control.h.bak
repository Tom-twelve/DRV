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
struct CurrentLoop_t
{
	float ExpectedCurrentD;
	float ExpectedCurrentQ;
	float ControlVoltageD;
	float ControlVoltageQ;
	float Kp_D;
	float Ki_D;
	float Kp_Q;
	float Ki_Q;
};

struct SpeedLoop_t
{
	float ExpectedMecAngularSpeed;	//degree per second
	float Acceleration;
	float Kp;
	float Ki;
};

struct PositionLoop_t
{
	float ExpectedMecAngle;	//degree
	float Kp;
	float Kd;
};

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CurrentControlLoopIntegralErrorLimit_D 		2.5f	//(A)

#define CurrentControlLoopIntegralErrorLimit_Q 		2.5f	//(A)

#define SpeedControlLoopIntegralErrorLimit 			(10.0f * 2 * PI)		//(°/s)

#define CURRENT_CONTROL_KP_D			(InductanceD * 500.f)		//d轴电感 * 电流环截止频率
#define CURRENT_CONTROL_KI_D			(PhaseResistance * 500.f)	//相电阻 * 电流环截止频率

#define CURRENT_CONTROL_KP_Q			(InductanceQ * 500.f)		//q轴电感 * 电流环截止频率
#define CURRENT_CONTROL_KI_Q			(PhaseResistance * 500.f)	//相电阻 * 电流环截止频率

#define SPEED_CONTROL_KP	
#define SPEED_CONTROL_KI

#define POSITION_CONTROL_KP	
#define POSITION_CONTROL_KD

#define	VoltageControlMode 			0
#define CurrentControlMode 			1
#define SpeedControlMode 			2
#define PositionControlMode 		3

#define WorkMode					1
#define TestMode 					2

/* USER CODE END PD */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

void MotorEnable(void);
void CurrentControlLoop(float expectedCurrentD, float expectedCurrentQ, float realityCurrentD, float realityCurrentQ, float *controlVoltageD, float *controlVoltageQ);
void SpeedControlLoop(float expectedMecAngularSpeed, float realityMecAngularSpeed, float *controlCurrentQ);
void PositionControlLoop(float expectedMecAngle, float realityMecAngle, float *controlCurrentQ);
float VelocitySlopeGenerator(float expectedVelocity);

/* USER CODE END PFP */


#endif

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

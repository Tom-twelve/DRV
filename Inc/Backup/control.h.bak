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
	float ControlCurrentD;
	float ControlCurrentQ;
	float ControlVoltageD;
	float ControlVoltageQ;
	float Kp_D;
	float Ki_D;
	float Kp_Q;
	float Ki_Q;
};

struct SpeedLoop_t
{
	float ExpectedMechanicalAngularSpeed;	//degree per second
	float Acceleration;
	float Kp;
	float Ki;
};

struct PositionLoop_t
{
	float ExpectedMechanicalAngle;	//degree
	float Kp;
	float Ki;
	float Kd;
};

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CurrentControlLoopIntegralErrorLimit_D 	2.5f	//(A)

#define CurrentControlLoopIntegralErrorLimit_Q 	2.5f	//(A)

#define SpeedControlLoopIntegralErrorLimit 	(10.0f * 360.f)		//(бу/s)

#define PositionControlLoopIntegralErrorLimit 	(0.5f * 360.f)		//(бу)

#define CURRENT_CONTROL_KP
#define CURRENT_CONTROL_KI

#define SPEED_CONTROL_KP	
#define SPEED_CONTROL_KI

#define POSITION_CONTROL_KP	
#define POSITION_CONTROL_KI
#define POSITION_CONTROL_KD

#define	VoltageControlMode 			0
#define CurrentControlMode 			1
#define SpeedControlMode 			2
#define PositionControlMode 		3

#define WorkMode				1
#define TestMode 				2

/* USER CODE END PD */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

void MotorEnable(void);
void CurrentControlLoop(float expectedCurrentD, float expectedCurrentQ, float realityCurrentD, float realityCurrentQ, float *controlVoltageD, float *controlVoltageQ);
void SpeedControlLoop(float expectedMechanicalAngularSpeed, float realityMechanicalAngularSpeed, float *controlCurrentQ);
void PositionControlLoop(float expectedMechanicalAngle, float realityMechanicalAngle, float *controlCurrentQ);
float VelocitySlopeGenerator(float expectedVelocity);

/* USER CODE END PFP */


#endif

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

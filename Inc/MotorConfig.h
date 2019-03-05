/**
 ******************************************************************************
 * @file		MotorConfig.h
 * @author		WrathWings
 * @version 	V1.0
 * @date		2018.1.17
 * @brief		The header file of MotorConfig.c
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTORCONFIG_H
#define __MOTORCONFIG_H

/* Private includes ----------------------------------------------------------*/
/* CODE BEGIN Includes */
#include "main.h"
/* CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RobotIdentifier		1U

#if RobotIdentifier == 1U
	#define CAN_ID_NUM			1U
		#if CAN_ID_NUM == 1
		#define MotorType				SUNNYSKY_X4125_9
		#define	PhaseSequence			NegativePhase
		#define EncoderType				Encoder_TLE5012
		#define GataDriverType			DRV8320
		#define MOSFET_Type				IPD053N08N3G_52nC_5mOhm3
		#define CurrentSensor			HallSensor_ACS781_150A
		#define GROUP_NUM           	0
		#elif CAN_ID_NUM == 2
		#define MotorType				N5055
		#define	PhaseSequence			NegativePhase
		#define EncoderType				Encoder_TLE5012
		#define GataDriveType			DRV8323
		#define MOSFET_Type				IPD053N08N3G_52nC_5mOhm3
		#define CurrentSensor			HallSensor_ACS781_150A
		#define GROUP_NUM          		0
		#endif
#endif



/* Motor Type*/
#if MotorType == SUNNYSKY_X4125_9	
#define MotorMagnetPairs					7.f
#define	MotorKV								350.f
#define RotatorFluxLinkage					( ONE_BY_SQRT3 * 60 / (2 * PI * (MotorMagnetPairs) * (MotorKV)))
#define PhaseResistance						0.0186698157f	//(Ohm)
#define InductanceD							0.0000065f		//(H)
#define InductanceQ							0.000012f		//(H)
#elif	MotorType == N5055
#define MotorMagnetPairs	7.f
#define	MotorKV				400.f
#define RotatorFluxLinkage	( ONE_BY_SQRT3 * 60 / (2 * PI * (MotorMagnetPairs) * (MotorKV)))
#define PhaseResistance		0.01955347f	//(Ohm)
#define InductanceD	
#define InductanceQ
#endif

/* Phase Sequence */
#define PositivePhase				1
#define NegativePhase				0

/* Motor Type */
#define SUNNYSKY_X4125_9			1
#define N5055						2

/* Encoder Type */
#define Encoder_TLE5012				1

/* Gate Driver Type */
#define DRV8323						1
#define DRV8320						2

/* MOSFET Type */
#define CDS18535_63nC_1mOhm6		1
#define IPD053N08N3G_52nC_5mOhm3	2

/* Current Sensor */
#define HallSensor_ACS781_150A		1
#define Resistance_1mOhm			2
#define Resistance_2mOhm			3

/* USER CODE END PD */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */


#endif

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

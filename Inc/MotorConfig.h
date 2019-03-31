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

/****************************************Type Define Begin****************************************/
/* Phase Sequence */
#define POSITIVE_SEQUENCE					1
#define NEGATIVE_SEQUENCE					2

/* Motor Type */
#define SUNNYSKY_X4125_9_KV350			1
#define N5055_KV400						2
#define TMOTOR_MN505_S_KV320			3

/* Position Sensor Type */
#define Encoder_TLE5012					1
#define HallSensor_DRV5053				2

/* Encoder Mode */
#define Encoder_AbsoluteMode			1
#define Encoder_IncrementalMode			2

/* Gate Driver Type */
#define GateDriver_DRV8323				1
#define GateDriver_DRV8320				2

/* MOSFET Type */
#define CDS18535_63nC_1mOhm6			1
#define IPD053N08N3G_52nC_5mOhm3		2

/* Current Sensor */
#define HallCurrentSensor_ACS781_150A	1
#define Resistance_1mOhm				2
#define Resistance_2mOhm				3

/****************************************Type Define End****************************************/

#define RobotIdentifier		1U

#if RobotIdentifier == 1U		//编码器
	#define CAN_ID_NUM			3
		#if CAN_ID_NUM == 1
		#define MOTOR_TYPE 				SUNNYSKY_X4125_9_KV350	
		#define	PHASE_SEQUENCE			NEGATIVE_SEQUENCE		
		#define POSITION_SENSOR_TYPE	Encoder_TLE5012
			#if POSITION_SENSOR_TYPE == Encoder_TLE5012
			#define ENCODER_MODE		Encoder_AbsoluteMode
			#endif
		#define GATE_DRIVER_TYPE		GateDriver_DRV8320
		#define MOSFET_TYPE				IPD053N08N3G_52nC_5mOhm3
		#define CURRENT_SENSOR			HallCurrentSensor_ACS781_150A
		#define GROUP_NUM           	0
		#elif CAN_ID_NUM == 2
		#define MOTOR_TYPE 				N5055_KV400	
		#define	PHASE_SEQUENCE			POSITIVE_SEQUENCE
		#define POSITION_SENSOR_TYPE	Encoder_TLE5012
			#if POSITION_SENSOR_TYPE == Encoder_TLE5012
			#define ENCODER_MODE		Encoder_AbsoluteMode
			#endif
		#define GATE_DRIVER_TYPE		GateDriver_DRV8320
		#define MOSFET_TYPE				IPD053N08N3G_52nC_5mOhm3
		#define CURRENT_SENSOR			HallCurrentSensor_ACS781_150A
		#define GROUP_NUM          		0
		#elif CAN_ID_NUM == 3
		#define MOTOR_TYPE 				TMOTOR_MN505_S_KV320	
		#define	PHASE_SEQUENCE			POSITIVE_SEQUENCE
		#define POSITION_SENSOR_TYPE	Encoder_TLE5012
			#if POSITION_SENSOR_TYPE == Encoder_TLE5012
			#define ENCODER_MODE		Encoder_AbsoluteMode
			#endif
		#define GATE_DRIVER_TYPE		GateDriver_DRV8320
		#define MOSFET_TYPE				IPD053N08N3G_52nC_5mOhm3
		#define CURRENT_SENSOR			HallCurrentSensor_ACS781_150A
		#define GROUP_NUM          		0
		#endif
#elif	RobotIdentifier == 2U	//霍尔
		#define CAN_ID_NUM			1
		#if CAN_ID_NUM == 1
		#define MOTOR_TYPE 				BallScrewMotor_KV320	
		#define	PHASE_SEQUENCE			NEGATIVE_SEQUENCE
		#define POSITION_SENSOR_TYPE	HallSensor_DRV5053
			#if POSITION_SENSOR_TYPE == Encoder_TLE5012
			#define ENCODER_MODE		Encoder_AbsoluteMode
			#endif
		#define GATE_DRIVER_TYPE		GateDriver_DRV8320
		#define MOSFET_TYPE				IPD053N08N3G_52nC_5mOhm3
		#define CURRENT_SENSOR			HallCurrentSensor_ACS781_150A
		#define GROUP_NUM          		0
		#elif CAN_ID_NUM == 2
		#define MOTOR_TYPE 				BallScrewMotor_KV320	
		#define	PHASE_SEQUENCE			POSITIVE_SEQUENCE
		#define POSITION_SENSOR_TYPE	HallSensor_DRV5053
			#if POSITION_SENSOR_TYPE == Encoder_TLE5012
			#define ENCODER_MODE		Encoder_AbsoluteMode
			#endif
		#define GATE_DRIVER_TYPE		GateDriver_DRV8320
		#define MOSFET_TYPE				IPD053N08N3G_52nC_5mOhm3
		#define CURRENT_SENSOR			HallCurrentSensor_ACS781_150A
		#define GROUP_NUM          		0
		#endif
#endif




/* Motor Type*/
#if MOTOR_TYPE == SUNNYSKY_X4125_9_KV350	  
	#define MotorPolePairs					7.f
	#define	MotorKV								350.f
	#define RotatorFluxLinkage					( ONE_BY_SQRT3 * 60 / (2 * PI * (MotorPolePairs) * (MotorKV)))
	#define PhaseResistance						0.0186698157f	//(Ohm)
	#define InductanceD							0.0000065f		//(H)
	#define InductanceQ							0.000012f		//(H)
#elif	MOTOR_TYPE == N5055_KV400
	#define MotorPolePairs					7.f
	#define	MotorKV								400.f
	#define RotatorFluxLinkage					( ONE_BY_SQRT3 * 60 / (2 * PI * (MotorPolePairs) * (MotorKV)))
	#define PhaseResistance						
	#define InductanceD	
	#define InductanceQ
#elif	MOTOR_TYPE == TMOTOR_MN505_S_KV320
	#define MotorPolePairs					14.f
	#define	MotorKV								320.f
	#define RotatorFluxLinkage					( ONE_BY_SQRT3 * 60 / (2 * PI * (MotorPolePairs) * (MotorKV)))
	#define PhaseResistance						0.019f	//(Ohm) T-MOTOR官方数据
	#define InductanceD							0.000008f	//(H)
	#define InductanceQ							0.000014f	//(H)
#elif	MOTOR_TYPE == BallScrewMotor_KV320	//与TMOTOR_MN505_S_KV320使用同一电枢
	#define MotorPolePairs					14.f
	#define	MotorKV								320.f
	#define RotatorFluxLinkage					( ONE_BY_SQRT3 * 60 / (2 * PI * (MotorPolePairs) * (MotorKV)))
	#define PhaseResistance						0.019f	//(Ohm) T-MOTOR官方数据
	#define InductanceD							0.000008f	//(H)
	#define InductanceQ							0.000014f	//(H)
#else
#error "Motor Type Invalid"
#endif



/* USER CODE END PD */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */


#endif

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

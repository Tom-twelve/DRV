/**
 ******************************************************************************
 * @file		MotorConfig.h
 * @author		WrathWings
 * @version 	V1.0
 * @date		2019.1.17
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
#define ENCODER_TLE5012					1

/* Encoder Mode */
#define Encoder_AbsoluteMode			1
#define Encoder_IncrementalMode			2

/* Gate Driver Type */
#define GATE_DRIVER_DRV8323				1
#define GATE_DRIVER_DRV8320				2

/* MOSFET Type */
#define CDS18535_63nC_1mOhm6			1
#define IPD053N08N3G_52nC_5mOhm3		2

/* Current Sensor */
#define HALL_CURR_SENSOR_ACS781_150A	1
#define RES_1mOhm						2
#define RES_2mOhm						3

/****************************************Type Define End****************************************/

#define ROBOT_ID		1U

#if ROBOT_ID == 1U		//编码器
	#define CAN_ID_NUM			3
		#if CAN_ID_NUM == 1
		#define MOTOR_TYPE 				SUNNYSKY_X4125_9_KV350	
		#define	PHASE_SEQUENCE			NEGATIVE_SEQUENCE		
		#define POSITION_SENSOR_TYPE	ENCODER_TLE5012
		#define ENCODER_MODE			Encoder_AbsoluteMode
		#define GATE_DRIVER_TYPE		GATE_DRIVER_DRV8320
		#define MOSFET_TYPE				IPD053N08N3G_52nC_5mOhm3
		#define CURRENT_SENSOR			HALL_CURR_SENSOR_ACS781_150A
		#define GROUP_NUM           	0
		#elif CAN_ID_NUM == 2
		#define MOTOR_TYPE 				N5055_KV400	
		#define	PHASE_SEQUENCE			POSITIVE_SEQUENCE
		#define POSITION_SENSOR_TYPE	ENCODER_TLE5012
		#define ENCODER_MODE			Encoder_AbsoluteMode
		#define GATE_DRIVER_TYPE		GATE_DRIVER_DRV8320
		#define MOSFET_TYPE				IPD053N08N3G_52nC_5mOhm3
		#define CURRENT_SENSOR			HALL_CURR_SENSOR_ACS781_150A
		#define GROUP_NUM          		0
		#elif CAN_ID_NUM == 3
		#define MOTOR_TYPE 				TMOTOR_MN505_S_KV320	
		#define	PHASE_SEQUENCE			POSITIVE_SEQUENCE
		#define POSITION_SENSOR_TYPE	ENCODER_TLE5012
		#define ENCODER_MODE			Encoder_AbsoluteMode
		#define GATE_DRIVER_TYPE		GATE_DRIVER_DRV8320
		#define MOSFET_TYPE				IPD053N08N3G_52nC_5mOhm3
		#define CURRENT_SENSOR			HALL_CURR_SENSOR_ACS781_150A
		#define GROUP_NUM          		0
		#endif
#endif




/* Motor Type*/
#if MOTOR_TYPE == SUNNYSKY_X4125_9_KV350	  
	#define MOTOR_POLE_PAIRS					7.f
	#define	MOTOR_KV							350.f
	#define ROTATOR_FLUX_LINKAGE				( ONE_BY_SQRT3 * 60 / (2 * PI * (MOTOR_POLE_PAIRS) * (MOTOR_KV)))
	#define PHASE_RES							0.0186698157f	//(Ohm)
	#define INDUCTANCE_D							0.0000065f		//(H)
	#define INDUCTANCE_Q							0.000012f		//(H)
#elif	MOTOR_TYPE == N5055_KV400
	#define MOTOR_POLE_PAIRS					7.f
	#define	MOTOR_KV							400.f
	#define ROTATOR_FLUX_LINKAGE				( ONE_BY_SQRT3 * 60 / (2 * PI * (MOTOR_POLE_PAIRS) * (MOTOR_KV)))
	#define PHASE_RES						
	#define INDUCTANCE_D	
	#define INDUCTANCE_Q
#elif	MOTOR_TYPE == TMOTOR_MN505_S_KV320
	#define MOTOR_POLE_PAIRS					14.f
	#define	MOTOR_KV							320.f
	#define ROTATOR_FLUX_LINKAGE				( ONE_BY_SQRT3 * 60 / (2 * PI * (MOTOR_POLE_PAIRS) * (MOTOR_KV)))
	#define PHASE_RES							0.019f	//(Ohm) T-MOTOR官方数据
	#define INDUCTANCE_D							0.000008f	//(H)
	#define INDUCTANCE_Q							0.000014f	//(H)
#elif	MOTOR_TYPE == BallScrewMotor_KV320	//与TMOTOR_MN505_S_KV320使用同一电枢
	#define MOTOR_POLE_PAIRS					14.f
	#define	MOTOR_KV							320.f
	#define ROTATOR_FLUX_LINKAGE				( ONE_BY_SQRT3 * 60 / (2 * PI * (MOTOR_POLE_PAIRS) * (MOTOR_KV)))
	#define PHASE_RES							0.019f	//(Ohm) T-MOTOR官方数据
	#define INDUCTANCE_D							0.000008f	//(H)
	#define INDUCTANCE_Q							0.000014f	//(H)
#else
#error "Motor Type Invalid"
#endif



/* USER CODE END PD */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */


#endif

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

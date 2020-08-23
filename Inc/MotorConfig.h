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
/*Robot ID*/
#define PASS_ROBOT						1U
#define TRY_ROBOT						2U

/* Phase Sequence */
#define POSITIVE_SEQUENCE				1
#define NEGATIVE_SEQUENCE				2

/* Motor Type */
#define MAD_XC5500_KV505				1
#define N5055_KV400						2
#define TMOTOR_U3_KV700					3
#define TMOTOR_P80_KV100				4	
#define TMOTOR_MN505_S_KV320			5	
#define SUNNYSKY_X4125_9_KV350			6
#define SUNNYSKY_X4125_3_KV210			7
#define IFLIGHT_T4214_KV660				8
#define LEOPARD_HOBBY_PH2820_KV780		9
#define CRAZY_MOTOR_5025_KV200			10
#define MAD_XC5000_KV380				11

/* Position Sensor Type */
#define ENCODER_TLE5012					1
#define HALL_SENSOR_DRV5053				2

/* Gate Driver Type */
#define GATE_DRIVER_DRV8323				1
#define GATE_DRIVER_DRV8320				2

/* MOSFET Type */
#define CDS18535_63nC_1mOhm6			1
#define IPD053N08N3G_52nC_5mOhm3		2
#define CSD88584Q5DC_52nC_5mOhm3		3

/* Current Sensor */
#define HALL_CURR_SENSOR_ACS781_50A		1
#define HALL_CURR_SENSOR_ACS781_150A	2
#define RES_1mOhm						3
#define RES_2mOhm						4

/****************************************Type Define End****************************************/

#define ROBOT_ID		PASS_ROBOT

#if ROBOT_ID == PASS_ROBOT
	#define CAN_ID_NUM			10
		#if CAN_ID_NUM == 1		//����
		#define MOTOR_TYPE 				MAD_XC5500_KV505	
		#define	PHASE_SEQUENCE			NEGATIVE_SEQUENCE
		#define POSITION_SENSOR_TYPE	ENCODER_TLE5012
		#define GATE_DRIVER_TYPE		GATE_DRIVER_DRV8323
		#define MOSFET_TYPE				CDS18535_63nC_1mOhm6
		#define INERTIA								(354.f*1e-7)
		#define CURRENT_SENSOR			RES_1mOhm
		#define GROUP_NUM           	1
		#elif CAN_ID_NUM == 2	//����
		#define MOTOR_TYPE 				MAD_XC5500_KV505	
		#define	PHASE_SEQUENCE			NEGATIVE_SEQUENCE
		#define POSITION_SENSOR_TYPE	ENCODER_TLE5012
		#define GATE_DRIVER_TYPE		GATE_DRIVER_DRV8323
		#define MOSFET_TYPE				CDS18535_63nC_1mOhm6
		#define INERTIA								(354.f*1e-7)
		#define CURRENT_SENSOR			RES_1mOhm
		#define GROUP_NUM           	1
		#elif CAN_ID_NUM == 3	//����
		#define MOTOR_TYPE 				MAD_XC5500_KV505	
		#define	PHASE_SEQUENCE			NEGATIVE_SEQUENCE
		#define POSITION_SENSOR_TYPE	ENCODER_TLE5012
		#define GATE_DRIVER_TYPE		GATE_DRIVER_DRV8323
		#define MOSFET_TYPE				CDS18535_63nC_1mOhm6
		#define CURRENT_SENSOR			RES_1mOhm
		#define INERTIA								(354.f*1e-7)
		#define GROUP_NUM           	1
		#elif CAN_ID_NUM == 4	//����
		#define MOTOR_TYPE 				MAD_XC5500_KV505	
		#define	PHASE_SEQUENCE			POSITIVE_SEQUENCE
		#define POSITION_SENSOR_TYPE	ENCODER_TLE5012
		#define GATE_DRIVER_TYPE		GATE_DRIVER_DRV8323
		#define MOSFET_TYPE				CDS18535_63nC_1mOhm6
		#define CURRENT_SENSOR			RES_1mOhm
		#define GROUP_NUM           	2
		#elif CAN_ID_NUM == 5	//����
		#define MOTOR_TYPE 				MAD_XC5500_KV505	
		#define	PHASE_SEQUENCE			POSITIVE_SEQUENCE
		#define POSITION_SENSOR_TYPE	ENCODER_TLE5012
		#define GATE_DRIVER_TYPE		GATE_DRIVER_DRV8323
		#define MOSFET_TYPE				CDS18535_63nC_1mOhm6
		#define CURRENT_SENSOR			RES_1mOhm
		#define GROUP_NUM           	2
		#elif CAN_ID_NUM == 6	//����
		#define MOTOR_TYPE 				MAD_XC5500_KV505	
		#define	PHASE_SEQUENCE			POSITIVE_SEQUENCE
		#define POSITION_SENSOR_TYPE	ENCODER_TLE5012
		#define GATE_DRIVER_TYPE		GATE_DRIVER_DRV8323
		#define MOSFET_TYPE				CDS18535_63nC_1mOhm6
		#define CURRENT_SENSOR			RES_1mOhm
		#define GROUP_NUM           	2
		#elif CAN_ID_NUM == 7	//����
		#define MOTOR_TYPE 				IFLIGHT_T4214_KV660	
		#define	PHASE_SEQUENCE			NEGATIVE_SEQUENCE
		#define POSITION_SENSOR_TYPE	ENCODER_TLE5012
		#define GATE_DRIVER_TYPE		GATE_DRIVER_DRV8323
		#define MOSFET_TYPE				CDS18535_63nC_1mOhm6
		#define CURRENT_SENSOR			RES_1mOhm
		#define INERTIA								(181.f*1e-7)
		#define GROUP_NUM           	2
		#elif CAN_ID_NUM == 8	//����
		#define MOTOR_TYPE 				LEOPARD_HOBBY_PH2820_KV780	
		#define	PHASE_SEQUENCE			NEGATIVE_SEQUENCE
		#define POSITION_SENSOR_TYPE	ENCODER_TLE5012
		#define GATE_DRIVER_TYPE		GATE_DRIVER_DRV8323
		#define MOSFET_TYPE				CDS18535_63nC_1mOhm6
		#define CURRENT_SENSOR			RES_1mOhm
		#define INERTIA								(354.f*1e-7)
		#define GROUP_NUM           	1
		#elif CAN_ID_NUM == 9	//����
		#define MOTOR_TYPE 				MAD_XC5500_KV505	
		#define	PHASE_SEQUENCE			NEGATIVE_SEQUENCE
		#define POSITION_SENSOR_TYPE	ENCODER_TLE5012
		#define GATE_DRIVER_TYPE		GATE_DRIVER_DRV8323
		#define MOSFET_TYPE				CDS18535_63nC_1mOhm6
		#define CURRENT_SENSOR			RES_1mOhm
		#define INERTIA								(354.f*1e-7)
		#define GROUP_NUM           	1
		#elif CAN_ID_NUM == 10	//����
		#define MOTOR_TYPE 				MAD_XC5500_KV505	
		#define	PHASE_SEQUENCE			NEGATIVE_SEQUENCE
		#define POSITION_SENSOR_TYPE	ENCODER_TLE5012
		#define GATE_DRIVER_TYPE		GATE_DRIVER_DRV8323
		#define MOSFET_TYPE				CDS18535_63nC_1mOhm6
		#define CURRENT_SENSOR			RES_1mOhm
		#define INERTIA								(354.f*1e-7)
		#define GROUP_NUM           	1
		#elif CAN_ID_NUM == 11	//����
		#define MOTOR_TYPE 				MAD_XC5500_KV505
		#define	PHASE_SEQUENCE			POSITIVE_SEQUENCE
		#define POSITION_SENSOR_TYPE	ENCODER_TLE5012
		#define GATE_DRIVER_TYPE		GATE_DRIVER_DRV8323
		#define MOSFET_TYPE				CDS18535_63nC_1mOhm6
		#define CURRENT_SENSOR			RES_1mOhm
		#define INERTIA								(354.f*1e-7)
		#define GROUP_NUM           	1
		#endif
#elif ROBOT_ID == TRY_ROBOT
	#define CAN_ID_NUM			7
		#if CAN_ID_NUM == 1		//����
		#define MOTOR_TYPE 				MAD_XC5500_KV505	
		#define	PHASE_SEQUENCE			NEGATIVE_SEQUENCE
		#define POSITION_SENSOR_TYPE	ENCODER_TLE5012
		#define GATE_DRIVER_TYPE		GATE_DRIVER_DRV8323
		#define MOSFET_TYPE				CDS18535_63nC_1mOhm6
		#define CURRENT_SENSOR			RES_1mOhm
		#define INERTIA								(155.f*1e-7)
		#define GROUP_NUM           	1
		#elif CAN_ID_NUM == 2	//����
		#define MOTOR_TYPE 				MAD_XC5500_KV505	
		#define	PHASE_SEQUENCE			NEGATIVE_SEQUENCE
		#define POSITION_SENSOR_TYPE	ENCODER_TLE5012
		#define GATE_DRIVER_TYPE		GATE_DRIVER_DRV8323
		#define MOSFET_TYPE				CDS18535_63nC_1mOhm6
		#define CURRENT_SENSOR			RES_1mOhm
		#define INERTIA								(132.f*1e-7)
		#define GROUP_NUM           	1
		#elif CAN_ID_NUM == 3	//����
		#define MOTOR_TYPE 				MAD_XC5500_KV505	
		#define	PHASE_SEQUENCE			NEGATIVE_SEQUENCE
		#define POSITION_SENSOR_TYPE	ENCODER_TLE5012
		#define GATE_DRIVER_TYPE		GATE_DRIVER_DRV8323
		#define MOSFET_TYPE				CDS18535_63nC_1mOhm6
		#define CURRENT_SENSOR			RES_1mOhm
		#define INERTIA								(115.f*1e-7)
		#define GROUP_NUM           	1
		#elif CAN_ID_NUM == 4	//����
		#define MOTOR_TYPE 				MAD_XC5500_KV505	
		#define	PHASE_SEQUENCE			POSITIVE_SEQUENCE
		#define POSITION_SENSOR_TYPE	ENCODER_TLE5012
		#define GATE_DRIVER_TYPE		GATE_DRIVER_DRV8323
		#define MOSFET_TYPE				CDS18535_63nC_1mOhm6
		#define INERTIA								(785.f*1e-7)
		#define CURRENT_SENSOR			RES_1mOhm
		#define GROUP_NUM           	2
		#elif CAN_ID_NUM == 5	//����
		#define MOTOR_TYPE 				MAD_XC5500_KV505	
		#define	PHASE_SEQUENCE			POSITIVE_SEQUENCE
		#define POSITION_SENSOR_TYPE	ENCODER_TLE5012
		#define GATE_DRIVER_TYPE		GATE_DRIVER_DRV8323
		#define MOSFET_TYPE				CDS18535_63nC_1mOhm6
		#define CURRENT_SENSOR			RES_1mOhm
		#define INERTIA								(613.f*1e-7)
		#define GROUP_NUM           	2
		#elif CAN_ID_NUM == 6	//����
		#define MOTOR_TYPE 				MAD_XC5500_KV505	
		#define	PHASE_SEQUENCE			POSITIVE_SEQUENCE
		#define POSITION_SENSOR_TYPE	ENCODER_TLE5012
		#define GATE_DRIVER_TYPE		GATE_DRIVER_DRV8323
		#define MOSFET_TYPE				CDS18535_63nC_1mOhm6
		#define CURRENT_SENSOR			RES_1mOhm
		#define INERTIA								(624.f*1e-7)
		#define GROUP_NUM           	2
		#elif CAN_ID_NUM == 7	//压球
		#define MOTOR_TYPE 				IFLIGHT_T4214_KV660	
		#define	PHASE_SEQUENCE			NEGATIVE_SEQUENCE
		#define POSITION_SENSOR_TYPE	ENCODER_TLE5012
		#define GATE_DRIVER_TYPE		GATE_DRIVER_DRV8323
		#define MOSFET_TYPE				CDS18535_63nC_1mOhm6
		#define CURRENT_SENSOR			RES_1mOhm
		#define INERTIA								(181.f*1e-7)
		#define GROUP_NUM           	2
		#endif
#endif

/* Motor Type*/
#if MOTOR_TYPE == MAD_XC5500_KV505	  
	#define MOTOR_POLE_PAIRS_NUM				7
	#define	MOTOR_KV							505.f
	#define ROTATOR_FLUX_LINKAGE				(5.513288954f / (MOTOR_KV * MOTOR_POLE_PAIRS_NUM))
	#define PHASE_RES							(14.f * (float)1e-3)	//(Ohm)
	#define INDUCTANCE_D						(14.f * (float)1e-6)	//(H)
	#define INDUCTANCE_Q						(14.f * (float)1e-6)	//(H)
	#define MAX_SPD								(GENERATRIX_VOL * MOTOR_KV * 0.10471975)	//rad/s
#elif	MOTOR_TYPE == N5055_KV400
	#define MOTOR_POLE_PAIRS_NUM				7
	#define	MOTOR_KV							400.f
	#define ROTATOR_FLUX_LINKAGE				(5.513288954f / (MOTOR_KV * MOTOR_POLE_PAIRS_NUM))
	#define PHASE_RES							(22.f * (float)1e-3)	//(Ohm)
	#define INDUCTANCE_D						(15.f * (float)1e-6)	//(H)
	#define INDUCTANCE_Q						(15.f * (float)1e-6)	//(H)
	#define MAX_SPD								(GENERATRIX_VOL * MOTOR_KV * 0.10471975)	//rad/s
#elif	MOTOR_TYPE == TMOTOR_U3_KV700
	#define MOTOR_POLE_PAIRS_NUM				7
	#define	MOTOR_KV							700.f
	#define ROTATOR_FLUX_LINKAGE				(5.513288954f / (MOTOR_KV * MOTOR_POLE_PAIRS_NUM))
	#define PHASE_RES							(85.f * (float)1e-3)	//(Ohm)
	#define INDUCTANCE_D						(22.f * (float)1e-6)	//(H)
	#define INDUCTANCE_Q						(22.f * (float)1e-6)	//(H)
	#define MAX_SPD								(GENERATRIX_VOL * MOTOR_KV * 0.10471975)	//rad/s
#elif	MOTOR_TYPE == TMOTOR_P80_KV100
	#define MOTOR_POLE_PAIRS_NUM				21
	#define	MOTOR_KV							100.f
	#define ROTATOR_FLUX_LINKAGE				(5.513288954f / (MOTOR_KV * MOTOR_POLE_PAIRS_NUM))
	#define PHASE_RES							(20.f * (float)1e-3)	//(Ohm)
	#define INDUCTANCE_D						(24.f * (float)1e-6)	//(H)
	#define INDUCTANCE_Q						(24.f * (float)1e-6)	//(H)
	#define MAX_SPD								(GENERATRIX_VOL * MOTOR_KV * 0.10471975)	//rad/s
#elif	MOTOR_TYPE == TMOTOR_MN505_S_KV320
	#define MOTOR_POLE_PAIRS_NUM				14
	#define	MOTOR_KV							320.f
	#define ROTATOR_FLUX_LINKAGE				(5.513288954f / (MOTOR_KV * MOTOR_POLE_PAIRS_NUM))
	#define PHASE_RES							(19.f * (float)1e-3)	//(Ohm)
	#define INDUCTANCE_D						(15.f * (float)1e-6)	//(H)
	#define INDUCTANCE_Q						(15.f * (float)1e-6)	//(H)
	#define MAX_SPD								(GENERATRIX_VOL * MOTOR_KV * 0.10471975)	//rad/s
#elif	MOTOR_TYPE == SUNNYSKY_X4125_9_KV350
	#define MOTOR_POLE_PAIRS_NUM				7
	#define	MOTOR_KV							350.f
	#define ROTATOR_FLUX_LINKAGE				(5.513288954f / (MOTOR_KV * MOTOR_POLE_PAIRS_NUM))
	#define PHASE_RES							(18.2f * (float)1e-3)	//(Ohm)
	#define INDUCTANCE_D						(12.f * (float)1e-6)	//(H)
	#define INDUCTANCE_Q						(12.f * (float)1e-6)	//(H)
	#define MAX_SPD								(GENERATRIX_VOL * MOTOR_KV * 0.10471975)	//rad/s
#elif	MOTOR_TYPE == SUNNYSKY_X4125_3_KV210	
	#define MOTOR_POLE_PAIRS_NUM				7
	#define	MOTOR_KV							210.f
	#define ROTATOR_FLUX_LINKAGE				(5.513288954f / (MOTOR_KV * MOTOR_POLE_PAIRS_NUM))
	#define PHASE_RES							(18.2f * (float)1e-3)	//(Ohm)
	#define INDUCTANCE_D						(15.f * (float)1e-6)	//(H)
	#define INDUCTANCE_Q						(15.f * (float)1e-6)	//(H)
	#define MAX_SPD								(GENERATRIX_VOL * MOTOR_KV * 0.10471975)	//rad/s
#elif	MOTOR_TYPE == IFLIGHT_T4214_KV660	
	#define MOTOR_POLE_PAIRS_NUM				7
	#define	MOTOR_KV							660.f
	#define ROTATOR_FLUX_LINKAGE				(5.513288954f / (MOTOR_KV * MOTOR_POLE_PAIRS_NUM))
	#define PHASE_RES							(18.2f * (float)1e-3)	//(Ohm)
	#define INDUCTANCE_D						(15.f * (float)1e-6)	//(H)
	#define INDUCTANCE_Q						(15.f * (float)1e-6)	//(H)
	#define MAX_SPD								(GENERATRIX_VOL * MOTOR_KV * 0.10471975)	//rad/s
#elif	MOTOR_TYPE == LEOPARD_HOBBY_PH2820_KV780	
	#define MOTOR_POLE_PAIRS_NUM				7
	#define	MOTOR_KV							780.f
	#define ROTATOR_FLUX_LINKAGE				(5.513288954f / (MOTOR_KV * MOTOR_POLE_PAIRS_NUM))
	#define PHASE_RES							(35.f * (float)1e-3)	//(Ohm)
	#define INDUCTANCE_D						(11.f * (float)1e-6)	//(H)
	#define INDUCTANCE_Q						(11.f * (float)1e-6)	//(H)
	#define MAX_SPD								(GENERATRIX_VOL * MOTOR_KV * 0.10471975)	//rad/s
#elif	MOTOR_TYPE == CRAZY_MOTOR_5025_KV200	
	#define MOTOR_POLE_PAIRS_NUM				7
	#define	MOTOR_KV							200.f
	#define ROTATOR_FLUX_LINKAGE				(5.513288954f / (MOTOR_KV * MOTOR_POLE_PAIRS_NUM))
	#define PHASE_RES							(30.f * (float)1e-3)	//(Ohm)
	#define INDUCTANCE_D						(25.f * (float)1e-6)	//(H)
	#define INDUCTANCE_Q						(25.f * (float)1e-6)	//(H)
	#define MAX_SPD								(GENERATRIX_VOL * MOTOR_KV * 0.10471975)	//rad/s
#elif	MOTOR_TYPE == MAD_XC5000_KV380	
	#define MOTOR_POLE_PAIRS_NUM				7
	#define	MOTOR_KV							380.f
	#define ROTATOR_FLUX_LINKAGE				(5.513288954f / (MOTOR_KV * MOTOR_POLE_PAIRS_NUM))
	#define PHASE_RES							(30.f * (float)1e-3)	//(Ohm)
	#define INDUCTANCE_D						(28.f * (float)1e-6)	//(H)
	#define INDUCTANCE_Q						(28.f * (float)1e-6)	//(H)
	#define MAX_SPD								(GENERATRIX_VOL * MOTOR_KV * 0.10471975)	//rad/s
#else
#error "Motor Type Invalid"
#endif



/* USER CODE END PD */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */


#endif

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

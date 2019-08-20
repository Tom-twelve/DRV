/**
 ******************************************************************************
 * @file		foc.h
 * @author		WrathWings
 * @version 	V1.0
 * @date		2019.1.17
 * @brief		The header file of foc.c
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FOC_H
#define __FOC_H

/* Private includes ----------------------------------------------------------*/
/* CODE BEGIN Includes */
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "math.h"
#include "arm_math.h"
#include "PositionSensor.h"
#include "control.h"
#include "usart.h"
#include "util.h"

/* CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

struct CoordTrans_t
{
	float CurrA;
	float CurrB;
	float CurrC;
	float VolAlpha;
	float VolBeta;
	float VolD;
	float VolQ;
	float CurrAlpha;
	float CurrBeta;
	float CurrD;
	float CurrQ;
};

struct MotorDynamicParameter_t
{
	float EleTorque;
};

struct MotorStaticParameter_t
{
	float PowerAngleComp_degree;	
	float PowerAngleComp_rad;
	uint8_t ControlMode;	//速度控制模式或位置控制模式
	uint8_t MotorMode;		//正常工作模式或测量电角度模式
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define	GENERATRIX_VOL						25.0f		//(V)
#define DEFAULT_CARRIER_FREQ				20000.f		//(Hz)
#define DEFAULT_CARRIER_PERIOD_s			(1.f / DEFAULT_CARRIER_FREQ)		//(s)
#define DEFAULT_CARRIER_PERIOD_us			(DEFAULT_CARRIER_PERIOD_s * 1000000.f)	//(us)
#define MCU_FREQ							180000000.f		//(Hz)
#define TIM8_ARR 							(int)((MCU_FREQ / DEFAULT_CARRIER_FREQ) / 2.f )
#define ADC_ExternalTrigger_CCR				(TIM8_ARR - 12)	//通过TIM8_CH4触发ADC, 超前量为ADC采样所需时间

#if		PHASE_SEQUENCE == POSITIVE_SEQUENCE
#define CCR_PhaseA      	TIM8->CCR3
#define CCR_PhaseB          TIM8->CCR2
#define CCR_PhaseC          TIM8->CCR1
#elif	PHASE_SEQUENCE == NEGATIVE_SEQUENCE
#define CCR_PhaseA          TIM8->CCR1
#define CCR_PhaseB          TIM8->CCR2
#define CCR_PhaseC          TIM8->CCR3
#else
#error "Phase Sequence Invalid"
#endif

/* USER CODE END PD */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void SpaceVectorModulation(float volAlpha, float volBeta);
void ParkTransform(float currentPhaseA, float currentPhaseB, float currentPhaseC, float *currentD, float *currentQ, float eleAngle);
void InverseParkTransform(float VolD, float VolQ, float *VolAlpha, float *VolBeta, float eleAngle);
void ClarkTransform(float currentPhaseA, float currentPhaseB, float currentPhaseC, float *currentAlpha, float *currentBeta);
void ParkTransform_arm(float currentAlpha, float currentBeta, float *currentD, float *currentQ, float eleAngle);
void ClarkTransform_arm(float currentPhaseA, float currentPhaseB, float *currentAlpha, float *currentBeta);
void PowerAngleComp(float expectedCurrentQ, float *powerAngleComp_degree);
void CalculateEleTorque(float actualCurrentQ, float *eleTorque);
void CalculateVoltage_dq(float actualCurrentQ, float *volD, float *volQ, float actualEleAngularSpeed_rad);

/* USER CODE END PFP */


#endif

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

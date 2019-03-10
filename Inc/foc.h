/**
 ******************************************************************************
 * @file		foc.h
 * @author		WrathWings
 * @version 	V1.0
 * @date		2018.1.17
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
#include "Encoder.h"
#include "control.h"
#include "usart.h"
#include "util.h"

/* CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

struct CoordinateTransformation_t
{
	float VoltageAlpha;
	float VoltageBeta;
	float VoltageD;
	float VoltageQ;
	float CurrentAlpha;
	float CurrentBeta;
	float CurrentD;
	float CurrentQ;
};

struct MotorDynamicParameter_t
{
	float VoltagePhaseA;
	float VoltagePhaseB;
	float VoltagePhaseC;
	float CurrentPhaseA;
	float CurrentPhaseB;
	float CurrentPhaseC;
	float AvgCurrentPhaseA;
	float AvgCurrentPhaseB;
	float AvgCurrentPhaseC;
	float ElectromagneticTorque;
};

struct MotorStaticParameter_t
{
	float PowerAngleCompensation_degree;	
	float PowerAngleCompensation_rad;
	uint8_t ControlMode;	//速度控制模式或位置控制模式
	uint8_t MotorMode;		//正常工作模式或测量电角度模式
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define	GeneratrixVoltage					25.0f		//(V)
#define	MaximumDistortionlessVoltage		(ONE_BY_SQRT3 * GeneratrixVoltage)	//(V)
#define CarrierFrequency					20000.f		//(Hz)
#define CarrierPeriod_s						(1.f / CarrierFrequency)		//(s)
#define CarrierPeriod_us					(CarrierPeriod_s * 1000000.f)	//(us)
#define MCU_Frequency						180000000.f		//(Hz)
#define TIM8_Autoreload 					(int)((MCU_Frequency / CarrierFrequency) / 2.f )
#define ADC_ExternalTrigger_CCR				(TIM8_Autoreload - 12)	//通过TIM8_CH4触发ADC, 超前量为ADC采样所需时间

#define DivideNum  20	//将360度n等分, 每次电角度增量为(360/DivideNum)

#if		PHASE_SEQUENCE == PositivePhase
#define CCR_PhaseA      	TIM8->CCR3
#define CCR_PhaseB          TIM8->CCR2
#define CCR_PhaseC          TIM8->CCR1
#elif	PHASE_SEQUENCE == NegativePhase
#define CCR_PhaseA          TIM8->CCR1
#define CCR_PhaseB          TIM8->CCR2
#define CCR_PhaseC          TIM8->CCR3
#else
#error "Phase Sequence Invalid"
#endif

/* USER CODE END PD */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void SpaceVectorPulseWidthModulation(float voltageAlpha, float voltageBeta);
void PowerAngleCompensation(float expectedCurrentQ, float *powerAngleCompensation_degree);
void ParkTransform(float currentPhaseA, float currentPhaseB, float currentPhaseC, float *currentD, float *currentQ, float EleAngle);
void InverseParkTransform_TwoPhase(float voltageD, float voltageQ,float *voltageAlpha,float *voltageBeta, float EleAngle);
void ClarkTransform(float currentPhaseA, float currentPhaseB, float currentPhaseC, float *currentAlpha, float *currentBeta);
void CurrentVoltageTransform(float controlCurrentQ, float *voltageD, float *voltageQ, float actualEleAngularSpeed_rad);
void CalculateElectromagneticTorque(float actualCurrentQ, float *electromagneticTorque);
void CalculateVoltage_dq(float actualCurrentQ, float *voltageD, float *voltageQ, float actualEleAngularSpeed_rad);
void MeasureEleAngle(float voltageD);

/* USER CODE END PFP */


#endif

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

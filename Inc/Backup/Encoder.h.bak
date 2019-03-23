/**
 ******************************************************************************
 * @file		Encoder.h
 * @author		WrathWings
 * @version 	V1.0
 * @date		2019.3.2
 * @brief		The header file of Encoder.c
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __Encoder_H
#define __Encoder_H

/* Private includes ----------------------------------------------------------*/
/* CODE BEGIN Includes */
#include <stdint.h>
#include <stdlib.h>
#include "gpio.h"
#include "tim.h"
#include "spi.h"
#include "MotorConfig.h"
#include "AngleTable.h"
#include "util.h"
#include "foc.h"
/* CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#if ENCODER_TYPE == Encoder_TLE5012
	#define SPI_TX_ON           GPIOB->MODER &= 0xFFFFF3FF; GPIOB->MODER |= 0x00000800	// PB5--MOSI复用
	#define SPI_TX_OFF          GPIOB->MODER &= 0xFFFFF3FF; GPIOB->MODER |= 0x00000000	//PB5--复位(输入模式)

	#define	TLE5012_UpdateTime_0		0.0000213f	//(us)	
	#define	TLE5012_UpdateTime_1		0.0000427f	//(us)	
	#define	TLE5012_UpdateTime_2		0.0000853f	//(us)	
	#define	TLE5012_UpdateTime_3		0.0001706f	//(us)	
		
	#define	TLE5012_SPI1_ChipSelect				LL_GPIO_ResetOutputPin(Encoder_SPI1_NSS_GPIO_Port, Encoder_SPI1_NSS_Pin)
	#define	TLE5012_SPI1_ChipDiselect			LL_GPIO_SetOutputPin(Encoder_SPI1_NSS_GPIO_Port, Encoder_SPI1_NSS_Pin)

	#define TLE5012_Command_ReadCurrentValue_AngleValue		0x8021	
	#define TLE5012_Command_ReadUpdatedValue_AngleValue		0x8421	

	#define TLE5012_Command_ReadCurrentValue_AngularSpeed 	0x8031	
	#define TLE5012_Command_ReadUpdatedValue_AngularSpeed	0x8431	
	
	#define TLE5012_AbsoluteModeResolution					32768.f
	#define TLE5012_IncrementalModeResolution				4096.f
#else
#error "Encoder Type Invalid"
#endif

/* USER CODE END PD */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#if ENCODER_TYPE == Encoder_TLE5012
	struct Encoder_t
	{
		uint16_t MecAngle_15bit;
		uint16_t MecAngle_14bit;
		int16_t MecAngularSpeed_Encoder_15bit;
		float MecAngularSpeed_Encoder;
		float MecAngle_degree;
		float MecAngle_rad;
		float MecAngularSpeed_degree;
		float MecAngularSpeed_rad;
		float AvgMecAngularSpeed_degree;
		float AvgMecAngularSpeed_rad;
		float EleAngle_degree;
		float EleAngle_rad;
		float EleAngularSpeed_degree;
		float EleAngularSpeed_rad;
		float AvgEleAngularSpeed_degree;
		float AvgEleAngularSpeed_rad;
		uint16_t OriginalMecAngle_14bit;
	};
#else
#error "Encoder Type Invalid"
#endif

/* USER CODE END PTD */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

#if ENCODER_TYPE == Encoder_TLE5012
	void GetPositionImformation(void);
	void GetMecAngle_15bit(void);
	void GetMecAngle_14bit(void);
	void GetMecAngle_degree(void);
	void GetMecAngle_rad(void);
	void GetMecAngularSpeed_degree(void);
	void GetMecAngularSpeed_rad(void);
	void GetAvgMecAngularSpeed_degree(void);
	void GetAvgMecAngularSpeed_rad(void);
	void CalculateEleAngle_degree(void);
	void CalculateEleAngle_rad(void);
	void GetEleAngularSpeed_degree(void);
	void GetEleAngularSpeed_rad(void);
	void GetAvgEleAngularSpeed_degree(void);
	void GetAvgEleAngularSpeed_rad(void);
	uint16_t TLE5012_ReadRegister(uint16_t command);
	void EncoderIncrementalModeEnable(void);
#else
#error "Encoder Type Invalid"
#endif

/* USER CODE END PFP */

#endif

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

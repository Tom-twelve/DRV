/**
 ******************************************************************************
 * @file		PositionSensor.h
 * @author		WrathWings
 * @version 	V1.0
 * @date		2019.3.2
 * @brief		The header file of PositionSensor.c
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PositionSensor_H
#define __PositionSensor_H

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
#if POSITION_SENSOR_TYPE == ENCODER_TLE5012
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
	
	#define DIVIDE_NUM  20	//将360度n等分, 每次电角度增量为(360/DIVIDE_NUM)
#else
#error "Position Sensor Type Invalid"
#endif

/* USER CODE END PD */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#if POSITION_SENSOR_TYPE == ENCODER_TLE5012
	struct PosSensor_t
	{
		uint16_t MecAngle_AbsoluteMode_15bit;
		uint16_t MecAngle_IncrementalMode_14bit;
		float MecAngle_degree;
		float MecAngle_rad;
		float MecAngularSpeed_rad;
		float EleAngle_degree;
		float EleAngle_rad;
		float EleAngularSpeed_rad;
		uint16_t OriginalMecAngle_14bit;
	};
#else
#error "Position Sensor Type Invalid"
#endif

/* USER CODE END PTD */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
	

#if POSITION_SENSOR_TYPE == ENCODER_TLE5012
	void GetPositionImformation(void);
	void GetMecAngle(void);
	void GetMecAngularSpeed(void);
	void GetEleAngle(void);
	void GetEleAngularSpeed(void);
	void GetMecAngle_AbsoluteMode_15bit(void);
	void GetMecAngle_IncrementalMode_14bit(void);
	uint16_t TLE5012_ReadRegister(uint16_t command);
	void EncoderIncrementalModeEnable(void);
	void MeasureEleAngle_Encoder(float VolD);
#else
#error "Position Sensor Type Invalid"
#endif

/* USER CODE END PFP */

#endif

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

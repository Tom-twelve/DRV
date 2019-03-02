/**
 ******************************************************************************
 * @file		Encoder.h
 * @author		WrathWings
 * @version 	V1.0
 * @date		2019.1.17
 * @brief		The header file of tle5012.c
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TLE5012_H
#define __TLE5012_H

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
#define TLE5012_Command_ReadUpdatedValue_AngularSpeed		0x8431	

/* USER CODE END PD */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct Encoder_t
{
	uint16_t MechanicalAngle_15bit;
	int16_t MechanicalAngularSpeed_Encoder_15bit;
	float MechanicalAngularSpeed_Encoder;
	float MechanicalAngle_degree;
	float MechanicalAngle_rad;
	float MechanicalAngularSpeed_degree;
	float MechanicalAngularSpeed_rad;
	float AverageMechanicalAngularSpeed_degree;
	float AverageMechanicalAngularSpeed_rad;
	float ElectricalAngle_degree;
	float ElectricalAngle_rad;
	float ElectricalAngularSpeed_degree;
	float ElectricalAngularSpeed_rad;
	float AverageElectricalAngularSpeed_degree;
	float AverageElectricalAngularSpeed_rad;
};

/* USER CODE END PTD */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void GetPositionImformation(void);
void GetMechanicalAngle_15bit(void);
void GetMechanicalAngle_degree(void);
void GetMechanicalAngle_rad(void);
void GetMechanicalAngularSpeed_degree(void);
void GetMechanicalAngularSpeed_rad(void);
void GetAverageMechanicalAngularSpeed_degree(void);
void GetAverageMechanicalAngularSpeed_rad(void);
void CalculateElectricalAngle_degree(void);
void CalculateElectricalAngle_rad(void);
void GetElectricalAngularSpeed_degree(void);
void GetElectricalAngularSpeed_rad(void);
void GetAverageElectricalAngularSpeed_degree(void);
void GetAverageElectricalAngularSpeed_rad(void);
uint16_t TLE5012_ReadRegister(uint16_t command);

/* USER CODE END PFP */

#endif

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

/**
 ******************************************************************************
 * @file		identify.h
 * @author		WrathWings
 * @version 	V1.0
 * @date		2019.9.5
 * @brief		The header file of identify.c
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IDENTIFY_H
#define __IDENTIFY_H

/* Private includes ----------------------------------------------------------*/
/* CODE BEGIN Includes */
#include "main.h"
#include "foc.h"
#include "control.h"
/* CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

uint8_t MeasureResidence(float sampleTimes, float currQ, float *residence);
uint8_t MeasureInductance(float sampleTimes, float *inductance);
void MeasureParameters(void);
void RotateInertiaTest(float sampleTime);

/* USER CODE END PFP */


#endif

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

/**
 ******************************************************************************
 * @file		observer.h
 * @author		WrathWings
 * @version 	V1.0
 * @date		2019.8.20
 * @brief		The header file of observer.c
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __OBSERVER_H
#define __OBSERVER_H

/* Private includes ----------------------------------------------------------*/
/* CODE BEGIN Includes */
#include "main.h"
#include "foc.h"
#include "control.h"
#include "observer.h"
/* CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
 float loadGain1;
 float loadGain2;
 float load;
 float w;
}LoadObserverType;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
extern LoadObserverType observer;
void LoadObserver(void);
/* USER CODE END PFP */


#endif

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

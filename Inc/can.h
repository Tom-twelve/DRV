/**
  ******************************************************************************
  * File Name          : CAN.h
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __can_H
#define __can_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "foc.h"
#include "PositionSensor.h"
#include "MotorConfig.h"
#include "usart.h"
/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */
#define CAN_SDO_CLIENT_STD_ID     (0x280 + CAN_ID_NUM)
#define CAN_SDO_SERVER_STD_ID     (0x300 + CAN_ID_NUM)

typedef union
{
	volatile uint32_t  data_uint32[2];
	volatile int32_t   data_int32[2];
	volatile float     data_float[2];
	volatile uint8_t   data_uint8[8];
} CANdata_t;

/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
void CANRespond(void);
void CANSendData(CANdata_t data);
void CAN_Enable(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ can_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

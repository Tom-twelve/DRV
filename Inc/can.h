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

#define DRIVER_CLIENT_BASE_ID		0x280
#define DRIVER_SERVER_BASE_ID		0x300

#define DRIVER_CLIENT_CAN_ID		(DRIVER_CLIENT_BASE_ID + CAN_ID_NUM)
#define DRIVER_SERVER_CAN_ID		(DRIVER_SERVER_BASE_ID + CAN_ID_NUM)
#define DRIVER_BROADCAST_ID			(DRIVER_SERVER_BASE_ID + 0x40 + GROUP_NUM)

typedef union
{
	volatile uint32_t  data_uint32[2];
	volatile int32_t   data_int32[2];
	volatile uint8_t   data_uint8[8];
} CAN_Data_t;

struct CAN_t
{
	uint32_t StdID;
	uint32_t MailBox;
	uint16_t Identifier;
	uint32_t ReceiveData;
	int32_t RecieveStatus;
	int32_t TransmitData;
	volatile CAN_Data_t Receive;
	volatile CAN_Data_t Transmit;
};

/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
void CANRespond_Test(void);
void CANRespond(void);
void CANSendData_Test(CAN_Data_t data, uint32_t identifier);
void CAN_Transmit_Test(CAN_Data_t data, uint8_t length);
void CAN_Transmit(CAN_Data_t data);
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

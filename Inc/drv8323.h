/**
 ******************************************************************************
 * @file		drv8323.h
 * @author		WrathWings
 * @version 	V1.0
 * @date		2018.1.17
 * @brief		The header file of drv8323.c
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRV8323_H
#define __DRV8323_H

/* Private includes ----------------------------------------------------------*/
/* CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "gpio.h"
#include "adc.h"
/* CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DRV8323_ReadCommandMessageNumber	7
#define DRV8323_WriteCommandMessageNumber	5

#define DRV8323_Enable							LL_GPIO_SetOutputPin(DRV8323_Enable_GPIO_Port, DRV8323_Enable_Pin)
#define DRV8323_Disable							LL_GPIO_ResetOutputPin(DRV8323_Enable_GPIO_Port, DRV8323_Enable_Pin)

#define DRV8323_SPI3_ChipSelect					LL_GPIO_ResetOutputPin(DRV8323_SPI3_NSS_GPIO_Port, DRV8323_SPI3_NSS_Pin)
#define DRV8323_SPI3_ChipDiselect				LL_GPIO_SetOutputPin(DRV8323_SPI3_NSS_GPIO_Port, DRV8323_SPI3_NSS_Pin)

#define DRV8323_CurrentSamplingCorrectStart		LL_GPIO_SetOutputPin(DRV8323_CAL_GPIO_Port, DRV8323_CAL_Pin)
#define DRV8323_CurrentSamplingCorrectOver		LL_GPIO_ResetOutputPin(DRV8323_CAL_GPIO_Port, DRV8323_CAL_Pin)

#define DRV8323_SPI_DATA_FORMATTER(RW, ADDR, DATA)    ((((RW) & 0x01) << 15) | (((ADDR) & 0x07) << 11) | (((DATA) & 0x07FF)))

enum DRV8323_RW_Cmd_t
{
	DRV8323_CMD_WRITE = 0,
	DRV8323_CMD_READ  = 1
};

enum DRV8323_Reg_Addr_t
{
	DRV8323_ADDR_FAULT_STATUS_REGISTER_1  = 0x00,
	DRV8323_ADDR_FAULT_STATUS_REGISTER_2  = 0x01,
	DRV8323_ADDR_DRIVER_CONTROL_REGISTER  = 0x02,
	DRV8323_ADDR_GATE_DRIVE_HS_REGISTER   = 0x03,
	DRV8323_ADDR_GATE_DRIVE_LS_REGISTER   = 0x04,
	DRV8323_ADDR_OCP_CONTROL_REGISTER     = 0x05,
	DRV8323_ADDR_CSA_CONTROL_REGISTER     = 0x06,
};

/* USER CODE END PD */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void DRV8323_GateDriverConfig(void);
void DRV8323_WriteGateDriver(void);
void DRV8323_ReadGateDriver(void);
/* USER CODE END PFP */


#endif

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

/**
 ******************************************************************************
 * @file		GateDriver.h
 * @author		WrathWings
 * @version 	V1.0
 * @date		2018.3.2
 * @brief		The header file of GateDriver.c
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GateDriver_H
#define __GateDriver_H

/* Private includes ----------------------------------------------------------*/
/* CODE BEGIN Includes */

#include "stm32f4xx_hal.h"
#include "gpio.h"
#include "adc.h"

/* CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#if	GateDriverType == GateDriver_DRV8323
	#define DRV8323_ReadCommandMessageNumber	7
	#define DRV8323_WriteCommandMessageNumber	5

	#define DRV8323_Enable							LL_GPIO_SetOutputPin(GateDriver_Enable_GPIO_Port, GateDriver_Enable_Pin)
	#define DRV8323_Disable							LL_GPIO_ResetOutputPin(GateDriver_Enable_GPIO_Port, GateDriver_Enable_Pin)

	#define DRV8323_SPI3_ChipSelect					LL_GPIO_ResetOutputPin(GateDriver_SPI3_NSS_GPIO_Port, GateDriver_SPI3_NSS_Pin)
	#define DRV8323_SPI3_ChipDiselect				LL_GPIO_SetOutputPin(GateDriver_SPI3_NSS_GPIO_Port, GateDriver_SPI3_NSS_Pin)

	#define DRV8323_CurrentSamplingCorrectStart		LL_GPIO_SetOutputPin(GateDriver_CAL_GPIO_Port, GateDriver_CAL_Pin)
	#define DRV8323_CurrentSamplingCorrectOver		LL_GPIO_ResetOutputPin(GateDriver_CAL_GPIO_Port, GateDriver_CAL_Pin)

	#define DRV8323_SPI_DATA_FORMATTER(RW, ADDR, DATA)    ((((RW) & 0x01) << 15) | (((ADDR) & 0x07) << 11) | (((DATA) & 0x07FF)))

	enum DRV8323_RW_CMD_t
	{
		DRV8323_CMD_WRITE = 0,
		DRV8323_CMD_READ  = 1
	};

	enum DRV8323_RegisterAddress_t
	{
		DRV8323_ADDR_FAULT_STATUS_REGISTER_1  = 0x00,
		DRV8323_ADDR_FAULT_STATUS_REGISTER_2  = 0x01,
		DRV8323_ADDR_DRIVER_CONTROL_REGISTER  = 0x02,
		DRV8323_ADDR_GATE_DRIVE_HS_REGISTER   = 0x03,
		DRV8323_ADDR_GATE_DRIVE_LS_REGISTER   = 0x04,
		DRV8323_ADDR_OCP_CONTROL_REGISTER     = 0x05,
		DRV8323_ADDR_CSA_CONTROL_REGISTER     = 0x06
	};
	
#elif GateDriverType == GateDriver_DRV8320
	#define DRV8320_ReadCommandMessageNumber	6
	#define DRV8320_WriteCommandMessageNumber	5

	#define DRV8320_Enable							LL_GPIO_SetOutputPin(GateDriver_Enable_GPIO_Port, GateDriver_Enable_Pin)
	#define DRV8320_Disable							LL_GPIO_ResetOutputPin(GateDriver_Enable_GPIO_Port, GateDriver_Enable_Pin)

	#define DRV8320_SPI3_ChipSelect					LL_GPIO_ResetOutputPin(GateDriver_SPI3_NSS_GPIO_Port, GateDriver_SPI3_NSS_Pin)
	#define DRV8320_SPI3_ChipDiselect				LL_GPIO_SetOutputPin(GateDriver_SPI3_NSS_GPIO_Port, GateDriver_SPI3_NSS_Pin)

	#define DRV8320_CurrentSamplingCorrectStart		LL_GPIO_SetOutputPin(GateDriver_CAL_GPIO_Port, GateDriver_CAL_Pin)
	#define DRV8320_CurrentSamplingCorrectOver		LL_GPIO_ResetOutputPin(GateDriver_CAL_GPIO_Port, GateDriver_CAL_Pin)

	#define DRV8320_SPI_DATA_FORMATTER(RW, ADDR, DATA)    ((((RW) & 0x01) << 15) | (((ADDR) & 0x07) << 11) | (((DATA) & 0x07FF)))

	enum DRV8320_RW_CMD_t
	{
		DRV8320_CMD_WRITE = 0,
		DRV8320_CMD_READ  = 1
	};

	enum DRV8320_RegisterAddress_t
	{
		DRV8320_ADDR_FAULT_STATUS_REGISTER_1  = 0x00,
		DRV8320_ADDR_FAULT_STATUS_REGISTER_2  = 0x01,
		DRV8320_ADDR_DRIVER_CONTROL_REGISTER  = 0x02,
		DRV8320_ADDR_GATE_DRIVE_HS_REGISTER   = 0x03,
		DRV8320_ADDR_GATE_DRIVE_LS_REGISTER   = 0x04,
		DRV8320_ADDR_OCP_CONTROL_REGISTER     = 0x05
	};
#else
#error "Gate Driver Type Invalid"
#endif

/* USER CODE END PD */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

void GateDriverConfig(void);

#if GateDriverType == GateDriver_DRV8323

	void DRV8323_ReadAllRegister(void);
	void DRV8323_WriteAllRegister(void);
	void DRV8323_ReadFaultStatusRegister(void);

#elif GateDriverType == GateDriver_DRV8320

	void DRV8320_ReadAllRegister(void);
	void DRV8320_WriteAllRegister(void);
	void DRV8320_ReadFaultStatusRegister(void);
	
#else
#error "Gate Driver Type Invalid"
#endif

/* USER CODE END PFP */


#endif

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

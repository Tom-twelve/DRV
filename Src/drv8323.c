/**
 ******************************************************************************
 * @file		drv8323.c
 * @author		WrathWings
 * @version 	V1.0
 * @date		2018.1.17
 * @brief		Set MOSFET the gate driver DRV8323
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
 
 /* Private includes ----------------------------------------------------------*/
/* CODE BEGIN Includes */
#include "drv8323.h"
/* CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* CODE BEGIN PTD */

/* CODE END PTD */

/* Private variables ---------------------------------------------------------*/
/* CODE BEGIN PV */
uint16_t DRV8323_SPI3_RxData[DRV8323_ReadCommandMessageNumber] = {0};

static const uint16_t DRV8323_ReadCommandMessage[DRV8323_ReadCommandMessageNumber] =
{
	DRV8323_SPI_DATA_FORMATTER(DRV8323_CMD_READ,  DRV8323_ADDR_FAULT_STATUS_REGISTER_1, 0x00),
	DRV8323_SPI_DATA_FORMATTER(DRV8323_CMD_READ,  DRV8323_ADDR_FAULT_STATUS_REGISTER_2, 0x00),
	DRV8323_SPI_DATA_FORMATTER(DRV8323_CMD_READ,  DRV8323_ADDR_DRIVER_CONTROL_REGISTER, 0x00),
	DRV8323_SPI_DATA_FORMATTER(DRV8323_CMD_READ,  DRV8323_ADDR_GATE_DRIVE_HS_REGISTER,  0x00),
	DRV8323_SPI_DATA_FORMATTER(DRV8323_CMD_READ,  DRV8323_ADDR_GATE_DRIVE_LS_REGISTER,  0x00),
	DRV8323_SPI_DATA_FORMATTER(DRV8323_CMD_READ,  DRV8323_ADDR_OCP_CONTROL_REGISTER,    0x00),
	DRV8323_SPI_DATA_FORMATTER(DRV8323_CMD_READ,  DRV8323_ADDR_CSA_CONTROL_REGISTER,    0x00),
};

static const uint16_t DRV8323_WriteCommandMessage[DRV8323_WriteCommandMessageNumber] =
{
	DRV8323_SPI_DATA_FORMATTER(DRV8323_CMD_WRITE, DRV8323_ADDR_DRIVER_CONTROL_REGISTER, 0x100),
	DRV8323_SPI_DATA_FORMATTER(DRV8323_CMD_WRITE, DRV8323_ADDR_GATE_DRIVE_HS_REGISTER,  0x366),
	DRV8323_SPI_DATA_FORMATTER(DRV8323_CMD_WRITE, DRV8323_ADDR_GATE_DRIVE_LS_REGISTER,  0x433),
	DRV8323_SPI_DATA_FORMATTER(DRV8323_CMD_WRITE, DRV8323_ADDR_OCP_CONTROL_REGISTER,    0x455),
	DRV8323_SPI_DATA_FORMATTER(DRV8323_CMD_WRITE, DRV8323_ADDR_CSA_CONTROL_REGISTER,    0x2E3),
};
/* CODE END PV */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern struct CurrentSampling_t
{
	uint32_t ADC_CurrentConvertedValue_PhaseA;
	uint32_t ADC_CurrentConvertedValue_PhaseB;
	uint32_t ADC_CurrentConvertedValue_PhaseC;
	float CurrentZeroDrift_PhaseA;
	float CurrentZeroDrift_PhaseB;
	float CurrentZeroDrift_PhaseC;
}CurrentSampling;

/* USER CODE END EV */

/* USER CODE BEGIN */

void DRV8323_GateDriverConfig(void)
{
	DRV8323_Enable;
	HAL_Delay(5);// 没有这个延时ADC会不准
	DRV8323_CurrentSamplingCorrectStart;
	HAL_Delay(1);
	DRV8323_CurrentSamplingCorrectOver;
	DRV8323_SPI3_ChipDiselect;
	HAL_Delay(2);
	
	DRV8323_WriteGateDriver();

	DRV8323_SPI3_ChipDiselect;
	
	HAL_Delay(2);
	
	DRV8323_ReadGateDriver();

}

void DRV8323_WriteGateDriver(void)
{
	for (int i = 0; i < DRV8323_WriteCommandMessageNumber; i++)
	{
		DRV8323_SPI3_ChipSelect;
		
		SPI_Transmit(SPI3, DRV8323_WriteCommandMessage[i], TimeOut);
		
		DRV8323_SPI3_ChipDiselect;
		
		HAL_Delay(1);
	}
}

void DRV8323_ReadGateDriver(void)
{
	for (int i = 0; i < DRV8323_ReadCommandMessageNumber; i++)
	{
		DRV8323_SPI3_ChipSelect;
		
		SPI_Transmit(SPI3, DRV8323_ReadCommandMessage[i], TimeOut);
		
		SPI_Receive(SPI3, DRV8323_SPI3_RxData, TimeOut);
		
		DRV8323_SPI3_ChipDiselect;
		
		HAL_Delay(1);
	}
}

/* USER CODE END */

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

/**
 ******************************************************************************
 * @file		drv8320.c
 * @author		WrathWings
 * @version 	V1.0
 * @date		2018.3.2
 * @brief		Set MOSFET the gate driver DRV8320
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
 
 /* Private includes ----------------------------------------------------------*/
/* CODE BEGIN Includes */
#include "drv8320.h"
/* CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* CODE BEGIN PTD */

/* CODE END PTD */

/* Private variables ---------------------------------------------------------*/
/* CODE BEGIN PV */

uint16_t DRV8320_SPI3_FaultStatusRegister[2] = {0};
uint16_t DRV8320_SPI3_RxData[DRV8320_ReadCommandMessageNumber] = {0};


static const uint16_t DRV8320_ReadCommandMessage[DRV8320_ReadCommandMessageNumber] =
{
	DRV8320_SPI_DATA_FORMATTER(DRV8320_CMD_READ,  DRV8320_ADDR_FAULT_STATUS_REGISTER_1, 0x00),
	DRV8320_SPI_DATA_FORMATTER(DRV8320_CMD_READ,  DRV8320_ADDR_FAULT_STATUS_REGISTER_2, 0x00),
	DRV8320_SPI_DATA_FORMATTER(DRV8320_CMD_READ,  DRV8320_ADDR_DRIVER_CONTROL_REGISTER, 0x00),
	DRV8320_SPI_DATA_FORMATTER(DRV8320_CMD_READ,  DRV8320_ADDR_GATE_DRIVE_HS_REGISTER,  0x00),
	DRV8320_SPI_DATA_FORMATTER(DRV8320_CMD_READ,  DRV8320_ADDR_GATE_DRIVE_LS_REGISTER,  0x00),
	DRV8320_SPI_DATA_FORMATTER(DRV8320_CMD_READ,  DRV8320_ADDR_OCP_CONTROL_REGISTER,    0x00),
};

static const uint16_t DRV8320_WriteCommandMessage[DRV8320_WriteCommandMessageNumber] =
{
	#if MOSFET_Type == CDS18535_63nC_1mOhm6
	DRV8320_SPI_DATA_FORMATTER(DRV8320_CMD_WRITE, DRV8320_ADDR_DRIVER_CONTROL_REGISTER, 0x100),
	DRV8320_SPI_DATA_FORMATTER(DRV8320_CMD_WRITE, DRV8320_ADDR_GATE_DRIVE_HS_REGISTER,  0x366),
	DRV8320_SPI_DATA_FORMATTER(DRV8320_CMD_WRITE, DRV8320_ADDR_GATE_DRIVE_LS_REGISTER,  0x433),
	DRV8320_SPI_DATA_FORMATTER(DRV8320_CMD_WRITE, DRV8320_ADDR_OCP_CONTROL_REGISTER,    0x456),
	#elif MOSFET_Type == IPD053N08N3G_52nC_5mOhm3
	DRV8320_SPI_DATA_FORMATTER(DRV8320_CMD_WRITE, DRV8320_ADDR_DRIVER_CONTROL_REGISTER, 0x100),
	DRV8320_SPI_DATA_FORMATTER(DRV8320_CMD_WRITE, DRV8320_ADDR_GATE_DRIVE_HS_REGISTER,  0x366),
	DRV8320_SPI_DATA_FORMATTER(DRV8320_CMD_WRITE, DRV8320_ADDR_GATE_DRIVE_LS_REGISTER,  0x433),
	DRV8320_SPI_DATA_FORMATTER(DRV8320_CMD_WRITE, DRV8320_ADDR_OCP_CONTROL_REGISTER,    0x45A),
	#endif
};

/* CODE END PV */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* USER CODE BEGIN */

void DRV8320_GateDriverConfig(void)
{
	DRV8320_Enable;
	
	HAL_Delay(2);
	
	DRV8320_SPI3_ChipDiselect;
	
	HAL_Delay(2);
	
	/*设置栅极驱动器*/
	DRV8320_WriteAllRegister();

	DRV8320_SPI3_ChipDiselect;
	
	HAL_Delay(2);
	
	/*读取数据*/
	DRV8320_ReadAllRegister();
	
	/*确认是否设置成功*/
	for (uint8_t i = 0; i < DRV8320_WriteCommandMessageNumber; i++)
	{
		if(DRV8320_ReadCommandMessage[i + 2] != DRV8320_WriteCommandMessage[i])
		{
			DRV8320_WriteAllRegister();
		}
	}
}

void DRV8320_ReadAllRegister(void)
{
	for (uint8_t i = 0; i < DRV8320_ReadCommandMessageNumber; i++)
	{
		DRV8320_SPI3_ChipSelect;

		HAL_SPI_Transmit(&hspi3, (uint8_t*) &DRV8320_ReadCommandMessage[i], 1, TimeOut);

		DRV8320_SPI3_ChipDiselect;

		LL_mDelay(1);
	}
}

void DRV8320_WriteAllRegister(void)
{
	for (uint8_t i = 0; i < DRV8320_WriteCommandMessageNumber; i++)
	{
		DRV8320_SPI3_ChipSelect;
		
		HAL_SPI_Transmit(&hspi3, (uint8_t*) &DRV8320_WriteCommandMessage[i], 1, TimeOut);
		
		DRV8320_SPI3_ChipDiselect;
		
		LL_mDelay(1);
	}
}

void DRV8320_ReadFaultStatusRegister(void)
{
	DRV8320_SPI3_ChipSelect;
	
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*) &DRV8320_ReadCommandMessage[0], (uint8_t*) &DRV8320_SPI3_FaultStatusRegister[0], 1, 2);
	
	DRV8320_SPI3_ChipDiselect;
	
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	
	DRV8320_SPI3_ChipSelect;
	
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*) &DRV8320_ReadCommandMessage[1], (uint8_t*) &DRV8320_SPI3_FaultStatusRegister[1], 1, 2);
	
	DRV8320_SPI3_ChipDiselect;
	
	DMAPRINTF("%d\t",(int)DRV8320_SPI3_FaultStatusRegister[0]);
	DMAPRINTF("%d\r\n",(int)DRV8320_SPI3_FaultStatusRegister[1]);
	
	LL_mDelay(1);
}

/* USER CODE END */

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

/**
 ******************************************************************************
 * @file		GateDriver.c
 * @author		WrathWings
 * @version 	V1.0
 * @date		2018.3.2
 * @brief		Set MOSFET gate driver
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
 
 /* Private includes ----------------------------------------------------------*/
/* CODE BEGIN Includes */

#include "GateDriver.h"

/* CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* CODE BEGIN PTD */

/* CODE END PTD */

/* Private variables ---------------------------------------------------------*/
/* CODE BEGIN PV */
#if GATE_DRIVER_TYPE == GATE_DRIVER_DRV8323
	uint16_t DRV8323_SPI3_RxData[DRV8323_ReadCommandMessageNumber] = {0};
	uint16_t DRV8323_SPI3_FaultStatusRegister[2] = {0};

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
		#if MOSFET_TYPE == CDS18535_63nC_1mOhm6
			DRV8323_SPI_DATA_FORMATTER(DRV8323_CMD_WRITE, DRV8323_ADDR_DRIVER_CONTROL_REGISTER, 0x100),
			DRV8323_SPI_DATA_FORMATTER(DRV8323_CMD_WRITE, DRV8323_ADDR_GATE_DRIVE_HS_REGISTER,  0x366),
			DRV8323_SPI_DATA_FORMATTER(DRV8323_CMD_WRITE, DRV8323_ADDR_GATE_DRIVE_LS_REGISTER,  0x466),
			DRV8323_SPI_DATA_FORMATTER(DRV8323_CMD_WRITE, DRV8323_ADDR_OCP_CONTROL_REGISTER,    0x45A),
			DRV8323_SPI_DATA_FORMATTER(DRV8323_CMD_WRITE, DRV8323_ADDR_CSA_CONTROL_REGISTER,    0x223),
		#elif MOSFET_TYPE == IPD053N08N3G_52nC_5mOhm3
			DRV8323_SPI_DATA_FORMATTER(DRV8323_CMD_WRITE, DRV8323_ADDR_DRIVER_CONTROL_REGISTER, 0x100),
			DRV8323_SPI_DATA_FORMATTER(DRV8323_CMD_WRITE, DRV8323_ADDR_GATE_DRIVE_HS_REGISTER,  0x366),
			DRV8323_SPI_DATA_FORMATTER(DRV8323_CMD_WRITE, DRV8323_ADDR_GATE_DRIVE_LS_REGISTER,  0x466),
			DRV8323_SPI_DATA_FORMATTER(DRV8323_CMD_WRITE, DRV8323_ADDR_OCP_CONTROL_REGISTER,    0x45A),
			DRV8323_SPI_DATA_FORMATTER(DRV8323_CMD_WRITE, DRV8323_ADDR_CSA_CONTROL_REGISTER,    0x2A3),
		#elif MOSFET_TYPE == NCEP0178AK_65nC_8mOhm5
			DRV8323_SPI_DATA_FORMATTER(DRV8323_CMD_WRITE, DRV8323_ADDR_DRIVER_CONTROL_REGISTER, 0x100),
			DRV8323_SPI_DATA_FORMATTER(DRV8323_CMD_WRITE, DRV8323_ADDR_GATE_DRIVE_HS_REGISTER,  0x3FE),
			DRV8323_SPI_DATA_FORMATTER(DRV8323_CMD_WRITE, DRV8323_ADDR_GATE_DRIVE_LS_REGISTER,  0x4FE),
			DRV8323_SPI_DATA_FORMATTER(DRV8323_CMD_WRITE, DRV8323_ADDR_OCP_CONTROL_REGISTER,    0x457),
			DRV8323_SPI_DATA_FORMATTER(DRV8323_CMD_WRITE, DRV8323_ADDR_CSA_CONTROL_REGISTER,    0x2A3),				
		#endif
	};
#elif GATE_DRIVER_TYPE == GATE_DRIVER_DRV8320

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
		#if MOSFET_TYPE == CDS18535_63nC_1mOhm6
			DRV8320_SPI_DATA_FORMATTER(DRV8320_CMD_WRITE, DRV8320_ADDR_DRIVER_CONTROL_REGISTER, 0x100),
			DRV8320_SPI_DATA_FORMATTER(DRV8320_CMD_WRITE, DRV8320_ADDR_GATE_DRIVE_HS_REGISTER,  0x366),
			DRV8320_SPI_DATA_FORMATTER(DRV8320_CMD_WRITE, DRV8320_ADDR_GATE_DRIVE_LS_REGISTER,  0x433),
			DRV8320_SPI_DATA_FORMATTER(DRV8320_CMD_WRITE, DRV8320_ADDR_OCP_CONTROL_REGISTER,    0x45A),
		#elif MOSFET_TYPE == IPD053N08N3G_52nC_5mOhm3
			DRV8320_SPI_DATA_FORMATTER(DRV8320_CMD_WRITE, DRV8320_ADDR_DRIVER_CONTROL_REGISTER, 0x100),
			DRV8320_SPI_DATA_FORMATTER(DRV8320_CMD_WRITE, DRV8320_ADDR_GATE_DRIVE_HS_REGISTER,  0x366),
			DRV8320_SPI_DATA_FORMATTER(DRV8320_CMD_WRITE, DRV8320_ADDR_GATE_DRIVE_LS_REGISTER,  0x433),
			DRV8320_SPI_DATA_FORMATTER(DRV8320_CMD_WRITE, DRV8320_ADDR_OCP_CONTROL_REGISTER,    0x45F),
		#endif
	};
#else
#error "Gate Driver Type Invalid"
#endif

/* CODE END PV */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* USER CODE BEGIN */

void GateDriverConfig(void)
{
	#if GATE_DRIVER_TYPE == GATE_DRIVER_DRV8323
		DRV8323_Enable;
		
		HAL_Delay(5);// 没有这个延时ADC会不准
		
		DRV8323_CurrentSamplingCorrectStart;
		
		HAL_Delay(1);
		
		DRV8323_CurrentSamplingCorrectOver;
		
		DRV8323_SPI3_ChipDiselect;
		
		HAL_Delay(2);
		/*设置栅极驱动器*/
		DRV8323_WriteAllRegister();

		DRV8323_SPI3_ChipDiselect;
		
		HAL_Delay(2);
		/*读取数据*/
		DRV8323_ReadAllRegister();
		
		/*确认是否设置成功*/
		for (uint8_t i = 0; i < DRV8323_WriteCommandMessageNumber; i++)
		{
			if(DRV8323_ReadCommandMessage[i + 2] != DRV8323_WriteCommandMessage[i])
			{
				DRV8323_WriteAllRegister();
			}
		}
		
	#elif GATE_DRIVER_TYPE == GATE_DRIVER_DRV8320
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
	#else
	#error "Gate Driver Type Invalid"
	#endif
}

#if GATE_DRIVER_TYPE == GATE_DRIVER_DRV8323
	void DRV8323_ReadAllRegister(void)
	{
		for (uint8_t i = 0; i < DRV8323_ReadCommandMessageNumber; i++)
		{
			DRV8323_SPI3_ChipSelect;

			HAL_SPI_Transmit(&hspi3, (uint8_t*) &DRV8323_ReadCommandMessage[i], 1, TimeOut);

			DRV8323_SPI3_ChipDiselect;

			LL_mDelay(1);
		}
	}

	void DRV8323_WriteAllRegister(void)
	{
		for (int i = 0; i < DRV8323_WriteCommandMessageNumber; i++)
		{
			DRV8323_SPI3_ChipSelect;
			
			HAL_SPI_Transmit(&hspi3, (uint8_t*) &DRV8323_WriteCommandMessage[i], 1, TimeOut);
			
			DRV8323_SPI3_ChipDiselect;
			
			LL_mDelay(1);
		}
	}

	void DRV8323_ReadFaultStatusRegister(void)
	{
		DRV8323_SPI3_ChipSelect;
		
		HAL_SPI_TransmitReceive(&hspi3, (uint8_t*) &DRV8323_ReadCommandMessage[0], (uint8_t*) &DRV8323_SPI3_FaultStatusRegister[0], 1, 2);
		
		DRV8323_SPI3_ChipDiselect;
		
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		
		DRV8323_SPI3_ChipSelect;
		
		HAL_SPI_TransmitReceive(&hspi3, (uint8_t*) &DRV8323_ReadCommandMessage[1], (uint8_t*) &DRV8323_SPI3_FaultStatusRegister[1], 1, 2);
		
		DRV8323_SPI3_ChipDiselect;
		
		UART_Transmit_DMA("%d\t",(int)DRV8323_SPI3_FaultStatusRegister[0]);
		UART_Transmit_DMA("%d\r\n",(int)DRV8323_SPI3_FaultStatusRegister[1]);
		
		LL_mDelay(1);
	}
#elif GATE_DRIVER_TYPE == GATE_DRIVER_DRV8320
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
		
		UART_Transmit_DMA("%d\t",(int)DRV8320_SPI3_FaultStatusRegister[0]);
		UART_Transmit_DMA("%d\r\n",(int)DRV8320_SPI3_FaultStatusRegister[1]);
		
		LL_mDelay(1);
	}
#else
#error "Gate Driver Type Invalid"
#endif 
/* USER CODE END */

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

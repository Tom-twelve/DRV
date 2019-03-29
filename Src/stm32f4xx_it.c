/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MotorConfig.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

extern struct Encoder_t Encoder;
extern struct SpeedLoop_t SpeedLoop;
extern struct CurrentLoop_t CurrentLoop;
extern struct PositionLoop_t PositionLoop;
extern struct MotorStaticParameter_t MotorStaticParameter;
extern struct MotorDynamicParameter_t MotorDynamicParameter;
extern struct CoordinateTransformation_t CoordinateTransformation;

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles CAN1 RX0 interrupt.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream7 global interrupt.
  */
void DMA2_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream7_IRQn 0 */

  /* USER CODE END DMA2_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA2_Stream7_IRQn 1 */

  /* USER CODE END DMA2_Stream7_IRQn 1 */
}

/* USER CODE BEGIN 1 */
uint16_t temp1 = 0;
uint16_t temp2 = 0;
uint16_t temp3 = 0;

void ADC_IRQHandler(void)
{
	GetPhaseCurrent();
	
	GetPositionImformation();
	
	switch(MotorStaticParameter.ControlMode)
	{
		case VoltageControlMode :	InverseParkTransform_TwoPhase(0.f, 5.0f, &CoordinateTransformation.VoltageAlpha, &CoordinateTransformation.VoltageBeta, Encoder.EleAngle_degree + MotorStaticParameter.PowerAngleCompensation_degree);
									
									SpaceVectorPulseWidthModulation(CoordinateTransformation.VoltageAlpha, CoordinateTransformation.VoltageBeta);

									/*测试用*/
									ParkTransform(MotorDynamicParameter.CurrentPhaseA, MotorDynamicParameter.CurrentPhaseB, MotorDynamicParameter.CurrentPhaseC, &CoordinateTransformation.CurrentD, &CoordinateTransformation.CurrentQ, Encoder.EleAngle_degree);
								
//									CalculateVoltage_dq(CoordinateTransformation.CurrentQ, &CoordinateTransformation.VoltageD, &CoordinateTransformation.VoltageQ, Encoder.AvgEleAngularSpeed_rad);
									
//									CalculateElectromagneticTorque(CoordinateTransformation.CurrentQ, &MotorDynamicParameter.ElectromagneticTorque);
		
//									UART_Transmit_DMA("%d\t",(int)(temp1));
//									UART_Transmit_DMA("%d\t",(int)(temp2));
//									UART_Transmit_DMA("%d\r\n",(int)(temp3));
//									UART_Transmit_DMA("%d\t",(int)(CoordinateTransformation.CurrentD * 1000));
//									UART_Transmit_DMA("%d\r\n",(int)(CoordinateTransformation.CurrentQ * 1000));
		
									break;

		case CurrentControlMode : 	/*进行Park变换, 将三相电流转换为dq轴电流*/
									ParkTransform(MotorDynamicParameter.CurrentPhaseA, MotorDynamicParameter.CurrentPhaseB, MotorDynamicParameter.CurrentPhaseC, &CoordinateTransformation.CurrentD, &CoordinateTransformation.CurrentQ, Encoder.EleAngle_degree + MotorStaticParameter.PowerAngleCompensation_degree);
									
									/*电流环PI控制器*/
									CurrentLoopController(CurrentLoop.ExpectedCurrentD, CurrentLoop.ExpectedCurrentQ, CoordinateTransformation.CurrentD, CoordinateTransformation.CurrentQ, &CurrentLoop.ControlVoltageD, &CurrentLoop.ControlVoltageQ);
													
									/*进行逆Park变换, 将转子坐标系下的dq轴电压转换为定子坐标系下的AlphaBeta轴电压*/
									InverseParkTransform_TwoPhase(CurrentLoop.ControlVoltageD, CurrentLoop.ControlVoltageQ, &CoordinateTransformation.VoltageAlpha, &CoordinateTransformation.VoltageBeta, Encoder.EleAngle_degree + MotorStaticParameter.PowerAngleCompensation_degree);
									
									/*利用SVPWM算法调制电压矢量*/
									SpaceVectorPulseWidthModulation(CoordinateTransformation.VoltageAlpha, CoordinateTransformation.VoltageBeta);
									
									UART_Transmit_DMA("%d\t",(int)(CoordinateTransformation.CurrentD * 1000));
									UART_Transmit_DMA("%d\r\n",(int)(CoordinateTransformation.CurrentQ * 1000));
									
									break;
		case SpeedControlMode :
								
								break;
		case PositionControlMode :	

									break;
	}
	
	__HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC);
	__HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_JEOC);
	__HAL_ADC_CLEAR_FLAG(&hadc3, ADC_FLAG_JEOC);
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DRV8323_SOC_Pin LL_GPIO_PIN_1
#define DRV8323_SOC_GPIO_Port GPIOC
#define DRV8323_SOB_Pin LL_GPIO_PIN_2
#define DRV8323_SOB_GPIO_Port GPIOC
#define DRV8323_SOA_Pin LL_GPIO_PIN_3
#define DRV8323_SOA_GPIO_Port GPIOC
#define IncrementalInterfaceB_Pin LL_GPIO_PIN_1
#define IncrementalInterfaceB_GPIO_Port GPIOA
#define IncrementalInterfaceA_Pin LL_GPIO_PIN_2
#define IncrementalInterfaceA_GPIO_Port GPIOA
#define DRV8323_CAL_Pin LL_GPIO_PIN_8
#define DRV8323_CAL_GPIO_Port GPIOA
#define DRV8323_SPI3_NSS_Pin LL_GPIO_PIN_15
#define DRV8323_SPI3_NSS_GPIO_Port GPIOA
#define DRV8323_SPI3_SCK_Pin LL_GPIO_PIN_10
#define DRV8323_SPI3_SCK_GPIO_Port GPIOC
#define DRV8323_SPI3_MISO_Pin LL_GPIO_PIN_11
#define DRV8323_SPI3_MISO_GPIO_Port GPIOC
#define DRV8323_SPI3_MOSI_Pin LL_GPIO_PIN_12
#define DRV8323_SPI3_MOSI_GPIO_Port GPIOC
#define Encoder_SPI1_NSS_Pin LL_GPIO_PIN_2
#define Encoder_SPI1_NSS_GPIO_Port GPIOD
#define Encoder_SPI1_SCK_Pin LL_GPIO_PIN_3
#define Encoder_SPI1_SCK_GPIO_Port GPIOB
#define Encoder_SPI1_MISO_Pin LL_GPIO_PIN_4
#define Encoder_SPI1_MISO_GPIO_Port GPIOB
#define Encoder_SPI1_MOSI_Pin LL_GPIO_PIN_5
#define Encoder_SPI1_MOSI_GPIO_Port GPIOB
#define DRV8323_Enable_Pin LL_GPIO_PIN_6
#define DRV8323_Enable_GPIO_Port GPIOB
#define DRV8323_nFault_Pin LL_GPIO_PIN_8
#define DRV8323_nFault_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
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

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA2_Stream7;
    hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

static char  USART_BUFF1[USART_BUF_CAPACITY] = { 0u };
static char  USART_BUFF2[USART_BUF_CAPACITY] = { 0u };
static char *USART_BuffNow                   = USART_BUFF1;
char *pBuf                            = USART_BUFF1;

void SendBuf(void)
{
	HAL_StatusTypeDef status;
	status = HAL_UART_Transmit_DMA(&DEBUG_UART_HANDLER, (uint8_t*)USART_BuffNow, pBuf - USART_BuffNow); //使能DMA发数1us
	if(status == HAL_OK)
	{
		USART_BuffNow = (USART_BuffNow == USART_BUFF1) ? USART_BUFF2 : USART_BUFF1;
		pBuf          = USART_BuffNow;
	}
	else
	{
		USART_BuffNow = (USART_BuffNow == USART_BUFF1) ? USART_BUFF2 : USART_BUFF1;
		pBuf          = USART_BuffNow;
		PutStr("DMA_Buffer_Overflow\r\n");
	}
}

void SendBufWaitWhenSending(void)
{
	HAL_StatusTypeDef status;
  int i=0;
  do
  {
    status = HAL_UART_Transmit_DMA(&DEBUG_UART_HANDLER, (uint8_t*)USART_BuffNow, pBuf - USART_BuffNow); //使能DMA发数1us
    
    if(status == HAL_OK)
    {
      break;
    }
    else
    {
      /*
      if status != HAL_OK, which means a Tx process is already ongoing, Wait 1ms and retry.
      break the loop when retry 2 times.
      */
      HAL_Delay(1);
      if(i++ > 2)
      {
        PutStr("DMA_Buffer_Overflow\r\n");
        break;
      }
    }
  }while(1);

  USART_BuffNow = (USART_BuffNow == USART_BUFF1) ? USART_BUFF2 : USART_BUFF1;
  pBuf          = USART_BuffNow;
}

uint32_t SendBufWhenFull(void)
{
	uint32_t bufSize = pBuf - USART_BuffNow;
	
	if (bufSize >= USART_BUF_SEND_THESHOLD)
	{                                                         //使能DMA发数
		SendBuf();
		if (bufSize > USART_BUF_CAPACITY)
		{
			PutStr("OVERFLOW");
			return 2u; //when it overflows
		}
		return 1u;
	}
	return 0u;
}



void USART_BufPush(char ch)
{
	*pBuf++ = ch;
	SendBufWhenFull();
}

static uint32_t t_pow(uint32_t x, uint32_t n)
{
	uint32_t  i       = 0u;
	int32_t   result  = 1u;

	for (i = 0u; i < n; i++)
		result = result * x;
	return result;
}

/**
 * @brief	把整型数转换为字符串
 * @note		注意此函数是有最高位数限制的
 * @param	value: 要转换的数字
 * @param	*str: 转换出的字符串输出的地址
 * @retval	strSize:
 */
static inline int IntToAscii(int value, char *str)
{
	int   i       = 0u;
	int   strSize = 0u;
	char  digit   = 0u;
	int   notMSB  = 0u;

	/*****   判断符号   *****/
	if (value < 0)
	{
		str[strSize++]  = '-';
		value           = -value;
	}
	else if (value == 0)
	{
		str[strSize++] = '0';
		return strSize;
	}

	digit = value / t_pow(10, NUM_SIZE - i);
	if (digit > 10u)
		str[strSize++] = '!';
	/*******     分解    ********/
	for (i = 0; i <= NUM_SIZE; i++)
	{
		digit = value / t_pow(10, NUM_SIZE - i) % 10;
		if (digit != 0)
		{
			str[strSize++]  = digit + '0';
			notMSB          = 1u;
		}
		else if (notMSB == 1u)
		{
			str[strSize++] = '0';
		}
	}
	return strSize;
}


/**************串口DMA发数*****************/
void PutNum(int data, char endSign)
{
	pBuf    += IntToAscii(data, pBuf);
	*pBuf++ = endSign;
	SendBufWhenFull();
}

void PutNumSign(int data, char endSign1, char endSign2)
{
	pBuf    += IntToAscii(data, pBuf);
	*pBuf++ = endSign1;
	*pBuf++ = endSign2;
	SendBufWhenFull();
}

uint32_t PutStr(const char *str)
{
	const char *pBufStart = pBuf;

	while (*str != 0)
	{
		*pBuf++ = *str++;
	}
	SendBufWhenFull();
	return pBuf - pBufStart;
}

void UART_Enable(void)
{
	__HAL_UART_ENABLE(&huart1);
}



/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

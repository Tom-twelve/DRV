/**
  ******************************************************************************
  * File Name          : USART.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */
extern char *pBuf;

#define NUM_SIZE                    15u
#define USART_BUF_CAPACITY          2000u
#define USART_BUF_SEND_THESHOLD     (USART_BUF_CAPACITY - NUM_SIZE - 30)
#define DEBUG_UART_HANDLER huart1

#define UART_Transmit_DMA(__RESTRICT_STR, ...)                     \
  do                                                       \
  {                                                        \
    static uint32_t __i_divSendTimes = 0;                  \
    if (__i_divSendTimes++ < 1)                            \
    {                                                      \
      __i_divSendTimes = 0;                                \
      pBuf += _sprintf(pBuf, __RESTRICT_STR, __VA_ARGS__); \
    }                                                      \
    SendBufWhenFull();                                 \
  } while (0)

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */
  
void SendBuf(void);
void SendBufWaitWhenSending(void);
uint32_t SendBufWhenFull(void);
void USART_BufPush(char ch);
static inline int IntToAscii(int value, char *str);
void PutNum(int data, char endSign);
uint32_t PutStr(const char *str);
void UART_Enable(void);

void USARTDMASend(USART_TypeDef* USARTx,uint8_t *buffAddr,uint8_t *sendBufAddr,uint16_t *buffPointer);
void USARTDMAOUT(USART_TypeDef* USARTx,uint8_t *buffAddr,uint16_t *buffPointer ,uint8_t *sendBufAddr,uint16_t bufferSize,const uint8_t *Data, ...);
void USARTDMASendData(USART_TypeDef* USARTx,uint8_t data,uint8_t *buffAddr,uint16_t *buffPointer ,uint8_t *sendBufAddr,uint16_t bufferSize);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

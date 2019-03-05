/**
  ******************************************************************************
  * File Name          : CAN.c
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */
#ifndef CAN_ID_NUM
 #error "CAN_ID_NUMBER is not undefined."
#endif

uint32_t CAN_RecieveStatus = 0u;
CAN_TxHeaderTypeDef TxMessage = { 0 };
CAN_RxHeaderTypeDef RxMessage0 = { 0 };
static uint32_t mbox;

extern struct Encoder_t Encoder;
extern struct SpeedLoop_t SpeedLoop;
extern struct PositionLoop_t PositionLoop;
extern struct MotorStaticParameter_t MotorStaticParameter;
extern struct CoordinateTransformation_t CoordinateTransformation;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration    
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();
  
    /**CAN1 GPIO Configuration    
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

#include "usart.h"

void CANSendData(canData_t data)
{
	TxMessage.StdId = CAN_SDO_CLIENT_STD_ID;
	TxMessage.ExtId = 0u;
	TxMessage.IDE   = CAN_ID_STD;
	TxMessage.RTR   = CAN_RTR_DATA;
	TxMessage.DLC   = 8u;
//	for (uint32_t i = 0u; i < 8; i++)
//		TxMessage.Data[i] = data.data_uint8[i];
	HAL_CAN_AddTxMessage(&hcan1, &TxMessage, (uint8_t *)&data, &mbox);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);
//	HAL_CAN_Transmit_IT(&hcan1);
//	PutStr("T");
//	PutNum(HAL_GetTick(), '\t');
}


void CANRespond(void)
{
	canData_t txData;

	switch (CAN_RecieveStatus)
	{
	case 0x40005155: //UQ   读取电压输出(V)
		txData.data_uint32[0] = 0x00005155;
		txData.data_float[1]  = CoordinateTransformation.VoltageQ;
		CANSendData(txData);
		CAN_RecieveStatus = 0;
		break;

	case 0x40005856: //VX   读取速度
	{
#if EncoderType == Encoder_TLE5012
		const float velReport = (Encoder.MechanicalAngle_degree /360.f * 32768.f);
		txData.data_uint32[0] = 0x00005856;
		txData.data_int32[1]  = (int32_t)(velReport);
		CANSendData(txData);
		CAN_RecieveStatus = 0;
#endif
		break;
	}
	case 0x40005149: //IQ	 读取转矩电流
		txData.data_uint32[0] = 0x00005149;
		txData.data_uint32[1] = (uint32_t)CoordinateTransformation.CurrentQ;
		CANSendData(txData);
		CAN_RecieveStatus = 0;
		break;

	case 0x40005850: //PX   读取位置
		txData.data_uint32[0] = 0x00005850;
		
		txData.data_int32[1]  = (int32_t)(Encoder.MechanicalAngle_degree /360.f * 32768.f) ;

		CANSendData(txData);
		CAN_RecieveStatus = 0;
		break;
	

	default:
		break;
	}
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	CANRespond();
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
	CANRespond();
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
	CANRespond();
};

uint32_t    StdId;
volatile canData_t receive;
volatile static float value_checkout = 0;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	//此处尚不知道如何区分 FIFO0 FIFO1
	
	first_line:
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxMessage0, (uint8_t *)&receive);
	StdId   = RxMessage0.StdId;

	if (StdId == CAN_SDO_SERVER_STD_ID)
	{
		switch (receive.data_uint32[0])
		{
		case 0x00004F4D: //MO
			if (receive.data_uint32[1] == 1)
			{
				SpeedLoop.ExpectedMechanicalAngularSpeed = 0.f;
				PWM_IT_CMD(ENABLE,ENABLE);
			}
			else
			{
				PWM_IT_CMD(DISABLE,ENABLE);
				SpeedLoop.ExpectedMechanicalAngularSpeed = 0.0f;
			}
			break;
		case 0x00004D55://先配置，再切换模式  //UM
			if(receive.data_int32[1] == SpeedControlMode)
			{
//				SpeedLoop.Kp = VEL_CONTROL_KP;
//				SpeedLoop.Ki = VEL_CONTROL_KI;
				MotorStaticParameter.ControlMode = SpeedControlMode;
			}
			else if(receive.data_int32[1] == PositionControlMode)
			{
//				PositionLoop.Kp = POS_CONTROL_KP;
//				PositionLoop.Kd = POS_CONTROL_KD;
				MotorStaticParameter.ControlMode = PositionControlMode;
				PositionLoop.ExpectedMechanicalAngle = Encoder.MechanicalAngle_degree + Encoder.AverageMechanicalAngularSpeed_degree * CarrierPeriod_s;
			}
			break;
		case 0x0000564A: //JV


		//value_checkout 
//		SpeedLoop.ExpectedMechanicalAngularSpeed =  -1 * SATURATION((float)(receive.data_int32[1]), -VEL_MAX, VEL_MAX)  * 0.000191650390625f;//主控以顺时针方向为正 
			break;

		case 0x00004341: //AC

			SpeedLoop.Acceleration =  (float)(receive.data_int32[1]) * 0.000191650390625f;
			
			break;

//		case 0x00005053: //SP
//			Driver.PosCtrl.DesiredVel = SATURATION((float)(receive.data_int32[1]) * 0.000191650390625f, -VEL_MAX, VEL_MAX);// 2 * PI / 32768
//			break;

		case 0x00004150: //PA绝对位置
			PositionLoop.ExpectedMechanicalAngle = -1 * (float)(receive.data_int32[1]);
			break;

		case 0x00005250: //PR相对位置
			PositionLoop.ExpectedMechanicalAngle = Encoder.MechanicalAngle_degree + -1 * (float)(receive.data_int32[1]);
			break;
		case 0x00005100:
			Encoder.MechanicalAngle_degree = receive.data_int32[1];
			break;
			
		case 0x40005155: //UQ  读取电压输出(mV)
			CAN_RecieveStatus = 0x40005155;
			break;

		case 0x40005149: //IQ	 读取电流
			CAN_RecieveStatus = 0x40005149;
			
			break;

		case 0x40005856: //VX   读取速度
			CAN_RecieveStatus = 0x40005856;
			break;

		case 0x40005850: //PX   读取位置
			CAN_RecieveStatus = 0x40005850;
			
			break; 
		default:
			break;
		}
	}
	else if (StdId == (0x300 + GROUP_NUM))
	{
		switch (receive.data_uint32[0])
		{
		case 0x40005155: //UQ  读取电压输出(mV)
			CAN_RecieveStatus = 0x40005155;
			break;

		case 0x40005149: //IQ	 读取电流
			CAN_RecieveStatus = 0x40005149;
			break;

		case 0x40005856: //VX   读取速度
			CAN_RecieveStatus = 0x40005856;
			break;

		case 0x40005850: //PX   读取位置
			CAN_RecieveStatus = 0x40005850;
			break;
		
		default:
			break;
		}
	}
	CANRespond();
	if(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0))
	{
		goto first_line;
	}
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	static uint16_t errTimes = 0;
	canData_t txData;
	PutStr("CAN Error Code:");
	PutNum(hcan->ErrorCode, '\n');
	txData.data_uint32[0] = 0x00111111;
	txData.data_uint32[1] = hcan->ErrorCode;
	//CANSendData(txData);
	if(errTimes++ <= 100)
	{
		errTimes++;
		HAL_CAN_DeInit(&hcan1);
		HAL_CAN_Init(&hcan1);
		__HAL_CAN_CLEAR_FLAG(&hcan1, CAN_FLAG_FOV0);
		HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
		CANSendData(txData);
	}
	else
	{
		PWM_IT_CMD(DISABLE,ENABLE);
	}
	SendBuf();
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

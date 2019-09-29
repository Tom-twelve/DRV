/**
  ******************************************************************************
  * File Name          : CAN.c
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#ifndef CAN_ID_NUM
 #error "CAN_ID_NUMBER is not undefined."
#endif

CAN_TxHeaderTypeDef TxMessage = { 0 };
CAN_RxHeaderTypeDef RxMessage0 = { 0 };

struct CAN_t CAN;

extern struct Driver_t Driver;
extern struct CurrLoop_t CurrLoop;
extern struct SpdLoop_t SpdLoop;
extern struct PosLoop_t PosLoop;
extern struct CoordTrans_t CoordTrans;
extern struct PosSensor_t PosSensor;
extern struct Regulator_t Regulator;
extern struct MainController_t MainController;

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_8TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
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
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
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
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	/*此处尚不知道如何区分 FIFO0 FIFO1*/
	
	first_line:
	
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxMessage0, (uint8_t *)&CAN.Receive);
	
	CAN.StdID = RxMessage0.StdId;
	
	CAN.Identifier = CAN.Receive.data_uint32[0] & 0xFFFF;
	
	CAN.ReceiveData = ((CAN.Receive.data_uint32[0] >> 16) | (CAN.Receive.data_uint32[1] << 16)) & 0xFFFFFFFF;
	
	/*ACTION驱动器指令*/
	if (CAN.StdID == DRIVER_SERVER_CAN_ID)
	{
		switch(CAN.Identifier)
		{
			case 0x4F4D: //MO
				
				if (CAN.ReceiveData == 0x00000001)
				{
					/*使能*/
					SpdLoop.ExptMecAngularSpeed_rad = 0.f;
					PWM_IT_CMD(ENABLE,ENABLE);
				}
				else
				{
					/*失能*/
					SpdLoop.ExptMecAngularSpeed_rad = 0.f;
					PWM_IT_CMD(DISABLE,ENABLE);
				}
				
				break;
				
			case 0x4D55: //UM
				
				if(CAN.ReceiveData == SPD_CURR_CTRL_MODE)
				{
					Driver.ControlMode = SPD_CURR_CTRL_MODE;
				}
				else if(CAN.ReceiveData == POS_SPD_CURR_CTRL_MODE)
				{
					Driver.ControlMode = POS_SPD_CURR_CTRL_MODE;
					PosLoop.ExptMecAngle_rad = PosSensor.MecAngle_rad + PosSensor.MecAngularSpeed_rad * Regulator.ActualPeriod_s;
				}
				
				break;
				
			case 0x564A: //JV
				
				/*期望速度*/
				Saturation_int((int*)&CAN.ReceiveData, MAX_SPD, -MAX_SPD);
				MainController.ExptMecAngularSpeed_pulse =  CAN.Receive.data_int32[1];
				SpdLoop.ExptMecAngularSpeed_rad = PULSE_TO_RAD(MainController.ExptMecAngularSpeed_pulse);
			
				break;

			case 0x4341: //AC
				
				/*设置加速度*/
				MainController.Acceleration_pulse = CAN.ReceiveData;
				SpdLoop.Acceleration = PULSE_TO_RAD(MainController.Acceleration_pulse);
				
				break;

			case 0x4344: //DC

				/*设置减速度*/
				MainController.Deceleration_pulse = CAN.ReceiveData;
				SpdLoop.Deceleration = PULSE_TO_RAD(MainController.Deceleration_pulse);

				break;
					
			case 0x5053: //SP
				
				/*设置位置环最大输出速度*/
				Saturation_int((int*)&CAN.ReceiveData, MAX_SPD, -MAX_SPD);
				MainController.MaxMecAngularSpeed_pulse = CAN.ReceiveData;
				PosLoop.MaxMecAngularSpeed_rad = PULSE_TO_RAD(MainController.MaxMecAngularSpeed_pulse);
				
				break;

			case 0x4150: //PA
			
				/*期望位置, 绝对位置模式*/
				MainController.ExptMecAngle_pulse = CAN.ReceiveData;
			
				break;

			case 0x5250: //PR
				
				/*期望位置, 相对位置模式*/
				MainController.ExptMecAngle_pulse = MainController.RefMecAngle_pulse + CAN.ReceiveData;
			
				break;

			case (0x4000 + 0x4449): //ID
				
				/*读取Id*/
				CAN.RecieveStatus = (0x4000 + 0x4449);
				
				break;
			case (0x4000 + 0x5149): //IQ
				
				/*读取Iq*/
				CAN.RecieveStatus = (0x4000 + 0x5149);
				
				break;
			
			case (0x4000 + 0x4455): //UD
				
				/*读取Vd*/
				CAN.RecieveStatus = (0x4000 + 0x4455);
			
				break;

			case (0x4000 + 0x5155): //UQ
				
				/*读取Vq*/
				CAN.RecieveStatus = (0x4000 + 0x5155);
			
				break;
						
			case (0x4000 + 0x5856): //VX
				
				/*读取速度*/
				CAN.RecieveStatus = (0x4000 + 0x5856);
			
				break;

			case (0x4000 + 0x5850): //PX
				
				/*读取位置*/
				CAN.RecieveStatus = (0x4000 + 0x5850);
		
				break; 
			
			default:
				
				break;
		}	
	}
	else if(CAN.StdID == DRIVER_BROADCAST_ID)
	{
		switch(CAN.Identifier)
		{
			case (0x4000 + 0x4449): //ID
				
				/*读取Id*/
				CAN.RecieveStatus = (0x4000 + 0x4449);
				
				break;
			
			case (0x4000 + 0x5149): //IQ
				
				/*读取Iq*/
				CAN.RecieveStatus = (0x4000 + 0x5149);
				
				break;
			
			case (0x4000 + 0x4455): //UD
				
				/*读取Vd*/
				CAN.RecieveStatus = (0x4000 + 0x4455);
			
				break;

			case (0x4000 + 0x5155): //UQ
				
				/*读取Vq*/
				CAN.RecieveStatus = (0x4000 + 0x5155);
			
				break;
						
			case (0x4000 + 0x5856): //VX
				
				/*读取速度*/
				CAN.RecieveStatus = (0x4000 + 0x5856);
			
				break;

			case (0x4000 + 0x5850): //PX
				
				/*读取位置*/
				CAN.RecieveStatus = (0x4000 + 0x5850);
		
				break; 
			
			default:
				
				break;
		}
	}
	
	CAN_Respond();
	
	/*检测RX FIFO剩余的信息数量*/
	if(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0))
	{
		goto first_line;
	}
}

void CAN_Respond(void)
{
	switch (CAN.RecieveStatus)
	{
		case (0x4000 + 0x5856): //VX
			
			CAN.Identifier = 0x5856;
			CAN.TransmitData = (int32_t)(PosSensor.MecAngularSpeed_rad /(2.f * PI) * TLE5012_ABS_MODE_RESOLUTION);
		
			/*发送当前速度*/
			CAN.Transmit.data_uint8[0] = (CAN.Identifier>>0)&0xff;
			CAN.Transmit.data_uint8[1] = (CAN.Identifier>>8)&0xff;
			CAN.Transmit.data_uint8[2] = (CAN.TransmitData>>0)&0xff;
			CAN.Transmit.data_uint8[3] = (CAN.TransmitData>>8)&0xff;
			CAN.Transmit.data_uint8[4] = (CAN.TransmitData>>16)&0xff;
			CAN.Transmit.data_uint8[5] = (CAN.TransmitData>>24)&0xff;
		
			CAN_Transmit(CAN.Transmit, 6);
		
			CAN.RecieveStatus = 0;
		
			break;

		case (0x4000 + 0x5850): //PX
			
			CAN.Identifier = 0x5850;
			CAN.TransmitData = (int32_t)MainController.RefMecAngle_pulse;
		
			/*发送当前位置*/
			CAN.Transmit.data_uint8[0] = (CAN.Identifier>>0)&0xff;
			CAN.Transmit.data_uint8[1] = (CAN.Identifier>>8)&0xff;
			CAN.Transmit.data_uint8[2] = (CAN.TransmitData>>0)&0xff;
			CAN.Transmit.data_uint8[3] = (CAN.TransmitData>>8)&0xff;
			CAN.Transmit.data_uint8[4] = (CAN.TransmitData>>16)&0xff;
			CAN.Transmit.data_uint8[5] = (CAN.TransmitData>>24)&0xff;
		
			CAN_Transmit(CAN.Transmit, 6);
		
			CAN.RecieveStatus = 0;
		
			break;
		
		case (0x4000 + 0x5445): //ET
			
			CAN.Identifier = 0x5445;
			CAN.TransmitData = (int32_t)(Driver.EleTorque * 1e3);
		
			/*发送当前电磁转矩*/	
			CAN.Transmit.data_uint8[0] = (CAN.Identifier>>0)&0xff;
			CAN.Transmit.data_uint8[1] = (CAN.Identifier>>8)&0xff;
			CAN.Transmit.data_uint8[2] = (CAN.TransmitData>>0)&0xff;
			CAN.Transmit.data_uint8[3] = (CAN.TransmitData>>8)&0xff;
			CAN.Transmit.data_uint8[4] = (CAN.TransmitData>>16)&0xff;
			CAN.Transmit.data_uint8[5] = (CAN.TransmitData>>24)&0xff;
		
			CAN_Transmit(CAN.Transmit, 6);
		
			CAN.RecieveStatus = 0;
		
			break;
				
		case (0x4000 + 0x4455): //UD
			
			CAN.Identifier = 0x4455;
			CAN.TransmitData = (int32_t)(CurrLoop.CtrlVolD * 1e3);
		
			/*发送当前Vd*/	
			CAN.Transmit.data_uint8[0] = (CAN.Identifier>>0)&0xff;
			CAN.Transmit.data_uint8[1] = (CAN.Identifier>>8)&0xff;
			CAN.Transmit.data_uint8[2] = (CAN.TransmitData>>0)&0xff;
			CAN.Transmit.data_uint8[3] = (CAN.TransmitData>>8)&0xff;
			CAN.Transmit.data_uint8[4] = (CAN.TransmitData>>16)&0xff;
			CAN.Transmit.data_uint8[5] = (CAN.TransmitData>>24)&0xff;
		
			CAN_Transmit(CAN.Transmit, 6);
		
			CAN.RecieveStatus = 0;
		
			break;
				
		case (0x4000 + 0x5155): //UQ
			
			CAN.Identifier = 0x5155;
			CAN.TransmitData = (int32_t)(CurrLoop.CtrlVolQ * 1e3);
		
			/*发送当前Vq*/	
			CAN.Transmit.data_uint8[0] = (CAN.Identifier>>0)&0xff;
			CAN.Transmit.data_uint8[1] = (CAN.Identifier>>8)&0xff;
			CAN.Transmit.data_uint8[2] = (CAN.TransmitData>>0)&0xff;
			CAN.Transmit.data_uint8[3] = (CAN.TransmitData>>8)&0xff;
			CAN.Transmit.data_uint8[4] = (CAN.TransmitData>>16)&0xff;
			CAN.Transmit.data_uint8[5] = (CAN.TransmitData>>24)&0xff;
		
			CAN_Transmit(CAN.Transmit, 6);
		
			CAN.RecieveStatus = 0;
		
			break;

		case (0x4000 + 0x4449): //ID
			
			CAN.Identifier = 0x4449;
			CAN.TransmitData = (int32_t)(CoordTrans.CurrD * 1e3);
		
			/*发送当前Id*/	
			CAN.Transmit.data_uint8[0] = (CAN.Identifier>>0)&0xff;
			CAN.Transmit.data_uint8[1] = (CAN.Identifier>>8)&0xff;
			CAN.Transmit.data_uint8[2] = (CAN.TransmitData>>0)&0xff;
			CAN.Transmit.data_uint8[3] = (CAN.TransmitData>>8)&0xff;
			CAN.Transmit.data_uint8[4] = (CAN.TransmitData>>16)&0xff;
			CAN.Transmit.data_uint8[5] = (CAN.TransmitData>>24)&0xff;
		
			CAN_Transmit(CAN.Transmit, 6);
		
			CAN.RecieveStatus = 0;
		
			break;
				
		case (0x4000 + 0x5149): //IQ
			
			CAN.Identifier = 0x5149;
			CAN.TransmitData = (int32_t)(CoordTrans.CurrQ * 1e3);
		
			/*发送当前Iq*/		
			CAN.Transmit.data_uint8[0] = (CAN.Identifier>>0)&0xff;
			CAN.Transmit.data_uint8[1] = (CAN.Identifier>>8)&0xff;
			CAN.Transmit.data_uint8[2] = (CAN.TransmitData>>0)&0xff;
			CAN.Transmit.data_uint8[3] = (CAN.TransmitData>>8)&0xff;
			CAN.Transmit.data_uint8[4] = (CAN.TransmitData>>16)&0xff;
			CAN.Transmit.data_uint8[5] = (CAN.TransmitData>>24)&0xff;
		
			CAN_Transmit(CAN.Transmit, 6);
		
			CAN.RecieveStatus = 0;
		
			break;
				
		default:
			
			break;
	}
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	CAN_Respond();
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
	CAN_Respond();
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
	CAN_Respond();
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	static uint16_t errTimes = 0;
	
	PutStr("CAN Error Code:");
	PutNum(hcan->ErrorCode, '\n');SendBuf();
	
	CAN.Identifier = 0x5245;	//ER
	CAN.TransmitData = hcan->ErrorCode;
	
	CAN.Transmit.data_uint8[0] = (CAN.Identifier>>0)&0xff;
	CAN.Transmit.data_uint8[1] = (CAN.Identifier>>8)&0xff;
	CAN.Transmit.data_uint8[2] = (CAN.TransmitData>>0)&0xff;
	CAN.Transmit.data_uint8[3] = (CAN.TransmitData>>8)&0xff;
	CAN.Transmit.data_uint8[4] = (CAN.TransmitData>>16)&0xff;
	CAN.Transmit.data_uint8[5] = (CAN.TransmitData>>24)&0xff;

	if(errTimes++ <= 100)
	{
		errTimes++;
		
		HAL_CAN_DeInit(&hcan1);
		HAL_CAN_Init(&hcan1);
		
		__HAL_CAN_CLEAR_FLAG(&hcan1, CAN_FLAG_FOV0);
		
		HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
		
		CAN_Transmit(CAN.Transmit, 6);
	}
	else
	{
		PWM_IT_CMD(DISABLE,ENABLE);
	}
}

void CAN_Transmit(CAN_Data_t data, uint8_t length)
{
	TxMessage.StdId = DRIVER_CLIENT_CAN_ID;
	TxMessage.ExtId = 0u;
	TxMessage.IDE   = CAN_ID_STD;
	TxMessage.RTR   = CAN_RTR_DATA;
	TxMessage.DLC   = length;
	
	HAL_CAN_AddTxMessage(&hcan1, &TxMessage, (uint8_t *)&data, &CAN.MailBox);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);
}

void CAN_Enable(void)
{
	/*配置过滤器*/
	CAN_FilterTypeDef CAN1_FilerConf    = {0};
	CAN1_FilerConf.FilterIdHigh         = 0X0000;
	CAN1_FilerConf.FilterIdLow          = 0X0000;
	CAN1_FilerConf.FilterMaskIdHigh     = 0X0000;
	CAN1_FilerConf.FilterMaskIdLow      = 0X0000;
	CAN1_FilerConf.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	CAN1_FilerConf.FilterBank           = 0;
	CAN1_FilerConf.FilterMode           = CAN_FILTERMODE_IDMASK;
	CAN1_FilerConf.FilterScale          = CAN_FILTERSCALE_32BIT;
	CAN1_FilerConf.FilterActivation     = ENABLE;
	CAN1_FilerConf.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan1, &CAN1_FilerConf);
		
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_LAST_ERROR_CODE);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_BUSOFF);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_ERROR);
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

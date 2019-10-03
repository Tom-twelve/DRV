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
	first_line:
	
	CAN_Receive(&CAN.StdID, &CAN.Identifier, &CAN.ReceiveData);
	
	/*ACTION驱动器指令*/
	/*点对点模式*/
	if (CAN.StdID == DRIVER_SERVER_CAN_ID)
	{
		switch(CAN.Identifier)
		{
			case 0x01:
				
				if (CAN.ReceiveData == 0x00000001)
				{
					/*使能*/
					SpdLoop.ExptMecAngularSpeed_rad = 0.f;
					PWM_IT_CMD(ENABLE,ENABLE);
				}
				else if(CAN.ReceiveData == 0x00000000)
				{
					/*失能*/
					SpdLoop.ExptMecAngularSpeed_rad = 0.f;
					PWM_IT_CMD(DISABLE,ENABLE);
				}
				
				break;
				
			case 0x02:
				
				/*期望速度*/
				MainController.ExptMecAngularSpeed_pulse =  MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
				SpdLoop.ExptMecAngularSpeed_rad = DRV_PULSE_TO_RAD(MainController.ExptMecAngularSpeed_pulse);
			
				break;
			
			case 0x03:
			
				/*期望位置, 绝对位置模式*/
				MainController.ExptMecAngle_pulse = MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
			
				break;

			case 0x04:
				
				/*期望位置, 相对位置模式*/
				MainController.ExptMecAngle_pulse = MainController.RefMecAngle_pulse +  MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
			
				break;
						
			case 0x05:
				
				/*配置驱动器控制模式*/
				if(CAN.ReceiveData == SPD_CURR_CTRL_MODE)
				{
					Driver.ControlMode = SPD_CURR_CTRL_MODE;
				}
				else if(CAN.ReceiveData == POS_SPD_CURR_CTRL_MODE)
				{
					Driver.ControlMode = POS_SPD_CURR_CTRL_MODE;
				}
				else if(CAN.ReceiveData == POS_CURR_CTRL_MODE)
				{
					Driver.ControlMode = POS_SPD_CURR_CTRL_MODE;
				}
				
				break;

			case 0x06:
				
				/*设置加速度*/
				MainController.Acceleration_pulse =  MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
				SpdLoop.Acceleration = DRV_PULSE_TO_RAD(MainController.Acceleration_pulse);
				
				break;

			case 0x07:

				/*设置减速度*/
				MainController.Deceleration_pulse =  MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
				SpdLoop.Deceleration = DRV_PULSE_TO_RAD(MainController.Deceleration_pulse);

				break;
					
			case 0x08:
				
				/*配置最大转矩电流*/
				CurrLoop.LimitCurrQ = (float)CAN.ReceiveData;
			
				break;
						
			case 0x09:
				
				/*配置速度环最大期望速度*/
				/*不超过电机的最大转速*/
				Saturation_int((int32_t*)&CAN.ReceiveData, MAX_SPD, -MAX_SPD);
				MainController.MaxMecAngularSpeed_pulse =  MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
				SpdLoop.MaxExptMecAngularSpeed_rad = DRV_PULSE_TO_RAD(MainController.MaxMecAngularSpeed_pulse);
				
				break;
			
			case 0x0A:
				
				/*配置位置环位置上限*/
				MainController.MecAngleUpperLimit_pulse =  MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
				PosLoop.MecAngleUpperLimit_rad = (float)DRV_PULSE_TO_RAD(MainController.MecAngleUpperLimit_pulse);
			
				break;
			
			case 0x0B:
				
				/*配置位置环位置下限*/
				MainController.MecAngleLowerLimit_pulse =  MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
				PosLoop.MecAngleLowerLimit_rad = (float)DRV_PULSE_TO_RAD(MainController.MecAngleLowerLimit_pulse);
			
				break;
			
			case (0x40 + 0x0C):
				
				/*读取速度*/
				CAN.RecieveStatus = (0x40 + 0x0C);
			
				break;

			case (0x40 + 0x0D):
				
				/*读取位置*/
				CAN.RecieveStatus = (0x40 + 0x0D);
		
				break; 
			
			case (0x40 + 0x0E):
				
				/*读取电磁转矩*/
				CAN.RecieveStatus = (0x40 + 0x0E);
				
				break;
			
			case (0x40 + 0x0F):
				
				/*读取Vq*/
				CAN.RecieveStatus = (0x40 + 0x0F);
			
				break;
			
			case (0x40 + 0x10):
				
				/*读取Iq*/
				CAN.RecieveStatus = (0x40 + 0x10);
				
				break;
			
			default:
				
				break;
		}	
	}
	/*广播模式*/
	else if(CAN.StdID == DRIVER_BROADCAST_ID)
	{
		switch(CAN.Identifier)
		{
			case 0x01:
				
				if (CAN.ReceiveData == 0x00000001)
				{
					/*使能*/
					SpdLoop.ExptMecAngularSpeed_rad = 0.f;
					PWM_IT_CMD(ENABLE,ENABLE);
				}
				else if(CAN.ReceiveData == 0x00000000)
				{
					/*失能*/
					SpdLoop.ExptMecAngularSpeed_rad = 0.f;
					PWM_IT_CMD(DISABLE,ENABLE);
				}
				
				break;
				
			case 0x02:
				
				/*期望速度*/
				MainController.ExptMecAngularSpeed_pulse =  MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
				SpdLoop.ExptMecAngularSpeed_rad = DRV_PULSE_TO_RAD(MainController.ExptMecAngularSpeed_pulse);
			
				break;
			
			case 0x03:
			
				/*期望位置, 绝对位置模式*/
				MainController.ExptMecAngle_pulse = MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
			
				break;

			case 0x04:
				
				/*期望位置, 相对位置模式*/
				MainController.ExptMecAngle_pulse = MainController.RefMecAngle_pulse +  MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
			
				break;
						
			case 0x05:
				
				/*配置驱动器控制模式*/
				if(CAN.ReceiveData == SPD_CURR_CTRL_MODE)
				{
					Driver.ControlMode = SPD_CURR_CTRL_MODE;
				}
				else if(CAN.ReceiveData == POS_SPD_CURR_CTRL_MODE)
				{
					Driver.ControlMode = POS_SPD_CURR_CTRL_MODE;
				}
				else if(CAN.ReceiveData == POS_CURR_CTRL_MODE)
				{
					Driver.ControlMode = POS_SPD_CURR_CTRL_MODE;
				}
				
				break;

			case 0x06:
				
				/*设置加速度*/
				MainController.Acceleration_pulse =  MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
				SpdLoop.Acceleration = DRV_PULSE_TO_RAD(MainController.Acceleration_pulse);
				
				break;

			case 0x07:

				/*设置减速度*/
				MainController.Deceleration_pulse =  MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
				SpdLoop.Deceleration = DRV_PULSE_TO_RAD(MainController.Deceleration_pulse);

				break;
					
			case 0x08:
				
				/*配置最大转矩电流*/
				CurrLoop.LimitCurrQ = (float)CAN.ReceiveData;
			
				break;
						
			case 0x09:
				
				/*配置速度环最大期望速度*/
				/*不超过电机的最大转速*/
				Saturation_int((int32_t*)&CAN.ReceiveData, MAX_SPD, -MAX_SPD);
				MainController.MaxMecAngularSpeed_pulse =  MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
				SpdLoop.MaxExptMecAngularSpeed_rad = DRV_PULSE_TO_RAD(MainController.MaxMecAngularSpeed_pulse);
				
				break;
			
			case 0x0A:
				
				/*配置位置环位置上限*/
				MainController.MecAngleUpperLimit_pulse =  MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
				PosLoop.MecAngleUpperLimit_rad = (float)DRV_PULSE_TO_RAD(MainController.MecAngleUpperLimit_pulse);
			
				break;
			
			case 0x0B:
				
				/*配置位置环位置下限*/
				MainController.MecAngleLowerLimit_pulse =  MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
				PosLoop.MecAngleLowerLimit_rad = (float)DRV_PULSE_TO_RAD(MainController.MecAngleLowerLimit_pulse);
			
				break;
			
			case (0x40 + 0x0C):
				
				/*读取速度*/
				CAN.RecieveStatus = (0x40 + 0x0C);
			
				break;

			case (0x40 + 0x0D):
				
				/*读取位置*/
				CAN.RecieveStatus = (0x40 + 0x0D);
		
				break; 
			
			case (0x40 + 0x0E):
				
				/*读取电磁转矩*/
				CAN.RecieveStatus = (0x40 + 0x0E);
				
				break;
			
			case (0x40 + 0x0F):
				
				/*读取Vq*/
				CAN.RecieveStatus = (0x40 + 0x0F);
			
				break;
			
			case (0x40 + 0x10):
				
				/*读取Iq*/
				CAN.RecieveStatus = (0x40 + 0x10);
				
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
		case (0x40 + 0x0C):
			
			CAN.Identifier = 0x0C;
			CAN.TransmitData = (int32_t)RAD_TO_MC_PULSE(PosSensor.MecAngularSpeed_rad);
		
			/*发送当前速度*/
			CAN_Transmit(CAN.Identifier, CAN.TransmitData, 4);
		
			CAN.RecieveStatus = 0;
		
			break;

		case (0x40 + 0x0D):
			
			CAN.Identifier = 0x0D;
			CAN.TransmitData = (int32_t)DRV_PULSE_TO_MC_PULSE(MainController.RefMecAngle_pulse);
		
			/*发送当前位置*/
			CAN_Transmit(CAN.Identifier, CAN.TransmitData, 4);
		
			CAN.RecieveStatus = 0;
		
			break;
		
		case (0x40 + 0x0E):
			
			CAN.Identifier = 0x0E;
			CAN.TransmitData = (int32_t)(Driver.EleTorque * 1e3);
		
			/*发送当前电磁转矩*/	
			CAN_Transmit(CAN.Identifier, CAN.TransmitData, 4);
		
			CAN.RecieveStatus = 0;
		
			break;
								
		case (0x40 + 0x0F):
			
			CAN.Identifier = 0x0F;
			CAN.TransmitData = (int32_t)(CurrLoop.CtrlVolQ * 1e3);
		
			/*发送当前Vq*/	
			CAN_Transmit(CAN.Identifier, CAN.TransmitData, 4);
		
			CAN.RecieveStatus = 0;
		
			break;

		case (0x40 + 0x10):
			
			CAN.Identifier = 0x10;
			CAN.TransmitData = (int32_t)(CoordTrans.CurrQ * 1e3);
		
			/*发送当前Iq*/		
			CAN_Transmit(CAN.Identifier, CAN.TransmitData, 4);
		
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
	
	UART_Transmit_DMA("CAN ERROR: %d\r\n", (uint32_t)hcan->ErrorCode);
	SendBuf();
	
	CAN.Identifier = 0xAA;
	CAN.TransmitData = hcan->ErrorCode;
	


	if(errTimes++ <= 100)
	{
		errTimes++;
		
		HAL_CAN_DeInit(&hcan1);
		HAL_CAN_Init(&hcan1);
		
		__HAL_CAN_CLEAR_FLAG(&hcan1, CAN_FLAG_FOV0);
		
		HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
		
		CAN_Transmit(CAN.Identifier, CAN.TransmitData, 4);
	}
	else
	{
		PWM_IT_CMD(DISABLE,ENABLE);
	}
}

void CAN_Transmit(uint8_t identifier, int32_t transmitData, uint8_t length)
{
	uint32_t absData = abs(transmitData);
	
	TxMessage.StdId = DRIVER_CLIENT_CAN_ID;
	TxMessage.ExtId = 0u;
	TxMessage.IDE   = CAN_ID_STD;
	TxMessage.RTR   = CAN_RTR_DATA;
	TxMessage.DLC   = length;
	
	CAN.Transmit.data_uint8[0] = (identifier>>0)&0xFF;
	
	if(transmitData >= 0)
	{
		for(uint8_t bytes = 1; bytes <= (length - 1); bytes++)
		{
			CAN.Transmit.data_uint8[bytes] = (absData>>((bytes - 1) * 8))&0xFF;
		}	
	}
	else if(transmitData < 0)
	{
		for(uint8_t bytes = 1; bytes <= (length - 1); bytes++)
		{

			CAN.Transmit.data_uint8[bytes] = (absData>>((bytes - 1) * 8))&0xFF;
			if(bytes == (length - 1))
			{
				CAN.Transmit.data_uint8[bytes] = ((absData>>((bytes - 1) * 8))&0xFF) | 0x80;
			}
		}	
	}
	
	HAL_CAN_AddTxMessage(&hcan1, &TxMessage, (uint8_t *)&CAN.Transmit, &CAN.MailBox);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);
}


void CAN_Receive(uint32_t *stdId, uint8_t *identifier, int32_t *receiveData)
{	
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxMessage0, (uint8_t *)&CAN.Receive);
	
	*stdId = RxMessage0.StdId;
	
	*identifier = CAN.Receive.data_uint8[0];
	
	if(((CAN.Receive.data_uint8[3]&0x80)>>7) == 0)
	{
		/*接受数据为正数时*/
		*receiveData = (int32_t)((CAN.Receive.data_uint8[3]<<16) | (CAN.Receive.data_uint8[2]<<8) | (CAN.Receive.data_uint8[1]<<0));
	}
	else if(((CAN.Receive.data_uint8[3]&0x80)>>7) == 1)
	{
		/*接受数据为负数时*/
		*receiveData = -(int32_t)(((CAN.Receive.data_uint8[3]&0x7F)<<16) | (CAN.Receive.data_uint8[2]<<8) | (CAN.Receive.data_uint8[1]<<0));
	}
}

void CAN_Enable(void)
{
	/*配置过滤器*/
	CAN_FilterTypeDef CAN1_FilerConf    = {0};
	CAN1_FilerConf.FilterIdHigh         = (DRIVER_BROADCAST_ID << 5) | (CAN_ID_STD << 4) | (CAN_RTR_DATA << 3);
	CAN1_FilerConf.FilterIdLow          = (DRIVER_SERVER_CAN_ID << 5) | (CAN_ID_STD << 4) | (CAN_RTR_DATA << 3);
	CAN1_FilerConf.FilterMaskIdHigh     = 0x0000;
	CAN1_FilerConf.FilterMaskIdLow      = 0x0000;
	CAN1_FilerConf.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	CAN1_FilerConf.FilterBank           = 0;
	CAN1_FilerConf.FilterMode           = CAN_FILTERMODE_IDLIST;
	CAN1_FilerConf.FilterScale          = CAN_FILTERSCALE_16BIT;
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

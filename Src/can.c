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

uint32_t StdId;
uint32_t ExtId;
volatile CANdata_t Receive;
volatile CANdata_t Transmit;
uint32_t CAN_RecieveStatus = 0u;
CAN_TxHeaderTypeDef TxMessage = { 0 };
CAN_RxHeaderTypeDef RxMessage0 = { 0 };
static uint32_t mbox;

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
	
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxMessage0, (uint8_t *)&Receive);
	
	StdId = RxMessage0.StdId;
	ExtId = RxMessage0.ExtId;
	
	if (StdId == CAN_SDO_SERVER_STD_ID)
	{
		/*ACTION驱动器指令*/
//		switch(ExtId)
//		{
//			case 0x00004F4D: //MO
//				
//				if (Receive.data_uint32[1] == 0x00000001)
//				{
//					/*使能*/
//					SpdLoop.ExptMecAngularSpeed_rad = 0.f;
//					PWM_IT_CMD(ENABLE,ENABLE);
//				}
//				else
//				{
//					/*失能*/
//					SpdLoop.ExptMecAngularSpeed_rad = 0.0f;
//					PWM_IT_CMD(DISABLE,ENABLE);
//				}
//				
//				break;
//				
//			case 0x00004D55://UM
//				
//				/*模式切换, 暂不可用*/
////				if(receive.data_int32[1] == SPD_CURR_CTRL_MODE)
////				{
////					Driver.ControlMode = SPD_CURR_CTRL_MODE;
////				}
////				else if(receive.data_int32[1] == POS_SPD_CURR_CTRL_MODE)
////				{
////					Driver.ControlMode = POS_SPD_CURR_CTRL_MODE;
////					PosLoop.ExptMecAngle_rad = PosSensor.MecAngle_rad + PosSensor.MecAngularSpeed_rad * Regulator.ActualPeriod_s;
////				}
//				
//				break;
//				
//			case 0x0000564A: //JV
//				
//				/*期望速度, 主控方向与驱动器相反*/
//				Saturation_int((int*)&Receive.data_int32[1], MAX_SPD, -MAX_SPD);
//				MainController.ExptMecAngularSpeed_pulse =  Receive.data_int32[1];
//				SpdLoop.ExptMecAngularSpeed_rad = PULSE_TO_RAD(MainController.ExptMecAngularSpeed_pulse);
//			
//				break;

//			case 0x00004341: //AC
//				
//				/*设置加速度*/
//				MainController.Acceleration_pulse = Receive.data_int32[1];
//				SpdLoop.Acceleration = PULSE_TO_RAD(MainController.Acceleration_pulse);
//				
//				break;

//			case 0x00004344: //DC

//				/*设置减速度*/
//				MainController.Deceleration_pulse = Receive.data_int32[1];
//				SpdLoop.Deceleration = PULSE_TO_RAD(MainController.Deceleration_pulse);

//				break;
//					
//			case 0x00005053: //SP
//				
//				/*设置位置环最大输出速度*/
//				Saturation_int((int*)&Receive.data_int32[1], MAX_SPD, -MAX_SPD);
//				MainController.MaxMecAngularSpeed_pulse = Receive.data_int32[1];
//				PosLoop.MaxMecAngularSpeed_rad = PULSE_TO_RAD(MainController.MaxMecAngularSpeed_pulse);
//				
//				break;

//			case 0x00004150: //PA
//			
//				/*期望位置, 绝对位置模式*/
//				MainController.ExptMecAngle_pulse = Receive.data_int32[1];
//			
//				break;

//			case 0x00005250: //PR
//				
//				/*期望位置, 相对位置模式*/
//				MainController.ExptMecAngle_pulse = MainController.RefMecAngle_pulse + Receive.data_int32[1];
//			
//				break;

//			case 0x40004449: //ID
//				
//				/*读取Id*/
//				CAN_RecieveStatus = 0x40004449;
//				
//				break;
//			case 0x40005149: //IQ
//				
//				/*读取Iq*/
//				CAN_RecieveStatus = 0x40005149;
//				
//				break;
//			
//			case 0x40004455: //UD
//				
//				/*读取Vq*/
//				CAN_RecieveStatus = 0x40004455;
//			
//				break;

//			case 0x40005155: //UQ
//				
//				/*读取Vq*/
//				CAN_RecieveStatus = 0x40005155;
//			
//				break;
//						
//			case 0x40005856: //VX
//				
//				/*读取速度*/
//				CAN_RecieveStatus = 0x40005856;
//			
//				break;

//			case 0x40005850: //PX
//				
//				/*读取位置*/
//				CAN_RecieveStatus = 0x40005850;
//		
//				break; 
//			
//			default:
//				
//				break;
//		}
		
		/*ELMO驱动器指令*/
		switch (Receive.data_uint32[0])
		{
			case 0x00004F4D: //MO
				
				if (Receive.data_uint32[1] == 0x00000001)
				{
					/*使能*/
					SpdLoop.ExptMecAngularSpeed_rad = 0.f;
					PWM_IT_CMD(ENABLE,ENABLE);
				}
				else
				{
					/*失能*/
					SpdLoop.ExptMecAngularSpeed_rad = 0.0f;
					PWM_IT_CMD(DISABLE,ENABLE);
				}
				
				break;
				
			case 0x00004D55://UM
				
				/*模式切换, 暂不可用*/
//				if(receive.data_int32[1] == SPD_CURR_CTRL_MODE)
//				{
//					Driver.ControlMode = SPD_CURR_CTRL_MODE;
//				}
//				else if(receive.data_int32[1] == POS_SPD_CURR_CTRL_MODE)
//				{
//					Driver.ControlMode = POS_SPD_CURR_CTRL_MODE;
//					PosLoop.ExptMecAngle_rad = PosSensor.MecAngle_rad + PosSensor.MecAngularSpeed_rad * Regulator.ActualPeriod_s;
//				}
				
				break;
				
			case 0x0000564A: //JV
				
				/*期望速度*/
				Saturation_int((int*)&Receive.data_int32[1], MAX_SPD, -MAX_SPD);
				MainController.ExptMecAngularSpeed_pulse =  Receive.data_int32[1];
				SpdLoop.ExptMecAngularSpeed_rad = PULSE_TO_RAD(MainController.ExptMecAngularSpeed_pulse);
			
				break;

			case 0x00004341: //AC
				
				/*设置加速度*/
				MainController.Acceleration_pulse = Receive.data_int32[1];
				SpdLoop.Acceleration = PULSE_TO_RAD(MainController.Acceleration_pulse);
				
				break;

			case 0x00004344: //DC

				/*设置减速度*/
				MainController.Deceleration_pulse = Receive.data_int32[1];
				SpdLoop.Deceleration = PULSE_TO_RAD(MainController.Deceleration_pulse);

				break;
					
			case 0x00005053: //SP
				
				/*设置位置环最大输出速度*/
				Saturation_int((int*)&Receive.data_int32[1], MAX_SPD, -MAX_SPD);
				MainController.MaxMecAngularSpeed_pulse = Receive.data_int32[1];
				PosLoop.MaxMecAngularSpeed_rad = PULSE_TO_RAD(MainController.MaxMecAngularSpeed_pulse);
				
				break;

			case 0x00004150: //PA
			
				/*期望位置, 绝对位置模式*/
				MainController.ExptMecAngle_pulse = Receive.data_int32[1];
			
				break;

			case 0x00005250: //PR
				
				/*期望位置, 相对位置模式*/
				MainController.ExptMecAngle_pulse = MainController.RefMecAngle_pulse + Receive.data_int32[1];
			
				break;

			case 0x40004449: //ID
				
				/*读取Id*/
				CAN_RecieveStatus = 0x40004449;
				
				break;
			case 0x40005149: //IQ
				
				/*读取Iq*/
				CAN_RecieveStatus = 0x40005149;
				
				break;
			
			case 0x40004455: //UD
				
				/*读取Vq*/
				CAN_RecieveStatus = 0x40004455;
			
				break;

			case 0x40005155: //UQ
				
				/*读取Vq*/
				CAN_RecieveStatus = 0x40005155;
			
				break;
						
			case 0x40005856: //VX
				
				/*读取速度*/
				CAN_RecieveStatus = 0x40005856;
			
				break;

			case 0x40005850: //PX
				
				/*读取位置*/
				CAN_RecieveStatus = 0x40005850;
		
				break; 
			
			default:
				
				break;
		}
	}
	else if (StdId == (0x300 + GROUP_NUM))
	{
		switch (Receive.data_uint32[0])
		{
			case 0x40004455: //UD
				
				/*读取Vd*/
				CAN_RecieveStatus = 0x40004455;
			
				break;
						
			case 0x40005155: //UQ
				
				/*读取Vq*/
				CAN_RecieveStatus = 0x40005155;
			
				break;
			
			case 0x40004449: //ID
				
				/*读取Id*/
				CAN_RecieveStatus = 0x40004449;
				
				break;
			
			case 0x40005149: //IQ
				
				/*读取Iq*/
				CAN_RecieveStatus = 0x40005149;
				
				break;

			case 0x40005856: //VX
				
				/*读取速度*/
				CAN_RecieveStatus = 0x40005856;
			
				break;

			case 0x40005850: //PX
				
				/*读取位置*/
				CAN_RecieveStatus = 0x40005850;
		
				break; 
			
			default:
				
				break;
		}
	}
	
	CANRespond();
	
	/*检测RX FIFO剩余的信息数量*/
	if(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0))
	{
		goto first_line;
	}
}

void CANRespond(void)
{
	switch (CAN_RecieveStatus)
	{
		case 0x40004455: //UD
			
			/*读取Vd*/
			Transmit.data_uint32[0] = 0x00004455;
			Transmit.data_int32[1]  = CurrLoop.CtrlVolD * 1e3;
		
			CANSendData(Transmit);
			CAN_RecieveStatus = 0;
		
			break;
				
		case 0x40005155: //UQ
			
			/*读取Vq*/
			Transmit.data_uint32[0] = 0x00005155;
			Transmit.data_int32[1]  = CurrLoop.CtrlVolQ * 1e3;
		
			CANSendData(Transmit);
			CAN_RecieveStatus = 0;
		
			break;

		case 0x40004449: //ID
			
			/*读取Id*/
			Transmit.data_uint32[0] = 0x00004449;
			Transmit.data_int32[1]  = CoordTrans.CurrD * 1e3;
		
			CANSendData(Transmit);
			CAN_RecieveStatus = 0;
		
			break;
				
		case 0x40005149: //IQ
			
			/*读取Iq*/
			Transmit.data_uint32[0] = 0x00005149;
			Transmit.data_int32[1]  = CoordTrans.CurrQ * 1e3;
		
			CANSendData(Transmit);
			CAN_RecieveStatus = 0;
		
			break;
				
		case 0x40005856: //VX
			
			/*读取速度*/
	#if POSITION_SENSOR_TYPE == ENCODER_TLE5012
			Transmit.data_uint32[0] = 0x00005856;
			Transmit.data_int32[1]  = (int32_t)((PosSensor.MecAngularSpeed_rad /(2.f * PI) * TLE5012_ABS_MODE_RESOLUTION));
		
			CANSendData(Transmit);
			CAN_RecieveStatus = 0;
	#endif
		
			break;

		case 0x40005850: //PX
			
			/*读取位置*/
			Transmit.data_uint32[0] = 0x00005850;
			Transmit.data_int32[1]  = MainController.RefMecAngle_pulse;
		
			CANSendData(Transmit);
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
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	static uint16_t errTimes = 0;
	
	CANdata_t txData;
	PutStr("CAN Error Code:");
	PutNum(hcan->ErrorCode, '\n');SendBuf();
	
	txData.data_uint32[0] = 0x00111111;
	txData.data_uint32[1] = hcan->ErrorCode;

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
}

void CANSendData_Test(CANdata_t data, uint32_t extId)
{
	TxMessage.StdId = CAN_SDO_CLIENT_STD_ID;
	TxMessage.ExtId = extId;
	TxMessage.IDE   = CAN_ID_STD;
	TxMessage.RTR   = CAN_RTR_DATA;
	TxMessage.DLC   = 8u;
	
	HAL_CAN_AddTxMessage(&hcan1, &TxMessage, (uint8_t *)&data, &mbox);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);
}

void CANSendData(CANdata_t data)
{
	TxMessage.StdId = CAN_SDO_CLIENT_STD_ID;
	TxMessage.ExtId = 0u;
	TxMessage.IDE   = CAN_ID_STD;
	TxMessage.RTR   = CAN_RTR_DATA;
	TxMessage.DLC   = 8u;
	
	HAL_CAN_AddTxMessage(&hcan1, &TxMessage, (uint8_t *)&data, &mbox);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);
}

void CAN_Enable(void)
{
	CAN_FilterTypeDef CAN1_FilerConf    = {0};
	CAN1_FilerConf.FilterIdHigh         = 0X0000;
	CAN1_FilerConf.FilterIdLow          = 0X0000;
	CAN1_FilerConf.FilterMaskIdHigh     = 0X0000;           //32 位MASK
	CAN1_FilerConf.FilterMaskIdLow      = 0X0000;
	CAN1_FilerConf.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	CAN1_FilerConf.FilterBank           = 0;
	CAN1_FilerConf.FilterMode           = CAN_FILTERMODE_IDMASK;
	CAN1_FilerConf.FilterScale          = CAN_FILTERSCALE_32BIT;
	CAN1_FilerConf.FilterActivation     = ENABLE;
	CAN1_FilerConf.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan1, &CAN1_FilerConf); //初始化过滤器
		
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_LAST_ERROR_CODE);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_BUSOFF);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_ERROR);
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

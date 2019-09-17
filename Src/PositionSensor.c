/**
 ******************************************************************************
 * @file		PositionSensor.c
 * @author		WrathWings
 * @version 	V1.0
 * @date		2019.3.2
 * @brief		Set and read position sensor
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
 
 /* Private includes ----------------------------------------------------------*/
/* CODE BEGIN Includes */
#include "PositionSensor.h"
/* CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* CODE BEGIN PTD */
extern struct Regulator_t Regulator;
extern struct MainController_t MainController;
/* CODE END PTD */

/* Private macro -------------------------------------------------------------*/
/* CODE BEGIN PM */

/* CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* CODE BEGIN PV */
#if	POSITION_SENSOR_TYPE == ENCODER_TLE5012
	struct PosSensor_t PosSensor;
	extern const short int EleAngleRef[DIVIDE_NUM * (uint8_t)MOTOR_POLE_PAIRS_NUM + 2];
	extern const int MecAngleRef[DIVIDE_NUM * (uint8_t)MOTOR_POLE_PAIRS_NUM + 2];
#else
#error "Position Sensor Type Invalid"
#endif

/* CODE END PV */

/* USER CODE BEGIN */

#if	POSITION_SENSOR_TYPE == ENCODER_TLE5012
	void GetPositionImformation(void)
	{
		#if	ENCODER_MODE == ENCODER_ABSOLUTE_MODE
		
		GetMecAngle_AbsoluteMode_15bit();  //绝对式, 读取编码器角度值寄存器
		
		#elif ENCODER_MODE == ENCODER_INCREMENTAL_MODE
		
		GetMecAngle_IncrementalMode_14bit();  //增量式, TLE5012的12位增量式编码器经过4倍频后精度提高至14位
		
		#else
		#error "Encoder Mode Invalid"
		#endif
		GetMecAngle(); //计算机械角度
		GetRefMecAngle(); //计算参考机械角度（主控用）
		GetMecAngularSpeed(); //计算机械角速度
		GetEleAngle(); //计算电角度
		GetEleAngularSpeed();  //计算电角速度
		TLE5012_ReadFSYNC();	//读取FSYNC值
		EncoderLostDetection();		//编码器异常检测
	}

	void GetMecAngle_AbsoluteMode_15bit(void)
	{
		PosSensor.MecAngle_AbsoluteMode_15bit = (TLE5012_ReadRegister(TLE5012_Command_ReadCurrentValue_AngleValue, &PosSensor.SafetyWord)) & 0x7FFF;
	}

	void GetMecAngle_IncrementalMode_14bit(void)
	{
		PosSensor.MecAngle_IncrementalMode_14bit = (uint16_t)(PosSensor.OriginalMecAngle_14bit + TIM2->CNT);
		
		if(PosSensor.MecAngle_IncrementalMode_14bit >= 16384)
		{
			PosSensor.MecAngle_IncrementalMode_14bit -= 16384;
		}
	}
	
	void GetMecAngle(void)
	{
		#if	ENCODER_MODE == ENCODER_ABSOLUTE_MODE
		
		PosSensor.MecAngle_degree = 360.f * (float)PosSensor.MecAngle_AbsoluteMode_15bit / TLE5012_ABS_MODE_RESOLUTION;
		
		PosSensor.MecAngle_rad = (float)PosSensor.MecAngle_degree / 360.f * 2.0f * PI;
		
		#elif ENCODER_MODE == ENCODER_INCREMENTAL_MODE
		
		PosSensor.MecAngle_degree = 360.f * (float)PosSensor.MecAngle_IncrementalMode_14bit / (TLE5012_IncrementalModeResolution * 4.f);
		
		PosSensor.MecAngle_rad = (float)PosSensor.MecAngle_degree / 360.f * 2.0f * PI;
		
		#else
		#error "Encoder Mode Invalid"
		#endif
	}
	
	void GetRefMecAngle(void)
	{
		int delta = 0;
		
		MainController.PresentMecAngle_pulse = PosSensor.MecAngle_AbsoluteMode_15bit;
		
		delta = MainController.PresentMecAngle_pulse - MainController.LastMecAngle_pulse;
		
		MainController.LastMecAngle_pulse = MainController.PresentMecAngle_pulse;
		
		if(delta < -(TLE5012_ABS_MODE_RESOLUTION / 2))
		{
			delta += TLE5012_ABS_MODE_RESOLUTION;
		}		
		else if(delta > (TLE5012_ABS_MODE_RESOLUTION / 2))
		{
			delta -= TLE5012_ABS_MODE_RESOLUTION;
		}
		
		MainController.RefMecAngle_pulse += delta;
	}
	
	void GetMecAngularSpeed(void)
	{
		float presentMecAngle = 0;
		static float lastMecAngle = 0;
		float angleDifference = 0;
		const uint8_t FilterOrder = 6;
		static float array[FilterOrder] = {0};
		static uint8_t pos = 0;
		static float sum = 0.f;
		static float avg = 0.f;
		static float old = 0.f;
		
		presentMecAngle = PosSensor.MecAngle_rad;
		
		angleDifference = presentMecAngle - lastMecAngle;
		
		lastMecAngle = presentMecAngle;
		
		if(angleDifference > PI)
		{
			angleDifference -= 2.f * PI;
		}
		
		else if(angleDifference < -PI)
		{
			angleDifference += 2.f * PI;
		}
		
		old = array[pos];

		#if ENCODER_MODE == ENCODER_ABSOLUTE_MODE
		
		array[pos] = angleDifference / Regulator.ActualPeriod_s;
		
		#elif ENCODER_MODE == ENCODER_INCREMENTAL_MODE
		
		array[pos] = angleDifference / Regulator.ActualPeriod_s;
		
		#else
		#error "Encoder Mode Invalid"
		#endif
			
		sum = (sum - old) + array[pos];
			
		avg = sum / FilterOrder;

		pos = (pos+1) % FilterOrder;
		
		PosSensor.MecAngularSpeed_rad = avg;
	}

	void GetEleAngle(void)
	{
		float normPos = 0;
		
		#if ENCODER_MODE == ENCODER_ABSOLUTE_MODE
		
//		normPos = fmodf(PosSensor.MecAngle_AbsoluteMode_15bit, TLE5012_ABS_MODE_RESOLUTION);	
//		
//		uint32_t index = UtilBiSearchInt(MecAngleRef, normPos, sizeof(MecAngleRef)/sizeof(MecAngleRef[0]));

//		uint32_t indexPlus1 = index + 1;
//		
//		PosSensor.EleAngle_degree = fmodf(utils_map(PosSensor.MecAngle_AbsoluteMode_15bit, MecAngleRef[index], MecAngleRef[indexPlus1], EleAngleRef[index], EleAngleRef[indexPlus1]), 360.f);
		
		PosSensor.EleAngle_degree = ((float)(PosSensor.MecAngle_AbsoluteMode_15bit - PosSensor.PosOffset) / (float)(TLE5012_ABS_MODE_RESOLUTION / MOTOR_POLE_PAIRS_NUM)) * 360.f;
		
		#elif ENCODER_MODE == ENCODER_INCREMENTAL_MODE
		
		normPos = fmodf(PosSensor.MecAngle_IncrementalMode_14bit, TLE5012_IncrementalModeResolution * 4.f);	
		
		uint32_t index = UtilBiSearchInt(MecAngleRef, normPos, sizeof(MecAngleRef)/sizeof(MecAngleRef[0]));

		uint32_t indexPlus1 = index + 1;
		
		PosSensor.EleAngle_degree = fmodf(utils_map(PosSensor.MecAngle_IncrementalMode_14bit, MecAngleRef[index], MecAngleRef[indexPlus1], EleAngleRef[index], EleAngleRef[indexPlus1]), 360.f);
		
		#else
		#error "Encoder Mode Invalid"
		#endif
	}

	void GetEleAngularSpeed(void)
	{
		const uint8_t FilterOrder = 6;
		static float array[FilterOrder] = {0};
		static uint8_t pos = 0;
		static float data = 0.f;
		static float sum = 0.f;
		static float avg = 0.f;
		static float old = 0.f;
			
		old = array[pos];
		
		array[pos] = PosSensor.MecAngularSpeed_rad * MOTOR_POLE_PAIRS_NUM;
			
		sum = (sum - old) + array[pos];
			
		avg = sum / FilterOrder;

		pos = (pos+1) % FilterOrder;
	
		PosSensor.EleAngularSpeed_rad = avg;
		
		PosSensor.EleAngularSpeed_degree = RAD_TO_DEGREE(PosSensor.EleAngularSpeed_rad);
	}

	void TLE5012_ReadFSYNC(void)
	{
		PosSensor.FSYNC = (TLE5012_ReadRegister(TLE5012_Command_ReadCurrentValue_FSYNC, &PosSensor.SafetyWord)) >> 9;
	}
	
	void EncoderLostDetection()
	{
		uint8_t cnt = 0;
		
		PosSensor.SafetyWord >>= 12;
		
		/*通过TLE5012的SafetyWord判断编码器是否异常*/
		if(PosSensor.SafetyWord != 0x07)
		{
			cnt++;
			
			if(cnt > 10)
			{
				PWM_IT_CMD(DISABLE,ENABLE);
				
				/*通过CAN总线向主控发送编码器异常*/
//				errorCode.data_int32[0] = 0xCCCCCCCC;
//				errorCode.data_int32[1] = 0xCCCCCCCC;
//					
//				CANSendData(errorCode);
				
				PutNum((uint16_t)PosSensor.SafetyWord,'\t');	
				PutStr("ENCODER LOST\r\n");	
				SendBuf();
				
				cnt = 0;
			}
		}
		else
		{
			if(cnt > 0)	
			{
				cnt--;
			}
		}
	}
	
	uint16_t TLE5012_ReadRegister(uint16_t command, uint16_t *safetyWord)
	{
		uint16_t data = 0;
		
		TLE5012_SPI1_ChipSelect;
		
		SPI_Transmit(SPI1, command, TimeOut);
		SPI_TX_OFF;
		
		SPI_Receive(SPI1, &data, TimeOut);
		SPI_Receive(SPI1, safetyWord, TimeOut);
		
		TLE5012_SPI1_ChipDiselect;
		SPI_TX_ON;

		return data;
	}
	
	void EncoderIncrementalModeEnable(void)
	{
		/*启动TIM2增量式编码器模式*/
		HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
		
		/*读取初始机械角度*/
		PosSensor.OriginalMecAngle_14bit = (uint16_t)(((float)(TLE5012_ReadRegister(TLE5012_Command_ReadCurrentValue_AngleValue, &PosSensor.SafetyWord) & 0x7FFF) / 32768.f) * 16384.f);
	}
	
	/**
	* @brief  Measure reference Ele angle
	* @param[in]  volD      voltage of axis d
	*/
	void MeasureEleAngle_Encoder(float volD)
	{
		float volAlpha = 0;
		float volBeta = 0;
		int16_t tempData[2] = {0};
		int EleAngle = 0;
		static int16_t index_5012b = 1;
		static int16_t index_bound = 0;
		static int tempMecAngleRef[DIVIDE_NUM * MOTOR_POLE_PAIRS_NUM + 2] = {0};
		static int tempEleAngleRef[DIVIDE_NUM * MOTOR_POLE_PAIRS_NUM + 2] = {0};
		static int16_t tempArray[DIVIDE_NUM * MOTOR_POLE_PAIRS_NUM] = {0};

		InverseParkTransform(volD, 0.f, &volAlpha, &volBeta, 0.f);	//设定Vq = 0, 电角度为零
		
		SpaceVectorModulation(volAlpha, volBeta);
		
		HAL_Delay(1000);
		
		for (int i = 0; i < MOTOR_POLE_PAIRS_NUM; i++)
		{
			for(int j = 0; j < DIVIDE_NUM; j ++)
			{
				GetPositionImformation();
				
				EleAngle = j * 360 / DIVIDE_NUM;
				
				InverseParkTransform(volD, 0.f, &volAlpha, &volBeta, EleAngle);
				
				SpaceVectorModulation(volAlpha, volBeta);
				
				HAL_Delay(200);

				
					#if ENCODER_MODE == ENCODER_ABSOLUTE_MODE
					
					tempMecAngleRef[index_5012b] = PosSensor.MecAngle_AbsoluteMode_15bit;
					
					#elif ENCODER_MODE == ENCODER_INCREMENTAL_MODE
					
					tempMecAngleRef[index_5012b] = PosSensor.MecAngle_IncrementalMode_14bit;
					
					#else
					#error "Encoder Mode Invalid"
					#endif
				

				if(tempMecAngleRef[index_5012b] < 1000 && tempMecAngleRef[index_5012b - 1] > 10000 && index_5012b > 1)
				{
					tempData[0] = tempMecAngleRef[index_5012b];
					
					tempData[1] = EleAngle;
					
					index_bound = index_5012b;
				}
				
				UART_Transmit_DMA("\t/*Angle*/\t %d \t /*Encoder*/\t %d \r\n", (int)(EleAngle), (int)tempMecAngleRef[index_5012b]); 
			
				SendBuf();

				index_5012b++;
				
				if(index_5012b > DIVIDE_NUM * MOTOR_POLE_PAIRS_NUM)
				{
					PutStr("EXCESS\r\n");SendBuf();
					break;
				}
			}
		}
		
		/*	发送电角度表	*/
		
		PutStr("\r\n\r\n\r\n\r\n");SendBuf();
		
		HAL_Delay(100);
		
		for(int i = index_bound, j = 0; i <= DIVIDE_NUM * MOTOR_POLE_PAIRS_NUM;i++,j++)
		{
			tempArray[j] =  tempMecAngleRef[i];
		}
		
		for(int i = index_bound - 1,k = DIVIDE_NUM * MOTOR_POLE_PAIRS_NUM; i >= 0; i--, k--)
		{
			tempMecAngleRef[k] = tempMecAngleRef[i];
		}
		
		for(int i = 1, k =0; k <=  DIVIDE_NUM * MOTOR_POLE_PAIRS_NUM - index_bound; i++,k++)
		{
			tempMecAngleRef[i] = tempArray[k];
		}
		
		tempEleAngleRef[1] = tempData[1];
		
		for(int i = 1; i <= DIVIDE_NUM * MOTOR_POLE_PAIRS_NUM; i++)
		{
			tempEleAngleRef[i + 1] = tempEleAngleRef[i] + 360 / DIVIDE_NUM;
		}
		
		#if POSITION_SENSOR_TYPE == ENCODER_TLE5012
			#if ENCODER_MODE == ENCODER_ABSOLUTE_MODE
			
			tempMecAngleRef[0] = tempMecAngleRef[DIVIDE_NUM * MOTOR_POLE_PAIRS_NUM] - TLE5012_ABS_MODE_RESOLUTION;
			
			tempMecAngleRef[DIVIDE_NUM * MOTOR_POLE_PAIRS_NUM + 1] = tempMecAngleRef[1] + TLE5012_ABS_MODE_RESOLUTION;
			
			#elif ENCODER_MODE == ENCODER_INCREMENTAL_MODE

			tempMecAngleRef[0] = tempMecAngleRef[DIVIDE_NUM * MOTOR_POLE_PAIRS_NUM] - (int32_t)(TLE5012_IncrementalModeResolution * 4.f);
			
			tempMecAngleRef[DIVIDE_NUM * MOTOR_POLE_PAIRS_NUM + 1] = tempMecAngleRef[1] + (int32_t)(TLE5012_IncrementalModeResolution * 4.f);
			
			#else
			#error "Encoder Mode Invalid"
			#endif
		#endif

		tempEleAngleRef[0] = tempEleAngleRef[1] - 360 / DIVIDE_NUM;

		for(int i = 0; i < DIVIDE_NUM * MOTOR_POLE_PAIRS_NUM + 2; i++)
		{
			UART_Transmit_DMA("%d\t,\t%d\t,\r\n", (int)tempEleAngleRef[i], (int)tempMecAngleRef[i]);
			
			SendBuf();
		
			HAL_Delay(15);
		}
		
		PWM_IT_CMD(DISABLE, DISABLE);
		
		while(1)
		{
			HAL_Delay(1);
		}
	}
	void CorrectPosOffset_Encoder(float volD)
	{
		float volAlpha = 0;
		float volBeta = 0;
		uint16_t position = 0;
		uint16_t lastPosition = 0;
		uint16_t count = 0;
		uint16_t times = 0;
		
		PutStr("Correct Begin...\r\n\r\n");SendBuf();
		
		InverseParkTransform(volD, 0.f, &volAlpha, &volBeta, 0.f);
		SpaceVectorModulation(volAlpha, volBeta);
		LL_mDelay(500);
		
		InverseParkTransform(volD, 0.f, &volAlpha, &volBeta, 60.f);
		SpaceVectorModulation(volAlpha, volBeta);
		LL_mDelay(500);
		
		InverseParkTransform(volD, 0.f, &volAlpha, &volBeta, 120.f);
		SpaceVectorModulation(volAlpha, volBeta);
		LL_mDelay(500);
		
		InverseParkTransform(volD, 0.f, &volAlpha, &volBeta, -120.f);
		SpaceVectorModulation(volAlpha, volBeta);
		LL_mDelay(500);
		
		InverseParkTransform(volD, 0.f, &volAlpha, &volBeta, -60.f);
		SpaceVectorModulation(volAlpha, volBeta);
		LL_mDelay(500);
		
		InverseParkTransform(volD, 0.f, &volAlpha, &volBeta, 0.f);
		SpaceVectorModulation(volAlpha, volBeta);
		LL_mDelay(500);
		
		GetPositionImformation();
		
		for(times = 0; times < 200; times++)
		{
			lastPosition = position;
			
			position = PosSensor.MecAngle_AbsoluteMode_15bit;
			
			if((times > 100) && (abs((int16_t)(position - lastPosition))<5))
			{
				count++;
				
				if(count > 30) 
				{
					PosSensor.PosOffset = position;
					
					UART_Transmit_DMA("Position Offset:	%d \r\nCorrect Finished", (int)PosSensor.PosOffset);SendBuf();
					
					break;
				}
			}
			
			LL_mDelay(5);
		}
		
		PWM_IT_CMD(DISABLE, DISABLE);
		
		while(1)
		{
			HAL_Delay(1);
		}
	}
	
#else
#error "Position Sensor Type Invalid"
#endif
  
/* USER CODE END */

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

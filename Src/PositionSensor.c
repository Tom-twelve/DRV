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

/* CODE END PTD */

/* Private macro -------------------------------------------------------------*/
/* CODE BEGIN PM */

/* CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* CODE BEGIN PV */
#if	POSITION_SENSOR_TYPE == ENCODER_TLE5012
	struct PosSensor_t PosSensor;
	extern const short int EleAngleRef[DIVIDE_NUM * (uint8_t)MOTOR_POLE_PAIRS + 2];
	extern const int MecAngleRef[DIVIDE_NUM * (uint8_t)MOTOR_POLE_PAIRS + 2];
#else
#error "Position Sensor Type Invalid"
#endif

/* CODE END PV */

/* USER CODE BEGIN */

#if	POSITION_SENSOR_TYPE == ENCODER_TLE5012
	void GetPositionImformation(void)
	{
		#if	ENCODER_MODE == Encoder_AbsoluteMode
		
		GetMecAngle_AbsoluteMode_15bit();  //绝对式, 读取编码器角度值寄存器
		
		#elif ENCODER_MODE == Encoder_IncrementalMode
		
		GetMecAngle_IncrementalMode_14bit();  //增量式, TLE5012的12位增量式编码器经过4倍频后精度提高至14位
		
		#else
		#error "Encoder Mode Invalid"
		#endif
		GetMecAngle(); //计算机械角度
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
		#if	ENCODER_MODE == Encoder_AbsoluteMode
		
		PosSensor.MecAngle_degree = 360.f * (float)PosSensor.MecAngle_AbsoluteMode_15bit / TLE5012_AbsoluteModeResolution;
		
		PosSensor.MecAngle_rad = (float)PosSensor.MecAngle_degree / 360.f * 2.0f * PI;
		
		#elif ENCODER_MODE == Encoder_IncrementalMode
		
		PosSensor.MecAngle_degree = 360.f * (float)PosSensor.MecAngle_IncrementalMode_14bit / (TLE5012_IncrementalModeResolution * 4.f);
		
		PosSensor.MecAngle_rad = (float)PosSensor.MecAngle_degree / 360.f * 2.0f * PI;
		
		#else
		#error "Encoder Mode Invalid"
		#endif
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

		#if ENCODER_MODE == Encoder_AbsoluteMode
		
		array[pos] = angleDifference / TLE5012_UpdateTime_1;
		
		#elif ENCODER_MODE == Encoder_IncrementalMode
		
		array[pos] = angleDifference / DEFAULT_CARRIER_PERIOD_s;
		
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
		
		#if ENCODER_MODE == Encoder_AbsoluteMode
		
		normPos = fmodf(PosSensor.MecAngle_AbsoluteMode_15bit, TLE5012_AbsoluteModeResolution);	
		
		uint32_t index = UtilBiSearchInt(MecAngleRef, normPos, sizeof(MecAngleRef)/sizeof(MecAngleRef[0]));

		uint32_t indexPlus1 = index + 1;
		
		PosSensor.EleAngle_degree = fmodf(utils_map(PosSensor.MecAngle_AbsoluteMode_15bit, MecAngleRef[index], MecAngleRef[indexPlus1], EleAngleRef[index], EleAngleRef[indexPlus1]), 360.f);
		
		#elif ENCODER_MODE == Encoder_IncrementalMode
		
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
		
		array[pos] = PosSensor.MecAngularSpeed_rad * MOTOR_POLE_PAIRS;
			
		sum = (sum - old) + array[pos];
			
		avg = sum / FilterOrder;

		pos = (pos+1) % FilterOrder;
	
		PosSensor.EleAngularSpeed_rad = avg;
		
		PosSensor.EleAngularSpeed_degree = RadToDegree(PosSensor.EleAngularSpeed_rad);
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
	* @param[in]  VolD      voltage of axis d
	*/
	void MeasureEleAngle_Encoder(float VolD)
	{
		float VolAlpha = 0;
		float VolBeta = 0;
		int16_t tmpData[2] = {0};
		int EleAngle = 0;
		static int16_t index_5012b = 1;
		static int16_t index_bound = 0;
		static int tempMecAngleRef[DIVIDE_NUM * (uint8_t)MOTOR_POLE_PAIRS + 2] = {0};
		static int tempEleAngleRef[DIVIDE_NUM * (uint8_t)MOTOR_POLE_PAIRS + 2] = {0};
		static int16_t tmpArray[DIVIDE_NUM * (uint8_t)MOTOR_POLE_PAIRS] = {0};

		InverseParkTransform(VolD, 0.f, &VolAlpha, &VolBeta, 0.f);	//设定Vq = 0, 电角度为零
		
		SpaceVectorModulation(VolAlpha, VolBeta);
		
		HAL_Delay(1000);
		
		for (int i = 0; i < MOTOR_POLE_PAIRS; i++)
		{
			for(int j = 0; j < DIVIDE_NUM; j ++)
			{
				GetPositionImformation();
				
				EleAngle = j * 360 / DIVIDE_NUM;
				
				InverseParkTransform(VolD, 0.f, &VolAlpha, &VolBeta, EleAngle);
				
				SpaceVectorModulation(VolAlpha, VolBeta);
				
				HAL_Delay(200);

				
					#if ENCODER_MODE == Encoder_AbsoluteMode
					
					tempMecAngleRef[index_5012b] = PosSensor.MecAngle_AbsoluteMode_15bit;
					
					#elif ENCODER_MODE == Encoder_IncrementalMode
					
					tempMecAngleRef[index_5012b] = PosSensor.MecAngle_IncrementalMode_14bit;
					
					#else
					#error "Encoder Mode Invalid"
					#endif
				

				if(tempMecAngleRef[index_5012b] < 1000 && tempMecAngleRef[index_5012b - 1] > 10000 && index_5012b > 1)
				{
					tmpData[0] = tempMecAngleRef[index_5012b];
					
					tmpData[1] = EleAngle;
					
					index_bound = index_5012b;
				}
				
				UART_Transmit_DMA("\t/*Angle*/\t %d \t /*Encoder*/\t %d \r\n", (int)(EleAngle), (int)tempMecAngleRef[index_5012b]); 
			
				SendBuf();

				index_5012b++;
				
				if(index_5012b > DIVIDE_NUM * (uint8_t)MOTOR_POLE_PAIRS)
				{
					PutStr("EXCESS\r\n");SendBuf();
					break;
				}
			}
		}
		
		/*	发送电角度表	*/
		
		PutStr("\r\n\r\n\r\n\r\n");
		
		HAL_Delay(100);
		
		for(int i = index_bound, j = 0; i <= DIVIDE_NUM * (uint8_t)MOTOR_POLE_PAIRS;i++,j++)
		{
			tmpArray[j] =  tempMecAngleRef[i];
		}
		
		for(int i = index_bound - 1,k = DIVIDE_NUM * (uint8_t)MOTOR_POLE_PAIRS; i >= 0; i--, k--)
		{
			tempMecAngleRef[k] = tempMecAngleRef[i];
		}
		
		for(int i = 1, k =0; k <=  DIVIDE_NUM * (uint8_t)MOTOR_POLE_PAIRS - index_bound; i++,k++)
		{
			tempMecAngleRef[i] = tmpArray[k];
		}
		
		tempEleAngleRef[1] = tmpData[1];
		
		for(int i = 1; i <= DIVIDE_NUM * (uint8_t)MOTOR_POLE_PAIRS; i++)
		{
			tempEleAngleRef[i + 1] = tempEleAngleRef[i] + 360 / DIVIDE_NUM;
		}
		
		#if POSITION_SENSOR_TYPE == ENCODER_TLE5012
			#if ENCODER_MODE == Encoder_AbsoluteMode
			
			tempMecAngleRef[0] = tempMecAngleRef[DIVIDE_NUM * (uint8_t)MOTOR_POLE_PAIRS] - (int32_t)TLE5012_AbsoluteModeResolution;
			
			tempMecAngleRef[DIVIDE_NUM * (uint8_t)MOTOR_POLE_PAIRS + 1] = tempMecAngleRef[1] + (int32_t)TLE5012_AbsoluteModeResolution;
			
			#elif ENCODER_MODE == Encoder_IncrementalMode

			tempMecAngleRef[0] = tempMecAngleRef[DIVIDE_NUM * (uint8_t)MOTOR_POLE_PAIRS] - (int32_t)(TLE5012_IncrementalModeResolution * 4.f);
			
			tempMecAngleRef[DIVIDE_NUM * (uint8_t)MOTOR_POLE_PAIRS + 1] = tempMecAngleRef[1] + (int32_t)(TLE5012_IncrementalModeResolution * 4.f);
			
			#else
			#error "Encoder Mode Invalid"
			#endif
		#endif

		tempEleAngleRef[0] = tempEleAngleRef[1] - 360 / DIVIDE_NUM;

		for(int i = 0; i < DIVIDE_NUM * (uint8_t)MOTOR_POLE_PAIRS + 2; i++)
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
	
#else
#error "Position Sensor Type Invalid"
#endif
  
/* USER CODE END */

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

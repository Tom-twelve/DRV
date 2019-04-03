/**
 ******************************************************************************
 * @file		Encoder.c
 * @author		WrathWings
 * @version 	V1.0
 * @date		2019.3.2
 * @brief		Set and read encoder
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
#if	POSITION_SENSOR_TYPE == Encoder_TLE5012
	struct PositionSensor_t PositionSensor;
	extern const short int EleAngleRef[DIVIDE_NUM * (uint8_t)MotorPolePairs + 2];
	extern const int MecAngleRef[DIVIDE_NUM * (uint8_t)MotorPolePairs + 2];
#elif	POSITION_SENSOR_TYPE == HallSensor_DRV5053
	struct PositionSensor_t PositionSensor;
	extern int GivenEle[CALIBRATE_NUM + 2];
	extern int HallEle[CALIBRATE_NUM + 2];
#else
#error "Position Sensor Type Invalid"
#endif

/* CODE END PV */

/* USER CODE BEGIN */



#if	POSITION_SENSOR_TYPE == Encoder_TLE5012
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
		GetAvgMecAngularSpeed(); //计算机械角速度均值
		GetEleAngle(); //计算电角度
		GetAvgEleAngularSpeed();  //计算电角速度均值
	}

	void GetMecAngle_AbsoluteMode_15bit(void)
	{
		PositionSensor.MecAngle_AbsoluteMode_15bit = (TLE5012_ReadRegister(TLE5012_Command_ReadCurrentValue_AngleValue) & 0x7FFF);
	}

	void GetMecAngle_IncrementalMode_14bit(void)
	{
		PositionSensor.MecAngle_IncrementalMode_14bit = (uint16_t)(PositionSensor.OriginalMecAngle_14bit + TIM2->CNT);
		
		if(PositionSensor.MecAngle_IncrementalMode_14bit >= 16384)
		{
			PositionSensor.MecAngle_IncrementalMode_14bit -= 16384;
		}
	}
	
	void GetMecAngle(void)
	{
		#if	ENCODER_MODE == Encoder_AbsoluteMode
		
		PositionSensor.MecAngle_degree = 360.f * (float)PositionSensor.MecAngle_AbsoluteMode_15bit / TLE5012_AbsoluteModeResolution;
		
		PositionSensor.MecAngle_rad = (float)PositionSensor.MecAngle_degree / 360.f * 2.0f * PI;
		
		#elif ENCODER_MODE == Encoder_IncrementalMode
		
		PositionSensor.MecAngle_degree = 360.f * (float)PositionSensor.MecAngle_IncrementalMode_14bit / (TLE5012_IncrementalModeResolution * 4.f);
		
		PositionSensor.MecAngle_rad = (float)PositionSensor.MecAngle_degree / 360.f * 2.0f * PI;
		
		#else
		#error "Encoder Mode Invalid"
		#endif
	}

	void GetMecAngularSpeed(void)
	{
		float presentMecAngle = 0;
		static float lastMecAngle = 0;
		float angleDifference = 0;
		
		presentMecAngle = PositionSensor.MecAngle_rad;
		
		angleDifference = presentMecAngle - lastMecAngle;
		
		if(angleDifference > PI)
		{
			angleDifference -= 2.f * PI;
		}
		
		else if(angleDifference < -PI)
		{
			angleDifference += 2.f * PI;
		}
		
		#if ENCODER_MODE == Encoder_AbsoluteMode
		
		PositionSensor.MecAngularSpeed_rad = angleDifference / TLE5012_UpdateTime_0;
		
		#elif ENCODER_MODE == Encoder_IncrementalMode
		
		PositionSensor.MecAngularSpeed_rad = angleDifference / CarrierPeriod_s;
		
		#else
		#error "Encoder Mode Invalid"
		#endif
		
		lastMecAngle = presentMecAngle;
	}

	void GetAvgMecAngularSpeed(void)
	{
		const uint8_t num = 6;
		static float array[num] = {0};
		static uint8_t pos = 0;
		static float data = 0.0f;
		static float sum = 0.0f;
		static float Avg = 0.0f;
		float old = array[pos];
		
		data = PositionSensor.MecAngularSpeed_rad;
		
		array[pos] = data;
		
		sum = (sum - old) + data;
		
		Avg = sum / num;

		pos = (pos+1) % num;

		PositionSensor.AvgMecAngularSpeed_rad = Avg;
	}

	void GetEleAngle(void)
	{
		float normPos = 0;
		
		#if ENCODER_MODE == Encoder_AbsoluteMode
		
		normPos = fmodf(PositionSensor.MecAngle_AbsoluteMode_15bit, TLE5012_AbsoluteModeResolution);	
		
		uint32_t index = UtilBiSearchInt(MecAngleRef, normPos, sizeof(MecAngleRef)/sizeof(MecAngleRef[0]));

		uint32_t indexPlus1 = index + 1;
		
		PositionSensor.EleAngle_degree = fmodf(utils_map(PositionSensor.MecAngle_AbsoluteMode_15bit, MecAngleRef[index], MecAngleRef[indexPlus1], EleAngleRef[index], EleAngleRef[indexPlus1]), 360.f);
		
		#elif ENCODER_MODE == Encoder_IncrementalMode
		
		normPos = fmodf(PositionSensor.MecAngle_IncrementalMode_14bit, TLE5012_IncrementalModeResolution * 4.f);	
		
		uint32_t index = UtilBiSearchInt(MecAngleRef, normPos, sizeof(MecAngleRef)/sizeof(MecAngleRef[0]));

		uint32_t indexPlus1 = index + 1;
		
		PositionSensor.EleAngle_degree = fmodf(utils_map(PositionSensor.MecAngle_IncrementalMode_14bit, MecAngleRef[index], MecAngleRef[indexPlus1], EleAngleRef[index], EleAngleRef[indexPlus1]), 360.f);
		
		#else
		#error "Encoder Mode Invalid"
		#endif
	}

	void GetAvgEleAngularSpeed(void)
	{
		PositionSensor.AvgEleAngularSpeed_rad = PositionSensor.AvgMecAngularSpeed_rad * MotorPolePairs;
	}

	uint16_t TLE5012_ReadRegister(uint16_t command)
	{
		uint16_t data = 0;
		uint16_t safetyWord = 0;
		
		TLE5012_SPI1_ChipSelect;
		
		SPI_Transmit(SPI1, command, TimeOut);
		SPI_TX_OFF;
		
		SPI_Receive(SPI1, &data, TimeOut);
		SPI_Receive(SPI1, &safetyWord, TimeOut);
		
		TLE5012_SPI1_ChipDiselect;
		SPI_TX_ON;

		return data;
	}
	
	void EncoderIncrementalModeEnable(void)
	{
		/*启动TIM2增量式编码器模式*/
		HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
		
		/*读取初始机械角度*/
		PositionSensor.OriginalMecAngle_14bit = (uint16_t)(((float)(TLE5012_ReadRegister(TLE5012_Command_ReadCurrentValue_AngleValue) & 0x7FFF) / 32768.f) * 16384.f);
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
		static int tempMecAngleRef[DIVIDE_NUM * (uint8_t)MotorPolePairs + 2] = {0};
		static int tempEleAngleRef[DIVIDE_NUM * (uint8_t)MotorPolePairs + 2] = {0};
		static int16_t tmpArray[DIVIDE_NUM * (uint8_t)MotorPolePairs] = {0};

		InverseParkTransform(VolD, 0.f, &VolAlpha, &VolBeta, 0.f);	//设定Uq = 0, 电角度为零
		
		SpaceVectorModulation(VolAlpha, VolBeta);
		
		HAL_Delay(1000);
		
		for (int i = 0; i < MotorPolePairs; i++)
		{
			for(int j = 0; j < DIVIDE_NUM; j ++)
			{
				GetPositionImformation();
				
				EleAngle = j * 360 / DIVIDE_NUM;
				
				InverseParkTransform(VolD, 0.f, &VolAlpha, &VolBeta, EleAngle);
				
				SpaceVectorModulation(VolAlpha, VolBeta);
				
				HAL_Delay(200);

				
					#if ENCODER_MODE == Encoder_AbsoluteMode
					
					tempMecAngleRef[index_5012b] = PositionSensor.MecAngle_AbsoluteMode_15bit;
					
					#elif ENCODER_MODE == Encoder_IncrementalMode
					
					tempMecAngleRef[index_5012b] = PositionSensor.MecAngle_IncrementalMode_14bit;
					
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
				
				if(index_5012b > DIVIDE_NUM * (uint8_t)MotorPolePairs)
				{
					PutStr("EXCESS\r\n");SendBuf();
					break;
				}
			}
		}
		
		/*	发送电角度表	*/
		
		PutStr("\r\n\r\n\r\n\r\n");
		
		HAL_Delay(100);
		
		for(int i = index_bound, j = 0; i <= DIVIDE_NUM * (uint8_t)MotorPolePairs;i++,j++)
		{
			tmpArray[j] =  tempMecAngleRef[i];
		}
		
		for(int i = index_bound - 1,k = DIVIDE_NUM * (uint8_t)MotorPolePairs; i >= 0; i--, k--)
		{
			tempMecAngleRef[k] = tempMecAngleRef[i];
		}
		
		for(int i = 1, k =0; k <=  DIVIDE_NUM * (uint8_t)MotorPolePairs - index_bound; i++,k++)
		{
			tempMecAngleRef[i] = tmpArray[k];
		}
		
		tempEleAngleRef[1] = tmpData[1];
		
		for(int i = 1; i <= DIVIDE_NUM * (uint8_t)MotorPolePairs; i++)
		{
			tempEleAngleRef[i + 1] = tempEleAngleRef[i] + 360 / DIVIDE_NUM;
		}
		
		#if POSITION_SENSOR_TYPE == Encoder_TLE5012
			#if ENCODER_MODE == Encoder_AbsoluteMode
			
			tempMecAngleRef[0] = tempMecAngleRef[DIVIDE_NUM * (uint8_t)MotorPolePairs] - (int32_t)TLE5012_AbsoluteModeResolution;
			
			tempMecAngleRef[DIVIDE_NUM * (uint8_t)MotorPolePairs + 1] = tempMecAngleRef[1] + (int32_t)TLE5012_AbsoluteModeResolution;
			
			#elif ENCODER_MODE == Encoder_IncrementalMode

			tempMecAngleRef[0] = tempMecAngleRef[DIVIDE_NUM * (uint8_t)MotorPolePairs] - (int32_t)(TLE5012_IncrementalModeResolution * 4.f);
			
			tempMecAngleRef[DIVIDE_NUM * (uint8_t)MotorPolePairs + 1] = tempMecAngleRef[1] + (int32_t)(TLE5012_IncrementalModeResolution * 4.f);
			
			#else
			#error "Encoder Mode Invalid"
			#endif
		#endif

		tempEleAngleRef[0] = tempEleAngleRef[1] - 360 / DIVIDE_NUM;

		for(int i = 0; i < DIVIDE_NUM * (uint8_t)MotorPolePairs + 2; i++)
		{
			UART_Transmit_DMA("%d\t,\t%d\t,\r\n", (int)tempEleAngleRef[i], (int)tempMecAngleRef[i]);
			
			SendBuf();
		
			HAL_Delay(15);
		}
	}
	
#elif	POSITION_SENSOR_TYPE == HallSensor_DRV5053
	void GetPositionImformation(void)
	{
		GetEleAngle(); //计算电角度
		GetEleAngularSpeed();  //计算电角速度
		GetAvgEleAngularSpeed();	//计算电角速度均值
		GetAvgMecAngularSpeed();	//计算机械角速度均值
	}
	
	void HallInit(void)
	{
		/*初始化霍尔的标定值*/
		#if	 RobotIdentifier == 2U
			#if CAN_ID_NUM == 1
				PositionSensor.HallMaxValue[0] = 1809;
				PositionSensor.HallMaxValue[1] = 1788;
				PositionSensor.HallMinValue[0] = 810;
				PositionSensor.HallMinValue[1] = 861;
				PositionSensor.HallZeroValue[0] = 1311;
				PositionSensor.HallZeroValue[1] = 1322;
			#elif CAN_ID_NUM == 2
				PositionSensor.HallMaxValue[0] = 1794;
				PositionSensor.HallMaxValue[1] = 1784;
				PositionSensor.HallMinValue[0] = 861;
				PositionSensor.HallMinValue[1] = 918;
				PositionSensor.HallZeroValue[0] = 1327;
				PositionSensor.HallZeroValue[1] = 1351;
			#endif
		#endif
		
		/*零点归一化处理*/
		PositionSensor.HallNormalizedActualValue[0] = Normalized((PositionSensor.HallZeroValue[0] - PositionSensor.HallMinValue[0]) / (PositionSensor.HallMaxValue[0] - PositionSensor.HallMinValue[0]));
		PositionSensor.HallNormalizedActualValue[1] = Normalized((PositionSensor.HallZeroValue[1] - PositionSensor.HallMinValue[1]) / (PositionSensor.HallMaxValue[1] - PositionSensor.HallMinValue[1]));
		
		HAL_Delay(10);
		
		
	}

	/**
	  * @brief 计算机械角速度均值
	  */
	void GetAvgMecAngularSpeed()
	{
		PositionSensor.AvgMecAngularSpeed_rad = PositionSensor.AvgEleAngularSpeed_rad / MotorPolePairs;
		
		PositionSensor.AvgMecAngularSpeed_degree = RadToDegree(PositionSensor.AvgMecAngularSpeed_rad);
	}
	
	/**
	  * @brief 计算电角度
	  */
	void GetEleAngle(void)
	{
		static float eleAngleCalculate = 0;
		
		/*归一化处理, 注意浮点数强制转换时候的符号问题*/
		PositionSensor.HallNormalizedActualValue[0] = Normalized((PositionSensor.HallActualValue[0] - PositionSensor.HallMinValue[0]) / (PositionSensor.HallMaxValue[0] - PositionSensor.HallMinValue[0]));
		PositionSensor.HallNormalizedActualValue[1] = Normalized((PositionSensor.HallActualValue[1] - PositionSensor.HallMinValue[1]) / (PositionSensor.HallMaxValue[1] - PositionSensor.HallMinValue[1]));

		#if	PHASE_SEQUENCE == POSITIVE_SEQUENCE
		
		eleAngleCalculate = (atan2f(PositionSensor.HallNormalizedActualValue[0] - PositionSensor.HallNormalizedZeroValue[0], PositionSensor.HallNormalizedActualValue[1] - PositionSensor.HallNormalizedZeroValue[1]) * 57.29578f);
		
		#elif PHASE_SEQUENCE == NEGATIVE_SEQUENCE
		
		eleAngleCalculate = (atan2f(PositionSensor.HallNormalizedActualValue[1] - PositionSensor.HallNormalizedZeroValue[1], PositionSensor.HallNormalizedActualValue[0] - PositionSensor.HallNormalizedZeroValue[0]) * 57.29578f);
		
		#endif
		
		/*查表补偿，得到的里程曲线与不使用查表法时基本一致（r^2均为1），因此不会随着运行时间变长而出现里程误差*/
		eleAngleCalculate = RadToDegree(HallAngleTableComp(eleAngleCalculate)); 
		
		PositionSensor.EleAngle_rad = eleAngleCalculate;
		
		PositionSensor.EleAngle_degree = RadToDegree(eleAngleCalculate);
	}
	
	/**
	  * @brief 计算电角速度
	  */
	void GetEleAngularSpeed(void)
	{
		float eleAngleDiff_rad = 0;
		static float eleAngleLast = 0;
		
		eleAngleDiff_rad = util_norm_float(PositionSensor.EleAngle_rad - eleAngleLast, -PI, PI, 2.f * PI);
		
		PositionSensor.EleAngularSpeed_rad = eleAngleDiff_rad / CarrierPeriod_s;
		
		PositionSensor.EleAngularSpeed_degree = RadToDegree(PositionSensor.EleAngularSpeed_rad);
	}
	
	/**
	  * @brief 计算电角速度均值
	  */
	void GetAvgEleAngularSpeed()
	{
		const uint8_t num = 6;
		static float array[num] = {0};
		static uint8_t pos = 0;
		static float data = 0.0f;
		static float sum = 0.0f;
		static float Avg = 0.0f;
		float old = array[pos];
		
		data = PositionSensor.EleAngularSpeed_rad;
		
		array[pos] = data;
		
		sum = (sum - old) + data;
		
		Avg = sum / num;

		pos = (pos+1) % num;

		PositionSensor.AvgEleAngularSpeed_rad = Avg;
		
		PositionSensor.AvgEleAngularSpeed_degree = RadToDegree(PositionSensor.AvgEleAngularSpeed_rad);
	}
	

	
	/**
	  * @brief 霍尔查表补偿
	  * @param[in]  hallValueSin  			作为sin的霍尔值
	  */
	float HallAngleTableComp(int eleAngleCalculate)
	{
		float eleAngleCompensated = 0;
		uint8_t index = 0;
		
		index = UtilBiSearchInt(HallEle, eleAngleCalculate, CALIBRATE_NUM + 2);
		
		eleAngleCompensated = (eleAngleCalculate - HallEle[index]) * (GivenEle[index + 1] - GivenEle[index]) / (HallEle[index + 1] - HallEle[index]) + GivenEle[index];
		
		return eleAngleCompensated;
	}

	/**
	  * @brief 测量电角度
	  * @param[in]  VolD  			给定d轴电压
	  */
	void MeasureEleAngle_HallSensor(float VolD)
	{
		float VolAlpha = 0;
		float VolBeta = 0;
		static float eleAngle = 0;
		static float hall1Value = 0;
		static float hall2Value = 0;
		static float eleAngleCalculate = 0;
		static float HallCalibrateTable[2][CALIBRATE_NUM + 2] = {0};
		static float hall1[(uint8_t)MotorPolePairs][DIVIDE_NUM] = {0};
		static float hall2[(uint8_t)MotorPolePairs][DIVIDE_NUM] = {0};
		
		InverseParkTransform_TwoPhase(VolD, 0.f, &VolAlpha, &VolBeta, 0.f);	//设定Vq = 0, 电角度为零
		
		SpaceVectorModulation(VolAlpha, VolBeta);
		
		/*零点归一化处理*/
		PositionSensor.HallNormalizedActualValue[0] = Normalized((PositionSensor.HallZeroValue[0] - PositionSensor.HallMinValue[0]) / (PositionSensor.HallMaxValue[0] - PositionSensor.HallMinValue[0]));
		PositionSensor.HallNormalizedActualValue[1] = Normalized((PositionSensor.HallZeroValue[1] - PositionSensor.HallMinValue[1]) / (PositionSensor.HallMaxValue[1] - PositionSensor.HallMinValue[1]));
		
		HAL_Delay(1000);
		
		/*测量TEST_ROUNDS圈并取平均值以保证霍尔值准确度*/
		for(uint8_t round = 0; round < TEST_ROUNDS; round++)
		{
			/*共MotorPolePairs个电气周期*/
			for (uint8_t pairs = 0; pairs < MotorPolePairs; pairs++)
			{
				/*每个电气周期切分为DIVIDE_NUM份*/
				for(uint8_t divNum = 0; divNum < DIVIDE_NUM; divNum ++)
				{
					eleAngle = divNum * 360 / DIVIDE_NUM;
					
					InverseParkTransform_TwoPhase(VolD, 0.f, &VolAlpha, &VolBeta, eleAngle);
					
					SpaceVectorModulation(VolAlpha, VolBeta);
					
					/*每个点取SAMPLING_TIMES个霍尔值取其平均值*/
					for(int times = 0 ; times < SAMPLING_TIMES; times++)
					{
						/*延时以保证ADC值已更新*/
						HAL_Delay(10);
						
						hall1Value += PositionSensor.HallActualValue[0] / SAMPLING_TIMES;
						hall2Value += PositionSensor.HallActualValue[1] / SAMPLING_TIMES;
					}				
			
					hall1[pairs][divNum] += hall1Value / TEST_ROUNDS;
					hall2[pairs][divNum] += hall2Value / TEST_ROUNDS;
					
					/***********************解算电角度***********************CODE BEGIN**********************/
					
					PositionSensor.HallNormalizedActualValue[0] = Normalized((PositionSensor.HallActualValue[0] - PositionSensor.HallMinValue[0]) / (PositionSensor.HallMaxValue[0] - PositionSensor.HallMinValue[0]));
					PositionSensor.HallNormalizedActualValue[1] = Normalized((PositionSensor.HallActualValue[1] - PositionSensor.HallMinValue[1]) / (PositionSensor.HallMaxValue[1] - PositionSensor.HallMinValue[1]));

					#if	PHASE_SEQUENCE == NEGATIVE_SEQUENCE
						eleAngleCalculate = RadToDegree((atan2f(PositionSensor.HallNormalizedActualValue[1] - PositionSensor.HallNormalizedActualValue[1], PositionSensor.HallNormalizedActualValue[0] - PositionSensor.HallNormalizedActualValue[0]) * 57.29578f));
					#elif PHASE_SEQUENCE == POSITIVE_SEQUENCE
						eleAngleCalculate = RadToDegree((PositionSensor.HallNormalizedActualValue[0] - PositionSensor.HallNormalizedActualValue[0], PositionSensor.HallNormalizedActualValue[1] - PositionSensor.HallNormalizedActualValue[1]) * 57.29578f));
					#endif
					
					/***********************解算电角度***********************CODE END************************/
					
					UART_Transmit_DMA("%d\t%d\t%d\t",(int)eleAngle,(int)hall1Value,(int)hall2Value);
					UART_Transmit_DMA("%d\r\n",(int)eleAngleCalculate);

					SendBuf();
					
					HAL_Delay(100);//一次SendBuf后，需要留足时间给DMA发送，否则会overflow
					
					hall1Value = 0;
					hall2Value = 0;
				}
			}
			HAL_Delay(5);
		}
		
		/***********************获取霍尔最大最小值及零点***********************CODE BEGIN***********/
		
		uint32_t hall1Average = 0;
		uint32_t hall2Average = 0;
		int32_t max1[(uint8_t)MotorPolePairs] = {0};
		int32_t max2[(uint8_t)MotorPolePairs] = {0};
		int32_t min1[(uint8_t)MotorPolePairs] = {0};
		int32_t min2[(uint8_t)MotorPolePairs] = {0};
		int32_t zero1[(uint8_t)MotorPolePairs] = {0};
		int32_t zero2[(uint8_t)MotorPolePairs] = {0};
		uint32_t maxIndex = 0, minIndex = 0;
		
		HAL_Delay(100);
		
		/*计算出霍尔值中的最大值、最小值、零点值*/
		for(int pairs = 0; pairs < MotorPolePairs; pairs++)
		{
			arm_max_q31((int32_t*)hall1[pairs], DIVIDE_NUM, (int32_t*)&max1[pairs], &maxIndex);
			arm_max_q31((int32_t*)hall2[pairs], DIVIDE_NUM, (int32_t*)&max2[pairs], &maxIndex);
			arm_min_q31((int32_t*)hall1[pairs], DIVIDE_NUM, (int32_t*)&min1[pairs], &minIndex);
			arm_min_q31((int32_t*)hall2[pairs], DIVIDE_NUM, (int32_t*)&min2[pairs], &minIndex);	
			arm_mean_q31((int32_t*)hall1[pairs], DIVIDE_NUM, (int32_t*)&zero1[pairs]);
			arm_mean_q31((int32_t*)hall2[pairs], DIVIDE_NUM, (int32_t*)&zero2[pairs]);
			
			UART_Transmit_DMA("%d\t%d\t%d\t%d\t%d\t%d\t\r\n",(int)max1[pairs], (int)max2[pairs], (int)min1[pairs], (int)min2[pairs], (int)zero1[pairs], (int)zero2[pairs]);
			SendBuf();
			HAL_Delay(50);
		}
		
		HAL_Delay(200);
		
		PositionSensor.HallMaxValue[0] = 0;
		PositionSensor.HallMaxValue[1] = 0;
		PositionSensor.HallMinValue[0] = 0;
		PositionSensor.HallMinValue[1] = 0;
		PositionSensor.HallZeroValue[0] = 0;
		PositionSensor.HallZeroValue[1] = 0;	
		
		for(int pairs = 0; pairs < MotorPolePairs; pairs++)
		{
			PositionSensor.HallMaxValue[0] += max1[pairs] / MotorPolePairs;
			PositionSensor.HallMaxValue[1] += max2[pairs] / MotorPolePairs;
			PositionSensor.HallMinValue[0] += min1[pairs] / MotorPolePairs;
			PositionSensor.HallMinValue[1] += min2[pairs] / MotorPolePairs;
			PositionSensor.HallZeroValue[0] += zero1[pairs] / MotorPolePairs;
			PositionSensor.HallZeroValue[1] += zero2[pairs] / MotorPolePairs;
		}
		
		PutStr("write in Hall Init:\r\n");SendBuf();
		
		HAL_Delay(10);
		
		UART_Transmit_DMA("%d\t%d\t%d\t%d\t%d\t%d\t\r\n",(int)PositionSensor.HallMaxValue[0], (int)PositionSensor.HallMaxValue[1], (int)PositionSensor.HallMinValue[0], (int)PositionSensor.HallMinValue[1], (int)PositionSensor.HallZeroValue[0], (int)PositionSensor.HallZeroValue[1]);
		
		SendBuf();
		
		HAL_Delay(50);
		
		/***********************获取霍尔最大最小值及零点***********************CODE END*************/
		
		/***************************获取霍尔补偿表格***************************CODE BEGIN***********/
		
		int angLast = 0;
		
		InverseParkTransform_TwoPhase(VolD, 0.f, &VolAlpha, &VolBeta, 0.f);
					
		SpaceVectorModulation(VolAlpha, VolBeta);
		
		/*零点归一化处理*/
		PositionSensor.HallNormalizedActualValue[0] = Normalized((PositionSensor.HallZeroValue[0] - PositionSensor.HallMinValue[0]) / (PositionSensor.HallMaxValue[0] - PositionSensor.HallMinValue[0]));
		PositionSensor.HallNormalizedActualValue[1] = Normalized((PositionSensor.HallZeroValue[1] - PositionSensor.HallMinValue[1]) / (PositionSensor.HallMaxValue[1] - PositionSensor.HallMinValue[1]));
		
		hall1Value = 0;
		hall2Value = 0;
		
		for(int div = 0; div < CALIBRATE_NUM + 1; div++)
		{
			InverseParkTransform_TwoPhase(VolD, 0.f, &VolAlpha, &VolBeta, div * 360.f / CALIBRATE_NUM);
					
			SpaceVectorModulation(VolAlpha, VolBeta);
			
			HAL_Delay(500);
			
			for(int times = 0 ; times < SAMPLING_TIMES; times++)
			{
				HAL_Delay(10);
				hall1Value += PositionSensor.HallActualValue[0] / SAMPLING_TIMES;//ADC2
				hall2Value += PositionSensor.HallActualValue[1] / SAMPLING_TIMES;
			}

			HallCalibrateTable[0][div] = div * 360.f / CALIBRATE_NUM;  
			
			/***********************解算电角度***********************CODE BEGIN**********************/
					
			PositionSensor.HallNormalizedActualValue[0] = Normalized((PositionSensor.HallActualValue[0] - PositionSensor.HallMinValue[0]) / (PositionSensor.HallMaxValue[0] - PositionSensor.HallMinValue[0]));
			PositionSensor.HallNormalizedActualValue[1] = Normalized((PositionSensor.HallActualValue[1] - PositionSensor.HallMinValue[1]) / (PositionSensor.HallMaxValue[1] - PositionSensor.HallMinValue[1]));

			#if	PHASE_SEQUENCE == NEGATIVE_SEQUENCE
			
				eleAngleCalculate = RadToDegree((atan2f(PositionSensor.HallNormalizedActualValue[1] - PositionSensor.HallNormalizedActualValue[1], PositionSensor.HallNormalizedActualValue[0] - PositionSensor.HallNormalizedActualValue[0]) * 57.29578f));
			
			#elif PHASE_SEQUENCE == POSITIVE_SEQUENCE
				
				eleAngleCalculate = RadToDegree((PositionSensor.HallNormalizedActualValue[0] - PositionSensor.HallNormalizedActualValue[0], PositionSensor.HallNormalizedActualValue[1] - PositionSensor.HallNormalizedActualValue[1]) * 57.29578f));
			
			#endif
			
			HallCalibrateTable[1][div] = eleAngleCalculate;
			
			/***********************解算电角度***********************CODE END************************/

			hall1Value = 0;
			hall2Value = 0;
			
			angLast = HallCalibrateTable[1][div] ;
		}
		
		PutStr("write in GiveEle and HallEle:\r\n");SendBuf();
		
		HAL_Delay(20);

		for(int div = 0; div < CALIBRATE_NUM + 2; div++)
		{
			if(div == 0)
			{
				UART_Transmit_DMA("%d,\t%d,\r\n",(int)( - HallCalibrateTable[0][div + 1] + 2 * HallCalibrateTable[0][div]), (int)( - HallCalibrateTable[1][div + 1] + 2 * HallCalibrateTable[1][div]));
				SendBuf();
			}
			if(div > 0 && div < CALIBRATE_NUM + 1)
			{
				UART_Transmit_DMA("%d,\t%d,\r\n",(int)HallCalibrateTable[0][div - 1], (int)HallCalibrateTable[1][div - 1]);
				SendBuf();		
			}
			if( div == CALIBRATE_NUM + 1)
			{
				UART_Transmit_DMA("%d,\t%d,\r\n",(int)(HallCalibrateTable[0][div - 1]), (int)(HallCalibrateTable[1][div - 1] + 360));
				SendBuf();		
			}
			HAL_Delay(20);
		}
		
		/***************************获取霍尔补偿表格***************************CODE END*************/
		
		HAL_Delay(250);
		
		/*失能驱动器输出*/
		PWM_IT_CMD(DISABLE,DISABLE);

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

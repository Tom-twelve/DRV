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
#elif	POSITION_SENSOR_TYPE == HALL_SENSOR_DRV5053
	struct PosSensor_t PosSensor;
	extern int GivenEle[CALIBRATE_NUM + 2];
	extern int HallEle[CALIBRATE_NUM + 2];
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
	}

	void GetMecAngle_AbsoluteMode_15bit(void)
	{
		PosSensor.MecAngle_AbsoluteMode_15bit = (TLE5012_ReadRegister(TLE5012_Command_ReadCurrentValue_AngleValue) & 0x7FFF);
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
		
		#if ENCODER_MODE == Encoder_AbsoluteMode
		
		PosSensor.MecAngularSpeed_rad = AverageFilter(angleDifference / TLE5012_UpdateTime_1);
		
		#elif ENCODER_MODE == Encoder_IncrementalMode
		
		PosSensor.MecAngularSpeed_rad = AverageFilter(angleDifference / CARRIER_PERIOD_S);
		
		#else
		#error "Encoder Mode Invalid"
		#endif
		
		
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
		PosSensor.EleAngularSpeed_rad = AverageFilter(PosSensor.MecAngularSpeed_rad * MOTOR_POLE_PAIRS);
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
		PosSensor.OriginalMecAngle_14bit = (uint16_t)(((float)(TLE5012_ReadRegister(TLE5012_Command_ReadCurrentValue_AngleValue) & 0x7FFF) / 32768.f) * 16384.f);
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

		InverseParkTransform(VolD, 0.f, &VolAlpha, &VolBeta, 0.f);	//设定Uq = 0, 电角度为零
		
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
	}
	
#elif	POSITION_SENSOR_TYPE == HALL_SENSOR_DRV5053
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
		#if	 ROBOT_ID == 2U
			#if CAN_ID_NUM == 1
				PosSensor.HallMaxValue[0] = 1809;
				PosSensor.HallMaxValue[1] = 1788;
				PosSensor.HallMinValue[0] = 810;
				PosSensor.HallMinValue[1] = 861;
				PosSensor.HallZeroValue[0] = 1311;
				PosSensor.HallZeroValue[1] = 1322;
			#elif CAN_ID_NUM == 2
				PosSensor.HallMaxValue[0] = 1794;
				PosSensor.HallMaxValue[1] = 1784;
				PosSensor.HallMinValue[0] = 861;
				PosSensor.HallMinValue[1] = 918;
				PosSensor.HallZeroValue[0] = 1327;
				PosSensor.HallZeroValue[1] = 1351;
			#endif
		#endif
		
		/*零点归一化处理*/
		PosSensor.HallNormalizedActualValue[0] = Normalized((PosSensor.HallZeroValue[0] - PosSensor.HallMinValue[0]) / (PosSensor.HallMaxValue[0] - PosSensor.HallMinValue[0]));
		PosSensor.HallNormalizedActualValue[1] = Normalized((PosSensor.HallZeroValue[1] - PosSensor.HallMinValue[1]) / (PosSensor.HallMaxValue[1] - PosSensor.HallMinValue[1]));
		
		HAL_Delay(10);
		
		
	}

	/**
	  * @brief 计算机械角速度均值
	  */
	void GetAvgMecAngularSpeed()
	{
		PosSensor.MecAngularSpeed_rad = PosSensor.EleAngularSpeed_rad / MOTOR_POLE_PAIRS;
		
		PosSensor.AvgMecAngularSpeed_degree = RadToDegree(PosSensor.MecAngularSpeed_rad);
	}
	
	/**
	  * @brief 计算电角度
	  */
	void GetEleAngle(void)
	{
		static float eleAngleCalculate = 0;
		
		/*归一化处理, 注意浮点数强制转换时候的符号问题*/
		PosSensor.HallNormalizedActualValue[0] = Normalized((PosSensor.HallActualValue[0] - PosSensor.HallMinValue[0]) / (PosSensor.HallMaxValue[0] - PosSensor.HallMinValue[0]));
		PosSensor.HallNormalizedActualValue[1] = Normalized((PosSensor.HallActualValue[1] - PosSensor.HallMinValue[1]) / (PosSensor.HallMaxValue[1] - PosSensor.HallMinValue[1]));

		#if	PHASE_SEQUENCE == POSITIVE_SEQUENCE
		
		eleAngleCalculate = (atan2f(PosSensor.HallNormalizedActualValue[0] - PosSensor.HallNormalizedZeroValue[0], PosSensor.HallNormalizedActualValue[1] - PosSensor.HallNormalizedZeroValue[1]) * 57.29578f);
		
		#elif PHASE_SEQUENCE == NEGATIVE_SEQUENCE
		
		eleAngleCalculate = (atan2f(PosSensor.HallNormalizedActualValue[1] - PosSensor.HallNormalizedZeroValue[1], PosSensor.HallNormalizedActualValue[0] - PosSensor.HallNormalizedZeroValue[0]) * 57.29578f);
		
		#endif
		
		/*查表补偿，得到的里程曲线与不使用查表法时基本一致（r^2均为1），因此不会随着运行时间变长而出现里程误差*/
		eleAngleCalculate = RadToDegree(HallAngleTableComp(eleAngleCalculate)); 
		
		PosSensor.EleAngle_rad = eleAngleCalculate;
		
		PosSensor.EleAngle_degree = RadToDegree(eleAngleCalculate);
	}
	
	/**
	  * @brief 计算电角速度
	  */
	void GetEleAngularSpeed(void)
	{
		float eleAngleDiff_rad = 0;
		static float eleAngleLast = 0;
		
		eleAngleDiff_rad = util_norm_float(PosSensor.EleAngle_rad - eleAngleLast, -PI, PI, 2.f * PI);
		
		PosSensor.EleAngularSpeed_rad = eleAngleDiff_rad / CARRIER_PERIOD_S;
		
		PosSensor.EleAngularSpeed_degree = RadToDegree(PosSensor.EleAngularSpeed_rad);
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
		
		data = PosSensor.EleAngularSpeed_rad;
		
		array[pos] = data;
		
		sum = (sum - old) + data;
		
		Avg = sum / num;

		pos = (pos+1) % num;

		PosSensor.EleAngularSpeed_rad = Avg;
		
		PosSensor.AvgEleAngularSpeed_degree = RadToDegree(PosSensor.EleAngularSpeed_rad);
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
		static float hall1[(uint8_t)MOTOR_POLE_PAIRS][DIVIDE_NUM] = {0};
		static float hall2[(uint8_t)MOTOR_POLE_PAIRS][DIVIDE_NUM] = {0};
		
		InverseParkTransform_TwoPhase(VolD, 0.f, &VolAlpha, &VolBeta, 0.f);	//设定Vq = 0, 电角度为零
		
		SpaceVectorModulation(VolAlpha, VolBeta);
		
		/*零点归一化处理*/
		PosSensor.HallNormalizedActualValue[0] = Normalized((PosSensor.HallZeroValue[0] - PosSensor.HallMinValue[0]) / (PosSensor.HallMaxValue[0] - PosSensor.HallMinValue[0]));
		PosSensor.HallNormalizedActualValue[1] = Normalized((PosSensor.HallZeroValue[1] - PosSensor.HallMinValue[1]) / (PosSensor.HallMaxValue[1] - PosSensor.HallMinValue[1]));
		
		HAL_Delay(1000);
		
		/*测量TEST_ROUNDS圈并取平均值以保证霍尔值准确度*/
		for(uint8_t round = 0; round < TEST_ROUNDS; round++)
		{
			/*共MOTOR_POLE_PAIRS个电气周期*/
			for (uint8_t pairs = 0; pairs < MOTOR_POLE_PAIRS; pairs++)
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
						
						hall1Value += PosSensor.HallActualValue[0] / SAMPLING_TIMES;
						hall2Value += PosSensor.HallActualValue[1] / SAMPLING_TIMES;
					}				
			
					hall1[pairs][divNum] += hall1Value / TEST_ROUNDS;
					hall2[pairs][divNum] += hall2Value / TEST_ROUNDS;
					
					/***********************解算电角度***********************CODE BEGIN**********************/
					
					PosSensor.HallNormalizedActualValue[0] = Normalized((PosSensor.HallActualValue[0] - PosSensor.HallMinValue[0]) / (PosSensor.HallMaxValue[0] - PosSensor.HallMinValue[0]));
					PosSensor.HallNormalizedActualValue[1] = Normalized((PosSensor.HallActualValue[1] - PosSensor.HallMinValue[1]) / (PosSensor.HallMaxValue[1] - PosSensor.HallMinValue[1]));

					#if	PHASE_SEQUENCE == NEGATIVE_SEQUENCE
						eleAngleCalculate = RadToDegree((atan2f(PosSensor.HallNormalizedActualValue[1] - PosSensor.HallNormalizedActualValue[1], PosSensor.HallNormalizedActualValue[0] - PosSensor.HallNormalizedActualValue[0]) * 57.29578f));
					#elif PHASE_SEQUENCE == POSITIVE_SEQUENCE
						eleAngleCalculate = RadToDegree((PosSensor.HallNormalizedActualValue[0] - PosSensor.HallNormalizedActualValue[0], PosSensor.HallNormalizedActualValue[1] - PosSensor.HallNormalizedActualValue[1]) * 57.29578f));
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
		int32_t max1[(uint8_t)MOTOR_POLE_PAIRS] = {0};
		int32_t max2[(uint8_t)MOTOR_POLE_PAIRS] = {0};
		int32_t min1[(uint8_t)MOTOR_POLE_PAIRS] = {0};
		int32_t min2[(uint8_t)MOTOR_POLE_PAIRS] = {0};
		int32_t zero1[(uint8_t)MOTOR_POLE_PAIRS] = {0};
		int32_t zero2[(uint8_t)MOTOR_POLE_PAIRS] = {0};
		uint32_t maxIndex = 0, minIndex = 0;
		
		HAL_Delay(100);
		
		/*计算出霍尔值中的最大值、最小值、零点值*/
		for(int pairs = 0; pairs < MOTOR_POLE_PAIRS; pairs++)
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
		
		PosSensor.HallMaxValue[0] = 0;
		PosSensor.HallMaxValue[1] = 0;
		PosSensor.HallMinValue[0] = 0;
		PosSensor.HallMinValue[1] = 0;
		PosSensor.HallZeroValue[0] = 0;
		PosSensor.HallZeroValue[1] = 0;	
		
		for(int pairs = 0; pairs < MOTOR_POLE_PAIRS; pairs++)
		{
			PosSensor.HallMaxValue[0] += max1[pairs] / MOTOR_POLE_PAIRS;
			PosSensor.HallMaxValue[1] += max2[pairs] / MOTOR_POLE_PAIRS;
			PosSensor.HallMinValue[0] += min1[pairs] / MOTOR_POLE_PAIRS;
			PosSensor.HallMinValue[1] += min2[pairs] / MOTOR_POLE_PAIRS;
			PosSensor.HallZeroValue[0] += zero1[pairs] / MOTOR_POLE_PAIRS;
			PosSensor.HallZeroValue[1] += zero2[pairs] / MOTOR_POLE_PAIRS;
		}
		
		PutStr("write in Hall Init:\r\n");SendBuf();
		
		HAL_Delay(10);
		
		UART_Transmit_DMA("%d\t%d\t%d\t%d\t%d\t%d\t\r\n",(int)PosSensor.HallMaxValue[0], (int)PosSensor.HallMaxValue[1], (int)PosSensor.HallMinValue[0], (int)PosSensor.HallMinValue[1], (int)PosSensor.HallZeroValue[0], (int)PosSensor.HallZeroValue[1]);
		
		SendBuf();
		
		HAL_Delay(50);
		
		/***********************获取霍尔最大最小值及零点***********************CODE END*************/
		
		/***************************获取霍尔补偿表格***************************CODE BEGIN***********/
		
		int angLast = 0;
		
		InverseParkTransform_TwoPhase(VolD, 0.f, &VolAlpha, &VolBeta, 0.f);
					
		SpaceVectorModulation(VolAlpha, VolBeta);
		
		/*零点归一化处理*/
		PosSensor.HallNormalizedActualValue[0] = Normalized((PosSensor.HallZeroValue[0] - PosSensor.HallMinValue[0]) / (PosSensor.HallMaxValue[0] - PosSensor.HallMinValue[0]));
		PosSensor.HallNormalizedActualValue[1] = Normalized((PosSensor.HallZeroValue[1] - PosSensor.HallMinValue[1]) / (PosSensor.HallMaxValue[1] - PosSensor.HallMinValue[1]));
		
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
				hall1Value += PosSensor.HallActualValue[0] / SAMPLING_TIMES;//ADC2
				hall2Value += PosSensor.HallActualValue[1] / SAMPLING_TIMES;
			}

			HallCalibrateTable[0][div] = div * 360.f / CALIBRATE_NUM;  
			
			/***********************解算电角度***********************CODE BEGIN**********************/
					
			PosSensor.HallNormalizedActualValue[0] = Normalized((PosSensor.HallActualValue[0] - PosSensor.HallMinValue[0]) / (PosSensor.HallMaxValue[0] - PosSensor.HallMinValue[0]));
			PosSensor.HallNormalizedActualValue[1] = Normalized((PosSensor.HallActualValue[1] - PosSensor.HallMinValue[1]) / (PosSensor.HallMaxValue[1] - PosSensor.HallMinValue[1]));

			#if	PHASE_SEQUENCE == NEGATIVE_SEQUENCE
			
				eleAngleCalculate = RadToDegree((atan2f(PosSensor.HallNormalizedActualValue[1] - PosSensor.HallNormalizedActualValue[1], PosSensor.HallNormalizedActualValue[0] - PosSensor.HallNormalizedActualValue[0]) * 57.29578f));
			
			#elif PHASE_SEQUENCE == POSITIVE_SEQUENCE
				
				eleAngleCalculate = RadToDegree((PosSensor.HallNormalizedActualValue[0] - PosSensor.HallNormalizedActualValue[0], PosSensor.HallNormalizedActualValue[1] - PosSensor.HallNormalizedActualValue[1]) * 57.29578f));
			
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

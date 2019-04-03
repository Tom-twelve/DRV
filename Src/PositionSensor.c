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
		
		GetMecAngle_AbsoluteMode_15bit();  //����ʽ, ��ȡ�������Ƕ�ֵ�Ĵ���
		
		#elif ENCODER_MODE == Encoder_IncrementalMode
		
		GetMecAngle_IncrementalMode_14bit();  //����ʽ, TLE5012��12λ����ʽ����������4��Ƶ�󾫶������14λ
		
		#else
		#error "Encoder Mode Invalid"
		#endif
		GetMecAngle(); //�����е�Ƕ�
		GetMecAngularSpeed(); //�����е���ٶ�
		GetAvgMecAngularSpeed(); //�����е���ٶȾ�ֵ
		GetEleAngle(); //�����Ƕ�
		GetAvgEleAngularSpeed();  //�������ٶȾ�ֵ
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
		/*����TIM2����ʽ������ģʽ*/
		HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
		
		/*��ȡ��ʼ��е�Ƕ�*/
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

		InverseParkTransform(VolD, 0.f, &VolAlpha, &VolBeta, 0.f);	//�趨Uq = 0, ��Ƕ�Ϊ��
		
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
		
		/*	���͵�Ƕȱ�	*/
		
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
		GetEleAngle(); //�����Ƕ�
		GetEleAngularSpeed();  //�������ٶ�
		GetAvgEleAngularSpeed();	//�������ٶȾ�ֵ
		GetAvgMecAngularSpeed();	//�����е���ٶȾ�ֵ
	}
	
	void HallInit(void)
	{
		/*��ʼ�������ı궨ֵ*/
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
		
		/*����һ������*/
		PositionSensor.HallNormalizedActualValue[0] = Normalized((PositionSensor.HallZeroValue[0] - PositionSensor.HallMinValue[0]) / (PositionSensor.HallMaxValue[0] - PositionSensor.HallMinValue[0]));
		PositionSensor.HallNormalizedActualValue[1] = Normalized((PositionSensor.HallZeroValue[1] - PositionSensor.HallMinValue[1]) / (PositionSensor.HallMaxValue[1] - PositionSensor.HallMinValue[1]));
		
		HAL_Delay(10);
		
		
	}

	/**
	  * @brief �����е���ٶȾ�ֵ
	  */
	void GetAvgMecAngularSpeed()
	{
		PositionSensor.AvgMecAngularSpeed_rad = PositionSensor.AvgEleAngularSpeed_rad / MotorPolePairs;
		
		PositionSensor.AvgMecAngularSpeed_degree = RadToDegree(PositionSensor.AvgMecAngularSpeed_rad);
	}
	
	/**
	  * @brief �����Ƕ�
	  */
	void GetEleAngle(void)
	{
		static float eleAngleCalculate = 0;
		
		/*��һ������, ע�⸡����ǿ��ת��ʱ��ķ�������*/
		PositionSensor.HallNormalizedActualValue[0] = Normalized((PositionSensor.HallActualValue[0] - PositionSensor.HallMinValue[0]) / (PositionSensor.HallMaxValue[0] - PositionSensor.HallMinValue[0]));
		PositionSensor.HallNormalizedActualValue[1] = Normalized((PositionSensor.HallActualValue[1] - PositionSensor.HallMinValue[1]) / (PositionSensor.HallMaxValue[1] - PositionSensor.HallMinValue[1]));

		#if	PHASE_SEQUENCE == POSITIVE_SEQUENCE
		
		eleAngleCalculate = (atan2f(PositionSensor.HallNormalizedActualValue[0] - PositionSensor.HallNormalizedZeroValue[0], PositionSensor.HallNormalizedActualValue[1] - PositionSensor.HallNormalizedZeroValue[1]) * 57.29578f);
		
		#elif PHASE_SEQUENCE == NEGATIVE_SEQUENCE
		
		eleAngleCalculate = (atan2f(PositionSensor.HallNormalizedActualValue[1] - PositionSensor.HallNormalizedZeroValue[1], PositionSensor.HallNormalizedActualValue[0] - PositionSensor.HallNormalizedZeroValue[0]) * 57.29578f);
		
		#endif
		
		/*��������õ�����������벻ʹ�ò��ʱ����һ�£�r^2��Ϊ1������˲�����������ʱ��䳤������������*/
		eleAngleCalculate = RadToDegree(HallAngleTableComp(eleAngleCalculate)); 
		
		PositionSensor.EleAngle_rad = eleAngleCalculate;
		
		PositionSensor.EleAngle_degree = RadToDegree(eleAngleCalculate);
	}
	
	/**
	  * @brief �������ٶ�
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
	  * @brief �������ٶȾ�ֵ
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
	  * @brief ���������
	  * @param[in]  hallValueSin  			��Ϊsin�Ļ���ֵ
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
	  * @brief ������Ƕ�
	  * @param[in]  VolD  			����d���ѹ
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
		
		InverseParkTransform_TwoPhase(VolD, 0.f, &VolAlpha, &VolBeta, 0.f);	//�趨Vq = 0, ��Ƕ�Ϊ��
		
		SpaceVectorModulation(VolAlpha, VolBeta);
		
		/*����һ������*/
		PositionSensor.HallNormalizedActualValue[0] = Normalized((PositionSensor.HallZeroValue[0] - PositionSensor.HallMinValue[0]) / (PositionSensor.HallMaxValue[0] - PositionSensor.HallMinValue[0]));
		PositionSensor.HallNormalizedActualValue[1] = Normalized((PositionSensor.HallZeroValue[1] - PositionSensor.HallMinValue[1]) / (PositionSensor.HallMaxValue[1] - PositionSensor.HallMinValue[1]));
		
		HAL_Delay(1000);
		
		/*����TEST_ROUNDSȦ��ȡƽ��ֵ�Ա�֤����ֵ׼ȷ��*/
		for(uint8_t round = 0; round < TEST_ROUNDS; round++)
		{
			/*��MotorPolePairs����������*/
			for (uint8_t pairs = 0; pairs < MotorPolePairs; pairs++)
			{
				/*ÿ�����������з�ΪDIVIDE_NUM��*/
				for(uint8_t divNum = 0; divNum < DIVIDE_NUM; divNum ++)
				{
					eleAngle = divNum * 360 / DIVIDE_NUM;
					
					InverseParkTransform_TwoPhase(VolD, 0.f, &VolAlpha, &VolBeta, eleAngle);
					
					SpaceVectorModulation(VolAlpha, VolBeta);
					
					/*ÿ����ȡSAMPLING_TIMES������ֵȡ��ƽ��ֵ*/
					for(int times = 0 ; times < SAMPLING_TIMES; times++)
					{
						/*��ʱ�Ա�֤ADCֵ�Ѹ���*/
						HAL_Delay(10);
						
						hall1Value += PositionSensor.HallActualValue[0] / SAMPLING_TIMES;
						hall2Value += PositionSensor.HallActualValue[1] / SAMPLING_TIMES;
					}				
			
					hall1[pairs][divNum] += hall1Value / TEST_ROUNDS;
					hall2[pairs][divNum] += hall2Value / TEST_ROUNDS;
					
					/***********************�����Ƕ�***********************CODE BEGIN**********************/
					
					PositionSensor.HallNormalizedActualValue[0] = Normalized((PositionSensor.HallActualValue[0] - PositionSensor.HallMinValue[0]) / (PositionSensor.HallMaxValue[0] - PositionSensor.HallMinValue[0]));
					PositionSensor.HallNormalizedActualValue[1] = Normalized((PositionSensor.HallActualValue[1] - PositionSensor.HallMinValue[1]) / (PositionSensor.HallMaxValue[1] - PositionSensor.HallMinValue[1]));

					#if	PHASE_SEQUENCE == NEGATIVE_SEQUENCE
						eleAngleCalculate = RadToDegree((atan2f(PositionSensor.HallNormalizedActualValue[1] - PositionSensor.HallNormalizedActualValue[1], PositionSensor.HallNormalizedActualValue[0] - PositionSensor.HallNormalizedActualValue[0]) * 57.29578f));
					#elif PHASE_SEQUENCE == POSITIVE_SEQUENCE
						eleAngleCalculate = RadToDegree((PositionSensor.HallNormalizedActualValue[0] - PositionSensor.HallNormalizedActualValue[0], PositionSensor.HallNormalizedActualValue[1] - PositionSensor.HallNormalizedActualValue[1]) * 57.29578f));
					#endif
					
					/***********************�����Ƕ�***********************CODE END************************/
					
					UART_Transmit_DMA("%d\t%d\t%d\t",(int)eleAngle,(int)hall1Value,(int)hall2Value);
					UART_Transmit_DMA("%d\r\n",(int)eleAngleCalculate);

					SendBuf();
					
					HAL_Delay(100);//һ��SendBuf����Ҫ����ʱ���DMA���ͣ������overflow
					
					hall1Value = 0;
					hall2Value = 0;
				}
			}
			HAL_Delay(5);
		}
		
		/***********************��ȡ���������Сֵ�����***********************CODE BEGIN***********/
		
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
		
		/*���������ֵ�е����ֵ����Сֵ�����ֵ*/
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
		
		/***********************��ȡ���������Сֵ�����***********************CODE END*************/
		
		/***************************��ȡ�����������***************************CODE BEGIN***********/
		
		int angLast = 0;
		
		InverseParkTransform_TwoPhase(VolD, 0.f, &VolAlpha, &VolBeta, 0.f);
					
		SpaceVectorModulation(VolAlpha, VolBeta);
		
		/*����һ������*/
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
			
			/***********************�����Ƕ�***********************CODE BEGIN**********************/
					
			PositionSensor.HallNormalizedActualValue[0] = Normalized((PositionSensor.HallActualValue[0] - PositionSensor.HallMinValue[0]) / (PositionSensor.HallMaxValue[0] - PositionSensor.HallMinValue[0]));
			PositionSensor.HallNormalizedActualValue[1] = Normalized((PositionSensor.HallActualValue[1] - PositionSensor.HallMinValue[1]) / (PositionSensor.HallMaxValue[1] - PositionSensor.HallMinValue[1]));

			#if	PHASE_SEQUENCE == NEGATIVE_SEQUENCE
			
				eleAngleCalculate = RadToDegree((atan2f(PositionSensor.HallNormalizedActualValue[1] - PositionSensor.HallNormalizedActualValue[1], PositionSensor.HallNormalizedActualValue[0] - PositionSensor.HallNormalizedActualValue[0]) * 57.29578f));
			
			#elif PHASE_SEQUENCE == POSITIVE_SEQUENCE
				
				eleAngleCalculate = RadToDegree((PositionSensor.HallNormalizedActualValue[0] - PositionSensor.HallNormalizedActualValue[0], PositionSensor.HallNormalizedActualValue[1] - PositionSensor.HallNormalizedActualValue[1]) * 57.29578f));
			
			#endif
			
			HallCalibrateTable[1][div] = eleAngleCalculate;
			
			/***********************�����Ƕ�***********************CODE END************************/

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
		
		/***************************��ȡ�����������***************************CODE END*************/
		
		HAL_Delay(250);
		
		/*ʧ�����������*/
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

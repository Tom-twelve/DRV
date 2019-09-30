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
	extern struct CAN_t CAN;
#else
#error "Position Sensor Type Invalid"
#endif

/* CODE END PV */

/* USER CODE BEGIN */

#if	POSITION_SENSOR_TYPE == ENCODER_TLE5012
	void GetEleImformation(void)
	{
		#if	ENCODER_MODE == ENCODER_ABSOLUTE_MODE
		
		GetMecAngle_AbsoluteMode_15bit();  //绝对式, 读取编码器角度值寄存器
		
		#elif ENCODER_MODE == ENCODER_INCREMENTAL_MODE
		
		GetMecAngle_IncrementalMode_14bit();  //增量式, TLE5012的12位增量式编码器经过4倍频后精度提高至14位
		
		#else
		#error "Encoder Mode Invalid"
		#endif
		GetEleAngle(); //计算电角度
		GetEleAngularSpeed();  //计算电角速度
		EncoderLostDetection();		//编码器异常检测
	}

	void GetMecImformation(void)
	{
		GetMecAngle(); //计算机械角度
		GetMecAngularSpeed(); //计算机械角速度
		GetRefMecAngle(); //计算参考机械角度（主控用）
	}
	
	void GetMecAngle_AbsoluteMode_15bit(void)
	{
		PosSensor.MecAngle_AbsoluteMode_15bit = (TLE5012_ReadRegister(TLE5012_COMMAND_READ_CURRENT_VALUE_ANGLE, &PosSensor.SafetyWord)) & 0x7FFF;
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
		
		PosSensor.MecAngle_degree = 360.f * (float)PosSensor.MecAngle_IncrementalMode_14bit / (TLE5012_INCREMENTAL_MODE_RESOLUTION * 4.f);
		
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
		const uint16_t FilterOrder = 10;
		
		float angleDifference = 0;
		float presentMecAngle = 0;
		static float lastMecAngle = 0;		
		static float array[FilterOrder] = {0};
		static float sum = 0.f;
		static float avg = 0.f;
		static float old = 0.f;
		static uint16_t pos = 0;
		
		presentMecAngle = PosSensor.MecAngle_rad;
		
		angleDifference = presentMecAngle - lastMecAngle;
		
		lastMecAngle = presentMecAngle;
		
		while(angleDifference > PI || angleDifference < -PI)
		{
			if(angleDifference > PI)
			{
				angleDifference -= 2.f * PI;
			}
			
			else if(angleDifference < -PI)
			{
				angleDifference += 2.f * PI;
			}
		}
		
		old = array[pos];
		
		array[pos] = angleDifference / (Regulator.ActualPeriod_s * PERIOD_MULTIPLE);
			
		sum = (sum - old) + array[pos];
			
		avg = sum / FilterOrder;

		pos = (pos+1) % FilterOrder;
		
		PosSensor.MecAngularSpeed_rad = avg;
	}

	void GetEleAngle(void)
	{
		float normPos = 0;
		
		#if ENCODER_MODE == ENCODER_ABSOLUTE_MODE
				
		PosSensor.EleAngle_degree = ((float)(PosSensor.MecAngle_AbsoluteMode_15bit - PosSensor.PosOffset) / (float)(TLE5012_ABS_MODE_RESOLUTION / MOTOR_POLE_PAIRS_NUM)) * 360.f;
		
		PosSensor.EleAngle_rad = DEGREE_TO_RAD(PosSensor.EleAngle_degree);
		
		#elif ENCODER_MODE == ENCODER_INCREMENTAL_MODE
		
		#else
		#error "Encoder Mode Invalid"
		#endif
	}

	void GetEleAngularSpeed(void)
	{
		const uint16_t FilterOrder = 6;
		
		float angleDifference = 0;
		float presentEleAngle = 0;
		static float lastEleAngle = 0;
		static float array[FilterOrder] = {0};
		static float sum = 0.f;
		static float avg = 0.f;
		static float old = 0.f;
		static uint16_t pos = 0;
		
		presentEleAngle = PosSensor.EleAngle_rad;
		
		angleDifference = presentEleAngle - lastEleAngle;
		
		lastEleAngle = presentEleAngle;
		
		while(angleDifference > PI || angleDifference < -PI)
		{
			if(angleDifference > PI)
			{
				angleDifference -= 2.f * PI;
			}
			
			else if(angleDifference < -PI)
			{
				angleDifference += 2.f * PI;
			}
		}
		
		old = array[pos];
		
		array[pos] = angleDifference / Regulator.ActualPeriod_s;
			
		sum = (sum - old) + array[pos];
			
		avg = sum / FilterOrder;

		pos = (pos+1) % FilterOrder;
		
		PosSensor.EleAngularSpeed_rad = avg;
		
		PosSensor.EleAngularSpeed_degree = RAD_TO_DEGREE(PosSensor.EleAngularSpeed_rad);
	}

	void TLE5012_ReadFSYNC(void)
	{
		PosSensor.FSYNC = (TLE5012_ReadRegister(TLE5012_COMMAND_READ_CURRENT_VALUE_FSYNC, &PosSensor.SafetyWord)) >> 9;
	}
	
	void EncoderLostDetection(void)
	{
		static uint8_t count = 0;
		
		TLE5012_ReadFSYNC();
		
		PosSensor.SafetyWord >>= 12;
		
		/*通过TLE5012的SafetyWord判断编码器是否异常*/
		if(PosSensor.SafetyWord != 0x07)
		{
			count++;
			
			if(count > 10)
			{
				PWM_IT_CMD(DISABLE,ENABLE);
				
				/*通过CAN总线告知主控编码器异常*/
				CAN.Identifier = 0x4C45;	//EL
				CAN.TransmitData = PosSensor.SafetyWord;
				
				CAN.Transmit.data_uint8[0] = (CAN.Identifier>>0)&0xff;
				CAN.Transmit.data_uint8[1] = (CAN.Identifier>>8)&0xff;
				CAN.Transmit.data_uint8[2] = (CAN.TransmitData>>0)&0xff;
				
				while(1)
				{
					CAN_Transmit(CAN.Transmit, 3);
				
					PutStr("ENCODER LOST:");	
					PutNum((uint16_t)PosSensor.SafetyWord, '\n');
					
					SendBuf();
					
					LL_mDelay(10);
				}
			}
		}
		else
		{
			if(count > 0)	
			{
				count--;
			}
		}
	}
	
	uint16_t TLE5012_ReadRegister(uint16_t command, uint16_t *safetyWord)
	{
		uint16_t data = 0;
		
		TLE5012_SPI1_CHIP_SELECT;
		
		SPI_Transmit(SPI1, command, TimeOut);
		SPI_TX_OFF;
		
		SPI_Receive(SPI1, &data, TimeOut);
		SPI_Receive(SPI1, safetyWord, TimeOut);
		
		TLE5012_SPI1_CHIP_DISELECT;
		SPI_TX_ON;

		return data;
	}
	
	void PosSensor_Init(void)
	{
		/*控制周期, 务必在所有运算开始之前赋值*/
		Regulator.ActualPeriod_s = DEFAULT_CARRIER_PERIOD_s;
		
		for(uint16_t times = 0; times <= 50; times++)
		{
			/*更新位置及速度, 防止上电位置跳动, 由于有均值滤波器, 故需重复执行多次以使速度变量接近零*/
			GetEleImformation();
		
			GetMecImformation();
			
			LL_mDelay(1);
		}

		RefAngleInit();
	}
	
	void EncoderIncrementalModeEnable(void)
	{
		/*启动TIM2增量式编码器模式*/
		HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
		
		/*读取初始机械角度*/
		PosSensor.OriginalMecAngle_14bit = (uint16_t)(((float)(TLE5012_ReadRegister(TLE5012_COMMAND_READ_CURRENT_VALUE_ANGLE, &PosSensor.SafetyWord) & 0x7FFF) / 32768.f) * 16384.f);
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
		
		/*通过旋转方向判断相序是否正确, 从编码器方向看, 应为顺时针旋转*/
		for(float eleAngle = 0; eleAngle <= 360; eleAngle += 15)
		{
			
			InverseParkTransform(volD, 0.f, &volAlpha, &volBeta, eleAngle);
			SpaceVectorModulation(volAlpha, volBeta);
			
			LL_mDelay(5);
			
			GetMecAngle_AbsoluteMode_15bit();
			UART_Transmit_DMA("EleAngle	%d\tMecPosition %d\r\n", (int)eleAngle, (int)PosSensor.MecAngle_AbsoluteMode_15bit);SendBuf();
			
			LL_mDelay(100);
		}
		
		InverseParkTransform(volD, 0.f, &volAlpha, &volBeta, 0);
		SpaceVectorModulation(volAlpha, volBeta);
		
		GetMecAngle_AbsoluteMode_15bit();
		
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
					
					UART_Transmit_DMA("Position Offset:	%d\r\nCorrect Finished", (int)PosSensor.PosOffset);SendBuf();
					
					break;
				}
			}
			
			LL_mDelay(5);
		}
		
		PWM_IT_CMD(DISABLE, DISABLE);
		
		while(1)
		{
			LL_mDelay(1);
		}
	}
	
#else
#error "Position Sensor Type Invalid"
#endif
  
/* USER CODE END */

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

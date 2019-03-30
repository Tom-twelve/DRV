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
#include "Encoder.h"
/* CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* CODE BEGIN PTD */

/* CODE END PTD */

/* Private macro -------------------------------------------------------------*/
/* CODE BEGIN PM */

/* CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* CODE BEGIN PV */
uint16_t tempAngle = 0;
extern const short int EleAngleRef[DivideNum * (uint8_t)MotorMagnetPairs + 2];
extern const int MecAngleRef[DivideNum * (uint8_t)MotorMagnetPairs + 2];
struct Encoder_t Encoder;

/* CODE END PV */

/* USER CODE BEGIN */

void GetPositionImformation(void)
{
	#if ENCODER_TYPE == Encoder_TLE5012
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
	#else
	#error "Encoder Type Invalid"
	#endif
}

#if ENCODER_TYPE == Encoder_TLE5012
	void GetMecAngle_AbsoluteMode_15bit(void)
	{
		Encoder.MecAngle_AbsoluteMode_15bit = (TLE5012_ReadRegister(TLE5012_Command_ReadCurrentValue_AngleValue) & 0x7FFF);
	}

	void GetMecAngle_IncrementalMode_14bit(void)
	{
		Encoder.MecAngle_IncrementalMode_14bit = (uint16_t)(Encoder.OriginalMecAngle_14bit + TIM2->CNT);
		
		if(Encoder.MecAngle_IncrementalMode_14bit >= 16384)
		{
			Encoder.MecAngle_IncrementalMode_14bit -= 16384;
		}
	}
	
	void GetMecAngle(void)
	{
		#if	ENCODER_MODE == Encoder_AbsoluteMode
		
		Encoder.MecAngle_degree = 360.f * (float)Encoder.MecAngle_AbsoluteMode_15bit / TLE5012_AbsoluteModeResolution;
		
		Encoder.MecAngle_rad = (float)Encoder.MecAngle_degree / 360.f * 2.0f * PI;
		
		#elif ENCODER_MODE == Encoder_IncrementalMode
		
		Encoder.MecAngle_degree = 360.f * (float)Encoder.MecAngle_IncrementalMode_14bit / (TLE5012_IncrementalModeResolution * 4.f);
		
		Encoder.MecAngle_rad = (float)Encoder.MecAngle_degree / 360.f * 2.0f * PI;
		
		#else
		#error "Encoder Mode Invalid"
		#endif
	}

	void GetMecAngularSpeed(void)
	{
		float presentMecAngle = 0;
		static float lastMecAngle = 0;
		float angleDifference = 0;
		
		presentMecAngle = Encoder.MecAngle_rad;
		
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
		
		Encoder.MecAngularSpeed_rad = angleDifference / TLE5012_UpdateTime_0;
		
		#elif ENCODER_MODE == Encoder_IncrementalMode
		
		Encoder.MecAngularSpeed_rad = angleDifference / CarrierPeriod_s;
		
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
		
		data = Encoder.MecAngularSpeed_rad;
		
		array[pos] = data;
		
		sum = (sum - old) + data;
		
		Avg = sum / num;

		pos = (pos+1) % num;

		Encoder.AvgMecAngularSpeed_rad = Avg;
	}

	void GetEleAngle(void)
	{
		float normPos = 0;
		
		#if ENCODER_MODE == Encoder_AbsoluteMode
		
		normPos = fmodf(Encoder.MecAngle_AbsoluteMode_15bit, TLE5012_AbsoluteModeResolution);	
		
		uint32_t index = UtilBiSearchInt(MecAngleRef, normPos, sizeof(MecAngleRef)/sizeof(MecAngleRef[0]));

		uint32_t indexPlus1 = index + 1;
		
		Encoder.EleAngle_degree = fmodf(utils_map(Encoder.MecAngle_AbsoluteMode_15bit, MecAngleRef[index], MecAngleRef[indexPlus1], EleAngleRef[index], EleAngleRef[indexPlus1]), 360.f);
		
		#elif ENCODER_MODE == Encoder_IncrementalMode
		
		normPos = fmodf(Encoder.MecAngle_IncrementalMode_14bit, TLE5012_IncrementalModeResolution * 4.f);	
		
		uint32_t index = UtilBiSearchInt(MecAngleRef, normPos, sizeof(MecAngleRef)/sizeof(MecAngleRef[0]));

		uint32_t indexPlus1 = index + 1;
		
		Encoder.EleAngle_degree = fmodf(utils_map(Encoder.MecAngle_IncrementalMode_14bit, MecAngleRef[index], MecAngleRef[indexPlus1], EleAngleRef[index], EleAngleRef[indexPlus1]), 360.f);
		
		#else
		#error "Encoder Mode Invalid"
		#endif
	}

	void GetAvgEleAngularSpeed(void)
	{
		Encoder.AvgEleAngularSpeed_rad = Encoder.AvgMecAngularSpeed_rad * MotorMagnetPairs;
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
		Encoder.OriginalMecAngle_14bit = (uint16_t)(((float)(TLE5012_ReadRegister(TLE5012_Command_ReadCurrentValue_AngleValue) & 0x7FFF) / 32768.f) * 16384.f);
	}
	
#else
#error "Encoder Type Invalid"
#endif
  
/* USER CODE END */

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

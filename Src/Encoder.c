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

extern const short int EleAngleRef[DivideNum * (uint8_t)MotorMagnetPairs + 2];
extern const int MecAngleRef[DivideNum * (uint8_t)MotorMagnetPairs + 2];
struct Encoder_t Encoder;

/* CODE END PV */

/* USER CODE BEGIN */

void GetPositionImformation(void)
{
	#if EncoderType == Encoder_TLE5012
		GetMecAngle_15bit();	//读取编码器角度值寄存器
		GetMecAngle_degree(); //计算角度制机械角度(0~360)
		GetMecAngle_rad(); //计算弧度制机械角度(0~360)
		GetMecAngularSpeed_degree(); //计算角度制机械角速度
		GetMecAngularSpeed_rad(); //计算弧度制机械角速度
		GetAvgMecAngularSpeed_degree(); //计算角度制机械角速度均值
		GetAvgMecAngularSpeed_rad(); //计算弧度制机械角速度均值
		CalculateEleAngle_degree(); //计算角度制电角度(0~360 * p)
		CalculateEleAngle_rad(); //计算弧度制电角度(0~360 * p)
		GetEleAngularSpeed_degree(); //计算角度制电角速度
		GetEleAngularSpeed_rad(); //计算弧度制电角速度
		GetAvgEleAngularSpeed_degree(); //计算角度制电角速度均值
		GetAvgEleAngularSpeed_rad(); //计算弧度制电角速度均值
	#else
	#error "Encoder Type Invalid"
	#endif
}

#if EncoderType == Encoder_TLE5012
	void GetMecAngle_15bit(void)
	{
		Encoder.MecAngle_15bit = (TLE5012_ReadRegister(TLE5012_Command_ReadCurrentValue_AngleValue) & 0x7FFF);
	}

	void GetMecAngle_degree(void)
	{
		Encoder.MecAngle_degree = 360.f * (float)Encoder.MecAngle_15bit / 32768.0f;
	}

	void GetMecAngle_rad(void)
	{
		Encoder.MecAngle_rad = (float)Encoder.MecAngle_15bit / 32768.0f * 2.0f * PI;
	}

	void GetMecAngularSpeed_degree(void)
	{
		float presentMecAngle = 0;
		static float lastMecAngle = 0;
		float angleDifference = 0;
		
		presentMecAngle = Encoder.MecAngle_degree;
		
		angleDifference = presentMecAngle - lastMecAngle;
		
		if(angleDifference > 180.f)
		{
			angleDifference -= 360.f;
		}
		
		else if(angleDifference < -180.f)
		{
			angleDifference += 360.f;
		}
		
		Encoder.MecAngularSpeed_degree = angleDifference / TLE5012_UpdateTime_2;
		
		lastMecAngle = presentMecAngle;
	}

	void GetMecAngularSpeed_rad(void)
	{
		Encoder.MecAngularSpeed_rad = (Encoder.MecAngularSpeed_degree / 360.f) * 2.0f * PI;
	}

	void GetAvgMecAngularSpeed_degree(void)
	{
		const uint8_t num = 6;
		static float array[num] = {0};
		static uint8_t pos = 0;
		static float data = 0.0f;
		static float sum = 0.0f;
		static float Avg = 0.0f;
		float old = array[pos];
		
		data = Encoder.MecAngularSpeed_degree;
		
		array[pos] = data;
		
		sum = (sum - old) + data;
		
		Avg = sum / num;

		pos = (pos+1) % num;

		Encoder.AvgMecAngularSpeed_degree = Avg;
	}

	void GetAvgMecAngularSpeed_rad(void)
	{
		Encoder.AvgMecAngularSpeed_rad = (Encoder.AvgMecAngularSpeed_degree / 360.f) * 2.0f * PI;
	}

	void CalculateEleAngle_degree(void)
	{
		float normPos = fmodf(Encoder.MecAngle_15bit, 32768);	
		
		uint32_t index = UtilBiSearchInt(MecAngleRef, normPos, sizeof(MecAngleRef)/sizeof(MecAngleRef[0]));

		uint32_t indexPlus1 = index + 1;
		
		Encoder.EleAngle_degree = fmodf(utils_map(Encoder.MecAngle_15bit, MecAngleRef[index], MecAngleRef[indexPlus1], EleAngleRef[index], EleAngleRef[indexPlus1]), 360);
	}

	void CalculateEleAngle_rad(void)
	{
		Encoder.EleAngle_rad = (Encoder.EleAngle_degree / 360.f) * 2.0f * PI;
	}

	void GetEleAngularSpeed_degree(void)
	{
		Encoder.EleAngularSpeed_degree = Encoder.MecAngularSpeed_degree * MotorMagnetPairs;
	}

	void GetEleAngularSpeed_rad(void)
	{
		Encoder.EleAngularSpeed_rad = (Encoder.EleAngularSpeed_degree / 360.f) * 2.0f * PI;
	}

	void GetAvgEleAngularSpeed_degree(void)
	{
		Encoder.AvgEleAngularSpeed_degree = Encoder.AvgMecAngularSpeed_degree * MotorMagnetPairs;
	}

	void GetAvgEleAngularSpeed_rad(void)
	{
		Encoder.AvgEleAngularSpeed_rad = (Encoder.AvgEleAngularSpeed_degree / 360.f) * 2.0f * PI;
	}

	void GetMecAngularSpeed_Encoder_15bit(void)
	{
		uint16_t data = 0;
		int16_t val = 0;
		data = TLE5012_ReadRegister(TLE5012_Command_ReadCurrentValue_AngularSpeed);
		// uint16 left shift
		// Pay attention to the type convertion :unsigned to sign
		val = (data << 1);
		// Sign16 right shift 符号位不动
		Encoder.MecAngularSpeed_Encoder_15bit = (val << 1);
	}

	void GetMecAngularSpeed_Encoder(void)
	{
		Encoder.MecAngularSpeed_Encoder = (float)Encoder.MecAngularSpeed_Encoder_15bit * 32.48776625f;
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
		HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	}
	
#else
#error "Encoder Type Invalid"
#endif
  
/* USER CODE END */

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

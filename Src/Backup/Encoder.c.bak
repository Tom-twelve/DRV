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

extern const short int ElectricalAngleRef[DivideNum * (uint8_t)MotorMagnetPairs + 2];
extern const int MechanicalAngleRef[DivideNum * (uint8_t)MotorMagnetPairs + 2];
struct Encoder_t Encoder;

/* CODE END PV */

/* USER CODE BEGIN */

void GetPositionImformation(void)
{
	#if EncoderType == Encoder_TLE5012
		GetMechanicalAngle_15bit();	//读取编码器角度值寄存器
		GetMechanicalAngle_degree(); //计算角度制机械角度(0~360)
		GetMechanicalAngle_rad(); //计算弧度制机械角度(0~360)
		GetMechanicalAngularSpeed_degree(); //计算角度制机械角速度
		GetMechanicalAngularSpeed_rad(); //计算弧度制机械角速度
		GetAverageMechanicalAngularSpeed_degree(); //计算角度制机械角速度均值
		GetAverageMechanicalAngularSpeed_rad(); //计算弧度制机械角速度均值
		CalculateElectricalAngle_degree(); //计算角度制电角度(0~360 * p)
		CalculateElectricalAngle_rad(); //计算弧度制电角度(0~360 * p)
		GetElectricalAngularSpeed_degree(); //计算角度制电角速度
		GetElectricalAngularSpeed_rad(); //计算弧度制电角速度
		GetAverageElectricalAngularSpeed_degree(); //计算角度制电角速度均值
		GetAverageElectricalAngularSpeed_rad(); //计算弧度制电角速度均值
	#else
	#error "Encoder Type Invalid"
	#endif
}

#if EncoderType == Encoder_TLE5012
	void GetMechanicalAngle_15bit(void)
	{
		Encoder.MechanicalAngle_15bit = (TLE5012_ReadRegister(TLE5012_Command_ReadCurrentValue_AngleValue) & 0x7FFF);
	}

	void GetMechanicalAngle_degree(void)
	{
		Encoder.MechanicalAngle_degree = 360.f * (float)Encoder.MechanicalAngle_15bit / 32768.0f;
	}

	void GetMechanicalAngle_rad(void)
	{
		Encoder.MechanicalAngle_rad = (float)Encoder.MechanicalAngle_15bit / 32768.0f * 2.0f * PI;
	}

	void GetMechanicalAngularSpeed_degree(void)
	{
		float presentMechanicalAngle = 0;
		static float lastMechanicalAngle = 0;
		float angleDifference = 0;
		
		presentMechanicalAngle = Encoder.MechanicalAngle_degree;
		
		angleDifference = presentMechanicalAngle - lastMechanicalAngle;
		
		if(angleDifference > 180.f)
		{
			angleDifference -= 360.f;
		}
		
		else if(angleDifference < -180.f)
		{
			angleDifference += 360.f;
		}
		
		Encoder.MechanicalAngularSpeed_degree = angleDifference / TLE5012_UpdateTime_2;
		
		lastMechanicalAngle = presentMechanicalAngle;
	}

	void GetMechanicalAngularSpeed_rad(void)
	{
		Encoder.MechanicalAngularSpeed_rad = (Encoder.MechanicalAngularSpeed_degree / 360.f) * 2.0f * PI;
	}

	void GetAverageMechanicalAngularSpeed_degree(void)
	{
		const uint8_t num = 6;
		static float array[num] = {0};
		static uint8_t pos = 0;
		static float data = 0.0f;
		static float sum = 0.0f;
		static float average = 0.0f;
		float old = array[pos];
		
		data = Encoder.MechanicalAngularSpeed_degree;
		
		array[pos] = data;
		
		sum = (sum - old) + data;
		
		average = sum / num;

		pos = (pos+1) % num;

		Encoder.AverageMechanicalAngularSpeed_degree = average;
	}

	void GetAverageMechanicalAngularSpeed_rad(void)
	{
		Encoder.AverageMechanicalAngularSpeed_rad = (Encoder.AverageMechanicalAngularSpeed_degree / 360.f) * 2.0f * PI;
	}

	void CalculateElectricalAngle_degree(void)
	{
		float normPos = fmodf(Encoder.MechanicalAngle_15bit, 32768);	
		
		uint32_t index = UtilBiSearchInt(MechanicalAngleRef, normPos, sizeof(MechanicalAngleRef)/sizeof(MechanicalAngleRef[0]));

		uint32_t indexPlus1 = index + 1;
		
		Encoder.ElectricalAngle_degree = fmodf(utils_map(Encoder.MechanicalAngle_15bit, MechanicalAngleRef[index], MechanicalAngleRef[indexPlus1], ElectricalAngleRef[index], ElectricalAngleRef[indexPlus1]), 360);
	}

	void CalculateElectricalAngle_rad(void)
	{
		Encoder.ElectricalAngle_rad = (Encoder.ElectricalAngle_degree / 360.f) * 2.0f * PI;
	}

	void GetElectricalAngularSpeed_degree(void)
	{
		Encoder.ElectricalAngularSpeed_degree = Encoder.MechanicalAngularSpeed_degree * MotorMagnetPairs;
	}

	void GetElectricalAngularSpeed_rad(void)
	{
		Encoder.ElectricalAngularSpeed_rad = (Encoder.ElectricalAngularSpeed_degree / 360.f) * 2.0f * PI;
	}

	void GetAverageElectricalAngularSpeed_degree(void)
	{
		Encoder.AverageElectricalAngularSpeed_degree = Encoder.AverageMechanicalAngularSpeed_degree * MotorMagnetPairs;
	}

	void GetAverageElectricalAngularSpeed_rad(void)
	{
		Encoder.AverageElectricalAngularSpeed_rad = (Encoder.AverageElectricalAngularSpeed_degree / 360.f) * 2.0f * PI;
	}

	void GetMechanicalAngularSpeed_Encoder_15bit(void)
	{
		uint16_t data = 0;
		int16_t val = 0;
		data = TLE5012_ReadRegister(TLE5012_Command_ReadCurrentValue_AngularSpeed);
		// uint16 left shift
		// Pay attention to the type convertion :unsigned to sign
		val = (data << 1);
		// Sign16 right shift 符号位不动
		Encoder.MechanicalAngularSpeed_Encoder_15bit = (val << 1);
	}

	void GetMechanicalAngularSpeed_Encoder(void)
	{
		Encoder.MechanicalAngularSpeed_Encoder = (float)Encoder.MechanicalAngularSpeed_Encoder_15bit * 32.48776625f;
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

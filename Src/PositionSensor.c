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
extern struct MainCtrl_t MainCtrl;
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
float lastESpeed;
#if	POSITION_SENSOR_TYPE == ENCODER_TLE5012
	void GetEleImformation(void)
	{		
		GetMecAngle_AbsoluteMode_15bit();  //����ʽ, ��ȡ�������Ƕ�ֵ�Ĵ���
		GetEleAngle(); //�����Ƕ�
		GetEleAngularSpeed();  //�������ٶ�
		EncodeErrorDetection();		//�������쳣���
	}
 
	void GetMecImformation(void)
	{
		GetMecAngle(); //�����е�Ƕ�
		GetMecAngularSpeed(); //�����е���ٶ�
		GetRefMecAngle(); //����ο���е�Ƕȣ������ã�
	}
	
	void GetMecAngle_AbsoluteMode_15bit(void)
	{
		PosSensor.MecAngle_15bit = (TLE5012_ReadRegister(TLE5012_COMMAND_READ_CURRENT_VALUE_ANGLE, &PosSensor.SafetyWord)) & 0x7FFF;
	}
	
	void GetMecAngle(void)
	{		
		PosSensor.MecAngle_degree = 360.f * (float)PosSensor.MecAngle_15bit / TLE5012_ABS_MODE_RESOLUTION;
		
		PosSensor.MecAngle_rad = (float)PosSensor.MecAngle_degree / 360.f * 2.0f * PI;
	}
	
	void GetRefMecAngle(void)
	{
		int delta = 0;
		
		MainCtrl.PresentMecAngle_pulse = PosSensor.MecAngle_15bit;
		
		delta = MainCtrl.PresentMecAngle_pulse - MainCtrl.LastMecAngle_pulse;
		
		MainCtrl.LastMecAngle_pulse = MainCtrl.PresentMecAngle_pulse;
		
		if(delta < -(TLE5012_ABS_MODE_RESOLUTION / 2))
		{
			delta += TLE5012_ABS_MODE_RESOLUTION;
		}		
		else if(delta > (TLE5012_ABS_MODE_RESOLUTION / 2))
		{
			delta -= TLE5012_ABS_MODE_RESOLUTION;
		}
		
		MainCtrl.RefMecAngle_pulse += delta;
		
		/*ֻ���������ֽڵ����ݸ�����, �޷��Է�ֹ�������*/
		if(MainCtrl.RefMecAngle_pulse > 1024 * 32768)
		{
			MainCtrl.RefMecAngle_pulse = 0;
		}
		else if(MainCtrl.RefMecAngle_pulse < -1024 * 32768)
		{
			MainCtrl.RefMecAngle_pulse = 0;
		}
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
		
		array[pos] = angleDifference / (DEFAULT_CARRIER_PERIOD_s * PERIOD_MULTIPLE);
			
		sum = (sum - old) + array[pos];
			
		avg = sum / FilterOrder;

		pos = (pos+1) % FilterOrder;
		
		PosSensor.MecAngularSpeed_rad = avg;
	}

	void GetEleAngle(void)
	{
		float normPos = 0;
		
		#if ENCODER_MODE == ENCODER_ABSOLUTE_MODE
				
		PosSensor.EleAngle_degree = ((float)(PosSensor.MecAngle_15bit - PosSensor.PosOffset) / (float)(TLE5012_ABS_MODE_RESOLUTION / MOTOR_POLE_PAIRS_NUM)) * 360.f;
		
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
//		UART_Transmit_DMA("P:%d\t%d\r\n",(int)PosSensor.EleAngle_degree,(int)lastEleAngle);
		lastEleAngle = presentEleAngle;
		
		lastESpeed =  angleDifference;
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
		
		array[pos] = angleDifference / DEFAULT_CARRIER_PERIOD_s;
			
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
	
	void EncodeErrorDetection(void)
	{
		static uint8_t count = 0;
		
		TLE5012_ReadFSYNC();
		
		PosSensor.SafetyWord >>= 12;
		
		/*ͨ��TLE5012��SafetyWord�жϱ������Ƿ��쳣*/
		if(PosSensor.SafetyWord != 0x07)
		{
			count++;
			
//			if(count > 10)
//			{
//				PWM_IT_CMD(DISABLE,ENABLE);
//				
//				/*ͨ��CAN���߸�֪���ر������쳣*/
//				CAN.Identifier = IDENTIFIER_ENCODER_ERROR;
//				CAN.TransmitData = PosSensor.SafetyWord;

//				while(1)
//				{
//					CAN_Transmit(CAN.Identifier, CAN.TransmitData, 2);
//				
//					UART_Transmit_DMA("ENCODER ERROR: %d\r\n", (uint8_t)PosSensor.SafetyWord);
//					SendBuf();
//					
//					LL_mDelay(10);
////				}
//			}
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
		for(uint16_t times = 0; times <= 50; times++)
		{
			/*����λ�ü��ٶ�, ��ֹ�ϵ�λ������, �����о�ֵ�˲���, �����ظ�ִ�ж����ʹ�ٶȱ����ӽ���*/
			GetEleImformation();
		
			GetMecImformation();
			
			LL_mDelay(1);
		}

		RefAngleInit();

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
		/*ͨ����ת�����ж������Ƿ���ȷ, �ӱ���������, ӦΪ˳ʱ����ת*/
		for(float eleAngle = 0; eleAngle <= 360; eleAngle += 15)
		{
			
			InverseParkTransform(volD, 0.f, &volAlpha, &volBeta, eleAngle);
			SpaceVectorModulation(volAlpha, volBeta);
			
			LL_mDelay(5);
			
			GetMecAngle_AbsoluteMode_15bit();
			UART_Transmit_DMA("EleAngle	%d\tMecPosition %d\r\n", (int)eleAngle, (int)PosSensor.MecAngle_15bit);SendBuf();
			
			LL_mDelay(100);
		}
		
		InverseParkTransform(volD, 0.f, &volAlpha, &volBeta, 0);
		SpaceVectorModulation(volAlpha, volBeta);
		
		GetMecAngle_AbsoluteMode_15bit();
		
		for(times = 0; times < 200; times++)
		{
			lastPosition = position;
			
			position = PosSensor.MecAngle_15bit;
			
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

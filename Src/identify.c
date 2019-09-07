/**
 ******************************************************************************
 * @file		identify.c
 * @author		WrathWings
 * @version 	V1.0
 * @date		2019.9.5
 * @brief		Functions to identify parameter
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
 
 /* Private includes ----------------------------------------------------------*/
/* CODE BEGIN Includes */
#include "identify.h"
/* CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* CODE BEGIN PTD */

/* CODE END PTD */

/* Private variables ---------------------------------------------------------*/
/* CODE BEGIN PV */

/* CODE END PV */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern struct CoordTrans_t	CoordTrans;
extern struct CurrLoop_t	CurrLoop;
extern struct PosSensor_t PosSensor;
/* USER CODE END EV */

/* USER CODE BEGIN */

 /** 
   * @brief		 Measure phase residence
   * @param[in]  targetSampleTimes     	期望采样次数
   * @param[in]  currQ      		    q轴电流
   * @param[out] residence 				相电阻
   */
uint8_t MeasureResidence(float targetSampleTimes, float currQ, float *residence)
{
	static uint8_t sendFlag = 0;
	static uint8_t status = 0;
	static uint16_t count = 0;
	static uint16_t sampleTime = 0;
	static float totalCurr = 0;
	static float totalVol = 0;

	PosSensor.EleAngle_degree = 0.f;
	
	switch(status)
	{
		case 0:
			count++;
			if(count >= 10)
			{
				status = 1;
				count = 0;
			}
			break;
		case 1:
			CurrentLoop(0.f, currQ, CoordTrans.CurrD, CoordTrans.CurrQ, &CurrLoop.CtrlVolD, &CurrLoop.CtrlVolQ);
			InverseParkTransform(CurrLoop.CtrlVolD, CurrLoop.CtrlVolQ, &CoordTrans.VolAlpha, &CoordTrans.VolBeta, PosSensor.EleAngle_degree);
			SpaceVectorModulation(CoordTrans.VolAlpha, CoordTrans.VolBeta);

			count++;
			if(count >= 10000)
			{
				totalCurr += sqrtf(SQUARE(CoordTrans.CurrD) + SQUARE(CoordTrans.CurrQ));
				totalVol += sqrtf(SQUARE(CurrLoop.CtrlVolD) + SQUARE(CurrLoop.CtrlVolQ));
				sampleTime++;
				
				if(sampleTime >= targetSampleTimes && sendFlag == 0)
				{
					sendFlag = 1;
					*residence = (totalVol / targetSampleTimes) / (totalCurr / targetSampleTimes) * 2.f / 3.f; //定子电阻与相电阻是 1.5:1的关系
					
					return 1;
				}
			}	
			break;
	}	
	return 0;
}

  /** 
   * @brief		 Measure phase inductance
   * @param[in]  targetSampleTimes     	期望采样次数
   * @param[in]  currQ      		    
   * @param[out] inductance 				相电感
   */
uint8_t MeasureInductance(float sampleTimes, float duty,float *inductance)
{
		static uint8_t inducState = 0;
	//duty 需要大于0.2
	static float V = 24.f; 
	int dutyCnt = duty * PWM_N - 1;
	static int sampleTime = 0;
	static float tot_current = 0;
//	static float currentRecord[3][SAMPLE] = {0};
	static uint8_t sendFlag = 0;
	static int count = 0;
#define TEST_OFFSET 5	

	count++;
	if(count >= 50)
	{		
		TIM1->ARR = dutyCnt;
		TIM1->CCR4 = dutyCnt-1;
		if(sampleTime < sample)
		{
			switch(inducState)
			{
				case 0:
					APHASE = 0;
					BPHASE = 0;
					CPHASE = 0;
					inducState++;
					break;
				case 1:
#if PHASE_SEQUENCE == POSITIVE_SEQUENCE			
					APHASE = 0;
					BPHASE = dutyCnt;
					CPHASE = dutyCnt;
					inducState++;
#elif PHASE_SEQUENCE ==  NEGATIVE_SEQUENCE
					APHASE = dutyCnt;
					BPHASE = 0;
					CPHASE = 0;
					inducState++;
#endif
					break;
				case 2:
					if(sampleTime >= TEST_OFFSET)
#if PHASE_SEQUENCE == POSITIVE_SEQUENCE		
						tot_current += Driver.Status.StateCurrent.ia;
#elif PHASE_SEQUENCE == NEGATIVE_SEQUENCE
						tot_current += -Driver.Status.StateCurrent.ia;
#endif					
//					currentRecord[0][sampleTime] = Driver.Status.StateCurrent.ia;
					APHASE = 0;
					BPHASE = 0;
					CPHASE = 0;
					sampleTime++;
					inducState++;
					break;
				case 3:
					inducState++;
					break;
				case 4:
					APHASE = dutyCnt;
					BPHASE = 0;
					CPHASE = dutyCnt;
					inducState++;
					break;
				case 5:
					if(sampleTime >= TEST_OFFSET)
						tot_current += Driver.Status.StateCurrent.ib;
//					currentRecord[1][sampleTime] = Driver.Status.StateCurrent.ib;
					APHASE = 0;
					BPHASE = 0;
					CPHASE = 0;
					sampleTime++;
					inducState++;
					break;
				case 6:
					inducState = 7;
					break;
				case 7:
#if PHASE_SEQUENCE == POSITIVE_SEQUENCE		
					APHASE = 0;
					BPHASE = 0;
					CPHASE = dutyCnt;
					inducState++;		
#elif PHASE_SEQUENCE ==  NEGATIVE_SEQUENCE
					APHASE = dutyCnt;
					BPHASE = dutyCnt;
					CPHASE = 0;
					inducState++;
#endif				
					break;
				case 8:
					if(sampleTime >= TEST_OFFSET)
#if PHASE_SEQUENCE == POSITIVE_SEQUENCE	
						tot_current +=  -Driver.Status.StateCurrent.ic;
#elif	PHASE_SEQUENCE == NEGATIVE_SEQUENCE	
						tot_current +=  Driver.Status.StateCurrent.ic;//(Driver.Status.StateCurrent.ia + Driver.Status.StateCurrent.ib);		
#endif
//					currentRecord[2][sampleTime] = Driver.Status.StateCurrent.ic;
					APHASE = 0;
					BPHASE = 0;
					CPHASE = 0;
					sampleTime++;
					inducState++;				
					break;
				case 9:
					inducState = 0;
					count = 0;
					break;
			}
		}
		else
		{
			APHASE = 0;
			BPHASE = 0;
			CPHASE = 0;
			if(sendFlag == 0)
			{
				*ind = (V - (tot_current / (sampleTime - TEST_OFFSET) * Driver.Status.StateStruct.res)) * (1 * duty * Driver.System.SysPeriod.s) / (tot_current / (sampleTime - TEST_OFFSET))  * 2.f / 3.f;
//				DMAPRINTF("%d %d %d\r\n",(int)(induc*1e6),(int)(tot_current / (sampleTime - TEST_OFFSET)*1e6),(int)(duty*10));
//				for(int i = 0; i < SAMPLE;i++)	
//				{
//					DMAPRINTF("%d %d %d\r\n",(int)(currentRecord[0][i] * 1000.f),(int)(currentRecord[1][i] * 1000.f),(int)(currentRecord[2][i] * 1000.f));
					sendFlag = 1;
//				}
//				resistor = voltage / (tot_current / (sampleTime - TEST_OFFSET))*1e6 * 2.f / 3.f;
//				DMAPRINTF("%d %d\r\n",(int)(resistor),(int)(tot_current / (sampleTime - TEST_OFFSET)*1e6));SendBuf();
			}
			else if(sendFlag == 1)
			{
//				SendBuf();
				sendFlag = 2;
			}
			else if(sendFlag == 2)
			{
				return 1;
			}
		}
	}
	return 0;
}

 /** 
   * @brief		 Measure parameters
   */
void MeasureParameters(void)
{
	float residence = 0;
	float inductance = 0;
	static int state = 0;
	static int flag = 0;
	static int wait = 0;
	
	switch(state)
	{
		case 0:
			PutStr("Parameters Measuring...\r\n"); SendBuf();
			state++;
			break;
		case 1 :
				flag = MeasureResidence(100.f, 15.f, &residence);
				if(flag)
				{
					flag = 0;
					state++;
					CCR_PHASE_A = 0;
					CCR_PHASE_B = 0;
					CCR_PHASE_C = 0;					
				}
			break;
		case 2:
		  wait++;
			if(wait > 5000)
			{
				state++;
			}
			break;
		case 3:
			flag = MeasureInductance(200.f ,0.7f, &inductance);
			if(flag)
			{
				flag = 0;
				state++;
			}
			break;
		case 4:
			UART_Transmit_DMA("phase residence: %d uOhm\t phase inductance: %d uH\r\n",(int)(residence * 1e6),(int)(inductance * 1e6));	SendBuf();			
			state++;
			break;
	}
}


/* USER CODE END */

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

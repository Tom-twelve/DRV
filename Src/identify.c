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
extern struct Regulator_t Regulator;
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
			CurrLoop.Kp_D = 0.5;												
			CurrLoop.Ki_D = 0.1;						
			CurrLoop.Kp_Q = 0.2;
			CurrLoop.Ki_Q = 0.1;
			ParkTransform(CoordTrans.CurrA, CoordTrans.CurrB, CoordTrans.CurrC, &CoordTrans.CurrD, &CoordTrans.CurrQ, PosSensor.EleAngle_degree);
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
   * @param[in]  duty      		    
   * @param[out] inductance 				相电感
   */
uint8_t MeasureInductance(float targetSampleTimes, float duty,float *inductance)
{
	//duty 需要大于0.2
	
	const uint8_t sampleOffsetTimes = 0;
	const uint32_t dutyCnt = duty * TIM8_ARR - 1;
	static uint8_t inductanceState = 0;
	static uint8_t sendFlag = 0;
	static uint16_t sampleTime = 0;
	static uint32_t count = 0;
	static float totalCurr = 0;

	count++;
	
	if(count >= 50)
	{		
		TIM1->ARR = dutyCnt;
		TIM1->CCR4 = dutyCnt-1;
		if(sampleTime < targetSampleTimes)
		{
			switch(inductanceState)
			{
				case 0:
					CCR_PHASE_A = 0;
					CCR_PHASE_B = 0;
					CCR_PHASE_C = 0;
					inductanceState++;
					break;
				case 1:
#if PHASE_SEQUENCE == POSITIVE_SEQUENCE			
					CCR_PHASE_A = 0;
					CCR_PHASE_B = dutyCnt;
					CCR_PHASE_C = dutyCnt;
					inductanceState++;
#elif PHASE_SEQUENCE == NEGATIVE_SEQUENCE
					CCR_PHASE_A = dutyCnt;
					CCR_PHASE_B = 0;
					CCR_PHASE_C = 0;
					inductanceState++;
#endif
					break;
				case 2:
					if(sampleTime >= sampleOffsetTimes)
#if PHASE_SEQUENCE == POSITIVE_SEQUENCE		
						totalCurr += CoordTrans.CurrA;
#elif PHASE_SEQUENCE == NEGATIVE_SEQUENCE
						totalCurr += -CoordTrans.CurrA;
#endif					
					CCR_PHASE_A = 0;
					CCR_PHASE_B = 0;
					CCR_PHASE_C = 0;
					sampleTime++;
					inductanceState++;
					
					break;
				case 3:
					inductanceState++;
				
					break;
				case 4:
					CCR_PHASE_A = dutyCnt;
					CCR_PHASE_B = 0;
					CCR_PHASE_C = dutyCnt;
					inductanceState++;
					break;
				case 5:
					if(sampleTime >= sampleOffsetTimes)
					{
						totalCurr += CoordTrans.CurrB;
					}
					CCR_PHASE_A = 0;
					CCR_PHASE_B = 0;
					CCR_PHASE_C = 0;
					
					sampleTime++;
					inductanceState++;
					
					break;
				case 6:
					inductanceState = 7;
				
					break;
				case 7:
#if PHASE_SEQUENCE == POSITIVE_SEQUENCE		
					CCR_PHASE_A = 0;
					CCR_PHASE_B = 0;
					CCR_PHASE_C = dutyCnt;
				
					inductanceState++;		
#elif PHASE_SEQUENCE == NEGATIVE_SEQUENCE
					CCR_PHASE_A = dutyCnt;
					CCR_PHASE_B = dutyCnt;
					CCR_PHASE_C = 0;
				
					inductanceState++;
#endif				
					break;
				case 8:
					if(sampleTime >= sampleOffsetTimes)
					{
						#if PHASE_SEQUENCE == POSITIVE_SEQUENCE	
						totalCurr +=  -CoordTrans.CurrC;
						#elif	PHASE_SEQUENCE == NEGATIVE_SEQUENCE	
						totalCurr +=  CoordTrans.CurrC;		
						#endif
					}
					
					CCR_PHASE_A = 0;
					CCR_PHASE_B = 0;
					CCR_PHASE_C = 0;
					
					sampleTime++;
					inductanceState++;	
					
					break;
				case 9:
					inductanceState = 0;
					count = 0;
				
					break;
			}
		}
		else
		{
			CCR_PHASE_A = 0;
			CCR_PHASE_B = 0;
			CCR_PHASE_C = 0;
			
			if(sendFlag == 0)
			{
				*inductance = GENERATRIX_VOL * (duty * Regulator.ActualPeriod_s) / (totalCurr / (sampleTime - sampleOffsetTimes))  * 2.f / 3.f;
				
				sendFlag = 1;
			}
			else if(sendFlag == 1)
			{
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
	static float residence = 0;
	static float inductance = 0;
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
				flag = MeasureResidence(100.f, 5.f, &residence);
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
			UART_Transmit_DMA("\r\nphase residence: %d mOhm \t phase inductance: %d uH \r\n",(int)(residence * 1e3),(int)(inductance * 1e6));	
			PutStr("\r\nParameters Measure Over \r\n");	
			SendBuf();
		
			state++;
			break;
	}
}


/* USER CODE END */

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

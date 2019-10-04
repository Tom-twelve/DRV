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
	static float tempVolD = 0;
	static float tempVolQ = 0;
	static float tempCurrD = 0;
	static float tempCurrQ = 0;
		
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
			CurrLoop.Kp_D = 0.2;												
			CurrLoop.Ki_D = 0.0;						
			CurrLoop.Kp_Q = 0.2;
			CurrLoop.Ki_Q = 0.0;
		
			ClarkTransform_arm(CoordTrans.CurrA, CoordTrans.CurrB, &CoordTrans.CurrAlpha, &CoordTrans.CurrBeta);
			ParkTransform_arm(CoordTrans.CurrAlpha, CoordTrans.CurrBeta, &CoordTrans.CurrD, &CoordTrans.CurrQ, PosSensor.EleAngle_degree);
			CurrentLoop(0.f, currQ, CoordTrans.CurrD, CoordTrans.CurrQ, &CurrLoop.CtrlVolD, &CurrLoop.CtrlVolQ);
			InverseParkTransform_arm(CurrLoop.CtrlVolD, CurrLoop.CtrlVolQ, &CoordTrans.VolAlpha, &CoordTrans.VolBeta, PosSensor.EleAngle_degree);
			SpaceVectorModulation(CoordTrans.VolAlpha, CoordTrans.VolBeta);

			count++;
			if(count >= 15000)
			{
				totalVol += sqrtf(SQUARE(CurrLoop.CtrlVolD) + SQUARE(CurrLoop.CtrlVolQ));
				totalCurr += sqrtf(SQUARE(CoordTrans.CurrD) + SQUARE(CoordTrans.CurrQ));
				
				tempVolD += CurrLoop.CtrlVolD;
				tempVolQ += CurrLoop.CtrlVolQ;
				
				tempCurrD += CoordTrans.CurrD;
				tempCurrQ += CoordTrans.CurrQ;
				
				sampleTime++;
				
				if(sampleTime >= targetSampleTimes && sendFlag == 0)
				{
					sendFlag = 1;
					*residence = (totalVol / targetSampleTimes) / (totalCurr / targetSampleTimes);
					
					UART_Transmit_DMA("\r\nVd: %d mV\tVq: %d mV\r\n", (int)((tempVolD / targetSampleTimes) * 1e3), (int)((tempVolQ / targetSampleTimes) * 1e3));
					UART_Transmit_DMA("\r\nId: %d mA\tIq: %d mA\r\n", (int)((tempCurrD / targetSampleTimes) * 1e3), (int)((tempCurrQ / targetSampleTimes) * 1e3));
					
					return 1;
				}
			}	
			break;
	}	
	return 0;
}

  /** 
   * @brief		 Measure phase inductance
   * @param[in]  targetSampleTimes     		期望采样次数    		    
   * @param[out] inductance 				相电感
   */
uint8_t MeasureInductance(float targetSampleTimes, float *inductance)
{
	const uint8_t sampleOffsetTimes = 0;
	static uint8_t inductanceState = 0;
	static uint8_t sendFlag = 0;
	static uint16_t sampleTime = 0;
	static uint32_t count = 0;
	static float totalCurr = 0;
	static float totalCurrA = 0;
	static float totalCurrB = 0;
	static float totalCurrC= 0;
	
	count++;
	
	if(count >= 50)
	{		
		TIM8->CCR4 = TIM8_ARR - 1;
		
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
					CCR_PHASE_A = 0;
					CCR_PHASE_B = TIM8_ARR;
					CCR_PHASE_C = TIM8_ARR;
				
					inductanceState++;

					break;
				
				case 2:
					if(sampleTime >= sampleOffsetTimes)
					{
						totalCurrA += CoordTrans.CurrA;
					}
			
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
					CCR_PHASE_A = TIM8_ARR;
					CCR_PHASE_B = 0;
					CCR_PHASE_C = TIM8_ARR;
				
					inductanceState++;
				
					break;
				
				case 5:
					if(sampleTime >= sampleOffsetTimes)
					{
						totalCurrB += CoordTrans.CurrB;
					}
					
					CCR_PHASE_A = 0;
					CCR_PHASE_B = 0;
					CCR_PHASE_C = 0;
					
					sampleTime++;
					inductanceState++;
					
					break;
					
				case 6:
					inductanceState++;
				
					break;
				case 7:	
					CCR_PHASE_A = TIM8_ARR;
					CCR_PHASE_B = TIM8_ARR;
					CCR_PHASE_C = 0;
				
					inductanceState++;		
				
					break;
				
				case 8:
					if(sampleTime >= sampleOffsetTimes)
					{
						totalCurrC +=  CoordTrans.CurrC;
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
				totalCurr = totalCurrA + totalCurrB + totalCurrC;
				
				*inductance = GENERATRIX_VOL * Regulator.ActualPeriod_s / (totalCurr / (sampleTime - sampleOffsetTimes))  * 2.f / 3.f;
				
				UART_Transmit_DMA("\r\nTotalCurrA: %d mA\r\n", (int)(totalCurrA * 1e3));
				UART_Transmit_DMA("\r\nTotalCurrB: %d mA\r\n", (int)(totalCurrB * 1e3));
				UART_Transmit_DMA("\r\nTotalCurrC: %d mA\r\n", (int)(totalCurrC * 1e3));
				UART_Transmit_DMA("\r\nAvgCurr: %d mA\r\n", (int)((totalCurr / (sampleTime - sampleOffsetTimes)) * 1e3));
				
				sendFlag = 1;
				
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
				flag = MeasureResidence(200.f, 10.f, &residence);
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
			if(wait > 10000)
			{
				state++;
			}
			break;
		case 3:
			flag = MeasureInductance(300.f, &inductance);
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

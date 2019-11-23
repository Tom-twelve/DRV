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
uint8_t MeasureResidence(float targetSampleTimes, float volD, float *residence)
{
	static uint8_t sendFlag = 0;
	static uint8_t status = 0;
	static uint32_t count = 0;
	static uint16_t sampleTime = 0;
	static float totalCurr = 0;
	static float totalVol = 0;
	static float tempCurrD = 0;
	static float tempCurrQ = 0;
	
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
			CurrLoop.CtrlVolD = volD;
			PosSensor.EleAngle_degree = 0.f;
		
			ClarkeTransform(CoordTrans.CurrA, CoordTrans.CurrB, &CoordTrans.CurrAlpha, &CoordTrans.CurrBeta);
			ParkTransform(CoordTrans.CurrAlpha, CoordTrans.CurrBeta, &CoordTrans.CurrD, &CoordTrans.CurrQ, PosSensor.EleAngle_degree);
			InverseParkTransform(volD, 0.f, &CoordTrans.VolAlpha, &CoordTrans.VolBeta, PosSensor.EleAngle_degree);
			SpaceVectorModulation(CoordTrans.VolAlpha, CoordTrans.VolBeta);

			count++;
		
			if(count >= 20000)
			{
				totalVol += volD;
				totalCurr += sqrtf(SQUARE(CoordTrans.CurrD) + SQUARE(CoordTrans.CurrQ));
				
				tempCurrD += CoordTrans.CurrD;
				tempCurrQ += CoordTrans.CurrQ;
				
				sampleTime++;
				
				if(sampleTime >= targetSampleTimes && sendFlag == 0)
				{
					sendFlag = 1;
					*residence = (totalVol / targetSampleTimes) / (totalCurr / targetSampleTimes);
					
					UART_Transmit_DMA("\r\ntotalVol: %d mV\r\n", (int)((totalVol / targetSampleTimes) * 1e3));
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
	static uint8_t inductanceState = 0;
	static uint8_t sendFlag = 0;
	static uint16_t sampleTime = 0;
	static uint32_t count = 0;
	static float avgTotalCurr = 0;
	static float totalCurrA = 0;
	static float totalCurrB = 0;
	static float totalCurrC= 0;
		
	if(sampleTime < targetSampleTimes)
	{
		switch(inductanceState)
		{
			case 0:
				count++;
			
				/*等待电感复位*/
				if(count >= 50)
				{
					count = 0;
					inductanceState++;
				}
				
				break;
								
			case 1:
				CCR_PHASE_A = 0;
				CCR_PHASE_B = 0;
				CCR_PHASE_C = 0;
				
				inductanceState++;
				
				break;
				
			case 2:		
				CCR_PHASE_A = 0;
				CCR_PHASE_B = TIM8_ARR;
				CCR_PHASE_C = TIM8_ARR;
				
				inductanceState++;

				break;
				
			case 3:
				totalCurrA += CoordTrans.CurrA;
			
				CCR_PHASE_A = 0;
				CCR_PHASE_B = 0;
				CCR_PHASE_C = 0;
					
				inductanceState++;
					
				break;
					
			case 4:
				count++;
			
				/*等待电感复位*/
				if(count >= 50)
				{
					count = 0;
					inductanceState++;
				}
				
				break;
				
			case 5:
				CCR_PHASE_A = TIM8_ARR;
				CCR_PHASE_B = 0;
				CCR_PHASE_C = TIM8_ARR;
				
				inductanceState++;
				
				break;
				
			case 6:
				totalCurrB += CoordTrans.CurrB;
					
				CCR_PHASE_A = 0;
				CCR_PHASE_B = 0;
				CCR_PHASE_C = 0;
				
				inductanceState++;
					
				break;
					
			case 7:
				count++;
			
				/*等待电感复位*/
				if(count >= 50)
				{
					count = 0;
					inductanceState++;
				}
				
				break;
			case 8:	
				CCR_PHASE_A = TIM8_ARR;
				CCR_PHASE_B = TIM8_ARR;
				CCR_PHASE_C = 0;
				
				inductanceState++;		
				
				break;
				
			case 9:
				totalCurrC += CoordTrans.CurrC;
					
				CCR_PHASE_A = 0;
				CCR_PHASE_B = 0;
				CCR_PHASE_C = 0;
				
				inductanceState++;	
					
				break;
					
			case 10:
				sampleTime++;
				inductanceState = 0;
				
				
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
				avgTotalCurr = (totalCurrA + totalCurrB + totalCurrC) / 3.f;
				
				*inductance = GENERATRIX_VOL * DEFAULT_CARRIER_PERIOD_s / (avgTotalCurr / sampleTime)  * 2.f / 3.f;
				
				UART_Transmit_DMA("\r\nL1: %d uH\r\n", (int)((GENERATRIX_VOL * DEFAULT_CARRIER_PERIOD_s / (totalCurrA / sampleTime)  * 2.f / 3.f) * 1e6));
				UART_Transmit_DMA("\r\nL2: %d uH\r\n", (int)((GENERATRIX_VOL * DEFAULT_CARRIER_PERIOD_s / (totalCurrB / sampleTime)  * 2.f / 3.f) * 1e6));
				UART_Transmit_DMA("\r\nL3: %d uH\r\n", (int)((GENERATRIX_VOL * DEFAULT_CARRIER_PERIOD_s / (totalCurrC / sampleTime)  * 2.f / 3.f) * 1e6));
				
				sendFlag = 1;
				
				return 1;
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
	static int count = 0;
	
	switch(state)
	{
		case 0:
			PutStr("Parameters Measuring...\r\n"); SendBuf();
			state++;
		
			break;
		
		case 1 :
			flag = MeasureResidence(200.f, 1.0f, &residence);
			if(flag)
			{			
				CCR_PHASE_A = 0;
				CCR_PHASE_B = 0;
				CCR_PHASE_C = 0;
				
				flag = 0;
				state++;		
			}
			
			break;
			
		case 2:
			count++;
			if(count > 20000)
			{
				state++;
			}
			
			break;
			
		case 3:
			flag = MeasureInductance(100.f, &inductance);
			if(flag)
			{
				flag = 0;
				state++;
			}
			
			break;
			
		case 4:
			UART_Transmit_DMA("\r\nRs: %d mOhm \t phase inductance: %d uH \r\n",(int)(residence * 1e3),(int)(inductance * 1e6));	
			PutStr("\r\nParameters Measure Over \r\n");	
			SendBuf();
		
			state++;
		
			break;
		
		default:
			
			break;
	}
}


/* USER CODE END */

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

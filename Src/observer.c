/**
 ******************************************************************************
 * @file		observer.c
 * @author		WrathWings
 * @version 	V1.0
 * @date		2019.8.20
 * @brief		State observer
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
 
 /* Private includes ----------------------------------------------------------*/
/* CODE BEGIN Includes */
#include "observer.h"
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
   * @brief		 Measure phase residence and inductance of axis q
   */
void MeasurePhaseRes(float iq, uint16_t sampleTime)
{
	static uint8_t status = 0;
	static int count = 0;
	static float eleAngle = 0;
	static float curTotal = 0;
	static float volTotal = 0;
	
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
				CurrentLoop(0.f, iq, CoordTrans.CurrD, CoordTrans.CurrQ, &CurrLoop.CtrlVolD, &CurrLoop.CtrlVolQ);
				InverseParkTransform(CurrLoop.CtrlVolD, CurrLoop.CtrlVolQ, &CoordTrans.VolAlpha, &CoordTrans.VolBeta, eleAngle);
				SpaceVectorModulation(CoordTrans.VolAlpha, CoordTrans.VolBeta);
		
				count++;
				if(count >= 10000)
				{
					curTotal += sqrtf(SQUARE(CoordTrans.CurrD) + SQUARE(CoordTrans.CurrQ));
					volTotal += sqrtf(SQUARE(CurrLoop.CtrlVolD) + SQUARE(CurrLoop.CtrlVolQ));
					sampleTime++;
					if(sampleTime >= sample && sendFlag == 0)
					{
						sendFlag = 1;
						*res = (volTotal / sample) / (curTotal / sample) * 2.f / 3.f; 
	//					DMAPRINTF("%d %d\r\n",(int)(*res * 1e6),(int)(vq*1000));	SendBuf();	
						return 1;
					}
				}	
			break;
	}	
	return 0;
}


/* USER CODE END */

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

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
/**
* @brief  负载观测器
* @param  *phase_var：观测到的速度
          dt:时间微分因子，在这里为运行周期      
					*x1：估算转速
					*x2：负载
					phase:位置观测器观测到的转子位置
*  @note  用PLL计算速度
 * @retval none
 */
LoadObserverType loadObserver;
extern struct TorqueCtrl_t TorqueCtrl;
extern struct PosSensor_t PosSensor;
void LoadObserver(void)
{
	static float load = 0;	
	static float x1_dot = 0, x2_dot = 0;
	static float err = 0,ki = 0.f * 1e-3;
	static int cnt = 0;
	float B = 0.f * 1e-6;

	#define AVERAGE_NUM_1  10
	#define AVERAGE_NUM_2  10
	
	static average_filter_t arr1[AVERAGE_NUM_1],arr2[AVERAGE_NUM_2]; 
	static uint32_t index1 = 0,index2 = 0; 
	static average_filter_t pSum1 = 0,pSum2 = 0;
	static float value = 0;

	err =  (PosSensor.MecAngularSpeed_rad - (loadObserver.w))* DEFAULT_CARRIER_PERIOD_s;
	cnt++;
//	if(cnt >=10)
//	{
////		DMAPRINTF("%d\t%d\r\n",(int)(1e6 * (err)),(int)(Driver.Status.StateVel.Speed.AngSpdMec));
////		DMAPRINTF("%d\t%d\r\n",(int)(1e6 * (*x2)),(int)(*x1));
//		cnt = 0;
//	}	
	(loadObserver.load) +=  loadObserver.loadGain2 * err ;//- (ki * (acc - x1_dot - e1 * err));
	if((loadObserver.load) >= 3.f) (loadObserver.load) = 3.f;
	else if((loadObserver.load) <= -3.f) (loadObserver.load) = -3.f;
  x1_dot = (TorqueCtrl.EleTorque_Nm - (loadObserver.load) - B*(loadObserver.load)) / INERTIA*DEFAULT_CARRIER_PERIOD_s;
//	  x1_dot = (Driver.Control.VelCtrl.output * Driver.Status.StateStruct.flux * 1.5f * Driver.Status.StateStruct.polePairs- (*x2) - B*(*x1)) / Driver.Status.StateStruct.inertia*dt;
	(loadObserver.w) +=  (x1_dot + loadObserver.loadGain1 * err) ;//1e6留到这里乘上，放上面可能会崩掉(未验证，有可能是出现了NAN)
//	(*x1) = UtilAverageFilter((*x1),arr1,&index1,&pSum1,AVERAGE_NUM);
	(loadObserver.load) = UtilAverageFilter(((loadObserver.load)),arr2,&index2,&pSum2,AVERAGE_NUM_2);
	loadObserver.load = (loadObserver.load);
//	Driver.Status.StateCurrent.torque = UtilAverageFilter((Driver.Status.StateCurrent.torque),arr1,&index1,&pSum1,AVERAGE_NUM_1);
	cnt++;
	if(cnt >= 100)
	{
//		DMAPRINTF("%d\t%d\r\n",(int)(Driver.Status.StateCurrent.torque * 1e3 * 19.2f),(int)(1e3 * (*x2 * 19.2f)));
//		DMAPRINTF("%d\t%d\r\n",(int)(Driver.Status.StateCurrent.id),(int)(Driver.Status.StateCurrent.iq));
//		DMAPRINTF("%d\t%d\t%d\r\n",(int)(1e3 * (*x2 * 19.2f)),(int)(*x1 / 19.2f /6.28f * 60.f),(int)(Driver.Status.StateVel.Speed.AngSpdMec));
//		UART_Transmit_DMA("%d\t%d\t%d\r\n",(int)(1e3*(loadObserver.load)),(int)((1e3*TorqueCtrl.EleTorque_Nm)),(int)(PosSensor.MecAngularSpeed_rad));
//		DMAPRINTF("%d\t%d\r\n",(int)((Driver.Status.StateVel.Speed.AngSpdMec)),(int)(1e3 * (*x2 * 19.2f)));
//		DMAPRINTF("%d\t%d\r\n",(int)(*x1 / 19.2f /6.28f * 60.f),(int)((Driver.Status.StateVel.Speed.AngSpdMec / 19.2f / 6.28f * 60.f)));
		cnt = 0;
	}
}
/* USER CODE END EV */

/* USER CODE BEGIN */


/* USER CODE END */

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

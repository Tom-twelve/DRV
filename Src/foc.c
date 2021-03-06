/**
 ******************************************************************************
 * @file		foc.c
 * @author		WrathWings
 * @version 	V1.0
 * @date		2019.1.17
 * @brief		Algorithm of Field Oriented Control
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
 
 /* Private includes ----------------------------------------------------------*/
/* CODE BEGIN Includes */
#include "foc.h"
/* CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* CODE BEGIN PTD */

/* CODE END PTD */

/* Private variables ---------------------------------------------------------*/
/* CODE BEGIN PV */
struct CoordTrans_t	CoordTrans;
struct Driver_t		Driver;
	
/* CODE END PV */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

extern struct PosSensor_t PosSensor;

/* USER CODE END EV */

/* USER CODE BEGIN */

 /** 
   * @brief		 Space vector pulse width modulation
   * @param[in]  volAlpha  		input two-phase vector coordinate alpha
   * @param[in]  volBeta   		input two-phase vector coordinate beta
   */
void SpaceVectorModulation(float volAlpha, float volBeta)
{
	const float StandardizationCoefficient = SQRT3 * TIM8_ARR / GENERATRIX_VOL;
	float U1 = 0;
	float U2 = 0;
	float U3 = 0;
	float Tx = 0;
	float Ty = 0;
	float tx = 0;
	float ty = 0;
	uint32_t TA = 0;
	uint32_t TB = 0;
	uint32_t TC = 0;
	uint8_t sectionFlag = 0;

	U1 =  volBeta;
	
	U2 = (SQRT3 * volAlpha - volBeta) / 2.0f;
	
	U3 = ( - SQRT3 * volAlpha - volBeta) / 2.0f;
	
	/*判断矢量电压所在扇区*/
	if (U1 > 0)
	{
		sectionFlag = 0x01;
	}
	else
	{
		sectionFlag = 0x00;
	}
	if (U2 > 0)
	{
		sectionFlag |= 0x02;
	}
	if (U3 > 0)
	{
		sectionFlag |= 0x04;
	}

	switch (sectionFlag)
	{
	case 3:  Tx = U2 * StandardizationCoefficient; 
			 Ty = U1 * StandardizationCoefficient;
			 break; 

	case 1:  Tx = - U2 * StandardizationCoefficient; 
			 Ty = - U3 * StandardizationCoefficient; 
			 break;

	case 5:  Tx = U1 * StandardizationCoefficient; 
			 Ty = U3 * StandardizationCoefficient; 
			 break;

	case 4:  Tx = - U1 * StandardizationCoefficient; 
			 Ty = - U2 * StandardizationCoefficient; 
			 break;

	case 6:  Tx = U3 * StandardizationCoefficient; 
			 Ty = U2 * StandardizationCoefficient; 
			 break; 

	case 2:  Tx = - U3 * StandardizationCoefficient; 
			 Ty = - U1 * StandardizationCoefficient; 
			 break; 

	default: Tx = 0u; 
			 Ty = 0u; 
			 break;
	}

	/*电压矢量超出正六边形范围时按比例缩小电压矢量, 防止波形失真*/
	if (Tx + Ty > TIM8_ARR)
	{
		tx  = Tx;
		ty  = Ty;
		Tx  = tx / (tx + ty) * TIM8_ARR;
		Ty  = ty / (tx + ty) * TIM8_ARR;
	}

	TA  = (TIM8_ARR - Tx - Ty) / 2.0f;
	TB  = TA + Tx;
	TC  = TB + Ty;
	
	switch (sectionFlag)
	{
	case 3:  CCR_PHASE_A = TA; 
			 CCR_PHASE_B = TB; 
			 CCR_PHASE_C = TC; 
			 break;

	case 1:  CCR_PHASE_A = TB; 
			 CCR_PHASE_B = TA; 
			 CCR_PHASE_C = TC; 
			 break;

	case 5:  CCR_PHASE_A = TC; 
			 CCR_PHASE_B = TA; 
			 CCR_PHASE_C = TB; 
			 break; 

	case 4:  CCR_PHASE_A = TC; 
			 CCR_PHASE_B = TB; 
			 CCR_PHASE_C = TA; 
			 break; 

	case 6:  CCR_PHASE_A = TB; 
			 CCR_PHASE_B = TC; 
			 CCR_PHASE_C = TA; 
			 break;

	case 2:  CCR_PHASE_A = TA; 
			 CCR_PHASE_B = TC; 
			 CCR_PHASE_C = TB; 
			 break;

	default: CCR_PHASE_A = TIM8_ARR; 
			 CCR_PHASE_B = TIM8_ARR; 
			 CCR_PHASE_C = TIM8_ARR; 
			 break;
	}
}

 /**
   * @brief  Floating-point Inverse Park transform
   * @param[in]  volD       	input coordinate of rotor reference frame d
   * @param[in]  volQ       	input coordinate of rotor reference frame q
   * @param[out] volAlpha 		output two-phase orthogonal vector axis alpha
   * @param[out] volBeta  		output two-phase orthogonal vector axis beta
   * @param[in]  EleAngle		value of Ele angle
   */
void InverseParkTransform(float volD, float volQ, float *volAlpha, float *volBeta, float eleAngle)
{
	float eleAngleSineValue = 0;
	float eleAngleCosineValue = 0;
	
	arm_sin_cos_f32((float)eleAngle,  &eleAngleSineValue,  &eleAngleCosineValue);
	
	arm_inv_park_f32((float)volD, (float)volQ, volAlpha, volBeta, (float)eleAngleSineValue, (float)eleAngleCosineValue);
}

 /**
   * @brief  Floating-point Park transform
   * @param[in]  volD       	input coordinate of rotor reference frame d
   * @param[in]  volQ       	input coordinate of rotor reference frame q
   * @param[out] volAlpha 		output two-phase orthogonal vector axis alpha
   * @param[out] volBeta  		output two-phase orthogonal vector axis beta
   * @param[in]  EleAngle		value of Ele angle
   */
void ParkTransform(float currAlpha, float currBeta, float *currD, float *currQ, float eleAngle)
{	
	float eleAngleSineValue = 0;
	float eleAngleCosineValue = 0;
	
	arm_sin_cos_f32((float)eleAngle,  &eleAngleSineValue,  &eleAngleCosineValue);
	
	arm_park_f32(currAlpha, currBeta, currD, currQ, eleAngleSineValue, eleAngleCosineValue);
}

 /**
   * @brief  Floating-point Clarke transform
   * @param[in]  volD       	input coordinate of rotor reference frame d
   * @param[in]  volQ       	input coordinate of rotor reference frame q
   * @param[out] volAlpha 		output two-phase orthogonal vector axis alpha
   * @param[out] volBeta  		output two-phase orthogonal vector axis beta
   * @param[in]  EleAngle		value of Ele angle
   */
void ClarkeTransform(float currA, float currB, float *currAlpha, float *currBeta)
{
	arm_clarke_f32(currA, currB, currAlpha, currBeta);
}

 /**
   * @brief  Calculate electromagnetic torque
   * @param[in]   realCurrQ      	current of axis q
   * @param[out]  EleTorque      	electromagnetic torque
   */
void CalculateEleTorque(float realCurrQ, float *EleTorque)
{
	*EleTorque = 1.5f * MOTOR_POLE_PAIRS_NUM * realCurrQ * ROTATOR_FLUX_LINKAGE;
}

/* USER CODE END */

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

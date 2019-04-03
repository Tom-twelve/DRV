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
struct CoordinateTransformation_t	CoordinateTransformation;
struct MotorDynamicParameter_t		MotorDynamicParameter;
struct MotorStaticParameter_t		MotorStaticParameter;
	
/* CODE END PV */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern struct PositionSensor_t PositionSensor;
extern struct CurrentLoop_t CurrentLoop;

/* USER CODE END EV */

/* USER CODE BEGIN */

 /** 
   * @brief		 Space vector pulse width modulation
   * @param[in]  volAlpha  		input two-phase vector coordinate alpha
   * @param[in]  volBeta   		input two-phase vector coordinate beta
   */
void SpaceVectorModulation(float volAlpha, float volBeta)
{
	const float StandardizationCoefficient = SQRT3 * TIM8_Autoreload / GeneratrixVoltage;
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
	if (Tx + Ty > TIM8_Autoreload)
	{
		tx  = Tx;
		ty  = Ty;
		Tx  = tx / (tx + ty) * TIM8_Autoreload;
		Ty  = ty / (tx + ty) * TIM8_Autoreload;
	}

	TA  = (TIM8_Autoreload - Tx - Ty) / 2.0f;
	TB  = TA + Tx;
	TC  = TB + Ty;
	
	switch (sectionFlag)
	{
	case 3:  CCR_PhaseA = TA; 
			 CCR_PhaseB = TB; 
			 CCR_PhaseC = TC; 
			 break;

	case 1:  CCR_PhaseA = TB; 
			 CCR_PhaseB = TA; 
			 CCR_PhaseC = TC; 
			 break;

	case 5:  CCR_PhaseA = TC; 
			 CCR_PhaseB = TA; 
			 CCR_PhaseC = TB; 
			 break; 

	case 4:  CCR_PhaseA = TC; 
			 CCR_PhaseB = TB; 
			 CCR_PhaseC = TA; 
			 break; 

	case 6:  CCR_PhaseA = TB; 
			 CCR_PhaseB = TC; 
			 CCR_PhaseC = TA; 
			 break;

	case 2:  CCR_PhaseA = TA; 
			 CCR_PhaseB = TC; 
			 CCR_PhaseC = TB; 
			 break;

	default: CCR_PhaseA = TIM8_Autoreload; 
			 CCR_PhaseB = TIM8_Autoreload; 
			 CCR_PhaseC = TIM8_Autoreload; 
			 break;
	}
}

 /**
   * @brief Floating-point Park transform
   * @param[in]  currentPhaseA  		current of phase A
   * @param[in]  currentPhaseB  		current of phase B
   * @param[in]  currentPhaseC  		current of phase C
   * @param[out] currentD     			points to output rotor reference frame d
   * @param[out] currentQ     			points to output rotor reference frame q
   * @param[in]  EleAngle				value of Electrical angle
   */
void ParkTransform(float currentPhaseA, float currentPhaseB, float currentPhaseC, float *currentD, float *currentQ, float eleAngle)
{
	const float Coefficient_ConstantPower = 0.8164965809f;
	const float Coefficient_ConstantAmplitude = 0.6666666667f;
	float eleAngleSineValue1 = 0;
	float eleAngleCosineValue1 = 0;
	float eleAngleSineValue2 = 0;
	float eleAngleCosineValue2 = 0;
	float eleAngleSineValue3 = 0;
	float eleAngleCosineValue3 = 0;
	
	arm_sin_cos_f32((float)eleAngle,  &eleAngleSineValue1,  &eleAngleCosineValue1);
	
	arm_sin_cos_f32((float)(eleAngle - 120.f),  &eleAngleSineValue2,  &eleAngleCosineValue2);
	
	arm_sin_cos_f32((float)(eleAngle + 120.f),  &eleAngleSineValue3,  &eleAngleCosineValue3);
	
	*currentD = Coefficient_ConstantAmplitude * (currentPhaseA * eleAngleSineValue1 + currentPhaseB * eleAngleSineValue2 + currentPhaseC * eleAngleSineValue3);
	
	*currentQ = Coefficient_ConstantAmplitude * (currentPhaseA * eleAngleCosineValue1 + currentPhaseB * eleAngleCosineValue2 + currentPhaseC * eleAngleCosineValue3);
}

   /**
   * @brief  Floating-point Inverse Park transform
   * @param[in]  VolD       	input coordinate of rotor reference frame d
   * @param[in]  VolQ       	input coordinate of rotor reference frame q
   * @param[out] VolAlpha 		output two-phase orthogonal vector axis alpha
   * @param[out] VolBeta  		output two-phase orthogonal vector axis beta
   * @param[in]  EleAngle			value of Ele angle
   */
void InverseParkTransform(float VolD, float VolQ, float *VolAlpha, float *VolBeta, float eleAngle)
{
	float eleAngleSineValue = 0;
	float eleAngleCosineValue = 0;
	
	arm_sin_cos_f32((float)eleAngle,  &eleAngleSineValue,  &eleAngleCosineValue);
	
	arm_inv_park_f32((float)VolD, (float)VolQ, VolAlpha, VolBeta, (float)eleAngleSineValue, (float)eleAngleCosineValue);
}

   /**
   * @brief  Floating-point Clark transform
   * @param[in]  currentPhaseA      current of phase A
   * @param[in]  currentPhaseB      current of phase B
   * @param[in]  currentPhaseC 		current of phase C
   * @param[out] currentAlpha  		output two-phase orthogonal vector axis alpha
   * @param[out] currentBeta		output two-phase orthogonal vector axis beta
   */
void ClarkTransform(float currentPhaseA, float currentPhaseB, float currentPhaseC, float *currentAlpha, float *currentBeta)
{
	const float Coefficient_ConstantPower = 0.8164965809f;
	const float Coefficient_ConstantAmplitude = 0.6666666667f;
	
	*currentAlpha = Coefficient_ConstantAmplitude * ((float)currentPhaseA - 0.5f * (float)currentPhaseB - 0.5f * (float)currentPhaseC);
	
	*currentBeta  = 0.5f * SQRT3 * Coefficient_ConstantAmplitude * ((float)currentPhaseB - (float)currentPhaseC);
}

void ParkTransform_arm(float currentAlpha, float currentBeta, float *currentD, float *currentQ, float eleAngle)
{	
	float eleAngleSineValue = 0;
	float eleAngleCosineValue = 0;
	
	arm_sin_cos_f32((float)eleAngle,  &eleAngleSineValue,  &eleAngleCosineValue);
	
	arm_park_f32(currentAlpha, currentBeta, currentD, currentQ, eleAngleSineValue, eleAngleCosineValue);
}

void ClarkTransform_arm(float currentPhaseA, float currentPhaseB, float *currentAlpha, float *currentBeta)
{
	arm_clarke_f32(currentPhaseA, currentPhaseB, currentAlpha, currentBeta);
}

   /**
   * @brief  power angle compensation
   * @param[in]  expectedCurrentQ  			current of axis d
   * @param[out] *powerAngleComp_degree  	compensation angle 
   */
void PowerAngleComp(float expectedCurrentQ, float *powerAngleComp_degree)
{
	float PowerAngleComp_rad = 0;
	
	PowerAngleComp_rad = arcsine((-2 * (InductanceD - InductanceQ) * expectedCurrentQ) / (RotatorFluxLinkage + sqrt_DSP(Square(RotatorFluxLinkage) + 8 * Square(InductanceD - InductanceQ) * Square(expectedCurrentQ))));

	*powerAngleComp_degree = PowerAngleComp_rad * 360.f	/ (2 * PI);
}

   /**
   * @brief  Calculate voltage of axis d and voltage of axis q
   * @param[in]  actualCurrentQ  					current of axis q
   * @param[out] volD  							voltage of axis d
   * @param[out] volQ							voltage of axis q
   * @param[in]  actualEleAngularSpeed  		actual ele angular speed(rad/s)
   */
void CalculateVoltage_dq(float actualCurrentQ, float *volD, float *volQ, float actualEleAngularSpeed_rad)
{
	*volD = -actualEleAngularSpeed_rad * InductanceQ * actualCurrentQ;
	
	*volQ = PhaseResistance * actualCurrentQ + actualEleAngularSpeed_rad * RotatorFluxLinkage;
}

   /**
   * @brief  Calculate electromagnetic torque
   * @param[in]   currentQ      				current of axis q
   * @param[out]  EleTorque      	electromagnetic torque
   */
void CalculateEleTorque(float actualCurrentQ, float *EleTorque)
{
	*EleTorque = 1.5f * MotorPolePairs * actualCurrentQ * RotatorFluxLinkage;
}

/* USER CODE END */

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

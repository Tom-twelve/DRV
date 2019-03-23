/**
 ******************************************************************************
 * @file		svpwm.c
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
extern struct Encoder_t Encoder;
extern struct CurrentLoop_t CurrentLoop;

/* USER CODE END EV */

/* USER CODE BEGIN */

 /** 
   * @brief		 Space vector pulse width modulation
   * @param[in]  voltageAlpha  		input two-phase vector coordinate alpha
   * @param[in]  voltageBeta   		input two-phase vector coordinate beta
   */
void SpaceVectorPulseWidthModulation(float voltageAlpha, float voltageBeta)
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

	U1 =  voltageBeta;
	
	U2 = (SQRT3 * voltageAlpha - voltageBeta) / 2.0f;
	
	U3 = ( - SQRT3 * voltageAlpha - voltageBeta) / 2.0f;
	
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
void ParkTransform(float currentPhaseA, float currentPhaseB, float currentPhaseC, float *currentD, float *currentQ, float EleAngle)
{
	const float Coefficient_ConstantPower = 0.8164965809f;
	const float Coefficient_ConstantAmplitude = 0.6666666667f;
	float EleAngleSineValue1 = 0;
	float EleAngleCosineValue1 = 0;
	float EleAngleSineValue2 = 0;
	float EleAngleCosineValue2 = 0;
	float EleAngleSineValue3 = 0;
	float EleAngleCosineValue3 = 0;
	
	arm_sin_cos_f32((float)EleAngle,  &EleAngleSineValue1,  &EleAngleCosineValue1);
	
	arm_sin_cos_f32((float)(EleAngle - 120.f),  &EleAngleSineValue2,  &EleAngleCosineValue2);
	
	arm_sin_cos_f32((float)(EleAngle + 120.f),  &EleAngleSineValue3,  &EleAngleCosineValue3);
	
	*currentD = Coefficient_ConstantAmplitude * (currentPhaseA * EleAngleSineValue1 + currentPhaseB * EleAngleSineValue2 + currentPhaseC * EleAngleSineValue3);
	
	*currentQ = Coefficient_ConstantAmplitude * (currentPhaseA * EleAngleCosineValue1 + currentPhaseB * EleAngleCosineValue2 + currentPhaseC * EleAngleCosineValue3);
}

   /**
   * @brief  Floating-point Inverse Park transform
   * @param[in]  voltageD       	input coordinate of rotor reference frame d
   * @param[in]  voltageQ       	input coordinate of rotor reference frame q
   * @param[out] voltageAlpha 		output two-phase orthogonal vector axis alpha
   * @param[out] voltageBeta  		output two-phase orthogonal vector axis beta
   * @param[in]  EleAngle	value of Ele angle
   */
void InverseParkTransform_TwoPhase(float voltageD, float voltageQ, float *voltageAlpha, float *voltageBeta, float EleAngle)
{
	float EleAngleSineValue = 0;
	float EleAngleCosineValue = 0;
	
	arm_sin_cos_f32((float)EleAngle,  &EleAngleSineValue,  &EleAngleCosineValue);
	
	arm_inv_park_f32((float)voltageD, (float)voltageQ, voltageAlpha, voltageBeta, (float)EleAngleSineValue, (float)EleAngleCosineValue);
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

   /**
   * @brief  Transform current to voltage under rotor coordinate
   * @param[in]  currentD  		current of axis d
   * @param[in]  currentQ  		current of axis q
   * @param[out] voltageD  		voltage of axis d
   * @param[out] voltageQ		voltage of axis q
   */
void PowerAngleCompensation(float expectedCurrentQ, float *powerAngleCompensation_degree)
{
	float powerAngleCompensation_rad = 0;
	
	powerAngleCompensation_rad = arcsine((-2 * (InductanceD - InductanceQ) * expectedCurrentQ) / (RotatorFluxLinkage + sqrt_DSP(Square(RotatorFluxLinkage) + 8 * Square(InductanceD - InductanceQ) * Square(expectedCurrentQ))));

	*powerAngleCompensation_degree = powerAngleCompensation_rad * 360.f	/ (2 * PI);
}

   /**
   * @brief  Transform current to voltage under rotor coordinate
   * @param[in]  controlCurrentQ  		output of current loop
   * @param[out] voltageD  				voltage of axis d
   * @param[out] voltageQ				voltage of axis q
   */
void CurrentVoltageTransform(float controlCurrentQ, float *voltageD, float *voltageQ, float actualEleAngularSpeed_rad)
{
	/*将80%的最大不失真Vq用于提供转速, 20%的最大不失真Vq用于提供转矩*/
	const float targetEleAngularSpeed_rad = 0.8f * MaximumDistortionlessVoltage / RotatorFluxLinkage;
	const float maximumTorque = 0.2f * (MaximumDistortionlessVoltage / PhaseResistance) * 1.5f * MotorMagnetPairs * RotatorFluxLinkage;
	
	/*输出电压*/
	*voltageD = -targetEleAngularSpeed_rad * InductanceQ * controlCurrentQ;
	
	*voltageQ = PhaseResistance * controlCurrentQ + targetEleAngularSpeed_rad * RotatorFluxLinkage;
}

   /**
   * @brief  Calculate voltage of axis d and voltage of axis q
   * @param[in]  actualCurrentQ  					current of axis q
   * @param[out] voltageD  							voltage of axis d
   * @param[out] voltageQ							voltage of axis q
   * @param[in]  actualEleAngularSpeed  		actual Ele angular speed(rad/s)
   */
void CalculateVoltage_dq(float actualCurrentQ, float *voltageD, float *voltageQ, float actualEleAngularSpeed_rad)
{
	*voltageD = -actualEleAngularSpeed_rad * InductanceQ * actualCurrentQ;
	
	*voltageQ = PhaseResistance * actualCurrentQ + actualEleAngularSpeed_rad * RotatorFluxLinkage;
}

   /**
   * @brief  Calculate electromagnetic torque
   * @param[in]   currentQ      				current of axis q
   * @param[out]  electromagneticTorque      	electromagnetic torque
   */
void CalculateElectromagneticTorque(float actualCurrentQ, float *electromagneticTorque)
{
	*electromagneticTorque = 1.5f * MotorMagnetPairs * actualCurrentQ * RotatorFluxLinkage;
}

   /**
   * @brief  Measure reference Ele angle
   * @param[in]  voltageD      voltage of axis d
   */
void MeasureEleAngle(float voltageD)
{
	float voltageAlpha = 0;
	float voltageBeta = 0;
	int16_t tmpData[2] = {0};
	int EleAngle = 0;
	static int16_t index_5012b = 1;
	static int16_t index_bound = 0;
	static int tempMecAngleRef[DivideNum * (uint8_t)MotorMagnetPairs + 2] = {0};
	static int tempEleAngleRef[DivideNum * (uint8_t)MotorMagnetPairs + 2] = {0};
	static int16_t tmpArray[DivideNum * (uint8_t)MotorMagnetPairs] = {0};

	InverseParkTransform_TwoPhase(voltageD, 0.f, &voltageAlpha, &voltageBeta, 0.f);	//设定Uq = 0, 电角度为零
	
	SpaceVectorPulseWidthModulation(voltageAlpha, voltageBeta);
	
	HAL_Delay(1000);
	
	for (int i = 0; i < MotorMagnetPairs; i++)
	{
		for(int j = 0; j < DivideNum; j ++)
		{
			GetPositionImformation();
			
			EleAngle = j * 360 / DivideNum;
			
			InverseParkTransform_TwoPhase(voltageD, 0.f, &voltageAlpha, &voltageBeta, EleAngle);
			
			SpaceVectorPulseWidthModulation(voltageAlpha, voltageBeta);
			
			HAL_Delay(200);

			#if ENCODER_MODE == Encoder_AbsoluteMode
			
			tempMecAngleRef[index_5012b] = Encoder.MecAngle_15bit;
			
			#elif ENCODER_MODE == Encoder_IncrementalMode
			
			tempMecAngleRef[index_5012b] = Encoder.MecAngle_14bit;
			
			#else
			#error "Encoder Mode Invalid"
			#endif

			if(tempMecAngleRef[index_5012b] < 1000 && tempMecAngleRef[index_5012b - 1] > 10000 && index_5012b > 1)
			{
				tmpData[0] = tempMecAngleRef[index_5012b];
				
				tmpData[1] = EleAngle;
				
				index_bound = index_5012b;
			}
			
			UART_Transmit_DMA("\t/*Angle*/\t %d \t /*Encoder*/\t %d \r\n", (int)(EleAngle), (int)tempMecAngleRef[index_5012b]); 
		
			SendBuf();

			index_5012b++;
			
			if(index_5012b > DivideNum * (uint8_t)MotorMagnetPairs)
			{
				PutStr("EXCESS\r\n");SendBuf();
				break;
			}
		}
	}
	
	/*	发送电角度表	*/
	
	PutStr("\r\n\r\n\r\n\r\n");
	
	HAL_Delay(100);
	
	for(int i = index_bound, j = 0; i <= DivideNum * (uint8_t)MotorMagnetPairs;i++,j++)
	{
		tmpArray[j] =  tempMecAngleRef[i];
	}
	
	for(int i = index_bound - 1,k = DivideNum * (uint8_t)MotorMagnetPairs; i >= 0; i--, k--)
	{
		tempMecAngleRef[k] = tempMecAngleRef[i];
	}
	
	for(int i = 1, k =0; k <=  DivideNum * (uint8_t)MotorMagnetPairs - index_bound; i++,k++)
	{
		tempMecAngleRef[i] = tmpArray[k];
	}
	
	tempEleAngleRef[1] = tmpData[1];
	
	for(int i = 1; i <= DivideNum * (uint8_t)MotorMagnetPairs; i++)
	{
		tempEleAngleRef[i + 1] = tempEleAngleRef[i] + 360 / DivideNum;
	}
	
	#if ENCODER_MODE == Encoder_AbsoluteMode
	
	tempMecAngleRef[0] = tempMecAngleRef[DivideNum * (uint8_t)MotorMagnetPairs] - (int32_t)TLE5012_AbsoluteModeResolution;
	
	tempMecAngleRef[DivideNum * (uint8_t)MotorMagnetPairs + 1] = tempMecAngleRef[1] + (int32_t)TLE5012_AbsoluteModeResolution;
	
	#elif ENCODER_MODE == Encoder_IncrementalMode

	tempMecAngleRef[0] = tempMecAngleRef[DivideNum * (uint8_t)MotorMagnetPairs] - (int32_t)(TLE5012_IncrementalModeResolution * 4.f);
	
	tempMecAngleRef[DivideNum * (uint8_t)MotorMagnetPairs + 1] = tempMecAngleRef[1] + (int32_t)(TLE5012_IncrementalModeResolution * 4.f);
	
	#else
	#error "Encoder Mode Invalid"
	#endif

	tempEleAngleRef[0] = tempEleAngleRef[1] - 360 / DivideNum;

	for(int i = 0; i < DivideNum * (uint8_t)MotorMagnetPairs + 2; i++)
	{
		UART_Transmit_DMA("%d\t,\t%d\t,\r\n", (int)tempEleAngleRef[i], (int)tempMecAngleRef[i]);
		
		SendBuf();
	
		HAL_Delay(15);
	}
}


/* USER CODE END */

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

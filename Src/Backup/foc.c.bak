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
   
   	uint32_t TA = 0;
	uint32_t TB = 0;
	uint32_t TC = 0;
void SpaceVectorPulseWidthModulation(float voltageAlpha, float voltageBeta)
{
	const float StandardizationCoefficient = SQRT3 * CarrierPeriod_us_square / GeneratrixVoltage;
	float U1 = 0;
	float U2 = 0;
	float U3 = 0;
	float T1 = 0;
	float T2 = 0;
	float t1 = 0;
	float t2 = 0;
	uint8_t sectionFlag = 0;

	/*为防止调制波形失真, 对Ualpha, Ubeta进行限幅*/
	if(voltageAlpha >= MaximumDistortionlessVoltage)
	{
		voltageAlpha = MaximumDistortionlessVoltage;
	}
	else if(voltageAlpha <= -MaximumDistortionlessVoltage)
	{
		voltageAlpha = -MaximumDistortionlessVoltage;
	}
	if(voltageBeta >= MaximumDistortionlessVoltage)
	{
		voltageBeta = MaximumDistortionlessVoltage;
	}
	else if(voltageBeta <= -MaximumDistortionlessVoltage)
	{
		voltageBeta = -MaximumDistortionlessVoltage;
	}
	
	U1 =  voltageBeta;
	
	U2 = (- voltageBeta + SQRT3 * voltageAlpha) / 2.0f;
	
	U3 = (- voltageBeta - SQRT3 * voltageAlpha) / 2.0f;
	

	/********************
	   判断矢量电压所在扇区
	 *********************/
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
	case 3:  T1 = U2 * StandardizationCoefficient; 
			 T2 = U1 * StandardizationCoefficient;
			 break; 

	case 1:  T1 = - U2 * StandardizationCoefficient; 
			 T2 = - U3 * StandardizationCoefficient; 
			 break;

	case 5:  T1 = U1 * StandardizationCoefficient; 
			 T2 = U3 * StandardizationCoefficient; 
			 break;

	case 4:  T1 = - U1 * StandardizationCoefficient; 
			 T2 = - U2 * StandardizationCoefficient; 
			 break;

	case 6:  T1 = U3 * StandardizationCoefficient; 
			 T2 = U2 * StandardizationCoefficient; 
			 break; 

	case 2:  T1 = - U3 * StandardizationCoefficient; 
			 T2 = - U1 * StandardizationCoefficient; 
			 break; 

	default: T1 = 0u; 
			 T2 = 0u; 
			 break;
	}

	if (T1 + T2 > CarrierPeriod_us_square)
	{
		t1  = T1;
		t2  = T2;
		T1  = t1 / (t1 + t2) * CarrierPeriod_us_square;
		T2  = t2 / (t1 + t2) * CarrierPeriod_us_square;
	}

	TA  = (CarrierPeriod_us_square - T1 - T2) / 2.0f;
	TB  = (CarrierPeriod_us_square + T1 - T2) / 2.0f;
	TC  = (CarrierPeriod_us_square + T1 + T2) / 2.0f;

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

	default: CCR_PhaseA = CarrierPeriod_us_square; 
			 CCR_PhaseB = CarrierPeriod_us_square; 
			 CCR_PhaseC = CarrierPeriod_us_square; 
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
   * @param[in]  electricalAngle		value of electrical angle
   */
void ParkTransform(float currentPhaseA, float currentPhaseB, float currentPhaseC, float *currentD, float *currentQ, float electricalAngle)
{
	const float Coefficient_ConstantPower = 0.8164965809f;
	const float Coefficient_ConstantAmplitude = 0.6666666667f;
	float electricalAngleSineValue1 = 0;
	float electricalAngleCosineValue1 = 0;
	float electricalAngleSineValue2 = 0;
	float electricalAngleCosineValue2 = 0;
	float electricalAngleSineValue3 = 0;
	float electricalAngleCosineValue3 = 0;
	
	arm_sin_cos_f32((float)electricalAngle,  &electricalAngleSineValue1,  &electricalAngleCosineValue1);
	
	arm_sin_cos_f32((float)(electricalAngle - 120.f),  &electricalAngleSineValue2,  &electricalAngleCosineValue2);
	
	arm_sin_cos_f32((float)(electricalAngle + 120.f),  &electricalAngleSineValue3,  &electricalAngleCosineValue3);
	
	*currentD = Coefficient_ConstantAmplitude * (currentPhaseA * electricalAngleSineValue1 + currentPhaseB * electricalAngleSineValue2 + currentPhaseC * electricalAngleSineValue3);
	
	*currentQ = Coefficient_ConstantAmplitude * (currentPhaseA * electricalAngleCosineValue1 + currentPhaseB * electricalAngleCosineValue2 + currentPhaseC * electricalAngleCosineValue3);
}

   /**
   * @brief  Floating-point Inverse Park transform
   * @param[in]  voltageD       	input coordinate of rotor reference frame d
   * @param[in]  voltageQ       	input coordinate of rotor reference frame q
   * @param[out] voltageAlpha 		output two-phase orthogonal vector axis alpha
   * @param[out] voltageBeta  		output two-phase orthogonal vector axis beta
   * @param[in]  electricalAngle	value of electrical angle
   */
void InverseParkTransform_TwoPhase(float voltageD, float voltageQ, float *voltageAlpha, float *voltageBeta, float electricalAngle)
{
	float electricalAngleSineValue = 0;
	float electricalAngleCosineValue = 0;
	
	arm_sin_cos_f32((float)electricalAngle,  &electricalAngleSineValue,  &electricalAngleCosineValue);
	
	arm_inv_park_f32((float)voltageD, (float)voltageQ, voltageAlpha, voltageBeta, (float)electricalAngleSineValue, (float)electricalAngleCosineValue);
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
	
	CoordinateTransformation.CurrentVector = sqrtf(Square(CoordinateTransformation.CurrentAlpha) + Square(CoordinateTransformation.CurrentBeta));	//总电流矢量大小
}

   /**
   * @brief  Transform current to voltage under rotor coordinate
   * @param[in]  currentD  		current of axis d
   * @param[in]  currentQ  		current of axis q
   * @param[out] voltageD  		voltage of axis d
   * @param[out] voltageQ		voltage of axis q
   */
void PowerAngleCompensation(float expectedCurrentQ, float *powerAngleCompensation_degree, float *powerAngleCompensation_rad)
{
	*powerAngleCompensation_rad = arcsine((-2 * (InductanceD - InductanceQ) * expectedCurrentQ) / (RotatorFluxLinkage + sqrt_DSP(Square(RotatorFluxLinkage) + 8 * Square(InductanceD - InductanceQ) * Square(expectedCurrentQ))));
	
	*powerAngleCompensation_degree = *powerAngleCompensation_rad * 360.f	/ (2 * PI);
}

   /**
   * @brief  Transform current to voltage under rotor coordinate
   * @param[in]  controlCurrentQ  		output of current loop
   * @param[out] voltageD  				voltage of axis d
   * @param[out] voltageQ				voltage of axis q
   */
void CurrentVoltageTransform(float controlCurrentQ, float *voltageD, float *voltageQ, float actualElectricalAngularSpeed_rad)
{
	/*将80%的最大不失真Uq用于提供转速, 20%的最大不失真Uq用于提供转矩*/
	const float targetElectricalAngularSpeed_rad = 0.8f * MaximumDistortionlessVoltage / RotatorFluxLinkage;
	const float maximumTorque = 0.2f * (MaximumDistortionlessVoltage / PhaseResistance) * 1.5f * MotorMagnetPairs * RotatorFluxLinkage;
	const float Kp = 15.0f;
	const float Ki = 15.0f;
	static float integralError = 0;
	float error = 0;
	float controlElectricalAngularSpeed_rad = 0;
	
	/*控制转速*/
	if(MotorDynamicParameter.ElectromagneticTorque <= 0.8f * maximumTorque)
	{
		error = targetElectricalAngularSpeed_rad - actualElectricalAngularSpeed_rad;
			
		controlElectricalAngularSpeed_rad = Kp * error + Ki * integralError;
			
		integralError += error * CarrierPeriod_s;
		
	}
	else if(MotorDynamicParameter.ElectromagneticTorque > 0.8f * maximumTorque)
	{
		controlElectricalAngularSpeed_rad = actualElectricalAngularSpeed_rad;
	}
		
	/*输出电压*/
	*voltageD = -controlElectricalAngularSpeed_rad * InductanceQ * controlCurrentQ;
	
	*voltageQ = PhaseResistance * controlCurrentQ + controlElectricalAngularSpeed_rad * RotatorFluxLinkage;
}

   /**
   * @brief  Calculate voltage of axis d and voltage of axis q
   * @param[in]  actualCurrentQ  					current of axis q
   * @param[out] voltageD  							voltage of axis d
   * @param[out] voltageQ							voltage of axis q
   * @param[in]  actualElectricalAngularSpeed  		actual electrical angular speed(rad/s)
   */
void CalculateVoltage_dq(float actualCurrentQ, float *voltageD, float *voltageQ, float actualElectricalAngularSpeed_rad)
{
	*voltageD = -actualElectricalAngularSpeed_rad * InductanceQ * actualCurrentQ;
	
	*voltageQ = PhaseResistance * actualCurrentQ + actualElectricalAngularSpeed_rad * RotatorFluxLinkage;
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
   * @brief  Measure reference electrical angle
   * @param[in]  voltageD      voltage of axis d
   */
void MeasureElectricalAngle(float voltageD)
{
	float voltageAlpha = 0;
	float voltageBeta = 0;
	int16_t tmpData[2] = {0};
	int electricalAngle = 0;
	static int16_t index_5012b = 1;
	static int16_t index_bound = 0;
	static int tempMechanicalAngleRef[DivideNum * (uint8_t)MotorMagnetPairs + 2] = {0};
	static int tempElectricalAngleRef[DivideNum * (uint8_t)MotorMagnetPairs + 2] = {0};
	static int16_t tmpArray[DivideNum * (uint8_t)MotorMagnetPairs] = {0};

	InverseParkTransform_TwoPhase(voltageD, 0.f, &voltageAlpha, &voltageBeta, 0.f);	//设定Uq = 0, 电角度为零
	
	SpaceVectorPulseWidthModulation(voltageAlpha, voltageBeta);
	
	HAL_Delay(1000);
	
	for (int i = 0; i < MotorMagnetPairs; i++)
	{
		for(int j = 0; j < DivideNum; j ++)
		{
			GetPositionImformation();
			
			electricalAngle = j * 360 / DivideNum;
			
			InverseParkTransform_TwoPhase(voltageD, 0.f, &voltageAlpha, &voltageBeta, electricalAngle);
			
			SpaceVectorPulseWidthModulation(voltageAlpha, voltageBeta);
			
			HAL_Delay(200);

			tempMechanicalAngleRef[index_5012b] = Encoder.MechanicalAngle_15bit;

			if(tempMechanicalAngleRef[index_5012b] < 1000 && tempMechanicalAngleRef[index_5012b - 1] > 10000 && index_5012b > 1)
			{
				tmpData[0] = tempMechanicalAngleRef[index_5012b];
				
				tmpData[1] = electricalAngle;
				
				index_bound = index_5012b;
			}
			
			DMAPRINTF("\t %d \t /*Angle \t %d \t Encoder*/ \t %d\r\n", (int)(electricalAngle), (int)tempMechanicalAngleRef[index_5012b]); 
		
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
		tmpArray[j] =  tempMechanicalAngleRef[i];
	}
	
	for(int i = index_bound - 1,k = DivideNum * (uint8_t)MotorMagnetPairs; i >= 0; i--, k--)
	{
		tempMechanicalAngleRef[k] = tempMechanicalAngleRef[i];
	}
	
	for(int i = 1, k =0; k <=  DivideNum * (uint8_t)MotorMagnetPairs - index_bound; i++,k++)
	{
		tempMechanicalAngleRef[i] = tmpArray[k];
	}
	
	tempElectricalAngleRef[1] = tmpData[1];
	
	for(int i = 1; i <= DivideNum * (uint8_t)MotorMagnetPairs; i++)
	{
		tempElectricalAngleRef[i + 1] = tempElectricalAngleRef[i] + 360 / DivideNum;
	}
	
	tempMechanicalAngleRef[0] = tempMechanicalAngleRef[DivideNum * (uint8_t)MotorMagnetPairs] - 32768;
	
	tempMechanicalAngleRef[DivideNum * (uint8_t)MotorMagnetPairs + 1] = tempMechanicalAngleRef[1] + 32768;

	tempElectricalAngleRef[0] = tempElectricalAngleRef[1] - 360 / DivideNum;

	for(int i = 0; i < DivideNum * (uint8_t)MotorMagnetPairs + 2; i++)
	{
		DMAPRINTF("%d\t,\t%d\t,\r\n", (int)tempElectricalAngleRef[i], (int)tempMechanicalAngleRef[i]);
		
		SendBuf();
	
		HAL_Delay(10);
	}
}


/* USER CODE END */

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

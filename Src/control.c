/**
 ******************************************************************************
 * @file		control.c
 * @author		WrathWings
 * @version 	V1.0
 * @date		2019.1.17
 * @brief		Algorithm of control
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
 
 /* Private includes ----------------------------------------------------------*/
/* CODE BEGIN Includes */
#include "control.h"
/* CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* CODE BEGIN PTD */

/* CODE END PTD */

/* Private variables ---------------------------------------------------------*/
/* CODE BEGIN PV */

struct CurrLoop_t CurrLoop;
struct SpdLoop_t SpdLoop;
struct PosLoop_t PosLoop;

/* CODE END PV */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern struct CoordTrans_t CoordTrans;
extern struct PosSensor_t PosSensor;
extern struct MotorStaticParameter_t MotorStaticParameter;
/* USER CODE END EV */

/* USER CODE BEGIN */

void MotorEnable(void)
{
	/*ʹ��PWM���*/
	PWM_IT_CMD(ENABLE,ENABLE);
	
	/*�趨����ģʽ*/
	MotorStaticParameter.ControlMode = SpeedControlMode;
	
	/*����Id = 0����, ���趨d�����Ϊ��*/
	CurrLoop.ExptCurrD = 0.f;
	
	switch(MotorStaticParameter.ControlMode)
	{
		case VoltageControlMode :	/*������*/
//									MotorStaticParameter.PowerAngleComp_degree = 5.0f * CARRIER_PERIOD_S * PosSensor.EleAngularSpeed_degree;
		
									break;
		
		case CurrentControlMode : 	/*�趨q�����*/
									CurrLoop.ExptCurrQ = 10.f;
									
									/*�趨������PI����*/
									CurrLoop.Kp_D = 0.5f;
																
									CurrLoop.Ki_D = 0.1f;
									
									CurrLoop.Kp_Q = 0.9f;
		
									CurrLoop.Ki_Q = 0.1f;
									
									break;
		
		case SpeedControlMode : 	/*�趨���ٶ�(rad/s)*/
									SpdLoop.ExptMecAngularSpeed = 50.f * 2 * PI;	//degree per second
																
									/*�趨���ٶ�(rad/s2)*/
									SpdLoop.Acceleration = 100.f * 2 * PI;	//degree per quadratic seconds
								
									/*�趨������PI����*/
									CurrLoop.Kp_D = 0.5f;
																
									CurrLoop.Ki_D = 0.1f;
									
									CurrLoop.Kp_Q = 0.4f;
		
									CurrLoop.Ki_Q = 0.1f;
								
									/*�趨�ٶȻ�PI����*/
									SpdLoop.Kp = 0.3f;
																
									SpdLoop.Ki = 0.4f;
								
									break;
		
		case PositionControlMode :	/*�趨������PI����*/
									CurrLoop.Kp_D = 0.1f;
																	
									CurrLoop.Ki_D = 0.035f;
																	
									CurrLoop.Kp_Q = 0.10f;

									CurrLoop.Ki_Q = 0.05f;
									
									/*�趨λ�û�PI����*/
									PosLoop.Kp = 0.0001f;
																	
									PosLoop.Kd = 0.f;
									
									break;
	}
}

 /**
   * @brief  ������
   * @param[in]  expectedCurrD     		����Id
   * @param[in]  expectedCurrQ      	����Iq
   * @param[in]  realityCurrD     		ʵ��Id
   * @param[in]  realityCurrQ      		ʵ��Iq
   * @param[out] controlVoltageD 		Vd���
   * @param[out] controlVoltageQ 		Vq���
   */
void CurrentLoop(float exptCurrD, float exptCurrQ, float realityCurrD, float realityCurrQ, float *ctrlVolD, float *ctrlVolQ)
{
	float errD = 0;
	float errQ = 0;
	float ctrlCurrD = 0;
	float ctrlCurrQ = 0;
	static float integralErrD = 0;
	static float integralErrQ = 0;
	
	errD = exptCurrD - realityCurrD;
	errQ = exptCurrQ - realityCurrQ;
	
	/*PI������*/
	ctrlCurrD = CurrLoop.Kp_D * errD + CurrLoop.Ki_D * integralErrD;
	ctrlCurrQ = CurrLoop.Kp_Q * errQ + CurrLoop.Ki_Q * integralErrQ;
	
	integralErrD += errD * CARRIER_PERIOD_S;
	integralErrQ += errQ * CARRIER_PERIOD_S;
	
	/*�����޷�*/
	if(integralErrD >= CURR_INTEGRAL_ERR_LIM_D)
	{
		integralErrD = CURR_INTEGRAL_ERR_LIM_D;
	}
	
	else if(integralErrD <= -CURR_INTEGRAL_ERR_LIM_D)
	{
		integralErrD = -CURR_INTEGRAL_ERR_LIM_D;
	}
	
	if(integralErrQ >= CURR_INTEGRAL_ERR_LIM_Q)
	{
		integralErrQ = CURR_INTEGRAL_ERR_LIM_Q;
	}
	
	else if(integralErrQ <= -CURR_INTEGRAL_ERR_LIM_Q)
	{
		integralErrQ = -CURR_INTEGRAL_ERR_LIM_Q;
	}
	
	/*���綯�Ʋ���*/
	
	*ctrlVolD = - ctrlCurrD * PosSensor.EleAngularSpeed_rad * INDUCTANCE_Q;
	*ctrlVolQ = ctrlCurrQ * PHASE_RES + PosSensor.EleAngularSpeed_rad * ROTATOR_FLUX_LINKAGE;
}

 /**
   * @brief  �ٶȻ�
   * @param[in]  expectedMecAngularSpeed     		������е���ٶ�
   * @param[in]  realityMecAngularSpeed      		ʵ�ʻ�е���ٶ�
   * @param[out] controlCurrentQ 					Iq���
   */
void SpeedLoop(float exptMecAngularSpeed, float realityMecAngularSpeed, float *ctrlCurrQ)
{
	float err = 0;
	static float integralErr = 0;
	
	err = exptMecAngularSpeed - realityMecAngularSpeed;
	
	*ctrlCurrQ = SpdLoop.Kp * err + SpdLoop.Ki * integralErr;
	
	integralErr += err * CARRIER_PERIOD_S;
	
	/*�����޷�*/
	if(integralErr >= SPD_INTEGRAL_ERR_LIM)
	{
		integralErr = SPD_INTEGRAL_ERR_LIM;
	}
	
	else if(integralErr <= -SPD_INTEGRAL_ERR_LIM)	
	{
		integralErr = -SPD_INTEGRAL_ERR_LIM;
	}
}

 /**
   * @brief  λ�û�
   * @param[in]  expectedMecAngle     		������е�Ƕ�
   * @param[in]  realityMecAngle      		ʵ�ʻ�е�Ƕ�
   * @param[out] controlAngularSpeed 		���ٶ����
   */
void PositionLoop(float exptMecAngle, float realityMecAngle, float *controlAngularSpeed)
{
	float err = 0;
	static float lastErr = 0;
	
	err = exptMecAngle - realityMecAngle;
	
	*controlAngularSpeed = PosLoop.Kp * err + PosLoop.Kd * (err - lastErr);
	
	lastErr = err;
}

 /**
   * @brief  б��������
   * @param[in]  expectedVelocity      �������ٶ�
   */
float VelocitySlopeGenerator(float exptVelocity)
{
	static float velocityProcessVolume = 0.0f;
	static float velocityStepValue = 0;
	
	velocityStepValue = SpdLoop.Acceleration * CARRIER_PERIOD_S;

	if (velocityProcessVolume < (exptVelocity - velocityStepValue))
	{
		velocityProcessVolume += velocityStepValue;
	}
	else if (velocityProcessVolume > (exptVelocity + velocityStepValue))
	{
		velocityProcessVolume -= velocityStepValue;
	}
	else
	{
		velocityProcessVolume = exptVelocity;
	}
	
	return velocityProcessVolume;
}

 /**
   * @brief  �����������޷�
   * @param[in]  exptCurr      ����Iq
   */
float CurrentExpectedLimit(float exptCurr)
{
	if(exptCurr >= CURR_EXPT_LIM_Q)
	{
		exptCurr = CURR_EXPT_LIM_Q;
	}
	
	else if(exptCurr <= -CURR_EXPT_LIM_Q)
	{
		exptCurr = -CURR_EXPT_LIM_Q;
	}
	
	else
	{
		exptCurr = exptCurr;
	}
	
	return exptCurr;
}

 /**
   * @brief  ����������
   */
void CurrentController(void)
{
	static float compRatio = 0;
	
	compRatio = 110.0f;
	
	MotorStaticParameter.PowerAngleComp_degree = compRatio * CARRIER_PERIOD_S * PosSensor.EleAngularSpeed_degree;
	
	/*����Park�任, ���������ת��Ϊdq�����*/
	ParkTransform(CoordTrans.CurrA, CoordTrans.CurrB, CoordTrans.CurrC, &CoordTrans.CurrD, &CoordTrans.CurrQ, PosSensor.EleAngle_degree + MotorStaticParameter.PowerAngleComp_degree);
	
	/*������PI������*/
	CurrentLoop(CurrLoop.ExptCurrD, CurrentExpectedLimit(CurrLoop.ExptCurrQ), CoordTrans.CurrD, CoordTrans.CurrQ, &CurrLoop.CtrlVolD, &CurrLoop.CtrlVolQ);
	
	/*������Park�任, ��ת������ϵ�µ�dq���ѹת��Ϊ��������ϵ�µ�AlphaBeta���ѹ*/
	InverseParkTransform(CurrLoop.CtrlVolD, CurrLoop.CtrlVolQ, &CoordTrans.VolAlpha, &CoordTrans.VolBeta, PosSensor.EleAngle_degree + MotorStaticParameter.PowerAngleComp_degree);
	
	/*����SVPWM�㷨���Ƶ�ѹʸ��*/
	SpaceVectorModulation(CoordTrans.VolAlpha, CoordTrans.VolBeta);
}

 /**
   * @brief  ת�ٿ�����
   */
void SpeedController(void)
{
	SpeedLoop(VelocitySlopeGenerator(SpdLoop.ExptMecAngularSpeed), PosSensor.MecAngularSpeed_rad, &CurrLoop.ExptCurrQ);
}

 /**
   * @brief  λ�ÿ�����
   */
void PositionController(void)
{
	PositionLoop(PosLoop.ExptMecAngle, PosSensor.MecAngle_rad, &SpdLoop.ExptMecAngularSpeed);
}

/* USER CODE END */

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

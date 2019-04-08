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
	/*使能PWM输出*/
	PWM_IT_CMD(ENABLE,ENABLE);
	
	/*设定控制模式*/
	MotorStaticParameter.ControlMode = SpeedControlMode;
	
	/*采用Id = 0控制, 故设定d轴电流为零*/
	CurrLoop.ExptCurrD = 0.f;
	
	switch(MotorStaticParameter.ControlMode)
	{
		case VoltageControlMode :	/*测试用*/
//									MotorStaticParameter.PowerAngleComp_degree = 5.0f * CARRIER_PERIOD_S * PosSensor.EleAngularSpeed_degree;
		
									break;
		
		case CurrentControlMode : 	/*设定q轴电流*/
									CurrLoop.ExptCurrQ = 10.f;
									
									/*设定电流环PI参数*/
									CurrLoop.Kp_D = 0.5f;
																
									CurrLoop.Ki_D = 0.1f;
									
									CurrLoop.Kp_Q = 0.9f;
		
									CurrLoop.Ki_Q = 0.1f;
									
									break;
		
		case SpeedControlMode : 	/*设定角速度(rad/s)*/
									SpdLoop.ExptMecAngularSpeed = 50.f * 2 * PI;	//degree per second
																
									/*设定加速度(rad/s2)*/
									SpdLoop.Acceleration = 100.f * 2 * PI;	//degree per quadratic seconds
								
									/*设定电流环PI参数*/
									CurrLoop.Kp_D = 0.5f;
																
									CurrLoop.Ki_D = 0.1f;
									
									CurrLoop.Kp_Q = 0.4f;
		
									CurrLoop.Ki_Q = 0.1f;
								
									/*设定速度环PI参数*/
									SpdLoop.Kp = 0.3f;
																
									SpdLoop.Ki = 0.4f;
								
									break;
		
		case PositionControlMode :	/*设定电流环PI参数*/
									CurrLoop.Kp_D = 0.1f;
																	
									CurrLoop.Ki_D = 0.035f;
																	
									CurrLoop.Kp_Q = 0.10f;

									CurrLoop.Ki_Q = 0.05f;
									
									/*设定位置环PI参数*/
									PosLoop.Kp = 0.0001f;
																	
									PosLoop.Kd = 0.f;
									
									break;
	}
}

 /**
   * @brief  电流环
   * @param[in]  expectedCurrD     		期望Id
   * @param[in]  expectedCurrQ      	期望Iq
   * @param[in]  realityCurrD     		实际Id
   * @param[in]  realityCurrQ      		实际Iq
   * @param[out] controlVoltageD 		Vd输出
   * @param[out] controlVoltageQ 		Vq输出
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
	
	/*PI控制器*/
	ctrlCurrD = CurrLoop.Kp_D * errD + CurrLoop.Ki_D * integralErrD;
	ctrlCurrQ = CurrLoop.Kp_Q * errQ + CurrLoop.Ki_Q * integralErrQ;
	
	integralErrD += errD * CARRIER_PERIOD_S;
	integralErrQ += errQ * CARRIER_PERIOD_S;
	
	/*积分限幅*/
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
	
	/*反电动势补偿*/
	
	*ctrlVolD = - ctrlCurrD * PosSensor.EleAngularSpeed_rad * INDUCTANCE_Q;
	*ctrlVolQ = ctrlCurrQ * PHASE_RES + PosSensor.EleAngularSpeed_rad * ROTATOR_FLUX_LINKAGE;
}

 /**
   * @brief  速度环
   * @param[in]  expectedMecAngularSpeed     		期望机械角速度
   * @param[in]  realityMecAngularSpeed      		实际机械角速度
   * @param[out] controlCurrentQ 					Iq输出
   */
void SpeedLoop(float exptMecAngularSpeed, float realityMecAngularSpeed, float *ctrlCurrQ)
{
	float err = 0;
	static float integralErr = 0;
	
	err = exptMecAngularSpeed - realityMecAngularSpeed;
	
	*ctrlCurrQ = SpdLoop.Kp * err + SpdLoop.Ki * integralErr;
	
	integralErr += err * CARRIER_PERIOD_S;
	
	/*积分限幅*/
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
   * @brief  位置环
   * @param[in]  expectedMecAngle     		期望机械角度
   * @param[in]  realityMecAngle      		实际机械角度
   * @param[out] controlAngularSpeed 		角速度输出
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
   * @brief  斜坡生成器
   * @param[in]  expectedVelocity      期望角速度
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
   * @brief  电流环输入限幅
   * @param[in]  exptCurr      期望Iq
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
   * @brief  电流控制器
   */
void CurrentController(void)
{
	static float compRatio = 0;
	
	compRatio = 110.0f;
	
	MotorStaticParameter.PowerAngleComp_degree = compRatio * CARRIER_PERIOD_S * PosSensor.EleAngularSpeed_degree;
	
	/*进行Park变换, 将三相电流转换为dq轴电流*/
	ParkTransform(CoordTrans.CurrA, CoordTrans.CurrB, CoordTrans.CurrC, &CoordTrans.CurrD, &CoordTrans.CurrQ, PosSensor.EleAngle_degree + MotorStaticParameter.PowerAngleComp_degree);
	
	/*电流环PI控制器*/
	CurrentLoop(CurrLoop.ExptCurrD, CurrentExpectedLimit(CurrLoop.ExptCurrQ), CoordTrans.CurrD, CoordTrans.CurrQ, &CurrLoop.CtrlVolD, &CurrLoop.CtrlVolQ);
	
	/*进行逆Park变换, 将转子坐标系下的dq轴电压转换为定子坐标系下的AlphaBeta轴电压*/
	InverseParkTransform(CurrLoop.CtrlVolD, CurrLoop.CtrlVolQ, &CoordTrans.VolAlpha, &CoordTrans.VolBeta, PosSensor.EleAngle_degree + MotorStaticParameter.PowerAngleComp_degree);
	
	/*利用SVPWM算法调制电压矢量*/
	SpaceVectorModulation(CoordTrans.VolAlpha, CoordTrans.VolBeta);
}

 /**
   * @brief  转速控制器
   */
void SpeedController(void)
{
	SpeedLoop(VelocitySlopeGenerator(SpdLoop.ExptMecAngularSpeed), PosSensor.MecAngularSpeed_rad, &CurrLoop.ExptCurrQ);
}

 /**
   * @brief  位置控制器
   */
void PositionController(void)
{
	PositionLoop(PosLoop.ExptMecAngle, PosSensor.MecAngle_rad, &SpdLoop.ExptMecAngularSpeed);
}

/* USER CODE END */

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

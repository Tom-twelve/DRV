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
struct Regulator_t Regulator;

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
	MotorStaticParameter.ControlMode = VoltageControlMode;
	
	/*采用Id = 0控制, 故设定d轴电流为零*/
	CurrLoop.ExptCurrD = 0.f;
	
	switch(MotorStaticParameter.ControlMode)
	{
		case VoltageControlMode :	/*测试用*/
		
									break;
		
		case CurrentControlMode : 	/*设定q轴电流*/
									CurrLoop.ExptCurrQ = 25.f;
									
									/*设定电流环PI参数*/
									CurrLoop.Kp_D = 2.75f;
																
									CurrLoop.Ki_D = 4.0f;
									
									CurrLoop.Kp_Q = 2.0f;
		
									CurrLoop.Ki_Q = 0.1f;
									
									break;
		
		case SpeedControlMode : 	/*设定角速度(rad/s)*/
									SpdLoop.ExptMecAngularSpeed = 50.f * 2 * PI;	//degree per second
																
									/*设定加速度(rad/s2)*/
									SpdLoop.Acceleration = 500.f * 2 * PI;	//degree per quadratic seconds
								
									/*设定电流环PI参数*/
									CurrLoop.Kp_D = 2.75f;
																
									CurrLoop.Ki_D = 4.0f;
									
									CurrLoop.Kp_Q = 2.0f;
		
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
   * @param[in]  realCurrD     			实际Id
   * @param[in]  realCurrQ      		实际Iq
   * @param[out] controlVoltageD 		Vd输出
   * @param[out] controlVoltageQ 		Vq输出
   */
void CurrentLoop(float exptCurrD, float exptCurrQ, float realCurrD, float realCurrQ, float *ctrlVolD, float *ctrlVolQ)
{
	float errD = 0;
	float errQ = 0;
	float ctrlCurrD = 0;
	float ctrlCurrQ = 0;
	static float integralErrD = 0;
	static float integralErrQ = 0;
	
	errD = exptCurrD - realCurrD;
	errQ = exptCurrQ - realCurrQ;
	
	/*PI控制器*/
	ctrlCurrD = CurrLoop.Kp_D * errD + CurrLoop.Ki_D * integralErrD;
	ctrlCurrQ = CurrLoop.Kp_Q * errQ + CurrLoop.Ki_Q * integralErrQ;
	
	integralErrD += errD * Regulator.ActualPeriod_s;
	integralErrQ += errQ * Regulator.ActualPeriod_s;
	
	/*积分限幅*/
	if(integralErrD >= CURR_INTEGRAL_ERR_LIM_D)
	{
		integralErrD = CURR_INTEGRAL_ERR_LIM_D;
	}
	
	else if(integralErrD <= - CURR_INTEGRAL_ERR_LIM_D)
	{
		integralErrD = - CURR_INTEGRAL_ERR_LIM_D;
	}
	
	if(integralErrQ >= CURR_INTEGRAL_ERR_LIM_Q)
	{
		integralErrQ = CURR_INTEGRAL_ERR_LIM_Q;
	}
	
	else if(integralErrQ <= - CURR_INTEGRAL_ERR_LIM_Q)
	{
		integralErrQ = - CURR_INTEGRAL_ERR_LIM_Q;
	}
	
	/*转速前馈*/

	*ctrlVolD = ctrlCurrD * PHASE_RES + CoordTrans.CurrD * PHASE_RES - PosSensor.EleAngularSpeed_rad * INDUCTANCE_Q * CoordTrans.CurrQ;
	*ctrlVolQ = ctrlCurrQ * PHASE_RES + CoordTrans.CurrQ * PHASE_RES + PosSensor.EleAngularSpeed_rad * INDUCTANCE_D * CoordTrans.CurrD + PosSensor.EleAngularSpeed_rad * ROTATOR_FLUX_LINKAGE;
}

 /**
   * @brief  速度环
   * @param[in]  expectedMecAngularSpeed     		期望机械角速度
   * @param[in]  realMecAngularSpeed      			实际机械角速度
   * @param[out] controlCurrentQ 					Iq输出
   */
void SpeedLoop(float exptMecAngularSpeed, float realMecAngularSpeed, float *ctrlCurrQ)
{
	float err = 0;
	static float integralErr = 0;
	
	err = exptMecAngularSpeed - realMecAngularSpeed;
	
	*ctrlCurrQ = SpdLoop.Kp * err + SpdLoop.Ki * integralErr;
	
	integralErr += err * Regulator.ActualPeriod_s;
	
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
   * @param[in]  realMecAngle      		实际机械角度
   * @param[out] controlAngularSpeed 		角速度输出
   */
void PositionLoop(float exptMecAngle, float realMecAngle, float *controlAngularSpeed)
{
	float err = 0;
	static float lastErr = 0;
	
	err = exptMecAngle - realMecAngle;
	
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
	
	velocityStepValue = SpdLoop.Acceleration * Regulator.ActualPeriod_s;

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
	
	compRatio = 70.0f;
	
	MotorStaticParameter.PowerAngleComp_degree = compRatio * Regulator.ActualPeriod_s * PosSensor.EleAngularSpeed_degree;
	
	/*进行Park变换, 将三相电流转换为dq轴电流*/
	ParkTransform(CoordTrans.CurrA, CoordTrans.CurrB, CoordTrans.CurrC, &CoordTrans.CurrD, &CoordTrans.CurrQ, PosSensor.EleAngle_degree + MotorStaticParameter.PowerAngleComp_degree);
	
	/*电流环PI控制器*/
	CurrentLoop(CurrLoop.ExptCurrD, CurrentExpectedLimit(CurrLoop.ExptCurrQ), CoordTrans.CurrD, CoordTrans.CurrQ, &CurrLoop.CtrlVolD, &CurrLoop.CtrlVolQ);
	
	/*进行逆Park变换, 将转子坐标系下的dq轴电压转换为定子坐标系下的AlphaBeta轴电压*/
	InverseParkTransform(CurrLoop.CtrlVolD, CurrLoop.CtrlVolQ, &CoordTrans.VolAlpha, &CoordTrans.VolBeta, PosSensor.EleAngle_degree + MotorStaticParameter.PowerAngleComp_degree);
	
	/*载波周期调节器, 尽可能使载波周期与编码器周期同步*/
	PeriodRegulator();
	
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

 /**
   * @brief  载波周期调节器
   */
void PeriodRegulator(void)
{
	/*PWM输出的系统周期，注意此变量不是系统实际运行的周期，是一个中间变量，是PID的输出，在幅值上是连续的变量*/
	/*定时器的周期是不能连续的设定的，只能以1/168000000 = 0.005952380952381us为步长设定。*/
	int16_t err = 0;
	static int16_t lastErr = 0;
	static int16_t preErr = 0;
	float diffTerm = 0;
	static float PWMPeriod_PID = DEFAULT_CARRIER_PERIOD_us;
	register int RegulatedARR; // PID控制器输出的ARR值, register关键字可加速运算
	
	/*设定PID参数及目标FSYNC值*/
	Regulator.Kp = PERIOD_REGULATOR_KP;
	Regulator.Ki = PERIOD_REGULATOR_KI;
	Regulator.Kd = PERIOD_REGULATOR_KD;
	Regulator.TargetFSYNC = 4;
	
	err = util_norm_int(Regulator.TargetFSYNC - (int)PosSensor.FSYNC, -16, 16, 32);

	/*增量式PI控制器*/
	PWMPeriod_PID = Regulator.Kp * (err - lastErr) + Regulator.Ki * err;

	/*由于无论系统周期比磁编码器周期更长还是更短，err大于零和err小于零的可能性都是一样的*/
	/*只应用PI调节时周期相差比较大时有可能出现正反馈或者不调节的情况，如系统周期比磁编码器周期更长，
	*但是err小于0的情况, 当误差绝对值增大时，才应用D调节，以期解决上述情况
	*/
	if ((err > 0 && err - lastErr > 0) || (err < 0 && err - lastErr < 0))
	{
		diffTerm = Regulator.Kd * (err - 2 * lastErr + preErr);
		
		PWMPeriod_PID += diffTerm;
	}
	
	lastErr = err;
	preErr = lastErr;
	
	/*PID限幅, 防止系统失控*/
	if(PWMPeriod_PID > DEFAULT_CARRIER_PERIOD_us + PERIOD_REGULATOR_LIM)
	{
		PWMPeriod_PID = DEFAULT_CARRIER_PERIOD_us;
	}
	if(PWMPeriod_PID < DEFAULT_CARRIER_PERIOD_us - PERIOD_REGULATOR_LIM)
	{
		PWMPeriod_PID = DEFAULT_CARRIER_PERIOD_us;
	}
	
	/*离散，得到实际的系统周期*/
	RegulatedARR = PWMPeriod_PID * 90;
	Regulator.ActualPeriod_s = PWMPeriod_PID * 1e-6;
	
	/*改变定时器的ARR寄存器，实际改变周期*/
	TIM1->ARR = RegulatedARR - 1;
}

/* USER CODE END */

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

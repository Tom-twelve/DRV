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
extern struct Driver_t Driver;
/* USER CODE END EV */

/* USER CODE BEGIN */

void DriverInit(void)
{
	/*使能PWM输出*/
	PWM_IT_CMD(ENABLE,ENABLE);
	
	/*设定控制模式*/
	Driver.ControlMode = SPD_CURR_CTRL_MODE;
	
	/*采用Id = 0控制, 故设定d轴电流为零*/
	CurrLoop.ExptCurrD = 0.f;
	
	switch(Driver.ControlMode)
	{
		case VOL_CTRL_MODE :			/*测试用*/
		
										break;
		
		case SPD_CURR_CTRL_MODE : 	
										CurrentLoopInit();

										SpeedLoopInit();

										break;
		
		case POS_SPD_CURR_CTRL_MODE :	
										CurrentLoopInit();
										
										SpeedLoopInit();
										
										PositionLoopInit();
			
										break;
	}
}

 /**
   * @brief  电流环参数初始化
   */
void CurrentLoopInit(void)
{
	/*设定电流环PI参数*/
	CurrLoop.Kp_D = CURRENT_CONTROL_KP_D * 1.0f;												
	CurrLoop.Ki_D = CURRENT_CONTROL_KI_D * 1.0f;						
	CurrLoop.Kp_Q = CURRENT_CONTROL_KP_Q * 1.0f;
	CurrLoop.Ki_Q = CURRENT_CONTROL_KI_Q * 1.0f;
}

 /**
   * @brief  速度环参数初始化
   */
void SpeedLoopInit(void)
{
	/*设定速度环PI参数*/
	SpdLoop.Kp = 1.5f;	
	SpdLoop.Ki = 0.1f;
	SpdLoop.ExptMecAngularSpeed = 100.f * 2 * PI;	//期望速度，degree per second
	SpdLoop.Acceleration = 5000.f * 2 * PI;	//期望加速度，degree per quadratic seconds
}

 /**
   * @brief  位置环参数初始化
   */
void PositionLoopInit(void)
{									
	PosLoop.Kp = 0.0001f;
	PosLoop.Kd = 0.f;		
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
	static float integralErrD = 0;
	static float integralErrQ = 0;
	
	errD = exptCurrD - realCurrD;
	errQ = exptCurrQ - realCurrQ;
	
	/*PI控制器*/
	*ctrlVolD = CurrLoop.Kp_D * errD + CurrLoop.Ki_D * integralErrD;
	*ctrlVolQ = CurrLoop.Kp_Q * errQ + CurrLoop.Ki_Q * integralErrQ;
	
	integralErrD += errD * Regulator.ActualPeriod_s;
	integralErrQ += errQ * Regulator.ActualPeriod_s;
	
	/*积分限幅*/
	AmplitudeLimit(&integralErrD, CURR_INTEGRAL_ERR_LIM_D, -CURR_INTEGRAL_ERR_LIM_D);
	AmplitudeLimit(&integralErrQ, CURR_INTEGRAL_ERR_LIM_Q, -CURR_INTEGRAL_ERR_LIM_Q);
	
	/*转速前馈*/ 
	*ctrlVolD = *ctrlVolD - PosSensor.EleAngularSpeed_rad * INDUCTANCE_Q * CoordTrans.CurrQ;
	*ctrlVolQ = *ctrlVolQ + CoordTrans.CurrQ * PHASE_RES + PosSensor.EleAngularSpeed_rad * ROTATOR_FLUX_LINKAGE;
	
	/*电流环d轴电流限幅, 若限幅过宽启动时振动严重*/
	AmplitudeLimit(ctrlVolD, 1.0f, -1.0f);
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
	
	/*速度环输出限幅*/
	AmplitudeLimit(ctrlCurrQ, CURR_EXPT_LIM_Q, -CURR_EXPT_LIM_Q);
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
   * @brief  电流控制器
   */
void CurrentController(void)
{
	/*进行Park变换, 将三相电流转换为dq轴电流*/
	ParkTransform(CoordTrans.CurrA, CoordTrans.CurrB, CoordTrans.CurrC, &CoordTrans.CurrD, &CoordTrans.CurrQ, PosSensor.EleAngle_degree);
	
	/*电流环PI控制器*/
	CurrentLoop(CurrLoop.ExptCurrD, CurrLoop.ExptCurrQ, CoordTrans.CurrD, CoordTrans.CurrQ, &CurrLoop.CtrlVolD, &CurrLoop.CtrlVolQ);
	
	/*进行逆Park变换, 将转子坐标系下的dq轴电压转换为定子坐标系下的AlphaBeta轴电压*/
	InverseParkTransform(CurrLoop.CtrlVolD, CurrLoop.CtrlVolQ, &CoordTrans.VolAlpha, &CoordTrans.VolBeta, PosSensor.EleAngle_degree);
	
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
	TIM8->ARR = RegulatedARR - 1;
}

/* USER CODE END */

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

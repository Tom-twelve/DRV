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
	/*ʹ��PWM���*/
	PWM_IT_CMD(ENABLE,ENABLE);
	
	/*�趨����ģʽ*/
	Driver.ControlMode = SPD_CURR_CTRL_MODE;
	
	/*����Id = 0����, ���趨d�����Ϊ��*/
	CurrLoop.ExptCurrD = 0.f;
	
	switch(Driver.ControlMode)
	{
		case VOL_CTRL_MODE :			/*������*/
		
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
   * @brief  ������������ʼ��
   */
void CurrentLoopInit(void)
{
	/*�趨������PI����*/
	CurrLoop.Kp_D = CURRENT_CONTROL_KP_D * 1.0f;												
	CurrLoop.Ki_D = CURRENT_CONTROL_KI_D * 1.0f;						
	CurrLoop.Kp_Q = CURRENT_CONTROL_KP_Q * 1.0f;
	CurrLoop.Ki_Q = CURRENT_CONTROL_KI_Q * 1.0f;
}

 /**
   * @brief  �ٶȻ�������ʼ��
   */
void SpeedLoopInit(void)
{
	/*�趨�ٶȻ�PI����*/
	SpdLoop.Kp = 1.5f;	
	SpdLoop.Ki = 0.1f;
	SpdLoop.ExptMecAngularSpeed = 100.f * 2 * PI;	//�����ٶȣ�degree per second
	SpdLoop.Acceleration = 5000.f * 2 * PI;	//�������ٶȣ�degree per quadratic seconds
}

 /**
   * @brief  λ�û�������ʼ��
   */
void PositionLoopInit(void)
{									
	PosLoop.Kp = 0.0001f;
	PosLoop.Kd = 0.f;		
}

 /**
   * @brief  ������
   * @param[in]  expectedCurrD     		����Id
   * @param[in]  expectedCurrQ      	����Iq
   * @param[in]  realCurrD     			ʵ��Id
   * @param[in]  realCurrQ      		ʵ��Iq
   * @param[out] controlVoltageD 		Vd���
   * @param[out] controlVoltageQ 		Vq���
   */
void CurrentLoop(float exptCurrD, float exptCurrQ, float realCurrD, float realCurrQ, float *ctrlVolD, float *ctrlVolQ)
{
	float errD = 0;
	float errQ = 0;
	static float integralErrD = 0;
	static float integralErrQ = 0;
	
	errD = exptCurrD - realCurrD;
	errQ = exptCurrQ - realCurrQ;
	
	/*PI������*/
	*ctrlVolD = CurrLoop.Kp_D * errD + CurrLoop.Ki_D * integralErrD;
	*ctrlVolQ = CurrLoop.Kp_Q * errQ + CurrLoop.Ki_Q * integralErrQ;
	
	integralErrD += errD * Regulator.ActualPeriod_s;
	integralErrQ += errQ * Regulator.ActualPeriod_s;
	
	/*�����޷�*/
	AmplitudeLimit(&integralErrD, CURR_INTEGRAL_ERR_LIM_D, -CURR_INTEGRAL_ERR_LIM_D);
	AmplitudeLimit(&integralErrQ, CURR_INTEGRAL_ERR_LIM_Q, -CURR_INTEGRAL_ERR_LIM_Q);
	
	/*ת��ǰ��*/ 
	*ctrlVolD = *ctrlVolD - PosSensor.EleAngularSpeed_rad * INDUCTANCE_Q * CoordTrans.CurrQ;
	*ctrlVolQ = *ctrlVolQ + CoordTrans.CurrQ * PHASE_RES + PosSensor.EleAngularSpeed_rad * ROTATOR_FLUX_LINKAGE;
	
	/*������d������޷�, ���޷���������ʱ������*/
	AmplitudeLimit(ctrlVolD, 1.0f, -1.0f);
}

 /**
   * @brief  �ٶȻ�
   * @param[in]  expectedMecAngularSpeed     		������е���ٶ�
   * @param[in]  realMecAngularSpeed      			ʵ�ʻ�е���ٶ�
   * @param[out] controlCurrentQ 					Iq���
   */
void SpeedLoop(float exptMecAngularSpeed, float realMecAngularSpeed, float *ctrlCurrQ)
{
	float err = 0;
	static float integralErr = 0;
	
	err = exptMecAngularSpeed - realMecAngularSpeed;
	
	*ctrlCurrQ = SpdLoop.Kp * err + SpdLoop.Ki * integralErr;
	
	integralErr += err * Regulator.ActualPeriod_s;
	
	/*�����޷�*/
	if(integralErr >= SPD_INTEGRAL_ERR_LIM)
	{
		integralErr = SPD_INTEGRAL_ERR_LIM;
	}
	
	else if(integralErr <= -SPD_INTEGRAL_ERR_LIM)	
	{
		integralErr = -SPD_INTEGRAL_ERR_LIM;
	}
	
	/*�ٶȻ�����޷�*/
	AmplitudeLimit(ctrlCurrQ, CURR_EXPT_LIM_Q, -CURR_EXPT_LIM_Q);
}

 /**
   * @brief  λ�û�
   * @param[in]  expectedMecAngle     		������е�Ƕ�
   * @param[in]  realMecAngle      		ʵ�ʻ�е�Ƕ�
   * @param[out] controlAngularSpeed 		���ٶ����
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
   * @brief  б��������
   * @param[in]  expectedVelocity      �������ٶ�
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
   * @brief  ����������
   */
void CurrentController(void)
{
	/*����Park�任, ���������ת��Ϊdq�����*/
	ParkTransform(CoordTrans.CurrA, CoordTrans.CurrB, CoordTrans.CurrC, &CoordTrans.CurrD, &CoordTrans.CurrQ, PosSensor.EleAngle_degree);
	
	/*������PI������*/
	CurrentLoop(CurrLoop.ExptCurrD, CurrLoop.ExptCurrQ, CoordTrans.CurrD, CoordTrans.CurrQ, &CurrLoop.CtrlVolD, &CurrLoop.CtrlVolQ);
	
	/*������Park�任, ��ת������ϵ�µ�dq���ѹת��Ϊ��������ϵ�µ�AlphaBeta���ѹ*/
	InverseParkTransform(CurrLoop.CtrlVolD, CurrLoop.CtrlVolQ, &CoordTrans.VolAlpha, &CoordTrans.VolBeta, PosSensor.EleAngle_degree);
	
	/*�ز����ڵ�����, ������ʹ�ز����������������ͬ��*/
	PeriodRegulator();
	
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

 /**
   * @brief  �ز����ڵ�����
   */
void PeriodRegulator(void)
{
	/*PWM�����ϵͳ���ڣ�ע��˱�������ϵͳʵ�����е����ڣ���һ���м��������PID��������ڷ�ֵ���������ı���*/
	/*��ʱ���������ǲ����������趨�ģ�ֻ����1/168000000 = 0.005952380952381usΪ�����趨��*/
	int16_t err = 0;
	static int16_t lastErr = 0;
	static int16_t preErr = 0;
	float diffTerm = 0;
	static float PWMPeriod_PID = DEFAULT_CARRIER_PERIOD_us;
	register int RegulatedARR; // PID�����������ARRֵ, register�ؼ��ֿɼ�������
	
	/*�趨PID������Ŀ��FSYNCֵ*/
	Regulator.Kp = PERIOD_REGULATOR_KP;
	Regulator.Ki = PERIOD_REGULATOR_KI;
	Regulator.Kd = PERIOD_REGULATOR_KD;
	Regulator.TargetFSYNC = 4;
	
	err = util_norm_int(Regulator.TargetFSYNC - (int)PosSensor.FSYNC, -16, 16, 32);

	/*����ʽPI������*/
	PWMPeriod_PID = Regulator.Kp * (err - lastErr) + Regulator.Ki * err;

	/*��������ϵͳ���ڱȴű��������ڸ������Ǹ��̣�err�������errС����Ŀ����Զ���һ����*/
	/*ֻӦ��PI����ʱ�������Ƚϴ�ʱ�п��ܳ������������߲����ڵ��������ϵͳ���ڱȴű��������ڸ�����
	*����errС��0�����, ��������ֵ����ʱ����Ӧ��D���ڣ����ڽ���������
	*/
	if ((err > 0 && err - lastErr > 0) || (err < 0 && err - lastErr < 0))
	{
		diffTerm = Regulator.Kd * (err - 2 * lastErr + preErr);
		
		PWMPeriod_PID += diffTerm;
	}
	
	lastErr = err;
	preErr = lastErr;
	
	/*PID�޷�, ��ֹϵͳʧ��*/
	if(PWMPeriod_PID > DEFAULT_CARRIER_PERIOD_us + PERIOD_REGULATOR_LIM)
	{
		PWMPeriod_PID = DEFAULT_CARRIER_PERIOD_us;
	}
	if(PWMPeriod_PID < DEFAULT_CARRIER_PERIOD_us - PERIOD_REGULATOR_LIM)
	{
		PWMPeriod_PID = DEFAULT_CARRIER_PERIOD_us;
	}
	
	/*��ɢ���õ�ʵ�ʵ�ϵͳ����*/
	RegulatedARR = PWMPeriod_PID * 90;
	Regulator.ActualPeriod_s = PWMPeriod_PID * 1e-6;
	
	/*�ı䶨ʱ����ARR�Ĵ�����ʵ�ʸı�����*/
	TIM8->ARR = RegulatedARR - 1;
}

/* USER CODE END */

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

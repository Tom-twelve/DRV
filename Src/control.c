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
struct VolCtrl_t VolCtrl;
struct Regulator_t Regulator;
struct MainController_t MainController;
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
	
	/*�Կ������ڸ���ֵ, ��ֹ����ʱ���ַ�ĸΪ0�����*/
	Regulator.ActualPeriod_s = DEFAULT_CARRIER_PERIOD_s;
	
	/*���ñ�����λ��ƫ����*/
	PosSensor.PosOffset = 21693;
	
	switch(Driver.ControlMode)
	{
		case SPD_CURR_CTRL_MODE : 	
										CurrentLoopInit();

										SpeedLoopInit();

										break;
		
		case POS_SPD_CURR_CTRL_MODE :	
										CurrentLoopInit();
										
										SpeedLoopInit();
										
										PositionLoopInit();
			
										break;
		
		case SPD_VOL_CTRL_MODE :		
										VoltageControllerInit();
		
										SpeedLoopInit();
		
										break;
		
		case POS_SPD_VOL_CTRL_MODE :	
										VoltageControllerInit();
		
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
   * @brief  ��ѹ������������ʼ��
   */
void VoltageControllerInit(void)
{
	/*�趨��ѹ�޷�*/
	VolCtrl.VolLimit = 8.0f;
	
	/*�趨���ǲ���ϵ��*/
	VolCtrl.CompRatio_forward = 3.8f;
	
	VolCtrl.CompRatio_reverse = 3.0f;
}

 /**
   * @brief  �ٶȻ�������ʼ��
   */
void SpeedLoopInit(void)
{
	if(Driver.ControlMode == SPD_CURR_CTRL_MODE)
	{
		/*�ٶ�-����˫������*/
		SpdLoop.Kp = 1.0f;	
		SpdLoop.Ki = 0.1f;
		SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;	//�����ٶȣ�degree per second
		SpdLoop.Acceleration = 5000.f * 2 * PI;	//�������ٶȣ�degree per quadratic seconds
		SpdLoop.Deceleration = SpdLoop.Acceleration;
	}
	else if(Driver.ControlMode == POS_SPD_CURR_CTRL_MODE)
	{
		/*λ��-�ٶ�-������������*/
		SpdLoop.Kp = 1.5f;	
		SpdLoop.Ki = 0.1f;
		SpdLoop.Acceleration = 5000.f * 2 * PI;	//�������ٶȣ�degree per quadratic seconds
		SpdLoop.Deceleration = SpdLoop.Acceleration;
	}
	else if(Driver.ControlMode == SPD_VOL_CTRL_MODE)
	{
		/*�ٶȻ���������*/
		SpdLoop.Kp = (ROTATOR_FLUX_LINKAGE * MOTOR_POLE_PAIRS_NUM * 5.5f) * 1.0f;	
		SpdLoop.Ki = (ROTATOR_FLUX_LINKAGE * MOTOR_POLE_PAIRS_NUM * 25.0f) * 1.0f;
		SpdLoop.ExptMecAngularSpeed_rad =  0.f * 2 * PI;	//�����ٶȣ�rad per second
		SpdLoop.Acceleration = 5000.f * 2 * PI;	//�������ٶȣ�rad per quadratic seconds
		SpdLoop.Deceleration = SpdLoop.Acceleration;
	}
	else if(Driver.ControlMode == POS_SPD_VOL_CTRL_MODE)
	{
		/*λ��-�ٶ�˫������*/
		SpdLoop.Kp = (ROTATOR_FLUX_LINKAGE * MOTOR_POLE_PAIRS_NUM * 5.5f) * 1.0f;	
		SpdLoop.Ki = (ROTATOR_FLUX_LINKAGE * MOTOR_POLE_PAIRS_NUM * 25.0f) * 0.0f;
		SpdLoop.Acceleration = 5000.f * 2 * PI;	//�������ٶȣ�degree per quadratic seconds
		SpdLoop.Deceleration = SpdLoop.Acceleration;
	}
}

 /**
   * @brief  λ�û�������ʼ��
   */
void PositionLoopInit(void)
{			
	/*��ȡ��ǰ��е�Ƕ�, ����ر�����ֵ��ֹλ�ÿ���ģʽ�²ο��Ƕȳ���*/
	GetMecAngle_AbsoluteMode_15bit();
	
	MainController.PresentMecAngle_pulse = PosSensor.MecAngle_AbsoluteMode_15bit;
	
	MainController.LastMecAngle_pulse = PosSensor.MecAngle_AbsoluteMode_15bit;
	
	/*�ǵ�ǰλ��Ϊ��λ*/
	MainController.RefMecAngle_pulse = 0;
	
	/*�趨��λƫ����*/
	MainController.ZeroPosOffset = 15000;
	
	/*�趨��λ, ������̶�����λ, �򲻵��øú���*/
	ZeroPosInit();
	
	if(Driver.ControlMode == POS_SPD_CURR_CTRL_MODE)
	{
		/*λ��-�ٶ�-������������*/
		PosLoop.Kp = 0.0f;
		PosLoop.Kd = 0.f;	
		PosLoop.MaxMecAngularSpeed_rad = 15.f * 2 * PI;
		MainController.RefMecAngle_pulse = 0;
		MainController.ExptMecAngle_pulse = 0;
	}
	else if(Driver.ControlMode == POS_SPD_VOL_CTRL_MODE)
	{
		/*λ��-�ٶ�˫������*/
		PosLoop.Kp = 80.5f * 1.0f;
		PosLoop.Kd = 8.5f * 0.1f;	
		PosLoop.MaxMecAngularSpeed_rad = 15.f * 2 * PI;
		
		MainController.ExptMecAngle_pulse = 0;
	}	
}

 /**
   * @brief  λ�ÿ���ģʽ�±궨��ʼλ��
   */
void ZeroPosInit(void)
{
	MainController.RefMecAngle_pulse = PosSensor.MecAngle_AbsoluteMode_15bit - MainController.ZeroPosOffset;
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
	CurrLoop.ErrD = exptCurrD - realCurrD;
	CurrLoop.ErrQ = exptCurrQ - realCurrQ;
	
	/*PI������*/
	*ctrlVolD = CurrLoop.Kp_D * CurrLoop.ErrD + CurrLoop.Ki_D * CurrLoop.IntegralErrD;
	*ctrlVolQ = CurrLoop.Kp_Q * CurrLoop.ErrQ + CurrLoop.Ki_Q * CurrLoop.IntegralErrQ;
	
	CurrLoop.IntegralErrD += CurrLoop.ErrD * Regulator.ActualPeriod_s;
	CurrLoop.IntegralErrQ += CurrLoop.ErrQ * Regulator.ActualPeriod_s;
	
	/*�����޷�*/
	Saturation_float(&CurrLoop.IntegralErrD, CURR_INTEGRAL_ERR_LIM_D, -CURR_INTEGRAL_ERR_LIM_D);
	Saturation_float(&CurrLoop.IntegralErrQ, CURR_INTEGRAL_ERR_LIM_Q, -CURR_INTEGRAL_ERR_LIM_Q);
	
	/*ת��ǰ��*/ 
	*ctrlVolD = *ctrlVolD - PosSensor.EleAngularSpeed_rad * INDUCTANCE_Q * CoordTrans.CurrQ;
	*ctrlVolQ = *ctrlVolQ + CoordTrans.CurrQ * PHASE_RES + PosSensor.EleAngularSpeed_rad * ROTATOR_FLUX_LINKAGE;
	
	/*������d������޷�, ���޷���������ʱ������*/
	Saturation_float(ctrlVolD, 1.0f, -1.0f);
}

 /**
   * @brief  �ٶȻ�
   * @param[in]  expectedMecAngularSpeed     		������е���ٶ�
   * @param[in]  realMecAngularSpeed      			ʵ�ʻ�е���ٶ�
   * @param[out] controlCurrentQ 					Iq���
   */
void SpeedLoop(float exptMecAngularSpeed, float realMecAngularSpeed, float *ctrlCurrQ)
{
	SpdLoop.Err = exptMecAngularSpeed - realMecAngularSpeed;
	
	*ctrlCurrQ = SpdLoop.Kp * SpdLoop.Err + SpdLoop.Ki * SpdLoop.IntegralErr;
	
	SpdLoop.IntegralErr += SpdLoop.Err * Regulator.ActualPeriod_s;
	
	/*�����޷�*/
	Saturation_float(&SpdLoop.IntegralErr, SPD_INTEGRAL_ERR_LIM, -SPD_INTEGRAL_ERR_LIM);

	if(Driver.ControlMode == SPD_CURR_CTRL_MODE || Driver.ControlMode == POS_SPD_CURR_CTRL_MODE)
	{
		/*�ٶȻ�����޷�*/
		Saturation_float(ctrlCurrQ, CURR_EXPT_LIM_Q, -CURR_EXPT_LIM_Q);
	}
}

 /**
   * @brief  λ�û�
   * @param[in]  expectedMecAngle     		������е�Ƕ�
   * @param[in]  realMecAngle      		ʵ�ʻ�е�Ƕ�
   * @param[out] controlAngularSpeed 		���ٶ����
   */
void PositionLoop(float exptMecAngle, float realMecAngle, float *ctrlAngularSpeed)
{
	float err = 0;
	static float lastErr = 0;
	static float diffErr = 0;
		
	err = exptMecAngle - realMecAngle;
	
	diffErr = (err - lastErr) / Regulator.ActualPeriod_s;
	
	*ctrlAngularSpeed = PosLoop.Kp * err + PosLoop.Kd * diffErr;
	
	lastErr = err;
	
	Saturation_float(ctrlAngularSpeed, PosLoop.MaxMecAngularSpeed_rad, -PosLoop.MaxMecAngularSpeed_rad);
}

 /**
   * @brief  б��������
   * @param[in]  expectedVelocity      �������ٶ�
   */
float VelocitySlopeGenerator(float exptVelocity)
{
	static float velocityProcessVolume = 0.0f;
	static float velocityStep_Acc = 0;
	static float velocityStep_Dec = 0;
	
	velocityStep_Acc = SpdLoop.Acceleration * Regulator.ActualPeriod_s;
	velocityStep_Dec = SpdLoop.Deceleration * Regulator.ActualPeriod_s;
	
	if(exptVelocity > 0 && velocityProcessVolume >= 0)
	{
		if(exptVelocity > (velocityProcessVolume - velocityStep_Acc))
		{
			velocityProcessVolume += velocityStep_Acc;
		}
		else if(exptVelocity < (velocityProcessVolume + velocityStep_Dec))
		{
			velocityProcessVolume -= velocityStep_Dec;
		}
		else
		{
			velocityProcessVolume = exptVelocity;
		}
	}
	else if(exptVelocity > 0 && velocityProcessVolume < 0)
	{
		velocityProcessVolume += velocityStep_Dec;
	}
	else if(exptVelocity < 0 && velocityProcessVolume >=  0)
	{
		velocityProcessVolume -= velocityStep_Dec;
	}
	else if(exptVelocity < 0 && velocityProcessVolume < 0)
	{
		if(exptVelocity > (velocityProcessVolume - velocityStep_Dec))
		{
			velocityProcessVolume += velocityStep_Dec;
		}
		else if(exptVelocity < (velocityProcessVolume + velocityStep_Acc))
		{
			velocityProcessVolume -= velocityStep_Acc;
		}
		else
		{
			velocityProcessVolume = exptVelocity;
		}
	}
	else if(exptVelocity == 0)
	{
		velocityProcessVolume = 0;
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
//	PeriodRegulator();
	
	/*����SVPWM�㷨���Ƶ�ѹʸ��*/
	SpaceVectorModulation(CoordTrans.VolAlpha, CoordTrans.VolBeta);
}

 /**
   * @brief  ��ѹ������
   */
void VoltageController(void)
{  
	/*����Id = 0����, ��Vd = 0ʱ, Id����Ϊ��*/
	VolCtrl.CtrlVolD = 0.f;
	
	/*����q�ᷴ�綯��*/
	VolCtrl.BEMF = ROTATOR_FLUX_LINKAGE * PosSensor.EleAngularSpeed_rad;
	
	/*Vq�޷�*/
	Saturation_float(&VolCtrl.CtrlVolQ, VolCtrl.BEMF + VolCtrl.VolLimit, VolCtrl.BEMF - VolCtrl.VolLimit);
	
	if(SpdLoop.ExptMecAngularSpeed_rad >= 0)
	{
		VolCtrl.CompRatio = VolCtrl.CompRatio_forward;
	}
	else if(SpdLoop.ExptMecAngularSpeed_rad < 0)
	{
		VolCtrl.CompRatio = VolCtrl.CompRatio_reverse;
	}
	
	/*������Park�任, ��ת������ϵ�µ�dq���ѹת��Ϊ��������ϵ�µ�AlphaBeta���ѹ*/
	InverseParkTransform(VolCtrl.CtrlVolD, VolCtrl.CtrlVolQ, &CoordTrans.VolAlpha, &CoordTrans.VolBeta, PosSensor.EleAngle_degree + VolCtrl.CompRatio * PosSensor.EleAngularSpeed_degree * Regulator.ActualPeriod_s);
	
	/*�ز����ڵ�����, ������ʹ�ز����������������ͬ��*/
//	PeriodRegulator();
	
	/*����SVPWM�㷨���Ƶ�ѹʸ��*/
	SpaceVectorModulation(CoordTrans.VolAlpha, CoordTrans.VolBeta);
}

 /**
   * @brief  ת�ٿ�����
   */
void SpeedController(void)
{	
	if(Driver.ControlMode == SPD_CURR_CTRL_MODE || Driver.ControlMode == POS_SPD_CURR_CTRL_MODE)
	{
		SpeedLoop(VelocitySlopeGenerator(SpdLoop.ExptMecAngularSpeed_rad), PosSensor.MecAngularSpeed_rad, &CurrLoop.ExptCurrQ);
	}
	else if(Driver.ControlMode == SPD_VOL_CTRL_MODE || Driver.ControlMode == POS_SPD_VOL_CTRL_MODE)
	{
		SpeedLoop(VelocitySlopeGenerator(SpdLoop.ExptMecAngularSpeed_rad), PosSensor.MecAngularSpeed_rad, &VolCtrl.CtrlVolQ);
	}
}

 /**
   * @brief  λ�ÿ�����
   */
void PositionController(void)
{
	PosLoop.ExptMecAngle_rad = PULSE_TO_RAD(MainController.ExptMecAngle_pulse);
	
	PositionLoop(PosLoop.ExptMecAngle_rad, PULSE_TO_RAD(MainController.RefMecAngle_pulse), &SpdLoop.ExptMecAngularSpeed_rad);
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
	else if(PWMPeriod_PID < DEFAULT_CARRIER_PERIOD_us - PERIOD_REGULATOR_LIM)
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

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
struct TorqueCtrl_t TorqueCtrl;
struct MainCtrl_t MainCtrl;
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
	
	#if ROBOT_ID == PASS_ROBOT
		#if CAN_ID_NUM == 1		//���� 
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 8541;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad =  0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 2	//����
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 8609;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 3	//����
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 28330;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 4	//����
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 23178;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 5	//����
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 4785;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 6	//����
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 26613;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 7	//����
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 12190;
			CurrLoop.LimitCurrQ = 350.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 8	//����
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 31848;
			CurrLoop.LimitCurrQ = 50.f;
			SpdLoop.ExptMecAngularSpeed_rad = 80.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 9	//����
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 5861;
			CurrLoop.LimitCurrQ = 50.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
			
		#elif CAN_ID_NUM == 10	//����
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 32089;
			CurrLoop.LimitCurrQ = 50.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 11	//����
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 27992;
			CurrLoop.LimitCurrQ = 50.f;
			SpdLoop.ExptMecAngularSpeed_rad = 20.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#endif
	#endif
	#if ROBOT_ID == TRY_ROBOT
		#if CAN_ID_NUM == 1
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 12190;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 2
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 26339;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;	
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 3
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 21030;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 4
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 12190;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 5
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 12190;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 6
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 12190;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#endif
	#endif
}

 /**
   * @brief  ������������ʼ��
   */
void CurrentLoopInit(void)
{
	/*�趨������PI����, q��������������ü򻯵��ģ��+I����, ��ͬ�ڴ�ͳPI������, ��I�������˹�����*/		
	CurrLoop.Kp_D = CURRENT_CONTROL_KP_D;
	CurrLoop.Ki_D = CURRENT_CONTROL_KI_D;						
	CurrLoop.Ki_Q = CURRENT_CONTROL_KI_Q;
	
	/*�趨�������ӳٲ���ϵ��*/
	PosSensor.CompRatio_forward = 3.2f;
	PosSensor.CompRatio_reverse = 3.2f;
	
	/*�趨���Iq*/
	CurrLoop.LimitCurrQ = 20.f;
}

 /**
   * @brief  �ٶȻ�������ʼ��
   */
void SpeedLoopInit(void)
{
	if(Driver.ControlMode == SPD_CURR_CTRL_MODE)
	{
		/*�ٶ�-����˫������*/
		SpdLoop.Kp = SPEED_CONTROL_KP;	
		SpdLoop.Ki = SPEED_CONTROL_KI;
		SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;	//�����ٶ�, rad per second
		SpdLoop.MaxExptMecAngularSpeed_rad = MAX_SPD;	//ת���޷�
		SpdLoop.Acceleration = 3000.f * 2 * PI;	//�������ٶ�, rad per quadratic seconds
		SpdLoop.Deceleration = SpdLoop.Acceleration;
	}
	else if(Driver.ControlMode == POS_SPD_CURR_CTRL_MODE)
	{
		/*λ��-�ٶ�-������������*/
		SpdLoop.Kp = SPEED_CONTROL_KP;
		SpdLoop.Ki = SPEED_CONTROL_KI;
		SpdLoop.MaxExptMecAngularSpeed_rad = 15.f * 2 * PI;	//ת���޷�
		SpdLoop.Acceleration = 3000.f * 2 * PI;	//�������ٶ�, rad per quadratic seconds
		SpdLoop.Deceleration = SpdLoop.Acceleration;
	}
	if(Driver.ControlMode == TORQUE_CTRL_MODE)
	{
		/*ת�ؿ���ģʽ��, ��ת�ٽӽ�ת���޷�ʱ�����ٶȻ�����*/
		SpdLoop.Kp = SPEED_CONTROL_KP;	
		SpdLoop.Ki = SPEED_CONTROL_KI;
		SpdLoop.MaxExptMecAngularSpeed_rad = TorqueCtrl.MaxMecSpd_rad;	//ת���޷�
	}
}

 /**
   * @brief  λ�û�������ʼ��
   */
void PositionLoopInit(void)
{					
	if(Driver.ControlMode == POS_SPD_CURR_CTRL_MODE)
	{
		/*λ��-�ٶ�-������������*/
		PosLoop.Kp = POSITION_CONTROL_KP;
		PosLoop.Kd = POSITION_CONTROL_KD;	
		MainCtrl.ExptMecAngle_pulse = 0;
		PosLoop.MecAngleUpperLimit_rad = 1024 * 2.f * PI;
		PosLoop.MecAngleLowerLimit_rad = -1024 * 2.f * PI;
	}
	else if(Driver.ControlMode == POS_CURR_CTRL_MODE)
	{
		/*λ��-����˫������*/
		PosLoop.Kp = 5.0f;
		PosLoop.Kd = 0.001f;
		MainCtrl.ExptMecAngle_pulse = 0;
		PosLoop.MecAngleUpperLimit_rad = 1024 * 2.f * PI;
		PosLoop.MecAngleLowerLimit_rad = -1024 * 2.f * PI;
	}
}

 /**
   * @brief  ת�ؿ�������ʼ��
   */
void TorqueCtrlInit(void)
{					
	TorqueCtrl.ExptTorque_Nm = 1.0f;	//����Ť��
	TorqueCtrl.MaxMecSpd_rad = 60.f * 2.f * PI;	//ת���޷�
	CurrLoop.LimitCurrQ = 200.f;		//Iq�޷�
}

 /**
   * @brief  λ�ÿ���ģʽ�±궨��ʼλ��
   */
void RefAngleInit(void)
{
	MainCtrl.PresentMecAngle_pulse = PosSensor.MecAngle_15bit;
	
	MainCtrl.LastMecAngle_pulse = PosSensor.MecAngle_15bit;
	
	/*�ǵ�ǰλ��Ϊ��λ*/
	MainCtrl.RefMecAngle_pulse = 0;
}

 /**
   * @brief  λ�ÿ���ģʽ�±궨��ʼλ��
   */
void ZeroPosSet(uint16_t posOffset)
{
	MainCtrl.RefMecAngle_pulse = PosSensor.MecAngle_15bit - posOffset;
}

 /**
   * @brief  ������
   * @param[in]  exptCurrD     		����Id
   * @param[in]  exptCurrQ      	����Iq
   * @param[in]  realCurrD     		ʵ��Id
   * @param[in]  realCurrQ      	ʵ��Iq
   * @param[out] ctrlVolD 			Vd���
   * @param[out] ctrlVolQ 			Vq���
   */
void CurrentLoop(float exptCurrD, float exptCurrQ, float realCurrD, float realCurrQ, float *ctrlVolD, float *ctrlVolQ)
{
	/*Iq�����޷�*/
	Saturation_float(&exptCurrQ, CurrLoop.LimitCurrQ, -CurrLoop.LimitCurrQ);
	
	CurrLoop.ErrD = exptCurrD - realCurrD;
	CurrLoop.ErrQ = exptCurrQ - realCurrQ;
	
	CurrLoop.IntegralErrD += CurrLoop.ErrD * DEFAULT_CARRIER_PERIOD_s;
	CurrLoop.IntegralErrQ += CurrLoop.ErrQ * DEFAULT_CARRIER_PERIOD_s;
	
	/*�����޷�*/
	Saturation_float(&CurrLoop.IntegralErrD, CURR_INTEGRAL_ERR_LIM_D, -CURR_INTEGRAL_ERR_LIM_D);
	Saturation_float(&CurrLoop.IntegralErrQ, CURR_INTEGRAL_ERR_LIM_Q, -CURR_INTEGRAL_ERR_LIM_Q);
	
	/*���ڵ��ģ�͵ĸĽ��͵���������, ���ü򻯵��ģ��+I����*/
	*ctrlVolD = CurrLoop.Kp_D * CurrLoop.ErrD + CurrLoop.Ki_D * CurrLoop.IntegralErrD;
	*ctrlVolQ = CurrLoop.Ki_Q * CurrLoop.IntegralErrQ + exptCurrQ * PHASE_RES + PosSensor.EleAngularSpeed_rad * ROTATOR_FLUX_LINKAGE;
	
	/*��ѹʸ���޷�*/
	CurrLoop.LimitVolD = GENERATRIX_VOL / SQRT3;
	arm_sqrt_f32(SQUARE(GENERATRIX_VOL) / 3.f - SQUARE(*ctrlVolD), &CurrLoop.LimitVolQ);
	
	Saturation_float(ctrlVolD, CurrLoop.LimitVolD, -CurrLoop.LimitVolD);
	Saturation_float(ctrlVolQ, CurrLoop.LimitVolQ, -CurrLoop.LimitVolQ);
}

 /**
   * @brief  �ٶȻ�
   * @param[in]  exptMecAngularSpeed     	������е���ٶ�
   * @param[in]  realMecAngularSpeed      	ʵ�ʻ�е���ٶ�
   * @param[out] ctrlCurrQ 					Iq���
   */
void SpeedLoop(float exptMecAngularSpeed, float realMecAngularSpeed, float *ctrlCurrQ)
{
	/*��������ٶ��޷�*/
	Saturation_float(&exptMecAngularSpeed, SpdLoop.MaxExptMecAngularSpeed_rad, -SpdLoop.MaxExptMecAngularSpeed_rad);
	
	SpdLoop.Err = exptMecAngularSpeed - realMecAngularSpeed;
	
	SpdLoop.IntegralErr += SpdLoop.Err * OUTER_LOOP_PERIOD;
	
	/*�����޷�*/
	Saturation_float(&SpdLoop.IntegralErr, SPD_INTEGRAL_ERR_LIM, -SPD_INTEGRAL_ERR_LIM);
	
	*ctrlCurrQ = SpdLoop.Kp * SpdLoop.Err + SpdLoop.Ki * SpdLoop.IntegralErr;
}

 /**
   * @brief  λ�û�
   * @param[in]  exptMecAngle     		������е�Ƕ�
   * @param[in]  realMecAngle      		ʵ�ʻ�е�Ƕ�
   * @param[out] ctrlAngularSpeed 		���ٶ����
   */
void PositionLoop(float exptMecAngle, float realMecAngle, float *ctrlAngularSpeed)
{
	/*λ���޷�*/
	Saturation_float(&exptMecAngle, PosLoop.MecAngleUpperLimit_rad, PosLoop.MecAngleLowerLimit_rad);
	
	PosLoop.Err = exptMecAngle - realMecAngle;
		
	PosLoop.DiffErr = (PosLoop.Err - PosLoop.LastErr) / OUTER_LOOP_PERIOD;
	
	*ctrlAngularSpeed = PosLoop.Kp * PosLoop.Err + PosLoop.Kd * PosLoop.DiffErr;
		
	PosLoop.LastErr = PosLoop.Err;
}

 /**
   * @brief  �ٶ�-����������
   */
void SpdCurrController(void)
{	
	static uint16_t Count = PERIOD_MULTIPLE - 1;	//ʹ�����һ��ִ���ж�ʱ, �����ٶȻ�, λ�û�
	
	Count++;
	
	/*����Id = 0����, ���趨d�����Ϊ��*/
	CurrLoop.ExptCurrD = 0.f;
	
	/*λ�û����ٶȻ��������ǵ��������ڵ�ʮ��*/
	if(Count == PERIOD_MULTIPLE)
	{
		/*���»�е�ٶȼ�λ����Ϣ*/
		GetMecImformation();
		
		SpeedLoop(VelSlopeGenerator(SpdLoop.ExptMecAngularSpeed_rad), PosSensor.MecAngularSpeed_rad, &CurrLoop.ExptCurrQ);
		
		Count = 0;
	}
	
	/*����Clark�任, ��abc����ϵת��ΪAlpha-Beta����ϵ*/
	ClarkeTransform(CoordTrans.CurrA, CoordTrans.CurrB, &CoordTrans.CurrAlpha, &CoordTrans.CurrBeta);

	/*����ת�������ӳٲ���ϵ����ͬ*/
	if(PosSensor.EleAngularSpeed_degree >= 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_forward;
	}
	else if(PosSensor.EleAngularSpeed_degree < 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_reverse;
	}
	
	PosSensor.CompAngle = PosSensor.CompRatio * PosSensor.EleAngularSpeed_degree * DEFAULT_CARRIER_PERIOD_s;
	
	/*����Park�任, ��Alpha-Beta����ϵת��Ϊdq����ϵ*/
	ParkTransform(CoordTrans.CurrAlpha, CoordTrans.CurrBeta, &CoordTrans.CurrD, &CoordTrans.CurrQ, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	/*������PI������*/
	CurrentLoop(CurrLoop.ExptCurrD, CurrLoop.ExptCurrQ, CoordTrans.CurrD, CoordTrans.CurrQ, &CurrLoop.CtrlVolD, &CurrLoop.CtrlVolQ);
	
	/*������Park�任, ��ת������ϵ�µ�dq���ѹת��Ϊ��������ϵ�µ�AlphaBeta����*/
	InverseParkTransform(CurrLoop.CtrlVolD, CurrLoop.CtrlVolQ, &CoordTrans.VolAlpha, &CoordTrans.VolBeta, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	/*����SVPWM�㷨���Ƶ�ѹʸ��*/
	SpaceVectorModulation(CoordTrans.VolAlpha, CoordTrans.VolBeta);
}

 /**
   * @brief  λ��-�ٶ�-����������
   */
void PosSpdCurrController(void)
{	
	static uint16_t Count = PERIOD_MULTIPLE - 1;	//ʹ�����һ��ִ���ж�ʱ, �����ٶȻ�, λ�û�
	
	Count++;
	
	/*����Id = 0����, ���趨d�����Ϊ��*/
	CurrLoop.ExptCurrD = 0.f;
	
	/*λ�û����ٶȻ��������ǵ��������ڵ�ʮ��*/
	if(Count == PERIOD_MULTIPLE)
	{
		/*���»�е�ٶȼ�λ����Ϣ*/
		GetMecImformation();
		
		PosLoop.ExptMecAngle_rad = DRV_PULSE_TO_RAD(MainCtrl.ExptMecAngle_pulse);
		
		PosLoop.RefMecAngle_rad = DRV_PULSE_TO_RAD(MainCtrl.RefMecAngle_pulse);
		
		PositionLoop(PosLoop.ExptMecAngle_rad, PosLoop.RefMecAngle_rad, &SpdLoop.ExptMecAngularSpeed_rad);
				
		SpeedLoop(VelSlopeGenerator(SpdLoop.ExptMecAngularSpeed_rad), PosSensor.MecAngularSpeed_rad, &CurrLoop.ExptCurrQ);
		
		Count = 0;
	}
	
	/*����Clark�任, ��abc����ϵת��ΪAlpha-Beta����ϵ*/
	ClarkeTransform(CoordTrans.CurrA, CoordTrans.CurrB, &CoordTrans.CurrAlpha, &CoordTrans.CurrBeta);

	/*����ת�������ӳٲ���ϵ����ͬ*/
	if(PosSensor.EleAngularSpeed_degree >= 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_forward;
	}
	else if(PosSensor.EleAngularSpeed_degree < 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_reverse;
	}
	
	PosSensor.CompAngle = PosSensor.CompRatio * PosSensor.EleAngularSpeed_degree * DEFAULT_CARRIER_PERIOD_s;
	
	/*����Park�任, ��Alpha-Beta����ϵת��Ϊdq����ϵ*/
	ParkTransform(CoordTrans.CurrAlpha, CoordTrans.CurrBeta, &CoordTrans.CurrD, &CoordTrans.CurrQ, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	/*������PI������*/
	CurrentLoop(CurrLoop.ExptCurrD, CurrLoop.ExptCurrQ, CoordTrans.CurrD, CoordTrans.CurrQ, &CurrLoop.CtrlVolD, &CurrLoop.CtrlVolQ);
	
	/*������Park�任, ��ת������ϵ�µ�dq���ѹת��Ϊ��������ϵ�µ�AlphaBeta����*/
	InverseParkTransform(CurrLoop.CtrlVolD, CurrLoop.CtrlVolQ, &CoordTrans.VolAlpha, &CoordTrans.VolBeta, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	/*����SVPWM�㷨���Ƶ�ѹʸ��*/
	SpaceVectorModulation(CoordTrans.VolAlpha, CoordTrans.VolBeta);
}

 /**
   * @brief  λ��-����������
   */
void PosCurrController(void)
{	
	static uint16_t Count = PERIOD_MULTIPLE - 1;	//ʹ�����һ��ִ���ж�ʱ, �����ٶȻ�, λ�û�
	
	Count++;
	
	/*����Id = 0����, ���趨d�����Ϊ��*/
	CurrLoop.ExptCurrD = 0.f;
	
	/*λ�û����ٶȻ��������ǵ��������ڵ�ʮ��*/
	if(Count == PERIOD_MULTIPLE)
	{
		/*���»�е�ٶȼ�λ����Ϣ*/
		GetMecImformation();
		
		PosLoop.ExptMecAngle_rad = DRV_PULSE_TO_RAD(MainCtrl.ExptMecAngle_pulse);
		
		PosLoop.RefMecAngle_rad = DRV_PULSE_TO_RAD(MainCtrl.RefMecAngle_pulse);
		
		PositionLoop(PosLoop.ExptMecAngle_rad, PosLoop.RefMecAngle_rad, &CurrLoop.ExptCurrQ);
		
		Count = 0;
	}
	
	/*����Clark�任, ��abc����ϵת��ΪAlpha-Beta����ϵ*/
	ClarkeTransform(CoordTrans.CurrA, CoordTrans.CurrB, &CoordTrans.CurrAlpha, &CoordTrans.CurrBeta);

	/*����ת�������ӳٲ���ϵ����ͬ*/
	if(PosSensor.EleAngularSpeed_degree >= 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_forward;
	}
	else if(PosSensor.EleAngularSpeed_degree < 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_reverse;
	}
	
	/*���㲹���Ƕ�*/
	PosSensor.CompAngle = PosSensor.CompRatio * PosSensor.EleAngularSpeed_degree * DEFAULT_CARRIER_PERIOD_s;
	
	/*����Park�任, ��Alpha-Beta����ϵת��Ϊdq����ϵ*/
	ParkTransform(CoordTrans.CurrAlpha, CoordTrans.CurrBeta, &CoordTrans.CurrD, &CoordTrans.CurrQ, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	/*������PI������*/
	CurrentLoop(CurrLoop.ExptCurrD, CurrLoop.ExptCurrQ, CoordTrans.CurrD, CoordTrans.CurrQ, &CurrLoop.CtrlVolD, &CurrLoop.CtrlVolQ);
	
	/*������Park�任, ��ת������ϵ�µ�dq���ѹת��Ϊ��������ϵ�µ�AlphaBeta����*/
	InverseParkTransform(CurrLoop.CtrlVolD, CurrLoop.CtrlVolQ, &CoordTrans.VolAlpha, &CoordTrans.VolBeta, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	/*����SVPWM�㷨���Ƶ�ѹʸ��*/
	SpaceVectorModulation(CoordTrans.VolAlpha, CoordTrans.VolBeta);
}

 /**
   * @brief  ת�ؿ�����
   */
void TorqueController(void)
{  
	static uint16_t Count = PERIOD_MULTIPLE - 1;
	
	Count++;
	
	/*����Id = 0����, ���趨d�����Ϊ��*/
	CurrLoop.ExptCurrD = 0.f;
		
	/*�ٶȻ��������ǵ��������ڵ�ʮ��*/
	if(Count == PERIOD_MULTIPLE)
	{
		/*���»�е�ٶȼ�λ����Ϣ*/
		GetMecImformation();
					
		/*ת�ٽӽ�ת���޷�ʱ, ����Iq��Ϊ�ٶȻ��ļ�����*/
		if(PosSensor.MecAngularSpeed_rad < (TorqueCtrl.MaxMecSpd_rad - 3.f * 2.f * PI))
		{		
			/*��Ŀ��ת��ת��ΪIq*/
			CurrLoop.ExptCurrQ = TorqueCtrl.ExptTorque_Nm / (1.5f * MOTOR_POLE_PAIRS_NUM * ROTATOR_FLUX_LINKAGE);
		}
		else if(PosSensor.MecAngularSpeed_rad >= (TorqueCtrl.MaxMecSpd_rad - 3.f * 2.f * PI))
		{
			SpeedLoop(TorqueCtrl.MaxMecSpd_rad, PosSensor.MecAngularSpeed_rad, &CurrLoop.ExptCurrQ);
			
			Saturation_float(&CurrLoop.ExptCurrQ, TorqueCtrl.ExptTorque_Nm / (1.5f * MOTOR_POLE_PAIRS_NUM * ROTATOR_FLUX_LINKAGE), -TorqueCtrl.ExptTorque_Nm / (1.5f * MOTOR_POLE_PAIRS_NUM * ROTATOR_FLUX_LINKAGE));
		}
			
		Count = 0;
	}

	/*����Clark�任, ��abc����ϵת��ΪAlpha-Beta����ϵ*/
	ClarkeTransform(CoordTrans.CurrA, CoordTrans.CurrB, &CoordTrans.CurrAlpha, &CoordTrans.CurrBeta);

	/*����ת�������ӳٲ���ϵ����ͬ*/
	if(PosSensor.EleAngularSpeed_degree >= 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_forward;
	}
	else if(PosSensor.EleAngularSpeed_degree < 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_reverse;
	}
	
	/*���㲹���Ƕ�*/
	PosSensor.CompAngle = PosSensor.CompRatio * PosSensor.EleAngularSpeed_degree * DEFAULT_CARRIER_PERIOD_s;
	
	/*����Park�任, ��Alpha-Beta����ϵת��Ϊdq����ϵ*/
	ParkTransform(CoordTrans.CurrAlpha, CoordTrans.CurrBeta, &CoordTrans.CurrD, &CoordTrans.CurrQ, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	/*������PI������*/
	CurrentLoop(CurrLoop.ExptCurrD, CurrLoop.ExptCurrQ, CoordTrans.CurrD, CoordTrans.CurrQ, &CurrLoop.CtrlVolD, &CurrLoop.CtrlVolQ);
	
	/*������Park�任, ��ת������ϵ�µ�dq���ѹת��Ϊ��������ϵ�µ�AlphaBeta����*/
	InverseParkTransform(CurrLoop.CtrlVolD, CurrLoop.CtrlVolQ, &CoordTrans.VolAlpha, &CoordTrans.VolBeta, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	/*����SVPWM�㷨���Ƶ�ѹʸ��*/
	SpaceVectorModulation(CoordTrans.VolAlpha, CoordTrans.VolBeta);
}

 /**
   * @brief  б��������
   * @param[in]  expectedVelocity      �������ٶ�
   */
float VelSlopeGenerator(float exptVel)
{
	static float velProcessVolume = 0.0f;
	static float velStep_Acc = 0;
	static float velStep_Dec = 0;
	
	velStep_Acc = SpdLoop.Acceleration * OUTER_LOOP_PERIOD;
	velStep_Dec = SpdLoop.Deceleration * OUTER_LOOP_PERIOD;
	
	if(exptVel > 0 && velProcessVolume >= 0)
	{
		if(velProcessVolume < (exptVel - velStep_Acc))
		{
			velProcessVolume += velStep_Acc;
		}
		else if(velProcessVolume > (exptVel + velStep_Dec))
		{
			velProcessVolume -= velStep_Dec;
		}
		else
		{
			velProcessVolume = exptVel;
		}
	}
	else if(exptVel > 0 && velProcessVolume < 0)
	{
		velProcessVolume += velStep_Dec;
	}
	else if(exptVel < 0 && velProcessVolume >=  0)
	{
		velProcessVolume -= velStep_Dec;
	}
	else if(exptVel < 0 && velProcessVolume < 0)
	{
		if(velProcessVolume > (exptVel + velStep_Acc))
		{
			velProcessVolume -= velStep_Acc;
		}
		else if(velProcessVolume < (exptVel - velStep_Dec))
		{
			velProcessVolume += velStep_Dec;	
		}
		else
		{
			velProcessVolume = exptVel;
		}
	}
	else if(exptVel == 0)
	{
		velProcessVolume = 0;
	}
	
	return velProcessVolume;
}

 /**
   * @brief  ����������ģʽ��ʼ��
   */
void DriverCtrlModeInit(void)
{		
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
		case POS_CURR_CTRL_MODE :	
										CurrentLoopInit();
										
										PositionLoopInit();
			
										break;
		case TORQUE_CTRL_MODE :	
										TorqueCtrlInit();	//ת�ؿ�������ʼ������ִ���ٶȻ��͵�������ʼ��֮ǰִ��
		
										CurrentLoopInit();
										
										SpeedLoopInit();		
		
										break;
	}
}

/* USER CODE END */

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

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
extern LoadObserverType loadObserver;
/* USER CODE END EV */

/* USER CODE BEGIN */

void DriverInit(void)                 
{

	#if ROBOT_ID == PASS_ROBOT
		#if CAN_ID_NUM == 1
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 27297;
			CurrLoop.LimitCurrQ = 20.f;
			SpdLoop.ExptMecAngularSpeed_rad = 10.f * 2 * PI;
			SpdLoop.MaxExptMecAngularSpeed_rad = 50.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.f;
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.f;
//			PosLoop.Kp = 80.0f;
//			PosLoop.Kd = 0.3f;
////			MainCtrl.ExptMecAngle_pulse = 1.2 * 32768;
		#elif CAN_ID_NUM == 2	
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 22706;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.3f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 50.f;
		#elif CAN_ID_NUM == 3	
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 9567;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.2f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 40.f;
		#elif CAN_ID_NUM == 4	
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 23208;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 5	
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 9550;
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
		#elif CAN_ID_NUM == 7
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 30845;
			CurrLoop.LimitCurrQ = 270.f;//270
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 2.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 6.0f;
			MainCtrl.ExptMecAngle_pulse = 0 * 4096;
			#elif CAN_ID_NUM == 8	
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 21679;
			CurrLoop.LimitCurrQ = 100.f;
			SpdLoop.ExptMecAngularSpeed_rad = -70.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 9	
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 10395;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.MaxExptMecAngularSpeed_rad = 70.f * 2 *PI; 
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.2f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 40.f;
		#elif CAN_ID_NUM == 10	
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 13724;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.MaxExptMecAngularSpeed_rad = 70.f * 2 *PI; 
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.2f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 40.0f;
		#elif CAN_ID_NUM == 11	
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 17092;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.MaxExptMecAngularSpeed_rad = 70.f * 2 *PI; 
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.2f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 40.0f;
		#endif		    
	#endif
	#if ROBOT_ID == TRY_ROBOT
		#if CAN_ID_NUM == 1
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 22629;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 2
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 18834;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;	
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 3
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 19102;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 4
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 20658;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 5
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 30831;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 6
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 32046;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 7
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 17047;
			CurrLoop.LimitCurrQ = 20.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 2.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 6.0f;
			MainCtrl.ExptMecAngle_pulse = 0 * 4096;
		#elif CAN_ID_NUM == 8
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 32146;
			CurrLoop.LimitCurrQ = 50.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 6.0f;
			MainCtrl.ExptMecAngle_pulse = 0 * 4096;
		#endif
	#endif
	loadObserver.loadGain1 = 3000.f;
	loadObserver.loadGain2 = -90.f;	
	/*读取编码器, 计算速度, 防止上电电机跳动*/
	PosSensor_Init();
	//发送初始配置
	PutStr("FIRST:\r\n");
	UART_Transmit_DMA("can id number:%d\r\ngroup number:%d\r\n",(int)CAN_ID_NUM,(int)GROUP_NUM);
	#if PHASE_SEQUENCE == POSITIVE_SEQUENCE
				PutStr("positive sequence\r\n");
	#elif PHASE_SEQUENCE == NEGATIVE_SEQUENCE
				PutStr("negative sequence\r\n");
	#endif
	
	UART_Transmit_DMA("phase resistor: %d\r\nphase inductance: %d\r\nmotor kv: %d\r\nmotor polePairs: %d\r\nmotor ampMax: %d\r\n",\
	(int)(PHASE_RES*1e6),(int)(INDUCTANCE_Q*1e6),\
		(int)MOTOR_KV,(int)MOTOR_POLE_PAIRS_NUM,\
			(int)CurrLoop.LimitCurrQ);	
	PutStr("list done\r\n");	
	PWM_IT_CMD(ENABLE,ENABLE);
}

 /**
   * @brief  
   */
void CurrentLoopInit(void)
{

	CurrLoop.Kp_D = CURRENT_CONTROL_KP_D;
	CurrLoop.Ki_D = CURRENT_CONTROL_KI_D;						
	CurrLoop.Ki_Q = CURRENT_CONTROL_KI_Q;
	

	PosSensor.CompRatio_forward = 3.2f;
	PosSensor.CompRatio_reverse = 3.2f;
	

	CurrLoop.LimitCurrQ = 20.f;
}

 /**
   * @brief  
   */
void SpeedLoopInit(void)
{
	if(Driver.ControlMode == SPD_CURR_CTRL_MODE)
	{

		SpdLoop.Kp = SPEED_CONTROL_KP;	
		SpdLoop.Ki = SPEED_CONTROL_KI;
		SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;	//rad per second
		SpdLoop.MaxExptMecAngularSpeed_rad = MAX_SPD;	
		SpdLoop.Acceleration = 3000.f * 2 * PI;	//rad per quadratic seconds
		SpdLoop.Deceleration = SpdLoop.Acceleration;
	}
	else if(Driver.ControlMode == POS_SPD_CURR_CTRL_MODE)
	{

		SpdLoop.Kp = SPEED_CONTROL_KP;
		SpdLoop.Ki = SPEED_CONTROL_KI;
		SpdLoop.MaxExptMecAngularSpeed_rad = 15.f * 2 * PI;	
		SpdLoop.Acceleration = 3000.f * 2 * PI;	//rad per quadratic seconds
		SpdLoop.Deceleration = SpdLoop.Acceleration;
	}
	if(Driver.ControlMode == TORQUE_CTRL_MODE)
	{

		SpdLoop.Kp = SPEED_CONTROL_KP;	
		SpdLoop.Ki = SPEED_CONTROL_KI;
		SpdLoop.MaxExptMecAngularSpeed_rad = TorqueCtrl.MaxMecSpd_rad;	
	}
}

 /**
   * @brief  
   */
void PositionLoopInit(void)
{					
	if(Driver.ControlMode == POS_SPD_CURR_CTRL_MODE)
	{

		PosLoop.Kp = POSITION_CONTROL_KP;
		PosLoop.Kd = POSITION_CONTROL_KD;	
		MainCtrl.ExptMecAngle_pulse = 0;
		PosLoop.MecAngleUpperLimit_rad = 1024 * 2.f * PI;
		PosLoop.MecAngleLowerLimit_rad = -1024 * 2.f * PI;
	}
	else if(Driver.ControlMode == POS_CURR_CTRL_MODE)
	{

		PosLoop.Kp = 5.0f;
		PosLoop.Kd = 0.001f;
		MainCtrl.ExptMecAngle_pulse = 0;
		PosLoop.MecAngleUpperLimit_rad = 1024 * 2.f * PI;
		PosLoop.MecAngleLowerLimit_rad = -1024 * 2.f * PI;
	}
}

 /**
   * @brief  
   */
void TorqueCtrlInit(void)
{					
	TorqueCtrl.ExptTorque_Nm = 1.0f;	
	TorqueCtrl.MaxMecSpd_rad = 60.f * 2.f * PI;	
	CurrLoop.LimitCurrQ = 200.f;	
}

 /**
   * @brief  
   */
void RefAngleInit(void)
{
	MainCtrl.PresentMecAngle_pulse = PosSensor.MecAngle_15bit;
	
	MainCtrl.LastMecAngle_pulse = PosSensor.MecAngle_15bit;
	

	MainCtrl.RefMecAngle_pulse = 0;
}

 /**
   * @brief  
   */
void ZeroPosSet(uint16_t posOffset)
{
	MainCtrl.RefMecAngle_pulse = PosSensor.MecAngle_15bit - posOffset;
}

 /**
   * @brief 
   * @param[in]  exptCurrD     		
   * @param[in]  exptCurrQ      	
   * @param[in]  realCurrD     		
   * @param[in]  realCurrQ      
   * @param[out] ctrlVolD 		
   * @param[out] ctrlVolQ 			
   */
extern float lastESpeed;
void CurrentLoop(float exptCurrD, float exptCurrQ, float realCurrD, float realCurrQ, float *ctrlVolD, float *ctrlVolQ)
{

	static int iii = 0;
	Saturation_float(&exptCurrQ, CurrLoop.LimitCurrQ, -CurrLoop.LimitCurrQ);
	
	CurrLoop.ErrD = exptCurrD - realCurrD;
	CurrLoop.ErrQ = exptCurrQ - realCurrQ;
	
	CurrLoop.IntegralErrD += CurrLoop.ErrD * DEFAULT_CARRIER_PERIOD_s;
	CurrLoop.IntegralErrQ += CurrLoop.ErrQ * DEFAULT_CARRIER_PERIOD_s;
	

	Saturation_float(&CurrLoop.IntegralErrD, CURR_INTEGRAL_ERR_LIM_D, -CURR_INTEGRAL_ERR_LIM_D);
	Saturation_float(&CurrLoop.IntegralErrQ, CURR_INTEGRAL_ERR_LIM_Q, -CURR_INTEGRAL_ERR_LIM_Q);
	

	*ctrlVolD = CurrLoop.Kp_D * CurrLoop.ErrD + CurrLoop.Ki_D * CurrLoop.IntegralErrD;
	*ctrlVolQ = CurrLoop.Ki_Q * CurrLoop.IntegralErrQ + exptCurrQ * PHASE_RES + PosSensor.EleAngularSpeed_rad * ROTATOR_FLUX_LINKAGE;
	

	CurrLoop.LimitVolD = GENERATRIX_VOL / SQRT3;
	arm_sqrt_f32(SQUARE(GENERATRIX_VOL) / 3.f - SQUARE(*ctrlVolD), &CurrLoop.LimitVolQ);
	
	Saturation_float(ctrlVolD, CurrLoop.LimitVolD, -CurrLoop.LimitVolD);
	Saturation_float(ctrlVolQ, CurrLoop.LimitVolQ, -CurrLoop.LimitVolQ);
	iii++;
	if(iii>10)
	{
//												UART_Transmit_DMA("%d\t", (int)(exptCurrQ*1e-1));
//		UART_Transmit_DMA("%d\t",(int)(exptCurrQ*1e3));
//		UART_Transmit_DMA("%d\r\n",(int)(CoordTrans.CurrQ * 1e3));
		iii = 0;
	}
	
//	UART_Transmit_DMA("%d\t%d\r\n", (int)(lastESpeed*1e2),(int)(PosSensor.EleAngularSpeed_rad * ROTATOR_FLUX_LINKAGE*1e2));
	
}

 /**
   * @brief  
   * @param[in]  exptMecAngularSpeed     
   * @param[in]  realMecAngularSpeed      	
   * @param[out] ctrlCurrQ 				
   */
void SpeedLoop(float exptMecAngularSpeed, float realMecAngularSpeed, float *ctrlCurrQ)
{

	Saturation_float(&exptMecAngularSpeed, SpdLoop.MaxExptMecAngularSpeed_rad, -SpdLoop.MaxExptMecAngularSpeed_rad);
	
	SpdLoop.Err = exptMecAngularSpeed - realMecAngularSpeed;
	
	SpdLoop.IntegralErr += SpdLoop.Err * OUTER_LOOP_PERIOD;
	

	Saturation_float(&SpdLoop.IntegralErr, SPD_INTEGRAL_ERR_LIM, -SPD_INTEGRAL_ERR_LIM);
	
	*ctrlCurrQ = SpdLoop.Kp * SpdLoop.Err + SpdLoop.Ki * SpdLoop.IntegralErr;
}

 /**
   * @brief  
   * @param[in]  exptMecAngle     	
   * @param[in]  realMecAngle      		
   * @param[out] ctrlAngularSpeed 		
   */
void PositionLoop(float exptMecAngle, float realMecAngle, float *ctrlAngularSpeed)
{

	Saturation_float(&exptMecAngle, PosLoop.MecAngleUpperLimit_rad, PosLoop.MecAngleLowerLimit_rad);
	
	PosLoop.Err = exptMecAngle - realMecAngle;
		
	PosLoop.DiffErr = (PosLoop.Err - PosLoop.LastErr) / OUTER_LOOP_PERIOD;
	
	*ctrlAngularSpeed = PosLoop.Kp * PosLoop.Err + PosLoop.Kd * PosLoop.DiffErr;
		
	PosLoop.LastErr = PosLoop.Err;
}

 /**
   * @brief  
   */
void SpdCurrController(void)
{	
	static uint16_t Count = PERIOD_MULTIPLE - 1;
	
	Count++;
	
	CurrLoop.ExptCurrD = 0.f;
	
	/* run each period of time    */
	/* speedloop -> Expt currentQ */
	if(Count == PERIOD_MULTIPLE)
	{

		GetMecImformation();
		
		SpeedLoop(VelSlopeGenerator(SpdLoop.ExptMecAngularSpeed_rad), PosSensor.MecAngularSpeed_rad, &CurrLoop.ExptCurrQ);
		
		Count = 0;
	}
	
	/* ClarkeTransform: CURRENT A & B -> CURRENT Alpha & Beta */
	ClarkeTransform(CoordTrans.CurrA, CoordTrans.CurrB, &CoordTrans.CurrAlpha, &CoordTrans.CurrBeta);

	/* get ele angel */
	if(PosSensor.EleAngularSpeed_degree >= 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_forward;
	}
	else if(PosSensor.EleAngularSpeed_degree < 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_reverse;
	}
	
	PosSensor.CompAngle = PosSensor.CompRatio * PosSensor.EleAngularSpeed_degree * DEFAULT_CARRIER_PERIOD_s;
	
	/* ParkTransform : CURRENT Alpha & Beta & ele Angle -> CURRENT D & Q  */
	ParkTransform(CoordTrans.CurrAlpha, CoordTrans.CurrBeta, &CoordTrans.CurrD, &CoordTrans.CurrQ, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	/* currentloop : Expt currentQ(from speedloop) & Expt currentD (0) AND CURRENT D & Q ---> OUTPUT CTRL VOLD & VOLQ */
	CurrentLoop(CurrLoop.ExptCurrD, CurrLoop.ExptCurrQ, CoordTrans.CurrD, CoordTrans.CurrQ, &CurrLoop.CtrlVolD, &CurrLoop.CtrlVolQ);
	
	/* Inverse park -> vol alpha & beta */
	InverseParkTransform(CurrLoop.CtrlVolD, CurrLoop.CtrlVolQ, &CoordTrans.VolAlpha, &CoordTrans.VolBeta, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	/* SVPWM Modulate */
	SpaceVectorModulation(CoordTrans.VolAlpha, CoordTrans.VolBeta);
	
}

 /**
   * @brief  
   */
void PosSpdCurrController(void)
{	
	static uint16_t Count = PERIOD_MULTIPLE - 1;	
	
	Count++;
		
	CurrLoop.ExptCurrD = 0.f;
	
	/* run each period of time  */	
	/* Positionloop -> Expt mechanical angle speed -> speedloop -> Expt CurrentQ */
	if(Count == PERIOD_MULTIPLE)
	{
	
		GetMecImformation();
		
		PosLoop.ExptMecAngle_rad = DRV_PULSE_TO_RAD(MainCtrl.ExptMecAngle_pulse);
		
		PosLoop.RefMecAngle_rad = DRV_PULSE_TO_RAD(MainCtrl.RefMecAngle_pulse);
		
		PositionLoop(PosLoop.ExptMecAngle_rad, PosLoop.RefMecAngle_rad, &SpdLoop.ExptMecAngularSpeed_rad);
				
		SpeedLoop(VelSlopeGenerator(SpdLoop.ExptMecAngularSpeed_rad), PosSensor.MecAngularSpeed_rad, &CurrLoop.ExptCurrQ);
		
		Count = 0;
	}
	
	/* ClarkeTransform: CURRENT A & B -> CURRENT Alpha & Beta */	
	ClarkeTransform(CoordTrans.CurrA, CoordTrans.CurrB, &CoordTrans.CurrAlpha, &CoordTrans.CurrBeta);

	/* get ele angel */
	if(PosSensor.EleAngularSpeed_degree >= 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_forward;
	}
	else if(PosSensor.EleAngularSpeed_degree < 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_reverse;
	}
	
	PosSensor.CompAngle = PosSensor.CompRatio * PosSensor.EleAngularSpeed_degree * DEFAULT_CARRIER_PERIOD_s;
	
	/* ParkTransform : CURRENT Alpha & Beta & ele Angle -> CURRENT D & Q  */
	ParkTransform(CoordTrans.CurrAlpha, CoordTrans.CurrBeta, &CoordTrans.CurrD, &CoordTrans.CurrQ, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	/* currentloop : Expt currentQ(from speedloop) & Expt currentD (0) AND CURRENT D & Q ---> OUTPUT CTRL VOLD & VOLQ */
	CurrentLoop(CurrLoop.ExptCurrD, CurrLoop.ExptCurrQ, CoordTrans.CurrD, CoordTrans.CurrQ, &CurrLoop.CtrlVolD, &CurrLoop.CtrlVolQ);
	
	/* Inverse park -> vol alpha & beta */
	InverseParkTransform(CurrLoop.CtrlVolD, CurrLoop.CtrlVolQ, &CoordTrans.VolAlpha, &CoordTrans.VolBeta, PosSensor.EleAngle_degree + PosSensor.CompAngle);

	/* SVPWM Modulate */	
	SpaceVectorModulation(CoordTrans.VolAlpha, CoordTrans.VolBeta);
}

 /**
   * @brief  
   */
void PosCurrController(void)
{	
	static uint16_t Count = PERIOD_MULTIPLE - 1;	
	
	Count++;
	
	CurrLoop.ExptCurrD = 0.f;
	
	/* run each period of time       */	
	/* Positionloop -> Expt CurrentQ */
	if(Count == PERIOD_MULTIPLE)
	{

		GetMecImformation();
		
		PosLoop.ExptMecAngle_rad = DRV_PULSE_TO_RAD(MainCtrl.ExptMecAngle_pulse);
		
		PosLoop.RefMecAngle_rad = DRV_PULSE_TO_RAD(MainCtrl.RefMecAngle_pulse);
		
		PositionLoop(PosLoop.ExptMecAngle_rad, PosLoop.RefMecAngle_rad, &CurrLoop.ExptCurrQ);
		
		Count = 0;
	}
	
	/* ClarkeTransform: CURRENT A & B -> CURRENT Alpha & Beta */		
	ClarkeTransform(CoordTrans.CurrA, CoordTrans.CurrB, &CoordTrans.CurrAlpha, &CoordTrans.CurrBeta);
	
	/* get ele angel */
	if(PosSensor.EleAngularSpeed_degree >= 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_forward;
	}
	else if(PosSensor.EleAngularSpeed_degree < 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_reverse;
	}
	
	PosSensor.CompAngle = PosSensor.CompRatio * PosSensor.EleAngularSpeed_degree * DEFAULT_CARRIER_PERIOD_s;
	
	/* ParkTransform : CURRENT Alpha & Beta & ele Angle -> CURRENT D & Q  */
	ParkTransform(CoordTrans.CurrAlpha, CoordTrans.CurrBeta, &CoordTrans.CurrD, &CoordTrans.CurrQ, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	/* currentloop : Expt currentQ(from speedloop) & Expt currentD (0) AND CURRENT D & Q ---> OUTPUT CTRL VOLD & VOLQ */
	CurrentLoop(CurrLoop.ExptCurrD, CurrLoop.ExptCurrQ, CoordTrans.CurrD, CoordTrans.CurrQ, &CurrLoop.CtrlVolD, &CurrLoop.CtrlVolQ);
	
	/* Inverse park -> vol alpha & beta */
	InverseParkTransform(CurrLoop.CtrlVolD, CurrLoop.CtrlVolQ, &CoordTrans.VolAlpha, &CoordTrans.VolBeta, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	/* SVPWM Modulate */	
	SpaceVectorModulation(CoordTrans.VolAlpha, CoordTrans.VolBeta);
}

 /** 
   * @brief  
   */
void TorqueController(void)
{  
	static uint16_t Count = PERIOD_MULTIPLE - 1;
	
	Count++;
	
	CurrLoop.ExptCurrD = 0.f;
		
	/* run each period of time    */	
	
	if(Count == PERIOD_MULTIPLE)
	{

		GetMecImformation();
					

		if(PosSensor.MecAngularSpeed_rad < (TorqueCtrl.MaxMecSpd_rad - 3.f * 2.f * PI))
		{		
			CurrLoop.ExptCurrQ = TorqueCtrl.ExptTorque_Nm / (1.5f * MOTOR_POLE_PAIRS_NUM * ROTATOR_FLUX_LINKAGE);
		}

		else if(PosSensor.MecAngularSpeed_rad >= (TorqueCtrl.MaxMecSpd_rad - 3.f * 2.f * PI))
		{
			SpeedLoop(TorqueCtrl.MaxMecSpd_rad, PosSensor.MecAngularSpeed_rad, &CurrLoop.ExptCurrQ);
			
			Saturation_float(&CurrLoop.ExptCurrQ, TorqueCtrl.ExptTorque_Nm / (1.5f * MOTOR_POLE_PAIRS_NUM * ROTATOR_FLUX_LINKAGE), -TorqueCtrl.ExptTorque_Nm / (1.5f * MOTOR_POLE_PAIRS_NUM * ROTATOR_FLUX_LINKAGE));
		}

		Count = 0;
	}


	ClarkeTransform(CoordTrans.CurrA, CoordTrans.CurrB, &CoordTrans.CurrAlpha, &CoordTrans.CurrBeta);


	if(PosSensor.EleAngularSpeed_degree >= 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_forward;
	}
	else if(PosSensor.EleAngularSpeed_degree < 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_reverse;
	}
	

	PosSensor.CompAngle = PosSensor.CompRatio * PosSensor.EleAngularSpeed_degree * DEFAULT_CARRIER_PERIOD_s;
	

	ParkTransform(CoordTrans.CurrAlpha, CoordTrans.CurrBeta, &CoordTrans.CurrD, &CoordTrans.CurrQ, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	

	CurrentLoop(CurrLoop.ExptCurrD, CurrLoop.ExptCurrQ, CoordTrans.CurrD, CoordTrans.CurrQ, &CurrLoop.CtrlVolD, &CurrLoop.CtrlVolQ);
	

	InverseParkTransform(CurrLoop.CtrlVolD, CurrLoop.CtrlVolQ, &CoordTrans.VolAlpha, &CoordTrans.VolBeta, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	

	SpaceVectorModulation(CoordTrans.VolAlpha, CoordTrans.VolBeta);
}

 /**
   * @brief 
   * @param[in]  expectedVelocity     
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
   * @brief  
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
										TorqueCtrlInit();	
		
										CurrentLoopInit();
										
										SpeedLoopInit();		
		
										break;
	}
}

/* USER CODE END */

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

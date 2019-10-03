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
struct VolCtrl_t VolCtrl;
struct Regulator_t Regulator;
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
	/*使能PWM输出*/
	PWM_IT_CMD(ENABLE,ENABLE);
	
	#if ROBOT_ID == PASS_ROBOT
		#if CAN_ID_NUM == 1
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 23504;
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
		#elif CAN_ID_NUM == 7
			Driver.ControlMode = POS_SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			ZeroPosSet(5540);
			PosSensor.PosOffset = 14132;
			CurrLoop.LimitCurrQ = 200.f;
			PosLoop.MaxMecAngularSpeed_rad = 60.f * 2 * PI;	//最大转速
			
			CurrLoop.Kp_D = CURRENT_CONTROL_KP_D;												
			CurrLoop.Ki_D = CURRENT_CONTROL_KI_D;						
			CurrLoop.Kp_Q = CURRENT_CONTROL_KP_Q;
			CurrLoop.Ki_Q = CURRENT_CONTROL_KI_Q;
	
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 0.0f;
		#endif
	#endif
}

 /**
   * @brief  电流环参数初始化
   */
void CurrentLoopInit(void)
{
	/*设定电流环PI参数*/
	CurrLoop.Kp_D = CURRENT_CONTROL_KP_D;												
	CurrLoop.Ki_D = CURRENT_CONTROL_KI_D * 0.001;						
	CurrLoop.Kp_Q = CURRENT_CONTROL_KP_Q;
	CurrLoop.Ki_Q = CURRENT_CONTROL_KI_Q * 0.001;
	
	/*设定编码器延迟补偿系数*/
	PosSensor.CompRatio_forward = 3.2f;
	PosSensor.CompRatio_reverse = 3.2f;
	
	/*设定最大Iq*/
	CurrLoop.LimitCurrQ = 20.f;
}

 /**
   * @brief  电压控制器参数初始化
   */
void VolCtrlInit(void)
{
	/*设定电压限幅*/
	VolCtrl.VolLimit = 8.0f;
	
	/*设定编码器延迟补偿系数*/
	PosSensor.CompRatio_forward = 3.8f;
	PosSensor.CompRatio_reverse = 3.0f;
}

 /**
   * @brief  速度环参数初始化
   */
void SpeedLoopInit(void)
{
	if(Driver.ControlMode == SPD_CURR_CTRL_MODE)
	{
		/*速度-电流双环控制*/
		SpdLoop.Kp = SPEED_CONTROL_KP;	
		SpdLoop.Ki = SPEED_CONTROL_KI;
		SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;	//期望速度，degree per second
		SpdLoop.MaxExptMecAngularSpeed_rad = MAX_SPD;	//最大转速
		SpdLoop.Acceleration = 3000.f * 2 * PI;	//期望加速度，degree per quadratic seconds
		SpdLoop.Deceleration = SpdLoop.Acceleration;
	}
	else if(Driver.ControlMode == POS_SPD_CURR_CTRL_MODE)
	{
		/*位置-速度-电流三环控制*/
		SpdLoop.Kp = SPEED_CONTROL_KP;
		SpdLoop.Ki = SPEED_CONTROL_KI;
		SpdLoop.MaxExptMecAngularSpeed_rad = 15.f * 2 * PI;	//最大转速
		SpdLoop.Acceleration = 3000.f * 2 * PI;	//期望加速度，degree per quadratic seconds
		SpdLoop.Deceleration = SpdLoop.Acceleration;
	}
	else if(Driver.ControlMode == SPD_VOL_CTRL_MODE)
	{
		/*速度环单环控制*/
		SpdLoop.Kp = (ROTATOR_FLUX_LINKAGE * MOTOR_POLE_PAIRS_NUM * 5.5f) * 1.0f;	
		SpdLoop.Ki = (ROTATOR_FLUX_LINKAGE * MOTOR_POLE_PAIRS_NUM * 25.0f) * 1.0f;
		SpdLoop.MaxExptMecAngularSpeed_rad = MAX_SPD;	//最大期望转速
		SpdLoop.ExptMecAngularSpeed_rad = 10.f * 2 * PI;	//期望速度，rad per second
		SpdLoop.Acceleration = 3000.f * 2 * PI;	//期望加速度，rad per quadratic seconds
		SpdLoop.Deceleration = SpdLoop.Acceleration;
	}
	else if(Driver.ControlMode == POS_SPD_VOL_CTRL_MODE)
	{
		/*位置-速度双环控制*/
		SpdLoop.Kp = (ROTATOR_FLUX_LINKAGE * MOTOR_POLE_PAIRS_NUM * 5.5f) * 1.0f;	
		SpdLoop.Ki = (ROTATOR_FLUX_LINKAGE * MOTOR_POLE_PAIRS_NUM * 25.0f) * 0.0f;
		SpdLoop.MaxExptMecAngularSpeed_rad = 15.f * 2 * PI;	//最大转速
		SpdLoop.Acceleration = 3000.f * 2 * PI;	//期望加速度，degree per quadratic seconds
		SpdLoop.Deceleration = SpdLoop.Acceleration;
	}
}

 /**
   * @brief  位置环参数初始化
   */
void PositionLoopInit(void)
{					
	if(Driver.ControlMode == POS_SPD_CURR_CTRL_MODE)
	{
		/*位置-速度-电流三环控制*/
		PosLoop.Kp = POSITION_CONTROL_KP;
		PosLoop.Kd = POSITION_CONTROL_KD;	
		MainCtrl.ExptMecAngle_pulse = 0;
		PosLoop.MecAngleUpperLimit_rad = 1024 * 2.f * PI;
		PosLoop.MecAngleLowerLimit_rad = -1024 * 2.f * PI;
	}
	else if(Driver.ControlMode == POS_CURR_CTRL_MODE)
	{
		/*位置-电流双环控制*/
		PosLoop.Kp = 5.0f;
		PosLoop.Kd = 0.001f;
		MainCtrl.ExptMecAngle_pulse = 0;
		PosLoop.MecAngleUpperLimit_rad = 1024 * 2.f * PI;
		PosLoop.MecAngleLowerLimit_rad = -1024 * 2.f * PI;
	}
	else if(Driver.ControlMode == POS_SPD_VOL_CTRL_MODE)
	{
		/*位置-速度双环控制*/
		PosLoop.Kp = 80.5f * 1.0f;
		PosLoop.Kd = 8.5f * 0.1f;	
		MainCtrl.ExptMecAngle_pulse = 0;
		PosLoop.MecAngleUpperLimit_rad = 1024 * 2.f * PI;
		PosLoop.MecAngleLowerLimit_rad = -1024 * 2.f * PI;
	}	
}

 /**
   * @brief  转矩控制器初始化
   */
void TorqueCtrlInit(void)
{					
	TorqueCtrl.ExptTorque = 1.0f;
	TorqueCtrl.MaxMecSpd = 100.f * 2.f * PI;
	MainCtrl.MaxTorque_Nm = 2.0f;
}

 /**
   * @brief  位置控制模式下标定初始位置
   */
void RefAngleInit(void)
{
	MainCtrl.PresentMecAngle_pulse = PosSensor.MecAngle_AbsoluteMode_15bit;
	
	MainCtrl.LastMecAngle_pulse = PosSensor.MecAngle_AbsoluteMode_15bit;
	
	/*记当前位置为零位*/
	MainCtrl.RefMecAngle_pulse = 0;
}

 /**
   * @brief  位置控制模式下标定初始位置
   */
void ZeroPosSet(uint16_t posOffset)
{
	MainCtrl.RefMecAngle_pulse = PosSensor.MecAngle_AbsoluteMode_15bit - posOffset;
}

 /**
   * @brief  电流环
   * @param[in]  exptCurrD     		期望Id
   * @param[in]  exptCurrQ      	期望Iq
   * @param[in]  realCurrD     		实际Id
   * @param[in]  realCurrQ      	实际Iq
   * @param[out] ctrlVolD 			Vd输出
   * @param[out] ctrlVolQ 			Vq输出
   */
void CurrentLoop(float exptCurrD, float exptCurrQ, float realCurrD, float realCurrQ, float *ctrlVolD, float *ctrlVolQ)
{
	/*Iq输入限幅*/
	Saturation_float(&exptCurrQ, CurrLoop.LimitCurrQ, -CurrLoop.LimitCurrQ);
	
	CurrLoop.ErrD = exptCurrD - realCurrD;
	CurrLoop.ErrQ = exptCurrQ - realCurrQ;
	
	CurrLoop.IntegralErrD += CurrLoop.ErrD * Regulator.ActualPeriod_s;
	CurrLoop.IntegralErrQ += CurrLoop.ErrQ * Regulator.ActualPeriod_s;
	
	/*积分限幅*/
	Saturation_float(&CurrLoop.IntegralErrD, CURR_INTEGRAL_ERR_LIM_D, -CURR_INTEGRAL_ERR_LIM_D);
	Saturation_float(&CurrLoop.IntegralErrQ, CURR_INTEGRAL_ERR_LIM_Q, -CURR_INTEGRAL_ERR_LIM_Q);
	
	/*基于电机模型的改进型电流控制器, 采用PI调节+电机模型补偿*/
	*ctrlVolD = CurrLoop.Kp_D * CurrLoop.ErrD + CurrLoop.Ki_D * CurrLoop.IntegralErrD - PosSensor.EleAngularSpeed_rad * INDUCTANCE_Q * CoordTrans.CurrQ;
	*ctrlVolQ = CurrLoop.Kp_Q * CurrLoop.ErrQ + CurrLoop.Ki_Q * CurrLoop.IntegralErrQ + PosSensor.EleAngularSpeed_rad * ROTATOR_FLUX_LINKAGE;
	
	/*电流环d轴电流限幅*/
	Saturation_float(ctrlVolD, GENERATRIX_VOL / SQRT3, -GENERATRIX_VOL / SQRT3);
	Saturation_float(ctrlVolQ, sqrt(SQUARE(GENERATRIX_VOL) / 3.f - SQUARE(*ctrlVolD)), -sqrt(SQUARE(GENERATRIX_VOL) / 3.f - SQUARE(*ctrlVolD)));
}

 /**
   * @brief  速度环
   * @param[in]  exptMecAngularSpeed     	期望机械角速度
   * @param[in]  realMecAngularSpeed      	实际机械角速度
   * @param[out] ctrlCurrQ 					Iq输出
   */
void SpeedLoop(float exptMecAngularSpeed, float realMecAngularSpeed, float *ctrlCurrQ)
{
	/*最大目标速度限幅*/
	Saturation_float(&exptMecAngularSpeed, SpdLoop.MaxExptMecAngularSpeed_rad, -SpdLoop.MaxExptMecAngularSpeed_rad);
	
	SpdLoop.Err = exptMecAngularSpeed - realMecAngularSpeed;
	
	SpdLoop.IntegralErr += SpdLoop.Err * OUTER_LOOP_PERIOD;
	
	/*积分限幅*/
	Saturation_float(&SpdLoop.IntegralErr, SPD_INTEGRAL_ERR_LIM, -SPD_INTEGRAL_ERR_LIM);
	
	*ctrlCurrQ = SpdLoop.Kp * SpdLoop.Err + SpdLoop.Ki * SpdLoop.IntegralErr;
}

 /**
   * @brief  位置环
   * @param[in]  exptMecAngle     		期望机械角度
   * @param[in]  realMecAngle      		实际机械角度
   * @param[out] ctrlAngularSpeed 		角速度输出
   */
void PositionLoop(float exptMecAngle, float realMecAngle, float *ctrlAngularSpeed)
{
	/*位置限幅*/
	Saturation_float(&exptMecAngle, PosLoop.MecAngleUpperLimit_rad, PosLoop.MecAngleLowerLimit_rad);
	
	PosLoop.Err = exptMecAngle - realMecAngle;
		
	PosLoop.DiffErr = (PosLoop.Err - PosLoop.LastErr) / OUTER_LOOP_PERIOD;
	
	*ctrlAngularSpeed = PosLoop.Kp * PosLoop.Err + PosLoop.Kd * PosLoop.DiffErr;
		
	PosLoop.LastErr = PosLoop.Err;
}

 /**
   * @brief  速度-电流控制器
   */
void SpdCurrController(void)
{	
	static uint16_t Count = PERIOD_MULTIPLE - 1;	//使程序第一次执行中断时, 运行速度环, 位置环
	
	Count++;
	
	/*采用Id = 0控制, 故设定d轴电流为零*/
	CurrLoop.ExptCurrD = 0.f;
	
	/*位置环与速度环的周期是电流环周期的十倍*/
	if(Count == PERIOD_MULTIPLE)
	{
		/*更新机械速度及位置信息*/
		GetMecImformation();
		
		SpeedLoop(VelSlopeGenerator(SpdLoop.ExptMecAngularSpeed_rad), PosSensor.MecAngularSpeed_rad, &CurrLoop.ExptCurrQ);
		
		Count = 0;
	}
	
	/*进行Clark变换, 将abc坐标系转换为Alpha-Beta坐标系*/
	ClarkTransform_arm(CoordTrans.CurrA, CoordTrans.CurrB, &CoordTrans.CurrAlpha, &CoordTrans.CurrBeta);

	/*正反转编码器延迟补偿系数不同*/
	if(PosSensor.EleAngularSpeed_degree >= 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_forward;
	}
	else if(PosSensor.EleAngularSpeed_degree < 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_reverse;
	}
	
	PosSensor.CompAngle = PosSensor.CompRatio * PosSensor.EleAngularSpeed_degree * Regulator.ActualPeriod_s;
	
	/*进行Park变换, 将Alpha-Beta坐标系转换为dq坐标系*/
	ParkTransform_arm(CoordTrans.CurrAlpha, CoordTrans.CurrBeta, &CoordTrans.CurrD, &CoordTrans.CurrQ, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	/*电流环PI控制器*/
	CurrentLoop(CurrLoop.ExptCurrD, CurrLoop.ExptCurrQ, CoordTrans.CurrD, CoordTrans.CurrQ, &CurrLoop.CtrlVolD, &CurrLoop.CtrlVolQ);
	
	/*进行逆Park变换, 将转子坐标系下的dq轴电压转换为定子坐标系下的AlphaBeta轴电压*/
	InverseParkTransform(CurrLoop.CtrlVolD, CurrLoop.CtrlVolQ, &CoordTrans.VolAlpha, &CoordTrans.VolBeta, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	/*利用SVPWM算法调制电压矢量*/
	SpaceVectorModulation(CoordTrans.VolAlpha, CoordTrans.VolBeta);
}

 /**
   * @brief  位置-速度-电流控制器
   */
void PosSpdCurrController(void)
{	
	static uint16_t Count = PERIOD_MULTIPLE - 1;	//使程序第一次执行中断时, 运行速度环, 位置环
	
	Count++;
	
	/*采用Id = 0控制, 故设定d轴电流为零*/
	CurrLoop.ExptCurrD = 0.f;
	
	/*位置环与速度环的周期是电流环周期的十倍*/
	if(Count == PERIOD_MULTIPLE)
	{
		/*更新机械速度及位置信息*/
		GetMecImformation();
		
		PosLoop.ExptMecAngle_rad = DRV_PULSE_TO_RAD(MainCtrl.ExptMecAngle_pulse);
		
		PosLoop.RefMecAngle_rad = DRV_PULSE_TO_RAD(MainCtrl.RefMecAngle_pulse);
		
		PositionLoop(PosLoop.ExptMecAngle_rad, PosLoop.RefMecAngle_rad, &SpdLoop.ExptMecAngularSpeed_rad);
				
		SpeedLoop(VelSlopeGenerator(SpdLoop.ExptMecAngularSpeed_rad), PosSensor.MecAngularSpeed_rad, &CurrLoop.ExptCurrQ);
		
		Count = 0;
	}
	
	/*进行Clark变换, 将abc坐标系转换为Alpha-Beta坐标系*/
	ClarkTransform_arm(CoordTrans.CurrA, CoordTrans.CurrB, &CoordTrans.CurrAlpha, &CoordTrans.CurrBeta);

	/*正反转编码器延迟补偿系数不同*/
	if(PosSensor.EleAngularSpeed_degree >= 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_forward;
	}
	else if(PosSensor.EleAngularSpeed_degree < 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_reverse;
	}
	
	PosSensor.CompAngle = PosSensor.CompRatio * PosSensor.EleAngularSpeed_degree * Regulator.ActualPeriod_s;
	
	/*进行Park变换, 将Alpha-Beta坐标系转换为dq坐标系*/
	ParkTransform_arm(CoordTrans.CurrAlpha, CoordTrans.CurrBeta, &CoordTrans.CurrD, &CoordTrans.CurrQ, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	/*电流环PI控制器*/
	CurrentLoop(CurrLoop.ExptCurrD, CurrLoop.ExptCurrQ, CoordTrans.CurrD, CoordTrans.CurrQ, &CurrLoop.CtrlVolD, &CurrLoop.CtrlVolQ);
	
	/*进行逆Park变换, 将转子坐标系下的dq轴电压转换为定子坐标系下的AlphaBeta轴电压*/
	InverseParkTransform(CurrLoop.CtrlVolD, CurrLoop.CtrlVolQ, &CoordTrans.VolAlpha, &CoordTrans.VolBeta, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	/*利用SVPWM算法调制电压矢量*/
	SpaceVectorModulation(CoordTrans.VolAlpha, CoordTrans.VolBeta);
}

 /**
   * @brief  位置-电流控制器
   */
void PosCurrController(void)
{	
	static uint16_t Count = PERIOD_MULTIPLE - 1;	//使程序第一次执行中断时, 运行速度环, 位置环
	
	Count++;
	
	/*采用Id = 0控制, 故设定d轴电流为零*/
	CurrLoop.ExptCurrD = 0.f;
	
	/*位置环与速度环的周期是电流环周期的十倍*/
	if(Count == PERIOD_MULTIPLE)
	{
		/*更新机械速度及位置信息*/
		GetMecImformation();
		
		PosLoop.ExptMecAngle_rad = DRV_PULSE_TO_RAD(MainCtrl.ExptMecAngle_pulse);
		
		PosLoop.RefMecAngle_rad = DRV_PULSE_TO_RAD(MainCtrl.RefMecAngle_pulse);
		
		PositionLoop(PosLoop.ExptMecAngle_rad, PosLoop.RefMecAngle_rad, &CurrLoop.ExptCurrQ);
		
		Count = 0;
	}
	
	/*进行Clark变换, 将abc坐标系转换为Alpha-Beta坐标系*/
	ClarkTransform_arm(CoordTrans.CurrA, CoordTrans.CurrB, &CoordTrans.CurrAlpha, &CoordTrans.CurrBeta);

	/*正反转编码器延迟补偿系数不同*/
	if(PosSensor.EleAngularSpeed_degree >= 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_forward;
	}
	else if(PosSensor.EleAngularSpeed_degree < 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_reverse;
	}
	
	/*计算补偿角度*/
	PosSensor.CompAngle = PosSensor.CompRatio * PosSensor.EleAngularSpeed_degree * Regulator.ActualPeriod_s;
	
	/*进行Park变换, 将Alpha-Beta坐标系转换为dq坐标系*/
	ParkTransform_arm(CoordTrans.CurrAlpha, CoordTrans.CurrBeta, &CoordTrans.CurrD, &CoordTrans.CurrQ, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	/*电流环PI控制器*/
	CurrentLoop(CurrLoop.ExptCurrD, CurrLoop.ExptCurrQ, CoordTrans.CurrD, CoordTrans.CurrQ, &CurrLoop.CtrlVolD, &CurrLoop.CtrlVolQ);
	
	/*进行逆Park变换, 将转子坐标系下的dq轴电压转换为定子坐标系下的AlphaBeta轴电压*/
	InverseParkTransform(CurrLoop.CtrlVolD, CurrLoop.CtrlVolQ, &CoordTrans.VolAlpha, &CoordTrans.VolBeta, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	/*利用SVPWM算法调制电压矢量*/
	SpaceVectorModulation(CoordTrans.VolAlpha, CoordTrans.VolBeta);
}

 /**
   * @brief  转矩控制器
   */
void TorqueController(void)
{  
	/*将目标转矩转换为目标Iq*/
	CurrLoop.ExptCurrQ = TorqueCtrl.ExptTorque / (1.5f * MOTOR_POLE_PAIRS_NUM * ROTATOR_FLUX_LINKAGE);
	
	/*采用Id = 0控制, 故设定d轴电流为零*/
	CurrLoop.ExptCurrD = 0.f;
	
	/*进行Clark变换, 将abc坐标系转换为Alpha-Beta坐标系*/
	ClarkTransform_arm(CoordTrans.CurrA, CoordTrans.CurrB, &CoordTrans.CurrAlpha, &CoordTrans.CurrBeta);
	
	/*正反转编码器延迟补偿系数不同*/
	if(PosSensor.EleAngularSpeed_degree >= 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_forward;
	}
	else if(PosSensor.EleAngularSpeed_degree < 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_reverse;
	}
	
	/*计算补偿角度*/
	PosSensor.CompAngle = PosSensor.CompRatio * PosSensor.EleAngularSpeed_degree * Regulator.ActualPeriod_s;
	
	/*进行Park变换, 将Alpha-Beta坐标系转换为dq坐标系*/
	ParkTransform_arm(CoordTrans.CurrAlpha, CoordTrans.CurrBeta, &CoordTrans.CurrD, &CoordTrans.CurrQ, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	/*电流环PI控制器*/
	CurrentLoop(CurrLoop.ExptCurrD, CurrLoop.ExptCurrQ, CoordTrans.CurrD, CoordTrans.CurrQ, &CurrLoop.CtrlVolD, &CurrLoop.CtrlVolQ);
	
	/*进行逆Park变换, 将转子坐标系下的dq轴电压转换为定子坐标系下的AlphaBeta轴电压*/
	InverseParkTransform(CurrLoop.CtrlVolD, CurrLoop.CtrlVolQ, &CoordTrans.VolAlpha, &CoordTrans.VolBeta, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	/*利用SVPWM算法调制电压矢量*/
	SpaceVectorModulation(CoordTrans.VolAlpha, CoordTrans.VolBeta);
}

 /**
   * @brief  速度-电压控制器
   */
void SpdVolController(void)
{  
	/*采用Id = 0控制, 设Vd = 0时, Id近似为零*/
	VolCtrl.CtrlVolD = 0.f;
	
	/*更新机械速度及位置信息*/
	GetMecImformation();
	
	SpeedLoop(VelSlopeGenerator(SpdLoop.ExptMecAngularSpeed_rad), PosSensor.MecAngularSpeed_rad, &VolCtrl.CtrlVolQ);
	
	/*计算q轴反电动势*/
	VolCtrl.BEMF = ROTATOR_FLUX_LINKAGE * PosSensor.EleAngularSpeed_rad;
	
	/*Vq限幅*/
	Saturation_float(&VolCtrl.CtrlVolQ, VolCtrl.BEMF + VolCtrl.VolLimit, VolCtrl.BEMF - VolCtrl.VolLimit);
	
	/*正反转编码器延迟补偿系数不同*/
	if(PosSensor.EleAngularSpeed_degree >= 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_forward;
	}
	else if(PosSensor.EleAngularSpeed_degree < 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_reverse;
	}
	
	/*计算补偿角度*/
	PosSensor.CompAngle = PosSensor.CompRatio * PosSensor.EleAngularSpeed_degree * Regulator.ActualPeriod_s;
	
	/*进行逆Park变换, 将转子坐标系下的dq轴电压转换为定子坐标系下的AlphaBeta轴电压*/
	InverseParkTransform(VolCtrl.CtrlVolD, VolCtrl.CtrlVolQ, &CoordTrans.VolAlpha, &CoordTrans.VolBeta, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	/*利用SVPWM算法调制电压矢量*/
	SpaceVectorModulation(CoordTrans.VolAlpha, CoordTrans.VolBeta);
}

 /**
   * @brief  位置-速度-电压控制器
   */
void PosSpdVolController(void)
{  
	/*采用Id = 0控制, 设Vd = 0时, Id近似为零*/
	VolCtrl.CtrlVolD = 0.f;
	
	/*更新机械速度及位置信息*/
	GetMecImformation();
	
	PosLoop.ExptMecAngle_rad = DRV_PULSE_TO_RAD(MainCtrl.ExptMecAngle_pulse);
		
	PosLoop.RefMecAngle_rad = DRV_PULSE_TO_RAD(MainCtrl.RefMecAngle_pulse);
		
	PositionLoop(PosLoop.ExptMecAngle_rad, PosLoop.RefMecAngle_rad, &CurrLoop.ExptCurrQ);
		
	SpeedLoop(VelSlopeGenerator(SpdLoop.ExptMecAngularSpeed_rad), PosSensor.MecAngularSpeed_rad, &VolCtrl.CtrlVolQ);
	
	/*计算q轴反电动势*/
	VolCtrl.BEMF = ROTATOR_FLUX_LINKAGE * PosSensor.EleAngularSpeed_rad;
	
	/*Vq限幅*/
	Saturation_float(&VolCtrl.CtrlVolQ, VolCtrl.BEMF + VolCtrl.VolLimit, VolCtrl.BEMF - VolCtrl.VolLimit);
	
	/*正反转编码器延迟补偿系数不同*/
	if(PosSensor.EleAngularSpeed_degree >= 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_forward;
	}
	else if(PosSensor.EleAngularSpeed_degree < 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_reverse;
	}
	
	/*计算补偿角度*/
	PosSensor.CompAngle = PosSensor.CompRatio * PosSensor.EleAngularSpeed_degree * Regulator.ActualPeriod_s;
	
	/*进行逆Park变换, 将转子坐标系下的dq轴电压转换为定子坐标系下的AlphaBeta轴电压*/
	InverseParkTransform(VolCtrl.CtrlVolD, VolCtrl.CtrlVolQ, &CoordTrans.VolAlpha, &CoordTrans.VolBeta, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	/*利用SVPWM算法调制电压矢量*/
	SpaceVectorModulation(CoordTrans.VolAlpha, CoordTrans.VolBeta);
}

 /**
   * @brief  斜坡生成器
   * @param[in]  expectedVelocity      期望角速度
   */
float VelSlopeGenerator(float exptVelocity)
{
	static float velocityProcessVolume = 0.0f;
	static float velocityStep_Acc = 0;
	static float velocityStep_Dec = 0;
	
	velocityStep_Acc = SpdLoop.Acceleration * OUTER_LOOP_PERIOD;
	velocityStep_Dec = SpdLoop.Deceleration * OUTER_LOOP_PERIOD;
	
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
   * @brief  驱动器控制模式初始化
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
										CurrentLoopInit();
		
										TorqueCtrlInit();
		
										break;
		
		case SPD_VOL_CTRL_MODE :		
										VolCtrlInit();
		
										SpeedLoopInit();
		
										break;
		
		case POS_SPD_VOL_CTRL_MODE :	
										VolCtrlInit();
		
										SpeedLoopInit();
		
										PositionLoopInit();

										break;
	}
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
	else if(PWMPeriod_PID < DEFAULT_CARRIER_PERIOD_us - PERIOD_REGULATOR_LIM)
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

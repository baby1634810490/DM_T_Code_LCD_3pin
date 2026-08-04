//
// Created by Lenovo on 2025/4/18.
//

/* Includes ------------------------------------------------------------------*/
#include "driver_motor.h"
#include "arm_math.h"

/* ----------------------- Internal Data ----------------------------------- */
RS_MotorInitStruct RS_Motor04Init = {
	.MotorTypeInit = MotorRS04,
	.MotorModeInit = RS_Motor_CUR,
	.CurModeInit = RS_Motor_CUR_Location,
	.ZeroState = 1,
	.ZeroPosition = 0,
	.CAN_ID = 0x30,
	.MasterID = 0x30,
	.PIDSpeed_KP = RS_Motor04_SPEED_KP,
	.PIDSpeed_AP = RS_Motor04_SPEED_AP,
	.PIDSpeed_BP = RS_Motor04_SPEED_BP,
	.PIDSpeed_CP = RS_Motor04_SPEED_CP,
	.PIDSpeed_KI = RS_Motor04_SPEED_KI,
	.PIDSpeed_KD = RS_Motor04_SPEED_KD,
	.PIDSpeed_OutMax = RS_Motor04_SPEED_OUTMAX,
	.PIDSpeed_UiOutMax = RS_Motor04_SPEED_UIOUTMAX,
	.Speed_MeanFilter_K_Init = 0.25f,
	.PIDLocation_KP = RS_Motor04_LOCATION_KP,
	.PIDLocation_OutMax = RS_Motor04_LOCATION_OUTMAX,
	.MotorThetaDiffFormula = Four_point_differential,
	.SpeedFF_K = RS_Motor04_SPEED_FF_K,
	.SpeedFF_MAX = RS_Motor04_SPEED_FF_MAX
};
RS_MotorInitStruct RS_Motor00Init = {
	.MotorTypeInit = MotorRS00,
	.MotorModeInit = RS_Motor_CUR,
	.CurModeInit = RS_Motor_CUR_Location,
	.ZeroState = 1,
	.ZeroPosition = 0,
	.CAN_ID = 0x36,
	.MasterID = 0x36,
	.PIDSpeed_KP = RS_Motor00_SPEED_KP,
	.PIDSpeed_AP = RS_Motor00_SPEED_AP,
	.PIDSpeed_BP = RS_Motor00_SPEED_BP,
	.PIDSpeed_CP = RS_Motor00_SPEED_CP,
	.PIDSpeed_KI = RS_Motor00_SPEED_KI,
	.PIDSpeed_KD = RS_Motor00_SPEED_KD,
	.PIDSpeed_OutMax = RS_Motor00_SPEED_OUTMAX,
	.PIDSpeed_UiOutMax = RS_Motor00_SPEED_UIOUTMAX,
	.Speed_MeanFilter_K_Init = 0.25f,
	.PIDLocation_KP = RS_Motor00_LOCATION_KP,
	.PIDLocation_OutMax = RS_Motor00_LOCATION_OUTMAX,
	.MotorThetaDiffFormula = Four_point_differential,
	.SpeedFF_K = RS_Motor00_SPEED_FF_K,
	.SpeedFF_MAX = RS_Motor00_SPEED_FF_MAX
};

RS_MotorStruct FirJointMotor;

/* ------------------------ Function Implements ----------------------------- */
/**
  * @brief  Initialize the motor data
  * @retval None
  */
void MotorInit(void)
{
	RS_MotorInit(&FirJointMotor, RS_Motor00Init);
}

/**
  * @brief  Update motor speed and location data
  * @param  safe_Flag : If the remote control signal is received? Yes, flag is ENABLE and no is DISABLE
  * @retval None
  */
void MotorDataUpdate(uint8_t safeFlag)
{
	RS_MotorDataUpdate(&FirJointMotor, safeFlag);
}

/**
  * @brief  Update motor speed and location setting
  * @retval None
  */
float testTOP;

void MotorSettingUpdate(void)
{
	float theta;
	float testM = 0.0f;
	float theta1;

	/* 摆臂的转动惯量是4167.833 kg*mm^2，质量是0.246kg，质心到旋转轴的距离是197.88mm
	 * 输出轴的转动惯量是20.262 kg*mm^2，质量是0.023kg，质心到旋转轴的距离是0.0mm
	 * 整体的转动惯量是0.00013821 kg*m^2，
	 */

	//*  灵足RS03电流控制
	FirJointMotor.Location.SetLocation -= 0.0f * 0.0000425f;
	theta1 = FirJointMotor.RawFeedback.Position - 4.73396f + 3.0f / 2.0f * PI; //减去摆臂垂下的时候位置值

	testTOP = 0.56f * 9.8f * 0.06497f * cosf(theta1)
		+ 2.93f * 9.8f * 0.2f * cosf(theta1);
	FirJointMotor.SetTorque = testTOP;
	if (FirJointMotor.Location.SetLocation > FIR_JOINT_LOCATION_MAX)
		FirJointMotor.Location.SetLocation = FIR_JOINT_LOCATION_MAX;
	if (FirJointMotor.Location.SetLocation < FIR_JOINT_LOCATION_MIN)
		FirJointMotor.Location.SetLocation = FIR_JOINT_LOCATION_MIN;

	/*  灵足RS02电流控制
		FirJointMotor.Location.SetLocation -= RemoteData.RY * 0.000125f;
		theta1 = FirJointMotor.RawFeedback.Position - 3.12524f + 3.0f / 2.0f * PI;//减去摆臂垂下的时候位置值

		testTOP= 0.56f * 9.8f * 0.06497f * cosf(theta1)
			+ 2.93f * 9.8f * 0.2f * cosf(theta1);
		FirJointMotor.SetTorque = testTOP;
		if (FirJointMotor.Location.SetLocation > FIR_JOINT_LOCATION_MAX)
			FirJointMotor.Location.SetLocation = FIR_JOINT_LOCATION_MAX;
		if (FirJointMotor.Location.SetLocation < FIR_JOINT_LOCATION_MIN)
			FirJointMotor.Location.SetLocation = FIR_JOINT_LOCATION_MIN;
		*/
	/*  灵足RS04电流控制
		FirJointMotor.Location.SetLocation -= RemoteData.RY * 0.000025f;
		theta1 = FirJointMotor.RawFeedback.Position - 3.602460f + 3.0f / 2.0f * PI;//减去摆臂垂下的时候位置值
		FirJointMotor.SetTorque = 1.332f * 9.8f * 0.311174f * cosf(theta1)
			+ 5.6f * 9.8f * 0.7f * cosf(theta1);
		if (FirJointMotor.Location.SetLocation > FIR_JOINT_LOCATION_MAX)
			FirJointMotor.Location.SetLocation = FIR_JOINT_LOCATION_MAX;
		if (FirJointMotor.Location.SetLocation < FIR_JOINT_LOCATION_MIN)
			FirJointMotor.Location.SetLocation = FIR_JOINT_LOCATION_MIN;
		*/
}

/**
  * @brief  The pid calculation of the motor
  * @param  safe_Flag : If the remote control signal is received? Yes, flag is ENABLE and no is DISABLE
  * @retval None
  */
void MotorCalculate(uint8_t safeFlag)
{
	RS_MotorPIDCalculate(&FirJointMotor, 1, safeFlag);
}

/**
  * @brief  Sending motor CAN data
  * @retval None
  */
void MotorCANSend(void)
{
	uint32_t extId;

	/* 第一关节电机发送数据解算 */
	RS_MotorSendDataUpdate(&FirJointMotor);
	extId = *((uint32_t*)&(FirJointMotor.SetValue.ExtId));
	if (FirJointMotor.SetValue.ExtId.id != 0 && FirJointMotor.SetValue.ExtId.data != 0)
		FDCAN1_Send_Msg(FirJointMotor.SendMessage, 8, extId, FDCAN_EXTENDED_ID);
}

/************************ (C) COPYRIGHT XJTU ROBOMASTER ********END OF FILE****/

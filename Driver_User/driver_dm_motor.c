//
// Created by Lenovo on 2023/12/28.
//

/**
  ******************************************************************************
  * @file    driver_dm_motor.c
  * @author  Shuai Yang
  * @brief   motor driver.
  *          This file provides functions to process dm motor data and update data:
  *           + Initialization function
  *           + Receive data function
  *           + Data update functions
  *           + Send data update functions
  *           + PID calculate functions
  *
  @verbatim
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================
    [..]
      (#) Initialize the motor by implementing the DM_MotorInit():
          (++) Initialize the motor parameters, including:
		       (+++) pid parameters

      (#) Get motor data by CAN by implementing the DM_MotorDataReceive():
          (++) Initialize the motor parameters, including:
		       (+++) ReceiveMessege

      (#) Update motor data by implementing the DM_MotorDataUpdate():
          (++) Update motor parameters, including:
      	       (+++) Position
      	       (+++) Velocity
      	       (+++) Torque
          (++) Update motor state

      (#) Update motor send data by implementing the DM_MotorSendDataUpdate():
          (++) Update motor SetValue, including:
               (+++) SetTorque
               (+++) SetVelocity
               (+++) SetPosition
          (++) Update motor SendMessage

      (#) PID calculate by implementing the DM_MotorPIDCalculate():
          (++) PID calculate, including:
               (+++) PID Speed Calculate
               (+++) PID Location Calculate
               (+++) PID output clear
          (++) Speed feedback calculate

  @endverbatim
  ******************************************************************************
  * @attention
  *
  * 这个文件已经完全封装好，不需要修改，所有宏定义都来自电机手册
  *
  * Don't forget the author
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "driver_dm_motor.h"

/* ------------------------ Internal Data ----------------------------------- */
/** @brief 上电自检完毕后需发送“使能”命令才可以进行控制。“使能”帧属于控制
  *        帧，帧 ID 等于设定的 CAN ID 值，不同的是数据段，无论处于哪种
  *        模式，“使能”的数据定义是相同的，如下
  */
const uint8_t EntryMotor[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};

/** @brief “失能”帧属于控制帧，帧 ID 等于设定的 CAN ID 值，数据段定义如下：
  */
const uint8_t ExitMotor[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

/** @brief “保存位置零点”帧属于控制帧，帧 ID 等于设定的 CAN ID 值，数据段
  *        定义如下：
  */
const uint8_t SavePositionZero[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};

/** @brief 电机出现过热等错误时，发送“清除”命令可以清除错误。“清除”帧属于控
  *        制帧，帧 ID 等于设定的 CAN ID 值，数据段定义如下：
  */
const uint8_t ClearErrors[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};

/* ------------------------ Function Implements ----------------------------- */
/**
  * @brief  Initialize the dm motor parameters
  * @param  motor pointer to a DM_MotorStruct structure that contains
  *         the PID information for the specified Motor.
  * @param  motorInitData Structure for motor initialization
  * @retval None
  */
void DM_MotorInit(DM_MotorStruct* motor, DM_MotorInitStruct motorInitData)
{
    /* 将CAN接收标志位设置为 DISABLE，意为未接收到数据 */
    motor->ReceiveFlag = DISABLE;
    /* 将电机状态设置为 Motor4310_Disable，意为电机此时为失能 */
    motor->MotorState = Motor4310_Disable;
    /* 设置电机的类型 */
    motor->MotorType = motorInitData.MotorTypeInit;
    /* 设置电机的模式 */
    motor->MotorMode = motorInitData.MotorModeInit;
    /* 若为 MIT 模式，则进一步设置其 MIT 状态下的模式 */
    if (motor->MotorMode == Motor4310_MIT)
        motor->MitMode = motorInitData.MitModeInit;
    /* 设置电机 MasterID */
    motor->MasterID = motorInitData.MasterID;
    motor->EnableCount = 0;
    motor->EncoderCount = 0;
    /* 设定电机CAN_ID */
    switch (motor->MotorMode)
    {
    case Motor4310_MIT:
        motor->CAN_ID = motorInitData.CAN_ID;
        motor->SendDataLen = 8;
        break;
    case Motor4310_POS:
        motor->CAN_ID = motorInitData.CAN_ID + 0x100;
        motor->SendDataLen = 8;
        break;
    case Motor4310_VEL:
        motor->CAN_ID = motorInitData.CAN_ID + 0x200;
        motor->SendDataLen = 4;
        break;
    }
    /* 设定电机位置环PID参数 */
    motor->PIDLocation.Kp = motorInitData.PIDLocation_KP;
    motor->PIDLocation.Ki = 0.0f;
    motor->PIDLocation.Kd = 0.0f;
    motor->PIDLocation.OutMax = motorInitData.PIDLocation_OutMax;
    motor->PIDLocation.UiOutMax = 0.0f;
    /* 设定电机速度反馈值的低通滤波系数 */
    motor->Speed_LowPassFilter_K = motorInitData.Speed_LowPassFilter_K;
    /* 设定电机速度环PID参数 */
    motor->PIDSpeed.Kp = motorInitData.PIDSpeed_KP;
    motor->PIDSpeed.Ap = motorInitData.PIDSpeed_AP;
    motor->PIDSpeed.Bp = motorInitData.PIDSpeed_BP;
    motor->PIDSpeed.Cp = motorInitData.PIDSpeed_CP;
    motor->PIDSpeed.Ki = motorInitData.PIDSpeed_KI;
    motor->PIDSpeed.Kd = motorInitData.PIDSpeed_KD;
    motor->PIDSpeed.OutMax = motorInitData.PIDSpeed_OutMax;
    motor->PIDSpeed.UiOutMax = motorInitData.PIDSpeed_UiOutMax;
    /* 设定电机速度环前馈参数 */
    motor->SpeedFeedForward.Diff.Formula = motorInitData.MotorThetaDiffFormula;
    motor->SpeedFeedForward.FeedForward_K = motorInitData.SpeedFF_K;
    motor->SpeedFeedForward.FeedForward_MAX = motorInitData.SpeedFF_MAX;
}

/**
  * @brief  Initialize the dm motor threshold value parameters
  *         数据来自2024达妙电机选型手册
  * @param  motorType Type of DM motor, from: DM_MotorType_Enum
  * @param  motorValue Structure for DM Motor threshold value
  * @retval None
  */
void DM_MotorThrValUpdate(uint8_t motorType, DM_MotorThresholdValue* motorValue)
{
    switch (motorType)
    {
    case Motor4310_24V:
        motorValue->P_Max = 12.5f;
        motorValue->V_Max = 30.0f;
        motorValue->T_Max = 10.0f;
        break;
    case Motor4310_48V:
        motorValue->P_Max = 12.5f;
        motorValue->V_Max = 50.0f;
        motorValue->T_Max = 10.0f;
        break;
    case Motor4340:
        motorValue->P_Max = 12.5f;
        motorValue->V_Max = 8.0f;
        motorValue->T_Max = 28.0f;
        break;
    case Motor6006:
        motorValue->P_Max = 12.5f;
        motorValue->V_Max = 45.0f;
        motorValue->T_Max = 20.0f;
        break;
    case Motor8006:
        motorValue->P_Max = 12.5f;
        motorValue->V_Max = 45.0f;
        motorValue->T_Max = 40.0f;
        break;
    case Motor8009:
        motorValue->P_Max = 12.5f;
        motorValue->V_Max = 45.0f;
        motorValue->T_Max = 54.0f;
        break;
    case Motor10010:
        motorValue->P_Max = 12.5f;
        motorValue->V_Max = 20.0f;
        motorValue->T_Max = 200.0f;
        break;
    case Motor10010L:
        motorValue->P_Max = 12.5f;
        motorValue->V_Max = 25.0f;
        motorValue->T_Max = 200.0f;
        break;
    default:
        motorValue->P_Max = 12.5f;
        motorValue->V_Max = 30.0f;
        motorValue->T_Max = 10.0f;
        break;
    }
}

/**
  * @brief  Record the data after getting an CAN frame
  * @param  motor pointer to a DM_MotorStruct structure that contains
  *         the physical condition for the specified Motor.
  * @param  CAN_Rx_Buffer CAN receive array
  * @retval None
  */
void DM_MotorDataReceive(DM_MotorStruct* motor, const uint8_t* CAN_Rx_Buffer)
{
    uint8_t i;
    /* 将CAN接收到的反馈帧数据记录到结构体变量中 */
    for (i = 0; i < 8; i++)
    {
        motor->ReceiveMessage[i] = CAN_Rx_Buffer[i];
    }
    /* 将标志位置为ENABLE，意为数据接收到但未处理 */
    motor->ReceiveFlag = ENABLE;
}

/**
  * @brief  Update motor feedback frame data
  * @param  motor pointer to a DM_MotorStruct structure that contains
  *         the physical condition for the Motor 4310.
  * @retval Whether the ID of the motor control frame is correct, if yes, it is 1 and if not, 0
  */
uint8_t Motor4310FeedbackFrameDataUpdate(DM_MotorStruct* motor, float P_Max, float V_Max, float T_Max)
{
    int pos_tmp, vel_tmp, tor_tmp;
    uint8_t i;
    float SpeedSum;

    /* 解算电机反馈帧的数据 */
    motor->RawFeedback.CAN_ID = (motor->ReceiveMessage[0]) & 0x0F;
    motor->RawFeedback.State = (motor->ReceiveMessage[0]) >> 4;
    pos_tmp = (motor->ReceiveMessage[1] << 8) | motor->ReceiveMessage[2];
    vel_tmp = (motor->ReceiveMessage[3] << 4) | (motor->ReceiveMessage[4] >> 4);
    tor_tmp = ((motor->ReceiveMessage[4] & 0xF) << 8) | motor->ReceiveMessage[5];
    motor->RawFeedback.Position = uint_to_float(pos_tmp, -1.0f * P_Max, P_Max, 16);
    motor->RawFeedback.Velocity = uint_to_float(vel_tmp, -1.0f * V_Max, V_Max, 12);
    motor->RawFeedback.Torque = uint_to_float(tor_tmp, -1.0f * T_Max, T_Max, 12);
    motor->RawFeedback.MosTemperature = (float)(motor->ReceiveMessage[6]);
    motor->RawFeedback.CoilTemperature = (float)(motor->ReceiveMessage[7]);
    /* 计算电机转动的圈数 */
    if (motor->RawFeedback.Position - motor->LastPosition > P_Max)
        motor->EncoderCount--;
    if (motor->RawFeedback.Position - motor->LastPosition < (-1.0f * P_Max))
        motor->EncoderCount++;
    /* 计算电机转子的位置 */
    motor->Location.Location =
        (float)motor->EncoderCount * P_Max / PI + motor->RawFeedback.Position / (2 * PI);
    /* 记录上一次位置电机反馈值 */
    motor->LastPosition = motor->RawFeedback.Position;
    /* 对速度值进行均值滤波 */
    SpeedSum = 0.0f;
    for (i = 9; i > 0; i--)
    {
        motor->Speed_array[i] = motor->Speed_array[i - 1];
        SpeedSum += motor->Speed_array[i];
    }
    motor->Speed_array[0] = motor->RawFeedback.Velocity;
    SpeedSum += motor->Speed_array[0];
    motor->MeanFilterSpeed = SpeedSum / 10.0f;
    /* 对速度值进行低通滤波 */
    motor->LowPassFilterSpeed = motor->Speed_array[0] * motor->Speed_LowPassFilter_K
        + motor->Speed_array[1] * (1 - motor->Speed_LowPassFilter_K);
    /* 对速度值进行归一化 */
    motor->Speed.Speed = motor->RawFeedback.Velocity / V_Max;

    return (motor->CAN_ID == motor->RawFeedback.CAN_ID);
}

/**
  * @brief  Update motor state
  * @param  motor pointer to a DM_MotorStruct structure that contains
  *         the physical condition for the Motor 4310.
  * @param  if_ON Whether to keep the motor on. The value is ENABLE or DISABLE
  * @retval None
  */
void Motor4310StateUpdate(DM_MotorStruct* motor, uint8_t if_ON)
{
    /* 保持20个任务周期的失能状态，然后变为使能状态 */
    if (motor->EnableCount > 20 && motor->MotorState == Motor4310_Disable)
    {
        motor->EnableCount = 0;
        motor->MotorState = Motor4310_Enable;
    }
    /* 判断电机的异常状态 */
    if (motor->RawFeedback.State >= Motor4310_OverVoltage && motor->RawFeedback.State <= Motor4310_Overload)
        motor->MotorState = motor->RawFeedback.State;
    /* 如果退出电机，则改变状态，保持为退出电机状态 */
    motor->MotorState = if_ON == DISABLE ? Motor4310_Exit : motor->MotorState;
    /* 重新开启电机时，变为失能状态 */
    if (if_ON == ENABLE && motor->MotorState == Motor4310_Exit)
        motor->MotorState = Motor4310_Disable;
}

/**
  * @brief  Update motor data, including feedback frame data and state
  * @param  motor pointer to a DM_MotorStruct structure that contains
  *         the physical condition for the Motor 4310.
  * @param  if_ON Whether to keep the motor on. The value is ENABLE or DISABLE
  * @retval None
  */
void DM_MotorDataUpdate(DM_MotorStruct* motor, uint8_t if_ON)
{
    if (motor->ReceiveFlag == ENABLE)
    {
        DM_MotorThresholdValue MV;
        DM_MotorThrValUpdate(motor->MotorType, &MV);

        Motor4310FeedbackFrameDataUpdate(motor, MV.P_Max, MV.V_Max, MV.T_Max);
        Motor4310StateUpdate(motor, if_ON);
        /* 完成数据和状态更新后，将标志位置为DISABLE，意为未收到新数据 */
        motor->ReceiveFlag = DISABLE;
    }
}

/**
  * @brief  Update motor send data
  * @param  motor pointer to a DM_MotorStruct structure that contains
  *         the physical condition for the Motor 4310.
  * @retval None
  */
void Motor4310SendDataCalculate(DM_MotorStruct* motor, float V_Max, float T_Max)
{
    switch (motor->MotorMode)
    {
    case Motor4310_MIT:
        if (motor->MitMode != Motor4310_MIT_Torque)
            motor->SetValue.Torque = motor->PIDSpeed.Out * T_Max + motor->SetTorque;
        else
            motor->SetValue.Torque = motor->SetTorque;
        if (motor->SetValue.Torque > T_Max)
            motor->SetValue.Torque = T_Max;
        if (motor->SetValue.Torque < -1.0f * T_Max)
            motor->SetValue.Torque = -1.0f * T_Max;
        motor->SetValue.Kp = 0.0f;
        motor->SetValue.Kd = 0.0f;
        motor->SetValue.Velocity = 0.0f;
        motor->SetValue.Position = 0.0f;
        break;
    case Motor4310_POS:
        motor->SetValue.Velocity = motor->PIDLocation.OutMax * V_Max;
        motor->SetValue.Position = motor->Location.SetLocation * (2 * PI);
        break;
    case Motor4310_VEL:
        motor->SetValue.Velocity = motor->Speed.SetSpeed * V_Max;
        break;
    }
}

/**
  * @brief  将发送的数据转换为MIT模式下的帧格式
  * @param  motor pointer to a DM_MotorStruct structure that contains
  *         the physical condition for the Motor 4310.
  * @retval None
  */
void Motor4310_MIT_SendFrameDataUpdate(DM_MotorStruct* motor, float P_Max, float V_Max, float T_Max)
{
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    pos_tmp = float_to_uint(motor->SetValue.Position, -1.0f * P_Max, P_Max, 16);
    vel_tmp = float_to_uint(motor->SetValue.Velocity, -1.0f * V_Max, V_Max, 12);
    kp_tmp = float_to_uint(motor->SetValue.Kp, DM_KP_MIN, DM_KP_MAX, 12);
    kd_tmp = float_to_uint(motor->SetValue.Kd, DM_KD_MIN, DM_KD_MAX, 12);
    tor_tmp = float_to_uint(motor->SetValue.Torque, -1.0f * T_Max, T_Max, 12);

    motor->SendMessage[0] = (pos_tmp >> 8);
    motor->SendMessage[1] = pos_tmp;
    motor->SendMessage[2] = (vel_tmp >> 4);
    motor->SendMessage[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    motor->SendMessage[4] = kp_tmp;
    motor->SendMessage[5] = (kd_tmp >> 4);
    motor->SendMessage[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    motor->SendMessage[7] = tor_tmp;
}

/**
  * @brief  将发送的数据转换为位置速度模式下的帧格式
  * @param  motor pointer to a DM_MotorStruct structure that contains
  *         the physical condition for the Motor 4310.
  * @retval None
  */
void Motor4310_POS_SendFrameDataUpdate(DM_MotorStruct* motor)
{
    uint8_t *pbuf, *vbuf;
    pbuf = (uint8_t*)&motor->SetValue.Position;
    vbuf = (uint8_t*)&motor->SetValue.Velocity;

    motor->SendMessage[0] = *pbuf;
    motor->SendMessage[1] = *(pbuf + 1);
    motor->SendMessage[2] = *(pbuf + 2);
    motor->SendMessage[3] = *(pbuf + 3);
    motor->SendMessage[4] = *vbuf;
    motor->SendMessage[5] = *(vbuf + 1);
    motor->SendMessage[6] = *(vbuf + 2);
    motor->SendMessage[7] = *(vbuf + 3);
}

/**
  * @brief  将发送的数据转换为速度模式下的帧格式
  * @param  motor pointer to a DM_MotorStruct structure that contains
  *         the physical condition for the Motor 4310.
  * @retval None
  */
void Motor4310_VEL_SendFrameDataUpdate(DM_MotorStruct* motor)
{
    uint8_t* vbuf;
    vbuf = (uint8_t*)&motor->SetValue.Velocity;

    motor->SendMessage[0] = *vbuf;
    motor->SendMessage[1] = *(vbuf + 1);
    motor->SendMessage[2] = *(vbuf + 2);
    motor->SendMessage[3] = *(vbuf + 3);
}

/**
  * @brief  将发送的数据转换为相应的帧格式
  * @param  motor pointer to a DM_MotorStruct structure that contains
  *         the physical condition for the Motor 4310.
  * @retval None
  */
void Motor4310SendFrameDataUpdate(DM_MotorStruct* motor, float P_Max, float V_Max, float T_Max)
{
    uint8_t i;
    /* 未使能状态下，发送使能帧 */
    if (motor->MotorState == Motor4310_Disable)
    {
        /* 将使能计时器加一，记录使能帧发送次数 */
        motor->EnableCount++;
        for (i = 0; i < 8; i++)
        {
            motor->SendMessage[i] = EntryMotor[i];
        }
    }
    /* 关闭电机状态下，发送失能帧 */
    else if (motor->MotorState == Motor4310_Exit)
    {
        for (i = 0; i < 8; i++)
        {
            motor->SendMessage[i] = ExitMotor[i];
        }
    }
    /* 电机异常状态下，发送清除错误帧 */
    else if (motor->MotorState == Motor4310_OverVoltage || motor->MotorState == Motor4310_UnderVoltage ||
        motor->MotorState == Motor4310_OverCurrent || motor->MotorState == Motor4310_MosOverTemperature ||
        motor->MotorState == Motor4310_CoilOverTemperature || motor->MotorState == Motor4310_CommunicationLos ||
        motor->MotorState == Motor4310_Overload)
    {
        for (i = 0; i < 8; i++)
        {
            motor->SendMessage[i] = ClearErrors[i];
        }
    }
    /* 电机已使能状态下，发送控制帧 */
    else if (motor->MotorState == Motor4310_Enable)
    {
        /* 判断模式并将发送的数据转换到该模式下的帧格式 */
        switch (motor->MotorMode)
        {
        case Motor4310_MIT:
            Motor4310_MIT_SendFrameDataUpdate(motor, P_Max, V_Max, T_Max);
            break;
        case Motor4310_POS:
            Motor4310_POS_SendFrameDataUpdate(motor);
            break;
        case Motor4310_VEL:
            Motor4310_VEL_SendFrameDataUpdate(motor);
            break;
        }
    }
}

/**
  * @brief  更新所需要发送的数据
  * @param  motor pointer to a DM_MotorStruct structure that contains
  *         the physical condition for the Motor 4310.
  * @retval None
  */
void DM_MotorSendDataUpdate(DM_MotorStruct* motor)
{
    DM_MotorThresholdValue MV;
    DM_MotorThrValUpdate(motor->MotorType, &MV);

    Motor4310SendDataCalculate(motor, MV.V_Max, MV.T_Max);
    Motor4310SendFrameDataUpdate(motor, MV.P_Max, MV.V_Max, MV.T_Max);
}

/**
  * @brief  计算电机PID速度环
  * @param  motor pointer to a DM_MotorStruct structure that contains
  * @retval None
  */
void DM_MotorSpeedCalculate(DM_MotorStruct* motor)
{
    motor->PIDSpeed.Ref = motor->Speed.SetSpeed;
    motor->PIDSpeed.Fdb = motor->Speed.Speed;
    PidCalc(&motor->PIDSpeed);
}

/**
  * @brief  计算电机PID位置环
  * @param  motor pointer to a DM_MotorStruct structure that contains
  *         the physical condition for the Motor 4310.
  * @retval None
  */
void DM_MotorLocationCalculate(DM_MotorStruct* motor)
{
    motor->PIDLocation.Ref = motor->Location.SetLocation;
    motor->PIDLocation.Fdb = motor->Location.Location;
    PidCalc(&motor->PIDLocation);
}

/**
  * @brief  Calculate motor speed feedforward data
  * @param  motor pointer to a DM_MotorStruct structure that contains
  *         the physical condition for the Motor 4310.
  * @param  interval The interval t between two SetLocation, in milliseconds
  * @retval None
  */
void DM_MotorSpeedFFCalculate(DM_MotorStruct* motor, double interval, float V_Max)
{
    double SetLocation;

    SetLocation = motor->Location.SetLocation * 2.0f * PI; //单位变为弧度

    interval = interval / 1000.0f; //单位变为秒
    /* 进行微分计算 */
    DiffCalc(&motor->SpeedFeedForward.Diff, SetLocation, interval);
    /* 速度单位为弧度每秒 */
    motor->SpeedFeedForward.FeedForward = (float)motor->SpeedFeedForward.Diff.diffValue;
    /* 速度归一化 */
    motor->SpeedFeedForward.FeedForward /= V_Max;
    motor->SpeedFeedForward.FeedForward *= motor->SpeedFeedForward.FeedForward_K;
    /* 对前馈进行限幅 */
    if (motor->SpeedFeedForward.FeedForward > motor->SpeedFeedForward.FeedForward_MAX)
        motor->SpeedFeedForward.FeedForward = motor->SpeedFeedForward.FeedForward_MAX;
    if (motor->SpeedFeedForward.FeedForward < -1 * motor->SpeedFeedForward.FeedForward_MAX)
        motor->SpeedFeedForward.FeedForward = -1 * motor->SpeedFeedForward.FeedForward_MAX;
}

/**
  * @brief  Motor PID Calculate and feedback Calculate
  * @param  motor pointer to a DM_MotorStruct structure that contains
  *         the physical condition for the Motor 4310.
  * @param  interval The interval t between two SetLocation, in milliseconds
  * @param  if_ON Whether to keep the motor on. The value is ENABLE or DISABLE
  * @retval None
  */
void DM_MotorPIDCalculate(DM_MotorStruct* motor, double interval, uint8_t if_ON)
{
    if (motor->MotorMode == Motor4310_MIT)
    {
        DM_MotorThresholdValue MV;
        DM_MotorThrValUpdate(motor->MotorType, &MV);

        switch (motor->MitMode)
        {
        case Motor4310_MIT_Torque:
            break;
        case Motor4310_MIT_Location:
            DM_MotorLocationCalculate(motor);
            DM_MotorSpeedFFCalculate(motor, interval, MV.V_Max);
            motor->Speed.SetSpeed = motor->PIDLocation.Out + motor->SpeedFeedForward.FeedForward;
        case Motor4310_MIT_Speed:
            DM_MotorSpeedCalculate(motor);
        }
        if (if_ON == DISABLE)
        {
            motor->SetTorque = 0.0f;
            if (motor->MitMode == Motor4310_MIT_Location)
            {
                motor->Location.SetLocation = motor->Location.Location;
                PidClear(&motor->PIDSpeed);
                PidClear(&motor->PIDLocation);
            }
            else if (motor->MitMode == Motor4310_MIT_Speed)
            {
                motor->Speed.SetSpeed = 0.0f;
                PidClear(&motor->PIDSpeed);
            }
        }
    }
    else
    {
        if (if_ON == DISABLE)
        {
            motor->Location.SetLocation = motor->Location.Location;
            motor->Speed.SetSpeed = 0.0f;
        }
    }
}

/***************************** (C) END OF FILE ******************************/

//
// Created by Lenovo on 2023/12/28.
//

/**
  ******************************************************************************
  * @file    driver_rs_motor.c
  * @author  Shuai Yang
  * @brief   motor driver 20251017
  *          This file provides functions to process rs motor data and update data:
  *           + Motor ID solve function
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
      (#) Initialize the motor by implementing the RS_MotorInit():
          (++) Initialize the motor parameters, including:
		       (+++) pid parameters

      (#) Solve the motor ID by implementing the Get_Motor_ID():
          (++) Solve the RS_Motor motor ID, including:
		       (+++) ExtId

      (#) Get motor data by CAN by implementing the RS_MotorDataReceive():
          (++) Initialize the motor parameters, including:
		       (+++) ReceiveMessege

      (#) Update motor data by implementing the RS_MotorDataUpdate():
          (++) Update motor parameters, including:
      	       (+++) Position
      	       (+++) Velocity
      	       (+++) Torque
          (++) Update motor state

      (#) Update motor send data by implementing the RS_MotorSendDataUpdate():
          (++) Update motor SetValue, including:
               (+++) SetTorque
               (+++) SetVelocity
               (+++) SetPosition
          (++) Update motor SendMessage

      (#) PID calculate by implementing the RS_MotorPIDCalculate():
          (++) PID calculate, including:
               (+++) PID Speed Calculate
               (+++) PID Location Calculate
               (+++) PID output clear
          (++) Speed feedback calculate

  @endverbatim
  ******************************************************************************
  * @version 20251017
  * @attention
  * 这个文件已经完全封装好，不需要修改，所有宏定义都来自电机手册
  *
  * Don't forget the author
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "driver_rs_motor.h"

/* ------------------------ Function Implements ----------------------------- */
/**
  * @brief  初始化灵足电机参数，主要包括CAN_ID、MasterID、控制模式以及PID参数
  * @param  motor pointer to a RS_MotorStruct structure that contains
  *         the PID information for the specified Motor.
  * @param  Structure for motor initialization
  * @retval None
  */
void RS_MotorInit(RS_MotorStruct* motor, RS_MotorInitStruct motorInitData)
{
    /* 设置电机的类型 */
    motor->MotorMode.MotorType = motorInitData.MotorTypeInit;
    /* 设置电机的控制模式 */
    motor->MotorMode.ControlModeSet = motorInitData.MotorModeInit;
    motor->MotorMode.ControlModeFdb = ~motorInitData.MotorModeInit;
    /* 进一步设置电机 CUR 模式下的控制模式 */
    motor->MotorMode.CurModeSet = motorInitData.CurModeInit;
    /* 将电机指令流设置为 RS_MotorInitState，意为电机此时未初始化 */
    motor->MotorMode.MotorFlow = RS_MotorInitState;
    motor->MotorMode.FlowState = 1; //重置指令流状态
    /* 将电机模式反馈值初始化为Reset模式，意为电机失能 */
    motor->MotorMode.MotorModeStateFdb = RS_MotorResetMode;
    /* 设置零点标志位，0代表0到2π，1代表-π到π */
    motor->MotorMode.ZeroStateSet = motorInitData.ZeroState;
    motor->MotorMode.ZeroStateFdb = ~motorInitData.ZeroState;
    /* 设置零位设置标志位，0代表关闭，1代表开启 */
    motor->MotorMode.ZeroPositionFlag = motorInitData.ZeroPosition;
    /* 将记录电机流的计时器进行初始化 */
    motor->StateCount = 0;
    /* 将CAN接收标志位设置为 0，意为未接收到数据 */
    motor->ReceiveFlag = 0;
    /* 设置电机 MasterID 和 CAN_ID */
    motor->CAN_ID = motorInitData.CAN_ID;
    motor->MasterID = motorInitData.MasterID;
    /* 将电机旋转的圈数初始化 */
    motor->EncoderCount = 0;
    /* 设定电机位置环PID参数 */
    motor->PIDLocation.Kp = motorInitData.PIDLocation_KP;
    motor->PIDLocation.Ki = 0.0f;
    motor->PIDLocation.Kd = 0.0f;
    motor->PIDLocation.OutMax = motorInitData.PIDLocation_OutMax;
    motor->PIDLocation.UiOutMax = 0.0f;
    /* 设定电机速度反馈值的均值滤波系数 */
    motor->Speed_MeanFilter_K = motorInitData.Speed_MeanFilter_K_Init;
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
  *         数据来自每一个电机的手册
  * @param  motorType Type of DM motor, from: DM_MotorType_Enum
  * @param  motorValue Structure for DM Motor threshold value
  * @retval None
  */
void RS_MotorThrValUpdate(uint8_t motorType, RS_MotorThresholdValue* motorValue)
{
    switch (motorType)
    {
    case MotorRS00:
        motorValue->V_Max = 33.0f;
        motorValue->T_Max = 14.0f;
        motorValue->I_Max = 16.0f;
        break;
    case MotorRS01:
    case MotorRS02:
        motorValue->V_Max = 44.0f;
        motorValue->T_Max = 17.0f;
        motorValue->I_Max = 23.0f;
        break;
    case MotorRS03:
        motorValue->V_Max = 20.0f;
        motorValue->T_Max = 60.0f;
        motorValue->I_Max = 43.0f;
        break;
    case MotorRS04:
        motorValue->V_Max = 15.0f;
        motorValue->T_Max = 120.0f;
        motorValue->I_Max = 90.0f;
        break;
    case MotorRS05:
        motorValue->V_Max = 50.0f;
        motorValue->T_Max = 5.5f;
        motorValue->I_Max = 11.0f;
        break;
    case MotorRS06:
        motorValue->V_Max = 50.0f;
        motorValue->T_Max = 36.0f;
        motorValue->I_Max = 57.0f;
        break;
    default:
        motorValue->V_Max = 33.0f;
        motorValue->T_Max = 14.0f;
        motorValue->I_Max = 16.0f;
        break;
    }
}

/**
  * @brief  判断主ID是否正确，接收电机反馈帧数据以及ID
  * @param  motor pointer to a RS_MotorStruct structure that contains
  *         the physical condition for the specified Motor.
  * @param  CAN receive array
  * @param  ExtId the extended identifier
  * @retval None
  */
void RS_MotorDataReceive(RS_MotorStruct* motor, const uint8_t* CAN_Rx_Buffer, uint32_t ExtId)
{
    uint8_t i;
    EXT_ID_t Ext_Id;

    Ext_Id = *((EXT_ID_t*)(&ExtId));
    if (Ext_Id.id == motor->MasterID)
    {
        motor->RawFeedback.ExtId = Ext_Id;
        for (i = 0; i < 8; i++)
        {
            motor->ReceiveMessage[i] = CAN_Rx_Buffer[i];
        }
        motor->ReceiveFlag = 1;
    }
}

/**
  * @brief  更新电机的反馈帧
  * @param  motor pointer to a RS_MotorStruct structure that contains
  *         the physical condition for the RS_Motor.
  * @retval 电机反馈的CAN_ID和MasterID是否与设置的相同，相同返回1，不同返回0
  */
uint8_t RS_MotorFeedbackFrameDataUpdate(RS_MotorStruct* motor, float V_Max, float T_Max)
{
    int pos_tmp, vel_tmp, tor_tmp, tem_tmp;
    uint8_t error;
    uint8_t masterID;
    uint8_t canID;

    /* 解算电机反馈帧中数据区2中(ID中bit7~0)的数据 */
    masterID = motor->RawFeedback.ExtId.id;
    /* 解算电机反馈帧中数据区2中(ID中bit15~8)的数据 */
    canID = motor->RawFeedback.ExtId.data & 0xFF; //当前电机CAN_ID
    if (motor->CAN_ID == canID && motor->MasterID == masterID)
    {
        /* 解算电机反馈帧中数据区2中(ID中bit23~16)的数据 */
        error = (motor->RawFeedback.ExtId.data >> 8) & 0x3F; //故障信息
        motor->ErrorState.RS_MotorFdbError = error == 0 ? 0 : 1;
        motor->ErrorState = *((RS_MotorErrorStateStruct*)(&error));
        motor->MotorMode.MotorModeStateFdb = (motor->RawFeedback.ExtId.data >> 14); //模式状态
        /* 解算电机反馈帧中数据区1中(即反馈帧的数据段)的数据 */
        pos_tmp = (motor->ReceiveMessage[0] << 8) | motor->ReceiveMessage[1];
        vel_tmp = (motor->ReceiveMessage[2] << 8) | motor->ReceiveMessage[3];
        tor_tmp = (motor->ReceiveMessage[4] << 8) | motor->ReceiveMessage[5];
        tem_tmp = (motor->ReceiveMessage[6] << 8) | motor->ReceiveMessage[7];
        motor->RawFeedback.Position = uint_to_float(pos_tmp, RS_P_MIN, RS_P_MAX, 16);
        motor->RawFeedback.Velocity = uint_to_float(vel_tmp, -1.0f * V_Max, V_Max, 16);
        motor->RawFeedback.Torque = uint_to_float(tor_tmp, -1.0f * T_Max, T_Max, 16);
        motor->RawFeedback.Temperature = (float)tem_tmp / 10.0f;
        return 1;
    }
    return 0;
}

/**
  * @brief  对电机的反馈帧进行解算
  * @param  motor pointer to a RS_MotorStruct structure that contains
  *         the physical condition for the RS_Motor.
  * @retval None
  */
void RS_MotorRawFeedbackDataCalculate(RS_MotorStruct* motor, float V_Max)
{
    uint8_t i;
    float SpeedSum;

    if (motor->RawFeedback.Position - motor->LastPosition > RS_P_MAX)
        motor->EncoderCount--;
    if (motor->RawFeedback.Position - motor->LastPosition < RS_P_MIN)
        motor->EncoderCount++;
    motor->Location.Location =
        (float)motor->EncoderCount * RS_P_MAX / PI + motor->RawFeedback.Position / (2 * PI);
    motor->LastPosition = motor->RawFeedback.Position;
    SpeedSum = 0.0f;
    for (i = 9; i > 0; i--)
    {
        motor->Speed_array[i] = motor->Speed_array[i - 1];
        SpeedSum += motor->Speed_array[i];
    }
    motor->Speed_array[0] = motor->RawFeedback.Velocity;
    SpeedSum += motor->Speed_array[0];
    motor->MeanFilterSpeed = SpeedSum / 10.0f;
    motor->LowPassFilterSpeed = motor->Speed_array[0] * motor->Speed_MeanFilter_K
        + motor->Speed_array[1] * (1 - motor->Speed_MeanFilter_K);
    motor->Speed.Speed = motor->RawFeedback.Velocity / V_Max;
}

/**
  * @brief  电机的参数读取帧
  * @param  motor pointer to a RS_MotorStruct structure that contains
  *         the physical condition for the RS_Motor.
  * @retval 是否读取成功，是返回1，否返回0
  */
uint8_t RS_MotorParamReadFrameDataUpdate(RS_MotorStruct* motor)
{
    /* ID中bit23~16的值为00则读取成功 */
    if ((motor->RawFeedback.ExtId.data >> 8) == 0)
    {
        uint16_t index;
        index = (motor->ReceiveMessage[1] << 8) | motor->ReceiveMessage[0];
        switch (index)
        {
        case RUN_MODE:
            motor->MotorMode.ControlModeFdb = motor->ReceiveMessage[4];
            break;
        case ZERO_STA:
            motor->MotorMode.ZeroStateFdb = motor->ReceiveMessage[4];
            break;
        default:
            return 0;
        }
        return 1;
    }
    return 0;
}

/**
  * @brief  电机故障参数反馈帧
  * @param  motor pointer to a RS_MotorStruct structure that contains
  *         the physical condition for the RS_Motor.
  * @retval 电机反馈的CAN_ID和MasterID是否与设置的相同，相同返回1，不同返回0
  */
uint8_t testErrorCount = 0;

uint8_t RS_MotorErrorFrameDataUpdate(RS_MotorStruct* motor)
{
    uint8_t error;
    uint8_t masterID;
    uint8_t canID;

    /* 解算故障反馈帧中数据区2中(ID中bit7~0)的数据 */
    masterID = motor->RawFeedback.ExtId.id;
    /* 解算故障反馈帧中数据区2中(ID中bit15~8)的数据 */
    canID = motor->RawFeedback.ExtId.data & 0xFF; //当前电机CAN_ID
    testErrorCount++;
    if (motor->CAN_ID == canID && motor->MasterID == masterID)
    {
        //		if (motor->ReceiveMessage[0] == 0 && motor->ReceiveMessage[1] == 0
        //			&& motor->ReceiveMessage[2] == 0 && motor->ReceiveMessage[3] == 0)
        //			motor->ErrorState.RS_MotorErrorFdb = 0;
        //		else
        //		{
        motor->ErrorState.RS_MotorErrorFdb = 1;
        //			/* 解算故障帧中数据区1中(即故障帧的数据段)的数据 */
        //			error = motor->ReceiveMessage[0] & 0x3F;//故障信息
        //			motor->ErrorState = *((RS_MotorErrorStateStruct*)(&error));
        //		}
        return 1;
    }
    return 0;
}

/**
  * @brief  Update motor operating state
  * @param  motor pointer to a RS_MotorStruct structure that contains
  *         the physical condition for the RS_Motor.
  * @param  if_ON Whether to keep the motor on. The value is ENABLE or DISABLE
  * @retval None
  */
void RS_MotorOperatingStateUpdate(RS_MotorStruct* motor, uint8_t if_ON)
{
    switch (motor->MotorMode.MotorFlow)
    {
    case RS_MotorInitState:
        if (motor->StateCount >= 80 && motor->MotorMode.FlowState == 1)
        {
            motor->MotorMode.FlowState = 0; //重置指令流状态
            motor->MotorMode.MotorFlow = RS_MotorResetState;
        }
        break;
    case RS_MotorResetState:
        if (motor->ErrorState.RS_MotorErrorFdb == 0 && motor->ErrorState.RS_MotorFdbError == 0)
            motor->MotorMode.FlowState = 1; //若已经没有故障，则本指令即完成
        if (if_ON && motor->MotorMode.FlowState == 1) //电机启动，且本指令已经完成
        {
            motor->MotorMode.FlowState = 0; //重置指令流状态
            motor->MotorMode.MotorFlow = RS_MotorRunningState; //设置为运行状态
        }
        break;
    case RS_MotorRunningState:
        if (motor->MotorMode.MotorModeStateFdb == RS_MotorRunningMode)
            motor->MotorMode.FlowState = 1; //电机的状态模式成为运行模式时，完成该指令状态
        if (if_ON == DISABLE)
        {
            motor->MotorMode.FlowState = 1; //重置指令流状态
            motor->MotorMode.MotorFlow = RS_MotorResetState;
        }
        break;
    case RS_MotorErrorState:
        if (motor->RawFeedback.Temperature < 80.0f) //如果故障已经可以清除
        {
            //motor->MotorMode.FlowState = 0;//重置指令流状态
            //motor->MotorMode.MotorFlow = RS_MotorResetState;
        }
        break;
    default:
        break;
    }
    //	if (motor->MotorMode.ControlModeSet != motor->MotorMode.ControlModeFdb
    //		|| motor->MotorMode.ZeroStateSet != motor->MotorMode.ZeroStateFdb
    //		|| motor->MotorMode.ZeroPositionFlag == 1)//如果电机的模式设置与反馈不同
    //	{
    //		motor->MotorMode.MotorFlow = RS_MotorInitState;
    //	}
    if (motor->ErrorState.RS_MotorErrorFdb != 0 || motor->ErrorState.RS_MotorFdbError != 0) //如果电机出现故障
    {
        motor->MotorMode.MotorFlow = RS_MotorErrorState;
    }
}

/**
  * @brief  Update motor data, including feedback frame data and state
  * @param  motor pointer to a RS_MotorStruct structure that contains
  *         the physical condition for the Motor RS_Motor.
  * @param  if_ON Whether to keep the motor on. The value is ENABLE or DISABLE
  * @retval None
  */
void RS_MotorDataUpdate(RS_MotorStruct* motor, uint8_t if_ON)
{
    if (motor->ReceiveFlag == 1)
    {
        RS_MotorThresholdValue MV;
        RS_MotorThrValUpdate(motor->MotorMode.MotorType, &MV);
        switch (motor->RawFeedback.ExtId.mode)
        {
        case RS_MotorFeedback:
        case RS_MotorReport:
            if (RS_MotorFeedbackFrameDataUpdate(motor, MV.V_Max, MV.T_Max) == 1)
                RS_MotorRawFeedbackDataCalculate(motor, MV.V_Max);
            break;
        case RS_MotorParamRead:
            RS_MotorParamReadFrameDataUpdate(motor);
            break;
        case RS_MotorFaultFeedback:
            RS_MotorErrorFrameDataUpdate(motor);
            break;
        default:
            break;
        }
        RS_MotorOperatingStateUpdate(motor, if_ON);
        /* 完成数据和状态更新后，将标志位置为DISABLE，意为未收到新数据 */
        motor->ReceiveFlag = 2;
    }
}

/**
  * @brief  Update motor send data
  * @param  motor pointer to a RS_MotorStruct structure that contains
  *         the physical condition for the RS_Motor.
  * @retval None
  */
void RS_MotorSendDataCalculate(RS_MotorStruct* motor, float V_Max, float T_Max, float I_Max)
{
    switch (motor->MotorMode.ControlModeSet)
    {
    case RS_Motor_CON:
        motor->SetValue.Torque = motor->SetTorque;
        motor->SetValue.Position = motor->Location.SetLocation * (2 * PI);
        motor->SetValue.Velocity = motor->Speed.SetSpeed * V_Max;
        if (motor->SetValue.Torque > T_Max)
            motor->SetValue.Torque = T_Max;
        if (motor->SetValue.Torque < -1.0f * T_Max)
            motor->SetValue.Torque = -1.0f * T_Max;
        break;
    case RS_Motor_POS:
        motor->SetValue.Position = motor->Location.SetLocation * (2 * PI);
        break;
    case RS_Motor_VEL:
        motor->SetValue.Velocity = motor->Speed.SetSpeed * V_Max;
        break;
    case RS_Motor_CUR:
        motor->SetValue.Current = motor->PIDSpeed.Out * I_Max + motor->SetTorque * I_Max / T_Max;
        if (motor->SetValue.Current > I_Max)
            motor->SetValue.Current = I_Max;
        if (motor->SetValue.Current < -1.0f * I_Max)
            motor->SetValue.Current = -1.0f * I_Max;
        break;
    }
}

/**
  * @brief  将发送的数据转换为“使能电机”的帧格式
  *         通信类型3
  * @param  motor pointer to a RS_MotorStruct structure that contains
  *         the physical condition for the RS_Motor.
  * @retval None
  */
void RS_MotorEnableSendFrameDataUpdate(RS_MotorStruct* motor)
{
    uint8_t i;

    motor->SetValue.ExtId.id = motor->CAN_ID;
    motor->SetValue.ExtId.data = motor->MasterID;
    motor->SetValue.ExtId.mode = RS_MotorEnableRunning;
    motor->SetValue.ExtId.res = 0;
    for (i = 0; i < 8; i++)
    {
        motor->SendMessage[i] = 0;
    }
}

/**
  * @brief  将发送的数据转换为“电机停止”的帧格式
  *         通信类型4
  * @param  motor pointer to a RS_MotorStruct structure that contains
  *         the physical condition for the RS_Motor.
  * @param  ifClearError 是否清除故障，1为是，0为否
  * @retval None
  */
void RS_MotorStopSendFrameDataUpdate(RS_MotorStruct* motor, uint8_t ifClearError)
{
    uint8_t i;

    motor->SetValue.ExtId.id = motor->CAN_ID;
    motor->SetValue.ExtId.data = motor->MasterID;
    motor->SetValue.ExtId.mode = RS_MotorStopRunning;
    motor->SetValue.ExtId.res = 0;

    for (i = 0; i < 8; i++)
    {
        motor->SendMessage[i] = 0;
    }
    if (ifClearError == 1)
        motor->SendMessage[0] = 1; //清故障
}

/**
  * @brief  将发送的数据转换为“模式初始化”的帧格式，默认电机的 ControlModeSet 已经赋值
  *         通信类型18
  * @param  motor pointer to a RS_MotorStruct structure that contains
  *         the physical condition for the RS_Motor.
  * @retval None
  */
void RS_MotorModeInitSendFrameDataUpdate(RS_MotorStruct* motor)
{
    uint8_t i;
    uint8_t runMode;
    uint16_t index;

    runMode = motor->MotorMode.ControlModeSet;
    index = RUN_MODE;
    motor->SetValue.ExtId.id = motor->CAN_ID;
    motor->SetValue.ExtId.data = motor->MasterID;
    motor->SetValue.ExtId.mode = RS_MotorParamWrite;
    motor->SetValue.ExtId.res = 0;
    for (i = 0; i < 8; i++)
    {
        motor->SendMessage[i] = 0;
    }
    memcpy(&motor->SendMessage[0], &index, 2);
    memcpy(&motor->SendMessage[4], &runMode, 1);
}

/**
  * @brief  将发送的数据转换为“零点标志位”的帧格式，默认电机的 ZeroStateSet 已经赋值
  *         通信类型18
  * @param  motor pointer to a RS_MotorStruct structure that contains
  *         the physical condition for the RS_Motor.
  * @retval None
  */
void RS_MotorModeZeroStaSendFrameDataUpdate(RS_MotorStruct* motor)
{
    uint8_t i;
    uint8_t zero_sta;
    uint16_t index;

    zero_sta = motor->MotorMode.ZeroStateSet;
    index = ZERO_STA;
    motor->SetValue.ExtId.id = motor->CAN_ID;
    motor->SetValue.ExtId.data = motor->MasterID;
    motor->SetValue.ExtId.mode = RS_MotorParamWrite;
    motor->SetValue.ExtId.res = 0;
    for (i = 0; i < 8; i++)
    {
        motor->SendMessage[i] = 0;
    }
    memcpy(&motor->SendMessage[0], &index, 2);
    memcpy(&motor->SendMessage[4], &zero_sta, 1);
}

/**
  * @brief  将发送的数据转换为“设置电机机械零位”的帧格式
  *         通信类型6
  * @param  motor pointer to a RS_MotorStruct structure that contains
  *         the physical condition for the RS_Motor.
  * @retval None
  */
void RS_MotorModeZeroPosSetFrameDataUpdate(RS_MotorStruct* motor)
{
    uint8_t i;

    motor->SetValue.ExtId.id = motor->CAN_ID;
    motor->SetValue.ExtId.data = motor->MasterID;
    motor->SetValue.ExtId.mode = RS_MotorSetZeroPos;
    motor->SetValue.ExtId.res = 0;
    for (i = 0; i < 8; i++)
    {
        motor->SendMessage[i] = 0;
    }
    motor->SendMessage[0] = 1;
}

/**
  * @brief  将发送的数据转换为“电机数据保存”的帧格式
  *         通信类型22
  * @param  motor pointer to a RS_MotorStruct structure that contains
  *         the physical condition for the RS_Motor.
  * @retval None
  */
void RS_MotorModeDataStorageSendFrameDataUpdate(RS_MotorStruct* motor)
{
    uint8_t i;

    motor->SetValue.ExtId.id = motor->CAN_ID;
    motor->SetValue.ExtId.data = motor->MasterID;
    motor->SetValue.ExtId.mode = RS_MotorDataStorage;
    motor->SetValue.ExtId.res = 0;
    for (i = 0; i < 8; i++)
    {
        motor->SendMessage[i] = i + 1;
    }
}

/**
  * @brief  空白帧格式
  * @param  motor pointer to a RS_MotorStruct structure that contains
  *         the physical condition for the RS_Motor.
  * @retval None
  */
void RS_MotorModePauseSendFrameDataUpdate(RS_MotorStruct* motor)
{
    uint8_t i;

    motor->SetValue.ExtId.id = 0;
    motor->SetValue.ExtId.data = 0;
    motor->SetValue.ExtId.mode = 0;
    motor->SetValue.ExtId.res = 0;
    for (i = 0; i < 8; i++)
    {
        motor->SendMessage[i] = 0;
    }
}

/**
  * @brief  单个参数读取帧格式，读取运行模式
  *         通信类型17
  * @param  motor pointer to a RS_MotorStruct structure that contains
  *         the physical condition for the RS_Motor.
  * @retval None
  */
void RS_MotorRunReadSendFrameDataUpdate(RS_MotorStruct* motor)
{
    uint8_t i;
    uint16_t index;

    index = RUN_MODE;
    motor->SetValue.ExtId.id = motor->CAN_ID;
    motor->SetValue.ExtId.data = motor->MasterID;
    motor->SetValue.ExtId.mode = RS_MotorParamRead;
    motor->SetValue.ExtId.res = 0;
    for (i = 0; i < 8; i++)
    {
        motor->SendMessage[i] = 0;
    }
    memcpy(&motor->SendMessage[0], &index, 2);
}

/**
  * @brief  单个参数读取帧格式，读取零点标志位
  *         通信类型17
  * @param  motor pointer to a RS_MotorStruct structure that contains
  *         the physical condition for the RS_Motor.
  * @retval None
  */
void RS_MotorZeroReadSendFrameDataUpdate(RS_MotorStruct* motor)
{
    uint8_t i;
    uint16_t index;
    index = ZERO_STA;
    motor->SetValue.ExtId.id = motor->CAN_ID;
    motor->SetValue.ExtId.data = motor->MasterID;
    motor->SetValue.ExtId.mode = RS_MotorParamRead;
    motor->SetValue.ExtId.res = 0;
    for (i = 0; i < 8; i++)
    {
        motor->SendMessage[i] = 0;
    }
    memcpy(&motor->SendMessage[0], &index, 2);
}

/**
  * @brief  将发送的数据转换为“CON模式”的帧格式
  *         通信类型1
  * @param  motor pointer to a RS_MotorStruct structure that contains
  *         the physical condition for the RS_Motor.
  * @retval None
  */
void RS_Motor_CON_SendFrameDataUpdate(RS_MotorStruct* motor, float V_Max, float T_Max)
{
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    pos_tmp = float_to_uint(motor->SetValue.Position, RS_P_MIN, RS_P_MAX, 16);
    vel_tmp = float_to_uint(motor->SetValue.Velocity, -1.0f * V_Max, V_Max, 16);
    kp_tmp = float_to_uint(motor->SetValue.Kp, RS_KP_MIN, RS_KP_MAX, 16);
    kd_tmp = float_to_uint(motor->SetValue.Kd, RS_KD_MIN, RS_KD_MAX, 16);
    tor_tmp = float_to_uint(motor->SetValue.Torque, -1.0f * T_Max, T_Max, 16);

    motor->SetValue.ExtId.id = motor->CAN_ID;
    motor->SetValue.ExtId.data = tor_tmp;
    motor->SetValue.ExtId.mode = RS_MotorControlCmd;
    motor->SetValue.ExtId.res = 0;
    motor->SendMessage[0] = (pos_tmp >> 8);
    motor->SendMessage[1] = pos_tmp;
    motor->SendMessage[2] = (vel_tmp >> 8);
    motor->SendMessage[3] = vel_tmp;
    motor->SendMessage[4] = (kp_tmp >> 8);
    motor->SendMessage[5] = kp_tmp;
    motor->SendMessage[6] = (kd_tmp >> 8);
    motor->SendMessage[7] = kd_tmp;
}

/**
  * @brief  将发送的数据转换为“PP位置模式”的帧格式
  *         通信类型18
  * @param  motor pointer to a RS_MotorStruct structure that contains
  *         the physical condition for the RS_Motor.
  * @retval None
  */
void RS_Motor_POS_SendFrameDataUpdate(RS_MotorStruct* motor)
{
    uint8_t i;
    uint16_t index;

    index = LOC_REF;
    motor->SetValue.ExtId.id = motor->CAN_ID;
    motor->SetValue.ExtId.data = motor->MasterID;
    motor->SetValue.ExtId.mode = RS_MotorParamWrite;
    motor->SetValue.ExtId.res = 0;
    for (i = 0; i < 8; i++)
    {
        motor->SendMessage[i] = 0;
    }
    memcpy(&motor->SendMessage[0], &index, 2);
    memcpy(&motor->SendMessage[4], &motor->SetValue.Position, 4);
}

/**
  * @brief  将发送的数据转换为“速度模式”的帧格式
  *         通信类型18
  * @param  motor pointer to a RS_MotorStruct structure that contains
  *         the physical condition for the RS_Motor.
  * @retval None
  */
void RS_Motor_VEL_SendFrameDataUpdate(RS_MotorStruct* motor)
{
    uint8_t i;
    uint16_t index;

    index = SPD_REF;
    motor->SetValue.ExtId.id = motor->CAN_ID;
    motor->SetValue.ExtId.data = motor->MasterID;
    motor->SetValue.ExtId.mode = RS_MotorParamWrite;
    motor->SetValue.ExtId.res = 0;
    for (i = 0; i < 8; i++)
    {
        motor->SendMessage[i] = 0;
    }
    memcpy(&motor->SendMessage[0], &index, 2);
    memcpy(&motor->SendMessage[4], &motor->SetValue.Velocity, 4);
}

/**
  * @brief  将发送的数据转换为“CUR模式”的帧格式
  *         通信类型18
  * @param  motor pointer to a RS_MotorStruct structure that contains
  *         the physical condition for the RS_Motor.
  * @retval None
  */
void RS_Motor_CUR_SendFrameDataUpdate(RS_MotorStruct* motor)
{
    uint8_t i;
    uint16_t index;

    index = IQ_REF;
    motor->SetValue.ExtId.id = motor->CAN_ID;
    motor->SetValue.ExtId.data = motor->MasterID;
    motor->SetValue.ExtId.mode = RS_MotorParamWrite;
    motor->SetValue.ExtId.res = 0;
    for (i = 0; i < 8; i++)
    {
        motor->SendMessage[i] = 0;
    }
    memcpy(&motor->SendMessage[0], &index, 2);
    memcpy(&motor->SendMessage[4], &motor->SetValue.Current, 4);
}

/**
  * @brief  将发送的数据转换为相应的帧格式
  * @param  motor pointer to a RS_MotorStruct structure that contains
  *         the physical condition for the RS_Motor.
  * @retval None
  */
void RS_MotorSendFrameDataUpdate(RS_MotorStruct* motor, float V_Max, float T_Max)
{
    switch (motor->MotorMode.MotorFlow)
    {
    case RS_MotorInitState:
        if (motor->MotorMode.ControlModeSet != motor->MotorMode.ControlModeFdb)
        {
            switch (motor->StateCount)
            {
            case 5: //查询控制模式，等待10次，若模式依然不对，则发送修改指令
                RS_MotorRunReadSendFrameDataUpdate(motor);
                break;
            case 10: //更改控制模式
                if (motor->ReceiveFlag != 0)
                {
                    RS_MotorModeInitSendFrameDataUpdate((RS_MotorStruct*)motor);
                    motor->ReceiveFlag = 0;
                }
                break;
            case 15: //跳转至查询
                motor->StateCount = 4;
            default: //发送空白帧
                RS_MotorModePauseSendFrameDataUpdate(motor);
                break;
            }
        }
        else if (motor->MotorMode.ZeroPositionFlag == 1)
        {
            switch (motor->StateCount)
            {
            case 15: //查询当前位置是否是零位
                RS_MotorStopSendFrameDataUpdate((RS_MotorStruct*)motor, 0);
                break;
            case 20: //设置电机机械零位
                if (motor->ReceiveFlag != 0 && motor->RawFeedback.ExtId.mode == RS_MotorFeedback)
                {
                    if (motor->RawFeedback.Position < 0.0005f && motor->RawFeedback.Position > -0.0005f)
                        motor->MotorMode.ZeroPositionFlag = 0; //满足零位要求，清除标志位
                    else
                        RS_MotorModeZeroPosSetFrameDataUpdate(motor); //设置零位
                    motor->ReceiveFlag = 0;
                }
                break;
            case 25: //跳转至查询
                motor->StateCount = 14;
            default: //发送空白帧
                RS_MotorModePauseSendFrameDataUpdate(motor);
                break;
            }
        }
        //		else if (motor->MotorMode.ZeroStateSet != motor->MotorMode.ZeroStateFdb)
        //		{
        //			switch (motor->StateCount)
        //			{
        //			case 25://查询零点模式，等待10次，若模式依然不对，则发送修改指令
        //				RS_MotorZeroReadSendFrameDataUpdate(motor);
        //				break;
        //			case 30://更改零点模式
        //				if (motor->ReceiveFlag != 0)
        //				{
        //					motor->MotorMode.FlowState = 0;
        //					RS_MotorModeZeroStaSendFrameDataUpdate((RS_MotorStruct*)motor);
        //					motor->ReceiveFlag = 0;
        //				}
        //				break;
        //			case 35://跳转至查询
        //				motor->StateCount = 24;
        //			default://发送空白帧
        //				RS_MotorModePauseSendFrameDataUpdate(motor);
        //				break;
        //			}
        //		}
        else if (motor->MotorMode.FlowState == 0)
        {
            switch (motor->StateCount)
            {
            case 35: //发送保存帧
                RS_MotorModeDataStorageSendFrameDataUpdate((RS_MotorStruct*)motor);
                break;
            case 100: //不再记数
                motor->MotorMode.FlowState = 1;
                RS_MotorStopSendFrameDataUpdate((RS_MotorStruct*)motor, 0);
            default: //发送
                RS_MotorModePauseSendFrameDataUpdate(motor);
                break;
            }
        }
        else
            RS_MotorStopSendFrameDataUpdate((RS_MotorStruct*)motor, 0);
        motor->StateCount++;
        break;
    case RS_MotorRunningState:
        if (motor->MotorMode.FlowState == 0) //电机还未使能
            RS_MotorEnableSendFrameDataUpdate((RS_MotorStruct*)motor);
        else //电机完成使能进入工作状态
        {
            switch (motor->MotorMode.ControlModeSet)
            {
            case RS_Motor_CON:
                RS_Motor_CON_SendFrameDataUpdate((RS_MotorStruct*)motor, V_Max, T_Max);
                break;
            case RS_Motor_POS:
                RS_Motor_POS_SendFrameDataUpdate((RS_MotorStruct*)motor);
                break;
            case RS_Motor_VEL:
                RS_Motor_VEL_SendFrameDataUpdate((RS_MotorStruct*)motor);
                break;
            case RS_Motor_CUR:
                RS_Motor_CUR_SendFrameDataUpdate((RS_MotorStruct*)motor);
                break;
            }
        }
        break;
    case RS_MotorResetState:
        if (motor->MotorMode.FlowState == 0) //未完成故障清除
            RS_MotorStopSendFrameDataUpdate((RS_MotorStruct*)motor, 1);
        else //完成了故障的清除
            RS_MotorStopSendFrameDataUpdate((RS_MotorStruct*)motor, 0);
        break;
    case RS_MotorErrorState:
        /* 出现故障，因此需要电机失能，但是不要清除故障 */
        RS_MotorStopSendFrameDataUpdate((RS_MotorStruct*)motor, 0);
        break;
    default:
        break;
    }
}

/**
  * @brief  更新所需要发送的数据
  * @param  motor pointer to a RS_MotorStruct structure that contains
  *         the physical condition for the RS_Motor.
  * @retval None
  */
void RS_MotorSendDataUpdate(RS_MotorStruct* motor)
{
    RS_MotorThresholdValue MV;
    RS_MotorThrValUpdate(motor->MotorMode.MotorType, &MV);

    RS_MotorSendDataCalculate(motor, MV.V_Max, MV.T_Max, MV.I_Max);
    RS_MotorSendFrameDataUpdate(motor, MV.V_Max, MV.T_Max);
}

/**
  * @brief  Calculate motor speed data
  * @param  motor pointer to a RS_MotorStruct structure that contains
  * @retval None
  */
void RS_MotorSpeedCalculate(RS_MotorStruct* motor)
{
    motor->PIDSpeed.Ref = motor->Speed.SetSpeed;
    motor->PIDSpeed.Fdb = motor->Speed.Speed;
    PidCalc(&motor->PIDSpeed);
}

/**
  * @brief  Calculate motor location data
  * @param  motor pointer to a RS_MotorStruct structure that contains
  * @retval None
  */
void RS_MotorLocationCalculate(RS_MotorStruct* motor)
{
    motor->PIDLocation.Ref = motor->Location.SetLocation;
    motor->PIDLocation.Fdb = motor->Location.Location;
    PidCalc(&motor->PIDLocation);
}

/**
  * @brief  Calculate motor speed feedforward data
  * @param  motor pointer to a RS_MotorStruct structure that contains
  * @param  interval The interval t between two SetLocation, in milliseconds
  * @retval None
  */
void RS_MotorSpeedFFCalculate(RS_MotorStruct* motor, double interval, float V_Max)
{
    double SetLocation;

    SetLocation = motor->Location.SetLocation * 2.0f * PI; //单位变为弧度

    interval = interval / 1000.0f; //单位变为秒
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
  * @param  motor pointer to a RS_MotorStruct structure that contains
  *         the physical condition for the Motor RS_Motor.
  * @param  interval The interval t between two SetLocation, in milliseconds
  * @param  if_ON Whether to keep the motor on. The value is ENABLE or DISABLE
  * @retval None
  */
void RS_MotorPIDCalculate(RS_MotorStruct* motor, double interval, uint8_t if_ON)
{
    if (motor->MotorMode.ControlModeSet == RS_Motor_CUR)
    {
        RS_MotorThresholdValue MV;
        RS_MotorThrValUpdate(motor->MotorMode.MotorType, &MV);

        switch (motor->MotorMode.CurModeSet)
        {
        case RS_Motor_NON_CUR:
        case RS_Motor_CUR_Torque:
            PidClear(&motor->PIDSpeed);
            break;
        case RS_Motor_CUR_Location:
            RS_MotorLocationCalculate(motor);
            RS_MotorSpeedFFCalculate(motor, interval, MV.V_Max);
            motor->Speed.SetSpeed = motor->PIDLocation.Out + motor->SpeedFeedForward.FeedForward;
        case RS_Motor_CUR_Speed:
            RS_MotorSpeedCalculate(motor);
        default:
            break;
        }
        if (if_ON == DISABLE)
        {
            if (motor->MotorMode.CurModeSet == RS_Motor_CUR_Location)
            {
                PidClear(&motor->PIDSpeed);
                PidClear(&motor->PIDLocation);
            }
            else if (motor->MotorMode.CurModeSet == RS_Motor_CUR_Speed)
            {
                PidClear(&motor->PIDSpeed);
            }
        }
    }
    if (if_ON == DISABLE)
    {
        motor->Location.SetLocation = motor->Location.Location;
        motor->Speed.SetSpeed = 0.0f;
        motor->SetTorque = 0.0f;
    }
}

/***************************** (C) END OF FILE ******************************/

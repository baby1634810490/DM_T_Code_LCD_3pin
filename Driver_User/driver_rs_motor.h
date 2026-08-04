//
// Created by Lenovo on 2023/12/28.
//

/**
  ******************************************************************************
  * @file    driver_rs_motor.h
  * @author  Shuai Yang
  * @brief   motor driver 20251017
  *          This file contains all the functions prototypes for the RS_Motor
  *          motor driver.
  ******************************************************************************
  * @version 20251017
  * @attention
  *
  * Don't forget the author
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DRIVER_RS_MOTOR_H
#define DRIVER_RS_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "pid.h"
#include "diff.h"
#include "convert.h"
#include "arm_math.h"

/* Exported types ------------------------------------------------------------*/
/**
  * @brief RS Motor type enumeration
  */
typedef enum
{
    MotorRS00 = 1U,

    MotorRS01 = 2U,

    MotorRS02 = 3U,

    MotorRS03 = 4U,

    MotorRS04 = 5U,

    MotorRS05 = 6U,

    MotorRS06 = 7U
} RS_MotorType_Enum;

/**
  * @brief RS_Motor Motor control mode enumeration
  */
typedef enum
{
    RS_Motor_CON = 0U, /*!< 运控模式，电机上电后默认处于运控模式，给定电机运控 5个参数，即：力矩、
                                目标位置、目标速度、Kp、Kd
                                控制过程为：
                                发送电机使能运行帧（通信类型 3）-->发送运控模式电机控制指令（通信类型 1）
                                -->收到电机反馈帧（通信类型 2）--> 发送电机停止运行帧（通信类型 4） */

    RS_Motor_POS = 1U, /*!< PP位置模式，给定电机指定的位置，电机将运行到该指定的位置
                                控制过程为：
                                发送电机模式参数写入命令（通信类型 18）设置 runmode 参数为 1 --> 发送电
                                机使能运行帧（通信类型 3）--> 发送电机模式参数写入命令（通信类型 18）设置
                                limit_spd 参数为预设最大速度指令-->发送电机模式参数写入命令（通信类型 18）
                                设置 loc_ref 参数为预设位置指令--> 发送电机停止运行帧（通信类型4） */

    RS_Motor_VEL = 2U, /*!< 速度模式，给定电机指定的运行速度
                                控制过程为：
                                发送电机模式参数写入命令（通信类型 18）设置 runmode 参数为 2 --->
                                发送电机使能运行帧（通信类型 3）--> 发送电机模式参数写入命令（通信类型 18）
                                设置 spd_ref 参数为预设速度指令--> 发送电机停止运行帧（通信类型 4） */

    RS_Motor_CUR = 3U /*!< 电流模式，给定电机指定的 Iq 电流
                                控制过程为：
                                发送电机模式参数写入命令（通信类型 18）设置 runmode 参数为 3 ---> 发送电
                                机使能运行帧（通信类型 3）--> 发送电机模式参数写入命令（通信类型 18）设置
                                iq_ref 参数为预设电流指令--> 发送电机停止运行帧（通信类型 4） */
} RS_MotorMode_Enum;

/**
  * @brief RS_Motor Motor mode of CON enumeration
  */
typedef enum
{
    RS_Motor_NON_CUR = 0U, /*!< 非电流模式 */

    RS_Motor_CUR_Torque = 1U, /*!< 电流模式下对力矩进行控制 */

    RS_Motor_CUR_Speed = 2U, /*!< 电流模式下对速度进行控制 */

    RS_Motor_CUR_Location = 3U /*!< 电流模式下对位置进行控制 */
} RS_MotorCurMode_Enum;

/**
  * @brief RS_Motor Motor operating flow enumeration
  */
typedef enum
{
    RS_MotorInitState = 0U, /*!< 电机未初始化状态 */

    RS_MotorResetState = 1U, /*!< 电机处于失能状态 */

    RS_MotorRunningState = 2U, /*!< 电机处于运行状态 */

    RS_MotorErrorState = 3U /*!< 电机出现故障 */
} RS_MotorFlow_Enum;

/**
  * @brief RS_Motor Motor mode state enumeration
  */
typedef enum
{
    RS_MotorResetMode = 0U, /*!< 复位模式 */

    RS_MotorCaliMode = 1U, /*!< 标定模式 */

    RS_MotorRunningMode = 2U, /*!< 运行模式 */
} RS_MotorModeState_Enum;

/**
  * @brief RS_Motor Motor error state structure definition
  */
typedef struct
{
    uint8_t RS_MotorUnderVoltage : 1; /*!< 电机欠压 */

    uint8_t RS_MotorDriveFailure : 1; /*!< 电机驱动故障 */

    uint8_t RS_MotorOverTemperature : 1; /*!< 过温 */

    uint8_t RS_MotorMagEncoderFault : 1; /*!< 磁编码故障 */

    uint8_t RS_MotorOverloadFault : 1; /*!< 堵转过载故障 */

    uint8_t RS_MotorUncalibrated : 1; /*!< 未标定 */

    uint8_t RS_MotorFdbError : 1; /*!< 电机反馈帧无错误，0无错误 */

    uint8_t RS_MotorErrorFdb : 1; /*!< 电机故障帧无错误，0无错误 */
} RS_MotorErrorStateStruct;

/**
  * @brief 灵足电机模式，包括所用的控制模式、电流模式下的控制模式，以及电机所处于的状态
  */
typedef struct
{
    uint8_t ControlModeFdb : 2; /*!< 控制模式反馈值 */

    uint8_t MotorModeStateFdb : 2; /*!< 电机模式状态反馈值 */

    uint8_t ZeroStateFdb : 1; /*!< 零点标志位反馈值。0代表0~2PI，1代表-PI~PI */

    uint8_t FlowState : 1; /*!< 指令流状态，1代表已经完成，0代表该状态进行中 */

    uint8_t MotorFlow : 2; /*!< 电机指令流 */

    uint8_t ControlModeSet : 2; /*!< 控制模式设定值 */

    uint8_t ZeroStateSet : 1; /*!< 零点标志位设定值。0代表0~2PI，1代表-PI~PI */

    uint8_t CurModeSet : 2; /*!< 电流模式下的控制模式设定值 */

    uint8_t MotorType : 3; /*!< 灵足电机类型 */

    uint8_t ZeroPositionFlag : 1; /*!< 零位设置标志位 */
} RS_MotorMode_Struct;

/**
  * @brief 32位扩展ID解析结构体，29位ID
  *        bit28~bit24  bit23~8  bit7~0
  *        通信类型      数据区 2  目标地址
  */
typedef struct
{
    uint32_t id : 8;
    uint32_t data : 16;
    uint32_t mode : 5;
    uint32_t res : 3;
} EXT_ID_t;

/**
  * @brief RS_Motor Motor Init structure definition
  */
typedef struct
{
    uint8_t MotorTypeInit : 3; /*!< Type of RS motor,
                                        This parameter should be MotorRS00, MotorRS01 or MotorRS02... */

    uint8_t MotorModeInit : 2; /*!< Mode of RS_Motor motor,
                                        This parameter should be RS_Motor_CON, RS_Motor_POS,
                                        RS_Motor_CUR or RS_Motor_VEL. */

    uint8_t CurModeInit : 2; /*!< Mode of RS_Motor motor in RS_Motor_CUR */

    uint8_t ZeroState : 1; /*!< Zero-point flag, 0 represents 0 to 2π, 1 represents -π to π */

    uint8_t ZeroPosition : 1; /*!< Zero-position flag, 0 represents off, 1 represents on */

    uint8_t CAN_ID; /*!< CAN of the command sent to the motor */

    uint8_t MasterID; /*!< CAN Received ID from the motor */

    float PIDSpeed_KP; /*!< PID speed KP, This parameter can be a value of @ref MOTOR_SPEED_KP */

    float PIDSpeed_AP; /*!< PID speed AP, This parameter can be a value of @ref MOTOR_SPEED_AP */

    float PIDSpeed_BP; /*!< PID speed BP, This parameter can be a value of @ref MOTOR_SPEED_BP */

    float PIDSpeed_CP; /*!< PID speed CP, This parameter can be a value of @ref MOTOR_SPEED_CP */

    float PIDSpeed_KI; /*!< PID speed KI, This parameter can be a value of @ref MOTOR_SPEED_KI */

    float PIDSpeed_KD; /*!< PID speed KD, This parameter can be a value of @ref MOTOR_SPEED_KD */

    float PIDSpeed_OutMax; /*!< PID speed out maximum, This parameter can be a number 1 */

    float PIDSpeed_UiOutMax; /*!< PID speed Integral out maximum */

    float Speed_MeanFilter_K_Init; /*!< Mean filter coefficient for speed,
                                        This parameter must be a number between Min_Data = -1 and Max_Data = 1. */

    float PIDLocation_KP; /*!< PID location KP, This parameter can be a value of @ref MOTOR_LOCATION_KP */

    float PIDLocation_OutMax; /*!< PID location out maximum, This parameter can be a number 1 */

    Diff_Formula_Enum MotorThetaDiffFormula; /*!< Differential formula type of motor theta */

    float SpeedFF_K; /*!< Speed feedforward coefficient */

    float SpeedFF_MAX; /*!< Speed feedforward max value */
} RS_MotorInitStruct;

/**
  * @brief MI Motor structure definition
  */
typedef struct
{
    RS_MotorMode_Struct MotorMode; /*!< 电机的模式 */

    uint8_t StateCount; /*!< 0-未开始初始化，5-查询控制模式，10-更改控制模式，15-查询零位标志位，
                                         20-更改零位，25-查询零点模式，30设置零点模式，35-保存设置，>100-完成保存 */

    uint8_t CAN_ID; /*!< CAN of the command sent to the motor */

    uint8_t MasterID; /*!< CAN Received ID from the motor */

    uint8_t SendMessage[8]; /*!< CAN send array */

    uint8_t ReceiveMessage[8]; /*!< CAN receive array */

    uint8_t ReceiveFlag; /*!< CAN receive array */

    RS_MotorErrorStateStruct ErrorState; /*!< Error State of RS_Motor motor. */

    struct
    {
        EXT_ID_t ExtId; /*!< 发送扩展ID设定值 */

        float Position; /*!< Position set value,单位为 rad
                                    	This parameter must be a number between Position_Min and Position_Max. */

        float Velocity; /*!< Velocity set value,单位为 rad/s
                                    	This parameter must be a number between Velocity_Min and Velocity_Max. */

        float Kp; /*!< Position Kp set value,
                                    	This parameter must be a number between Min_Data = 0.0f and Max_Data = 500.0f. */

        float Kd; /*!< Position Kd set value,
                                    	This parameter must be a number between Min_Data = 0.0f and Max_Data = 5.0f. */

        float Torque; /*!< Torque set value,
                                    	This parameter must be a number between Torque_Min and Torque_Max. */

        float Current; /*!< Current set value,
                                    	This parameter must be a number between I_MIN and I_MAX. */
    } SetValue; /*!< The set value sent to the motor */

    struct
    {
        EXT_ID_t ExtId; /*!< 反馈数据包的扩展ID */
        float Position;
        float Velocity;
        float Torque;
        float Temperature;
    } RawFeedback; /*!< Motor feedback frame data, this frame data id is Master ID */

    float LastPosition; /*!< The last position value,单位为 rad
                                    	This parameter must be a number between Position_Min and Position_Max. */

    int16_t EncoderCount; /*!< The number of motor turns */

    float Speed_array[10]; /*!< The first ten times motor rotor speed,单位为 rad/s
                                    	This parameter must be a number between Velocity_Min and Velocity_Max. */

    float MeanFilterSpeed; /*!< Motor rotor speed passing through the mean filter,单位为 rad/s
                                    	This parameter must be a number between Velocity_Min and Velocity_Max. */

    float Speed_MeanFilter_K; /*!< Mean filter coefficient,
                                        This parameter must be a number between Min_Data = -1 and Max_Data = 1. */

    float LowPassFilterSpeed; /*!< Motor rotor speed after passing through a first-order low-pass filter,单位为 rad/s
                                    	This parameter must be a number between Velocity_Min and Velocity_Max. */

    struct
    {
        float SetSpeed; /*!< Speed setting value,
                                          This parameter can be a number between Min_Data = -1 and Max_Data = 1. */

        float Speed; /*!< Speed values after normalization,
                                          This parameter can be a number between Min_Data = -1 and Max_Data = 1. */
    } Speed;

    struct
    {
        float SetLocation; /*!< Location setting value */

        float Location; /*!< Location values after normalization */
    } Location;

    float SetTorque; /*!< Torque setting value */

    FeedForward SpeedFeedForward;

    PID PIDSpeed; /*!< Speed pid struct */

    PID PIDLocation; /*!< Location pid struct */
} RS_MotorStruct;

/**
  * @brief RS Motor threshold value structure definition
  */
typedef struct
{
    float V_Max;
    float T_Max;
    float I_Max;
} RS_MotorThresholdValue;

/* Exported constants --------------------------------------------------------*/
/** @defgroup 解算发送和反馈帧的范围值
  * @{
  */
#define RS_P_MIN     (-12.57f)//rad
#define RS_P_MAX     (12.57f)//rad
#define RS_V_MIN     (-33.0f)//rad/s
#define RS_V_MAX     (33.0f)//rad/s
#define RS_KP_MIN    (0.0f)
#define RS_KP_MAX    (500.0f)
#define RS_KD_MIN    (0.0f)
#define RS_KD_MAX    (5.0f)
#define RS_T_MIN     (-14.0f)
#define RS_T_MAX     (14.0f)
#define RS_I_MAX     (27.0f)
/**
  * @}
  */

/** @defgroup 灵足电机通信类型
  * @{
  */
#define RS_MotorGetDeviceID      (0U)     /*!< 获取设备ID，包含发送帧和反馈帧 */
#define RS_MotorControlCmd       (1U)     /*!< 运控模式电机控制指令，仅包含发送帧，反馈类型 2 */
#define RS_MotorFeedback         (2U)     /*!< 电机反馈数据，仅包含反馈帧 */
#define RS_MotorEnableRunning    (3U)     /*!< 电机使能运行，仅包含发送帧，反馈类型 2 */
#define RS_MotorStopRunning      (4U)     /*!< 电机停止运行，仅包含发送帧，反馈类型 2 */
#define RS_MotorSetZeroPos       (6U)     /*!< 设置电机机械零位，仅包含发送帧，反馈类型 2 */
#define RS_MotorSetCAN_ID        (7U)     /*!< 设置电机 CAN_ID，仅包含发送帧，反馈类型 0 */
#define RS_MotorParamRead        (17U)    /*!< 单个参数读取，包含发送帧和反馈帧 */
#define RS_MotorParamWrite       (18U)    /*!< 单个参数写入，仅包含发送帧，反馈类型 2 */
#define RS_MotorFaultFeedback    (21U)    /*!< 故障反馈，仅包含反馈帧 */
#define RS_MotorDataStorage      (22U)    /*!< 电机数据保存，反馈类型 2 */
#define RS_MotorBaudModify       (23U)    /*!< 电机波特率修改，反馈类型 0 */
#define RS_MotorReport           (24U)    /*!< 电机主动上报，包含反馈帧 */
#define RS_MotorProtocolModify   (25U)    /*!< 电机协议修改，反馈类型 0 */
/**
  * @}
  */

/** @defgroup 参数表(index)，以下参数均为可读/可写
  * @{
  */
#define RUN_MODE         (0x7005)
#define IQ_REF           (0x7006)
#define SPD_REF          (0x700A)
#define LIMIT_TORQUE     (0x700B)
#define CUR_KP           (0x7010)
#define CUR_KI           (0x7011)
#define CUR_FILT_GAIN    (0x7014)
#define LOC_REF          (0x7016)
#define LIMIT_SPD        (0x7017)
#define LIMIT_CUR        (0x7018)
#define ZERO_STA         (0x7029)
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
void RS_MotorInit(RS_MotorStruct* motor, RS_MotorInitStruct motorInitData);

void RS_MotorDataReceive(RS_MotorStruct* motor, const uint8_t* CAN_Rx_Buffer, uint32_t ExtId);

void RS_MotorDataUpdate(RS_MotorStruct* motor, uint8_t if_ON);

void RS_MotorSendDataUpdate(RS_MotorStruct* motor);

void RS_MotorPIDCalculate(RS_MotorStruct* motor, double interval, uint8_t if_ON);

#endif //DRIVER_RS_MOTOR_H

/***************************** (C) END OF FILE ******************************/

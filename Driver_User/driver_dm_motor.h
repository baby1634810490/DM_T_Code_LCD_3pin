//
// Created by Lenovo on 2023/12/28.
//

/**
  ******************************************************************************
  * @file    driver_dm_motor.h
  * @author  Shuai Yang
  * @brief   This file contains all the functions prototypes for the DM 4310
  *          motor driver.
  ******************************************************************************
  * @attention
  *
  * Don't forget the author
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DRIVER_DM_MOTOR_H
#define DRIVER_DM_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "pid.h"
#include "diff.h"
#include "convert.h"
#include "arm_math.h"

/* Exported types ------------------------------------------------------------*/
/**
  * @brief DM Motor type enumeration
  */
typedef enum
{
    Motor4310_24V = 1U,

    Motor4310_48V = 2U,

    Motor4340 = 3U,

    Motor6006 = 4U,

    Motor8006 = 5U,

    Motor8009 = 6U,

    Motor10010 = 7U,

    Motor10010L = 8U
} DM_MotorType_Enum;

/**
  * @brief DM Motor mode enumeration
  */
typedef enum
{
    Motor4310_MIT = 1U, /*!< MIT模式是为了兼容原版MIT模式所设计,可以在实现无缝切换的同时,能够灵活设
                                定控制范围(P_MAX,V_MAX,T_MAX),电调将接收到的 CAN 数据转化成控制变量进
                                行运算得到扭矩值作为电流环的电流给定,电流环根据其调节规律最终达到给定的扭矩电流 */

    Motor4310_POS = 2U, /*!< 位置串级模式是采用三环串联控制的模式,位置环作为最外环,其输出作为速度环的给
                                定,而速度环的输出作为内环电流环的给定,用以控制实际的电流输出 */

    Motor4310_VEL = 3U /*!< 速度模式能让电机稳定运行在设定的速度 */
} DM_MotorMode_Enum;

/**
  * @brief DM Motor mode of MIT enumeration
  */
typedef enum
{
    Motor4310_MIT_Torque = 1U, /*!< MIT模式下对力矩进行控制 */

    Motor4310_MIT_Speed = 2U, /*!< MIT模式下对速度进行控制 */

    Motor4310_MIT_Location = 3U /*!< MIT模式下对位置进行控制 */
} DM_MotorMitMode_Enum;

/**
  * @brief DM Motor state enumeration
  */
typedef enum
{
    Motor4310_Disable = 0U, /*!< 电机初始状态，没有被使能 */

    Motor4310_Enable = 1U, /*!< 电机没有异常状态，已经使能并可以正常工作 */

    Motor4310_Exit = 2U, /*!< 关闭电机 */

    Motor4310_OverVoltage = 0x8, /*!< 电机超压，电压高于32V，过压将退出“使能模式” */

    Motor4310_UnderVoltage = 0x9, /*!< 电机欠压，电压低于15V，欠压将退出“使能模式” */

    Motor4310_OverCurrent = 0xA, /*!< 电机过电流，电流高于9.8A，过流将退出“使能模式” */

    Motor4310_MosOverTemperature = 0xB, /*!< MOS过温，温度大于120℃将退出“使能模式” */

    Motor4310_CoilOverTemperature = 0xC, /*!< 线圈过温，温度大于120℃将退出“使能模式” */

    Motor4310_CommunicationLos = 0xD, /*!< 通讯丢失防护，设定周期内没有收到 CAN 指令将自动退出“使能模式” */

    Motor4310_Overload = 0xE /*!< 电机过载 */
} DM_MotorState_Enum;

/**
  * @brief DM Motor Init structure definition
  */
typedef struct
{
    DM_MotorType_Enum MotorTypeInit; /*!< Type of DM motor,
                                            This parameter should be Motor4310, Motor4030 or Motor6006... */

    DM_MotorMode_Enum MotorModeInit; /*!< Mode of DM motor,
                                            This parameter should be Motor4310_MIT, Motor4310_POS or Motor4310_VEL. */

    DM_MotorMitMode_Enum MitModeInit; /*!< Mode of 4310 motor in Motor4310_MIT */

    uint32_t CAN_ID; /*!< CAN of the command sent to the motor */

    uint32_t MasterID; /*!< CAN Received ID from the motor */

    float PIDSpeed_KP; /*!< PID speed KP, This parameter can be a value of @ref MOTOR_SPEED_KP */

    float PIDSpeed_AP; /*!< PID speed AP, This parameter can be a value of @ref MOTOR_SPEED_AP */

    float PIDSpeed_BP; /*!< PID speed BP, This parameter can be a value of @ref MOTOR_SPEED_BP */

    float PIDSpeed_CP; /*!< PID speed CP, This parameter can be a value of @ref MOTOR_SPEED_CP */

    float PIDSpeed_KI; /*!< PID speed KI, This parameter can be a value of @ref MOTOR_SPEED_KI */

    float PIDSpeed_KD; /*!< PID speed KD, This parameter can be a value of @ref MOTOR_SPEED_KD */

    float PIDSpeed_OutMax; /*!< PID speed out maximum, This parameter can be a number 1 */

    float PIDSpeed_UiOutMax; /*!< PID speed Integral out maximum */

    float Speed_LowPassFilter_K; /*!< Mean filter coefficient for speed,
                                        This parameter must be a number between Min_Data = -1 and Max_Data = 1. */

    float PIDLocation_KP; /*!< PID location KP, This parameter can be a value of @ref MOTOR_LOCATION_KP */

    float PIDLocation_OutMax; /*!< PID location out maximum, This parameter can be a number 1 */

    Diff_Formula_Enum MotorThetaDiffFormula; /*!< Differential formula type of motor theta */

    float SpeedFF_K; /*!< Speed feedforward coefficient */

    float SpeedFF_MAX; /*!< Speed feedforward max value */
} DM_MotorInitStruct;

/**
  * @brief DM Motor structure definition
  */
typedef struct
{
    DM_MotorType_Enum MotorType; /*!< Type of DM motor. */

    DM_MotorState_Enum MotorState; /*!< State of DM motor. */

    DM_MotorMode_Enum MotorMode; /*!< Mode of DM motor,
                                        This parameter should be Motor4310_MIT, Motor4310_POS or Motor4310_VEL. */

    DM_MotorMitMode_Enum MitMode; /*!< Mode of 4310 motor in Motor4310_MIT */

    uint8_t EnableCount; /*!< The number of times an enable message is sent when entering the motor */

    uint32_t CAN_ID; /*!< CAN of the command sent to the motor */

    uint32_t MasterID; /*!< CAN Received ID from the motor */

    uint8_t SendDataLen; /*!< The length of the data frame that CAN send */

    uint8_t SendMessage[8]; /*!< CAN send array */

    uint8_t ReceiveMessage[8]; /*!< CAN receive array */

    uint8_t ReceiveFlag; /*!< CAN receive flag,
                                        receiving unprocessed data is ENABLE, otherwise it is DISABLE */

    struct
    {
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
    } SetValue; /*!< The set value sent to the motor */

    struct
    {
        uint8_t State;
        uint8_t CAN_ID;
        float Position;
        float Velocity;
        float Torque;
        float MosTemperature;
        float CoilTemperature;
    } RawFeedback; /*!< Motor feedback frame data, this frame data id is Master ID */

    float LastPosition; /*!< The last position value,单位为 rad
                                    	This parameter must be a number between Position_Min and Position_Max. */

    int16_t EncoderCount; /*!< The number of motor turns */

    float Speed_array[10]; /*!< The first ten times motor rotor speed,单位为 rad/s
                                    	This parameter must be a number between Velocity_Min and Velocity_Max. */

    float MeanFilterSpeed; /*!< Motor rotor speed passing through the mean filter,单位为 rad/s
                                    	This parameter must be a number between Velocity_Min and Velocity_Max. */

    float Speed_LowPassFilter_K; /*!< LowPassFilter coefficient,
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
} DM_MotorStruct;

/**
  * @brief DM Motor threshold value structure definition
  */
typedef struct
{
    float P_Max;
    float V_Max;
    float T_Max;
} DM_MotorThresholdValue;

/* Exported constants --------------------------------------------------------*/
#define MOTOR4310_DECELERATION_RATIO (10.0f / 1.0f)  /* 4310减速比 */

#define MOTOR4310_RATED_SPEED (120.0f)  /* 4310额定速度，单位：rpm */

/* 以下参数来源于 4310 电机手册，用于帧格式的转换 */
#define DM_P_MIN     (-12.5f)
#define DM_P_MAX     (12.5f)
#define DM_V_MIN     (-30.0f)
#define DM_V_MAX     (30.0f)
#define DM_KP_MIN    (0.0f)
#define DM_KP_MAX    (500.0f)
#define DM_KD_MIN    (0.0f)
#define DM_KD_MAX    (5.0f)
#define DM_T_MIN     (-10.0f)
#define DM_T_MAX     (10.0f)

/* Exported functions --------------------------------------------------------*/
void DM_MotorInit(DM_MotorStruct* motor, DM_MotorInitStruct motorInitData);

void DM_MotorDataReceive(DM_MotorStruct* motor, const uint8_t* CAN_Rx_Buffer);

void DM_MotorDataUpdate(DM_MotorStruct* motor, uint8_t if_ON);

void DM_MotorSendDataUpdate(DM_MotorStruct* motor);

void DM_MotorPIDCalculate(DM_MotorStruct* motor, double interval, uint8_t if_ON);

#endif //DRIVER_DM_MOTOR_H

/***************************** (C) END OF FILE ******************************/

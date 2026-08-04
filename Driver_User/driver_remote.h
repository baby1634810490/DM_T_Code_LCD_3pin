//
// Created by Lenovo on 2025/1/6.
//

/**
  ******************************************************************************
  * @file    driver_remote.h
  * @author  Shuai Yang
  * @brief   This file contains all the functions prototypes for the remote
  *          driver.
  ******************************************************************************
  * @attention
  *
  * Please add comments after adding or deleting functions to ensure code
  * specification.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DRIVER_REMOTE_H
#define DRIVER_REMOTE_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/
/**
  * @brief BetaFPV Stick 3 state enumeration
  */
typedef enum
{
    SW_Up3 = 0U,
    SW_Mid3 = 1U,
    SW_Down3 = 2U
} Switch_3_Enum;

/**
  * @brief BetaFPV Stick 2 state enumeration
  */
typedef enum
{
    SW_Up2 = 3U,
    SW_Down2 = 4U
} Switch_2_Enum;

/**
  * @brief Receiver channel data structure definition
  */
typedef struct Crsf_Data
{
    uint16_t CH1;
    uint16_t CH2;
    uint16_t CH3;
    uint16_t CH4;
    uint16_t CH5;
    uint16_t CH6;
    uint16_t CH7;
    uint16_t CH8;
    uint16_t CH9;
    uint16_t CH10;
    uint16_t CH11;
    uint16_t CH12;
    uint16_t CH13;
    uint16_t CH14;
    uint16_t CH15;
    uint16_t CH16;
} CrsfData_t;

/**
  * @brief Remote data structure definition
  */
typedef struct
{
    float LX;
    float LY;
    float RX;
    float RY;
    Switch_3_Enum L_SW3;
    Switch_3_Enum R_SW3;
    Switch_2_Enum L_SW2;
    Switch_2_Enum R_SW2;
} RemoteDataPortStruct;

/**
  * @brief Robot Control data structure definition
  */
typedef struct
{
    float X_Set;
    float Y_Set;
    float Z_Set;
    float Spin_Set;
} RobotCtlPortStruct;

/* Exported constants --------------------------------------------------------*/
#define CH_VALUE_MAX    (1811U)
#define CH_VALUE_MIN    (174U)
#define CH_VALUE_OFFSET (992.5f)
#define SW_UP   (1792U)
#define SW_MID  (997U)
#define SW_DOWN (191U)

/* Exported variables --------------------------------------------------------*/
extern uint8_t remoteData[50];

extern CrsfData_t CrsfData;

extern RemoteDataPortStruct RobotRemote;

extern RobotCtlPortStruct RobotControl;

/* Exported functions --------------------------------------------------------*/
uint8_t CrsfDataUpdate(const uint8_t* data, CrsfData_t* crsfData);

void RemoteDataUpdate(CrsfData_t crsfData, RemoteDataPortStruct* RemoteDataStruct);

void RobotCtlDataUpdate(RemoteDataPortStruct RemoteDataStruct, RobotCtlPortStruct* RobotCtl);

#endif //DRIVER_REMOTE_H

/************************ (C) COPYRIGHT XJTU ROBOMASTER ********END OF FILE****/

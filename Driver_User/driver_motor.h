//
// Created by Lenovo on 2025/4/18.
//

#ifndef DRIVER_MOTOR_H
#define DRIVER_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "bsp_fdcan.h"
#include "bsp_pwm.h"
#include "driver_dm_motor.h"
#include "driver_rs_motor.h"

/* Exported types ------------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/
/** @defgroup 电机的参数
  * @{
  */
#define RS_Motor04_SPEED_KP    (0.0f)
#define RS_Motor04_SPEED_AP    (1.45f)
#define RS_Motor04_SPEED_BP    (2.6f)
#define RS_Motor04_SPEED_CP    (0.1f)
#define RS_Motor04_SPEED_KI    (0.01f)
#define RS_Motor04_SPEED_KD    (0.0f)
#define RS_Motor04_SPEED_OUTMAX    (0.6f)
#define RS_Motor04_SPEED_UIOUTMAX    (0.2f)
#define RS_Motor04_LOCATION_KP    (4.50f)
#define RS_Motor04_LOCATION_OUTMAX    (0.1f)
#define RS_Motor04_SPEED_FF_K    (0.0f)
#define RS_Motor04_SPEED_FF_MAX    (0.10f)

#define RS_Motor00_SPEED_KP    (0.0f)
#define RS_Motor00_SPEED_AP    (4.00f)
#define RS_Motor00_SPEED_BP    (2.6f)
#define RS_Motor00_SPEED_CP    (0.1f)
#define RS_Motor00_SPEED_KI    (0.015f)
#define RS_Motor00_SPEED_KD    (0.0f)
#define RS_Motor00_SPEED_OUTMAX    (0.6f)
#define RS_Motor00_SPEED_UIOUTMAX    (0.01f)
#define RS_Motor00_LOCATION_KP    (1.5f)
#define RS_Motor00_LOCATION_OUTMAX    (0.1f)
#define RS_Motor00_SPEED_FF_K    (0.0f)
#define RS_Motor00_SPEED_FF_MAX    (0.20f)
/**
  * @}
  */

#define FIR_JOINT_LOCATION_MAX    (1.00450f)
#define FIR_JOINT_LOCATION_MIN    (0.449f)

/* Exported variables --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void MotorInit(void);

void MotorDataUpdate(uint8_t safeFlag);

void MotorSettingUpdate(void);

void MotorCalculate(uint8_t safeFlag);

void MotorCANSend(void);

#endif //DRIVER_MOTOR_H

/***************************** (C) END OF FILE ******************************/

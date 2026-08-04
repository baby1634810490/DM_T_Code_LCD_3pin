//
// Created by Lenovo on 2024/10/11.
//

/**
  ******************************************************************************
  * @file    pid.h
  * @author  XJTU ROBOMASTER Team
  * @brief   pid计算
  ******************************************************************************
  * @attention
  *
  * Don't forget the author
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PID_H
#define PID_H

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    float Ref; // Input: Reference input
    float Fdb; // Input: Feedback input
    float Err; // Variable: Error

    float Kp; // Parameter: Proportional gain
    float Ki; // Parameter: Integral gain
    float Kd; // Parameter: Derivative gain

    float Ap;
    float Bp;
    float Cp;
    float Ai;
    float Ci;
    float Ad;
    float Bd;
    float Cd;
    float Kl;
    float ErrRef;


    float Up; // Variable: Proportional output
    float Ui; // Variable: Integral output
    float Ud; // Variable: Derivative output
    float OutPreSat; // Variable: Pre-saturated output
    float OutMax; // Parameter: Maximum output
    float UiOutMax; // Parameter: Maximum Ui output
    float Out; // Output: PID output
    float SatErr; // Variable: Saturated difference
    float Kc; // Parameter: Integral correction gain
    float Up1; // History: Previous proportional output
} PID;

/* Exported functions --------------------------------------------------------*/
void PidCalc(PID* v);

void PidClear(PID* v);

void PidCalc2(PID* v);

#endif //PID_H

/************************ (C) COPYRIGHT XJTU ROBOMASTER ********END OF FILE****/

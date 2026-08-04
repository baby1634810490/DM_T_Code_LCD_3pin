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

/* Includes ------------------------------------------------------------------*/
#include "pid.h"
#include "math.h"

//将PID函数放在RAM中
#pragma arm section code = "RAMCODE"

/* ----------------------- Function Implements ---------------------------- */
/**
  * @brief  PID计算
  * @param  v pointer to a PID structure.
  * @retval None
  */
void PidCalc(PID* v)
{
    /* 变结构PID，若Ap、Bp、Cp不为0，则计算Kp */
    if (fabsf(v->Ap) >= 0.0001f || fabsf(v->Bp) >= 0.0001f || fabsf(v->Cp) >= 0.0001f)
        v->Kp = v->Ap + v->Bp * (1 - expf(-v->Cp * fabsf(v->Err)));

    // Compute the error
    v->Err = v->Ref - v->Fdb;
    // Compute the proportional output
    v->Up = v->Kp * v->Err;
    // Compute the integral output
    v->Ui = v->Ui + v->Ki * v->Up + v->Kc * v->SatErr;
    // Compute the derivative output
    v->Ud = v->Kd * (v->Up - v->Up1);
    // Compute the pre-saturated output
    v->OutPreSat = v->Up + v->Ui + v->Ud;
    /* 积分限幅 */
    if (v->Ui > v->UiOutMax)
    {
        v->Ui = v->UiOutMax;
    }
    if (v->Ui < ((-1) * v->UiOutMax))
    {
        v->Ui = (-1) * v->UiOutMax;
    }

    // Saturate the output
    if (v->OutPreSat > v->OutMax)
        v->Out = v->OutMax;
    else if (v->OutPreSat < ((-1) * v->OutMax))
        v->Out = (-1) * v->OutMax;
    else
        v->Out = v->OutPreSat;

    // Compute the saturate difference
    v->SatErr = v->Out - v->OutPreSat;
    // Update the previous proportional output
    v->Up1 = v->Up;
}

/**
  * @brief  PID计算
  * @param  v pointer to a PID structure.
  * @retval None
  */
void PidCalc2(PID* v)
{
    // Compute the error
    v->Err = v->Ref - v->Fdb;


    //算Kp
    v->Kp = v->Ap + v->Bp * (1 - expf(-v->Cp * fabsf(v->Err)));
    //算Ki
    v->Ki = v->Ai * expf(-v->Ci * fabsf(v->Err));
    //算Kd
    v->Kd = v->Ad - v->Bd * (1 - expf(-v->Cd * fabsf(v->Err)));

    // Compute the proportional output
    v->Up = v->Kp * v->Err;
    // Compute the integral output
    //	v->Ui = v->Ui + v->Ki*v->Up + v->Kc*v->SatErr;

    //积分分离PID控制算法
    if (fabsf(v->Err) > v->ErrRef)
    {
        v->Kl = 0.0f;
        v->Ui = 0.0f;
    }
    else
        v->Kl = 1.0f;

    v->Ui = v->Ui + v->Kl * v->Ki * v->Up + v->Kc * v->SatErr;

    if ((v->Err > 0 && v->Ui > 0) || (v->Err < 0 && v->Ui < 0))
    {
        //误差限额
        if (v->Ui > 0.2f)
            v->Ui = 0.2f;
        if (v->Ui < -0.2f)
            v->Ui = -0.2f;
    }
    else
        v->Ui = 0;

    // Compute the derivative output
    v->Ud = v->Kd * (v->Up - v->Up1);
    // Compute the pre-saturated output
    v->OutPreSat = v->Up + v->Ui + v->Ud;


    // Saturate the output
    if (v->OutPreSat > v->OutMax)
        v->Out = v->OutMax;
    else if (v->OutPreSat < ((-1) * v->OutMax))
        v->Out = (-1) * v->OutMax;
    else
        v->Out = v->OutPreSat;

    // Compute the saturate difference
    v->SatErr = v->Out - v->OutPreSat;
    // Update the previous proportional output
    v->Up1 = v->Up;
}

/**
  * @brief  PID清零
  * @param  v pointer to a PID structure.
  * @retval None
  */
void PidClear(PID* v)
{
    /* 将所有输出清零 */
    v->Ui = 0;
    v->Ud = 0;
    v->Up = 0;
    v->OutPreSat = 0;
    v->Out = 0;
}

#pragma arm section

/************************ (C) COPYRIGHT XJTU ROBOMASTER ********END OF FILE****/

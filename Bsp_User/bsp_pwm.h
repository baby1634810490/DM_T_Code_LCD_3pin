//
// Created by Lenovo on 2025/5/24.
//

/**
  ******************************************************************************
  * @file    bsp_pwm.h
  * @author  Shuai Yang
  * @brief   这个文件提供函数对 PWM 的输出设置
  ******************************************************************************
  * @attention
  *
  * Don't forget the author
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_PWM_H
#define BSP_PWM_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported constants --------------------------------------------------------*/
#define BEEP    (TIM12->CCR2)//取值从0-1999

/* Exported functions --------------------------------------------------------*/
void PWM_Init(void);

void BeepSet(uint16_t loudness);

#endif //BSP_PWM_H

/***************************** (C) END OF FILE ******************************/

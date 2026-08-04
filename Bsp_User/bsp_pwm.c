//
// Created by Lenovo on 2025/5/24.
//

/**
  ******************************************************************************
  * @file    bsp_pwm.c
  * @author  Shuai Yang
  * @brief   bsp pwm
  *          这个文件提供函数对 PWM 的输出设置
  *           + PWM Init
  *           + PWM ARR SET
  *
  @verbatim
  ==============================================================================

  @endverbatim
  ******************************************************************************
  * @attention
  *
  * Don't forget the author
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_pwm.h"

/* Exported variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim12;

/* ----------------------- Function Implements ---------------------------- */
/**
  * @brief  对 PWM 进行初始化
  * @retval None.
  */
void PWM_Init(void)
{
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
}

/**
  * @brief  设置蜂鸣器的响度
  * @param  loudness 响度的大小
  * @retval None.
  */
void BeepSet(uint16_t loudness)
{
    BEEP = loudness;
}

/***************************** (C) END OF FILE ******************************/

//
// Created by Lenovo on 2025/5/24.
//

/**
  ******************************************************************************
  * @file    driver_power.c
  * @author  Shuai Yang
  * @brief   power driver.
  *          这个文件提供函数以实现可控电源的开关:
  *           + 打开电源
  *           + 关闭电源
  *
  ******************************************************************************
  * @attention
  *
  * 这个文件使用设置 IO 引脚的高低电平，控制可控电源的通断
  *
  * Don't forget the author
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "driver_power.h"

/* ------------------------ Function Implements ----------------------------- */
/**
  * @brief  打开电源 1 ，将 xt30 朝向自己，左侧的电源接口即为电源 1
  * @retval None
  */
void PowerOut1TurnON(void)
{
    HAL_GPIO_WritePin(POWER_OUT1_GPIO_Port, POWER_OUT1_Pin, GPIO_PIN_SET);
}

/**
  * @brief  关闭电源 1 ，将 xt30 朝向自己，左侧的电源接口即为电源 1
  * @retval None
  */
void PowerOut1TurnOFF(void)
{
    HAL_GPIO_WritePin(POWER_OUT1_GPIO_Port, POWER_OUT1_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  打开电源 2 ，将 xt30 朝向自己，右侧的电源接口即为电源 2
  * @retval None
  */
void PowerOut2TurnON(void)
{
    HAL_GPIO_WritePin(POWER_OUT2_GPIO_Port, POWER_OUT2_Pin, GPIO_PIN_SET);
}

/**
  * @brief  关闭电源 2 ，将 xt30 朝向自己，右侧的电源接口即为电源 2
  * @retval None
  */
void PowerOut2TurnOFF(void)
{
    HAL_GPIO_WritePin(POWER_OUT2_GPIO_Port, POWER_OUT2_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  打开电源 5V ，该电源包含 4 个排针供电、串口10、串口7、CAN3
  * @retval None
  */
void PowerOut5VTurnON(void)
{
    HAL_GPIO_WritePin(POWER_OUT5V_GPIO_Port, POWER_OUT5V_Pin, GPIO_PIN_SET);
}

/**
  * @brief  关闭电源 5V ，该电源包含 4 个排针供电、串口10、串口7、CAN3
  * @retval None
  */
void PowerOut5VTurnOFF(void)
{
    HAL_GPIO_WritePin(POWER_OUT5V_GPIO_Port, POWER_OUT5V_Pin, GPIO_PIN_RESET);
}

/***************************** (C) END OF FILE ******************************/

//
// Created by Lenovo on 2024/10/11.
//

/**
  ******************************************************************************
  * @file    convert.c
  * @author  达妙科技
  * @brief   这个文件提供函数以完成浮点数和整数的相互转换，来自于达妙电机例程
  *           + float_to_uint
  *           + uint_to_float
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
#include "convert.h"

/* ------------------------ Function Implements ----------------------------- */
/**
  * @brief  将浮点数转换为无符号整数
  * @param  x 要转换的浮点数
  * @param  x_min 浮点数的最小值
  * @param  x_max 浮点数的最大值
  * @param  bits 无符号整数的位数
  * @retval None.
  */
int float_to_uint(float x, float x_min, float x_max, int bits)
{
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max) x = x_max;
    else if (x < x_min) x = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
  * @brief  采用浮点数据等比例转换成整数
  * @param  x_int 要转换的无符号整数
  * @param  x_min 目标浮点数的最小值
  * @param  x_max 目标浮点数的最大值
  * @param  bits 无符号整数的位数
  * @retval None.
  */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/***************************** (C) END OF FILE ******************************/

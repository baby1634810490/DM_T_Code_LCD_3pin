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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CONVERT_H
#define CONVERT_H

/* Exported functions --------------------------------------------------------*/
int float_to_uint(float x, float x_min, float x_max, int bits);

float uint_to_float(int x_int, float x_min, float x_max, int bits);

#endif //CONVERT_H

/***************************** (C) END OF FILE ******************************/

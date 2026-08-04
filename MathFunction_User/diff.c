//
// Created by Lenovo on 2024/10/11.
//

/**
  ******************************************************************************
  * @file    diff.c
  * @author  Shuai Yang
  * @brief   微分
  *          This file provides functions to solve for the derivative with respect to time:
  *           + Initialization function
  *           + Calculate data function
  *
  @verbatim
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================
    [..]
      (#) Initialize the diff by implementing the DiffInit():
          (++) Initialize the diff parameters, including:
		       (+++) 微分公式
		       (+++) 初始值

      (#) Solve for the derivative with respect to time by implementing the DiffCalc():
          (++) 计算对时间的导数, including:
		       (+++) 两点
		       (+++) 三点
		       (+++) 四点
		       (+++) 五点
		       (+++) 六点

  @endverbatim
  ******************************************************************************
  * @attention
  *
  * 这个文件包含两点、三点、四点、五点的数值微分，对时间求导
  *
  * Don't forget the author
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "diff.h"

/* ----------------------- Function Implements ---------------------------- */
/**
  * @brief  Initialize the motor parameters
  * @param  ft pointer to a DIFF structure that contains
  *         the raw data and diff data.
  * @param  RawValue The latest value of the f(t)
  * @param  interval The interval t between two values
  * @retval None
  */
void DiffCalc(DIFF* ft, double RawValue, double interval)
{
    switch (ft->Formula)
    {
    case Two_point_differential:
        ft->RawData.Value1 = ft->RawData.Value0;
        ft->RawData.Value0 = RawValue;
        ft->DiffData.Value0 = (ft->RawData.Value0 - ft->RawData.Value1) / interval;
        ft->diffValue = ft->DiffData.Value0;
        break;
    case Three_point_differential:
        ft->RawData.Value2 = ft->RawData.Value1;
        ft->RawData.Value1 = ft->RawData.Value0;
        ft->RawData.Value0 = RawValue;
        ft->DiffData.Value0 =
            (3.0f * ft->RawData.Value0 - 4.0f * ft->RawData.Value1 + ft->RawData.Value2) / (2.0f * interval);
        ft->DiffData.Value1 = (ft->RawData.Value0 - ft->RawData.Value2) / (2.0f * interval);
        ft->DiffData.Value2 =
            (-1.0f * ft->RawData.Value0 + 4.0f * ft->RawData.Value1 - 3.0f * ft->RawData.Value2) /
            (2.0f * interval);
        ft->diffValue = ft->DiffData.Value2;
        break;
    case Four_point_differential:
        ft->RawData.Value3 = ft->RawData.Value2;
        ft->RawData.Value2 = ft->RawData.Value1;
        ft->RawData.Value1 = ft->RawData.Value0;
        ft->RawData.Value0 = RawValue;
        ft->DiffData.Value0 = (11.0f * ft->RawData.Value0 - 18.0f * ft->RawData.Value1 + 9.0 * ft->RawData.Value2 -
            2.0f * ft->RawData.Value3) / (6.0f * interval);
        ft->DiffData.Value1 = (2.0f * ft->RawData.Value0 + 3.0f * ft->RawData.Value1 - 6.0 * ft->RawData.Value2 +
            1.0f * ft->RawData.Value3) / (6.0f * interval);
        ft->DiffData.Value2 = (-1.0f * ft->RawData.Value0 + 6.0f * ft->RawData.Value1 - 3.0 * ft->RawData.Value2 -
            2.0f * ft->RawData.Value3) / (6.0f * interval);
        ft->DiffData.Value3 = (2.0f * ft->RawData.Value0 - 9.0f * ft->RawData.Value1 + 18.0 * ft->RawData.Value2 -
            11.0f * ft->RawData.Value3) / (6.0f * interval);
        ft->diffValue = ft->DiffData.Value3;
        break;
    case Five_point_differential:
        ft->RawData.Value4 = ft->RawData.Value3;
        ft->RawData.Value3 = ft->RawData.Value2;
        ft->RawData.Value2 = ft->RawData.Value1;
        ft->RawData.Value1 = ft->RawData.Value0;
        ft->RawData.Value0 = RawValue;
        ft->diffValue = ft->DiffData.Value4;
        break;
    case Six_point_differential:
        ft->RawData.Value5 = ft->RawData.Value4;
        ft->RawData.Value4 = ft->RawData.Value3;
        ft->RawData.Value3 = ft->RawData.Value2;
        ft->RawData.Value2 = ft->RawData.Value1;
        ft->RawData.Value1 = ft->RawData.Value0;
        ft->RawData.Value0 = RawValue;
        ft->diffValue = ft->DiffData.Value5;
        break;
    default:
        ft->RawData.Value1 = ft->RawData.Value0;
        ft->RawData.Value0 = RawValue;
        ft->DiffData.Value0 = (ft->RawData.Value0 - ft->RawData.Value1) / interval;
        ft->diffValue = ft->DiffData.Value0;
    }
}

/***************************** (C) END OF FILE ******************************/

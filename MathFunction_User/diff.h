//
// Created by Lenovo on 2024/10/11.
//

/**
  ******************************************************************************
  * @file    diff.h
  * @author  Shuai Yang
  * @brief   包含对数据微分的两点、三点、四点、五点、六点公式，求解对时间的导数
  ******************************************************************************
  * @attention
  *
  * Don't forget the author
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DIFF_H
#define DIFF_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/
/**
  * @brief Differential formula type enumeration
  */
typedef enum
{
    Two_point_differential = 2U, /*!< 两点微分公式 */
    Three_point_differential = 3U, /*!< 三点微分公式 */
    Four_point_differential = 4U, /*!< 四点微分公式 */
    Five_point_differential = 5U, /*!< 五点微分公式 */
    Six_point_differential = 6U /*!< 六点微分公式 */
} Diff_Formula_Enum;

/**
  * @brief  diff structure definition
  */
typedef struct
{
    struct
    {
        double Value0; /*!< 横轴上最右边变量对应的f(t)的函数值 */

        double Value1; /*!< Value0左移t处变量对应的f(t)的函数值 */

        double Value2; /*!< Value0左移2t处变量对应的f(t)的函数值 */

        double Value3; /*!< Value0左移3t处变量对应的f(t)的函数值 */

        double Value4; /*!< Value0左移4t处变量对应的f(t)的函数值 */

        double Value5; /*!< Value0左移5t处变量对应的f(t)的函数值 */
    } RawData; /*!< 函数值f(t),一般是x，即转角或者位移 */

    struct
    {
        double Value0; /*!< 横轴上最右边变量对应的导数df/dt的函数值 */

        double Value1; /*!< Value0左移t处变量对应的导数df/dt的函数值 */

        double Value2; /*!< Value0左移2t处变量对应的导数df/dt的函数值 */

        double Value3; /*!< Value0左移3t处变量对应的导数df/dt的函数值 */

        double Value4; /*!< Value0左移4t处变量对应的导数df/dt的函数值 */

        double Value5; /*!< Value0左移5t处变量对应的导数df/dt的函数值 */
    } DiffData; /*!< 一阶导数值df/dt,一般是v，即转速或者速度 */

    double diffValue; /*!< 微分结果值 */

    Diff_Formula_Enum Formula; /*!< 求导时所用的微分公式 */
} DIFF;

/**
  * @brief  FeedForward structure definition
  */
typedef struct
{
    float FeedForward; /*!< Feedforward values after normalization,
                                This parameter can be a number between Min_Data = -1 and Max_Data = 1. */

    float FeedForward_K; /*!< Feedforward coefficient */

    float FeedForward_MAX; /*!< Feedforward max value */

    DIFF Diff; /*!< Differential struct of speed */
} FeedForward;

/* Exported functions --------------------------------------------------------*/
void DiffCalc(DIFF* fx, double RawValue, double interval);

#endif //DIFF_H

/***************************** (C) END OF FILE ******************************/

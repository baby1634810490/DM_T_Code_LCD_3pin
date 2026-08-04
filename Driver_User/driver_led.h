//
// Created by Lenovo on 2025/5/20.
//

/**
  ******************************************************************************
  * @file    driver_led.h
  * @author  Shuai Yang
  * @brief   这个文件提供函数以实现 led 的驱动
  *
  ******************************************************************************
  * @attention
  *
  * Don't forget the author
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DRIVER_LED_H
#define DRIVER_LED_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/
/**
  * @brief LED color enumeration
  */
typedef enum
{
    Red = 0U,
    Green = 1U,
    Blue = 2U,
    White = 3U,
    Black = 4U,
    Cyan = 5U,
    Yellow = 6U,
    Purple = 7U
} LED_Color_Enum;

/* Exported constants --------------------------------------------------------*/
#define WS2812_LowLevel    0xC0     // 0码
#define WS2812_HighLevel   0xF0     // 1码

/* Exported functions --------------------------------------------------------*/
void LED_Light_Up(uint8_t color);

#endif //DRIVER_LED_H

/***************************** (C) END OF FILE ******************************/

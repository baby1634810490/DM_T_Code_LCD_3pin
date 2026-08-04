//
// Created by Lenovo on 2025/5/20.
//

/**
  ******************************************************************************
  * @file    driver_led.c
  * @author  Shuai Yang
  * @brief   dm-mc02 led driver.
  *          这个文件提供函数以实现 led 的驱动：
  *           + LED 点灯函数
  *
  @verbatim
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================
    [..]
      (#) 设置灯 WS2812 三个灯珠的亮度，通过 WS2812_Ctrl():
          (++) Initialize the led parameters, including:
		       (+++) r
		       (+++) g
		       (+++) b

      (#) 设置灯的颜色，通过 LED_Light_Up():
          (++) Initialize the led parameters, including:
		       (+++) color

  @endverbatim
  ******************************************************************************
  * @attention
  *
  * 这个文件使用 SPI 通讯实现对灯的颜色和亮度的控制，函数基本来自例程
  *
  * Don't forget the author
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "driver_led.h"

/* Exported variables --------------------------------------------------------*/
extern SPI_HandleTypeDef hspi6;

/* ------------------------ Function Implements ----------------------------- */
/**
  * @brief  WS2812彩灯控制程序
  * @param  r 红灯亮度，0-255
  * @param  g 绿灯亮度，0-255
  * @param  b 蓝灯亮度，0-255
  * @retval None
  */
void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t txbuf[24];
    uint8_t res = 0;
    for (int i = 0; i < 8; i++)
    {
        txbuf[7 - i] = (((g >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
        txbuf[15 - i] = (((r >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
        txbuf[23 - i] = (((b >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
    }
    HAL_SPI_Transmit(&hspi6, &res, 0, 0xFFFF);
    while (hspi6.State != HAL_SPI_STATE_READY);
    HAL_SPI_Transmit(&hspi6, txbuf, 24, 0xFFFF);
    for (int i = 0; i < 100; i++)
    {
        HAL_SPI_Transmit(&hspi6, &res, 1, 0xFFFF);
    }
}

/**
  * @brief  设置灯的颜色和亮度
  * @param  color 灯的颜色
  * @retval None
  */
void LED_Light_Up(uint8_t color)
{
    uint8_t r, g, b;
    switch (color)
    {
    case Red:
        r = 255;
        g = 0;
        b = 0;
        break;
    case Green:
        r = 0;
        g = 255;
        b = 0;
        break;
    case Blue:
        r = 0;
        g = 0;
        b = 255;
        break;
    case White:
        r = 255;
        g = 255;
        b = 255;
        break;
    case Black:
        r = 0;
        g = 0;
        b = 0;
        break;
    case Cyan:
        r = 0;
        g = 255;
        b = 255;
        break;
    case Yellow:
        r = 255;
        g = 255;
        b = 0;
        break;
    case Purple:
        r = 255;
        g = 0;
        b = 255;
        break;
    default:
        r = 0;
        g = 0;
        b = 0;
        break;
    }
    WS2812_Ctrl(r, g, b);
}

/***************************** (C) END OF FILE ******************************/

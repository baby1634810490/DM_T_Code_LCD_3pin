//
// Created by Lenovo on 2025/1/6.
//

/**
  ******************************************************************************
  * @file    driver_usart.h
  * @author  Shuai Yang
  * @brief   This file contains all the usart initialization function
  ******************************************************************************
  * @attention
  *
  * Don't forget the author
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_USART_H
#define BSP_USART_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported functions --------------------------------------------------------*/
void UART7ConfigEnable(uint8_t* DMA_Memory0BaseAddr, uint8_t DMA_BufferSize);

#endif //BSP_USART_H

/***************************** (C) END OF FILE ******************************/

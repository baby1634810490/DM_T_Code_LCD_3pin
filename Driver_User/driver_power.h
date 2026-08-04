//
// Created by Lenovo on 2025/5/24.
//

/**
  ******************************************************************************
  * @file    driver_power.h
  * @author  Shuai Yang
  * @brief   这个文件提供函数以实现可控电源的开关
  *
  ******************************************************************************
  * @attention
  *
  * Don't forget the author
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DRIVER_POWER_H
#define DRIVER_POWER_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported functions --------------------------------------------------------*/
void PowerOut1TurnON(void);

void PowerOut1TurnOFF(void);

void PowerOut2TurnON(void);

void PowerOut2TurnOFF(void);

void PowerOut5VTurnON(void);

void PowerOut5VTurnOFF(void);

#endif //DRIVER_POWER_H

/***************************** (C) END OF FILE ******************************/

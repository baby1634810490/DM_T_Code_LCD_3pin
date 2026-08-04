//
// Created by Lenovo on 2025/5/26.
//

/**
  ******************************************************************************
  * @file    bsp_fdcan.h
  * @author  Shuai Yang
  * @brief   这个文件提供函数对 fdcan 进行初始化、发送、回调
  ******************************************************************************
  * @attention
  *
  * Don't forget the author
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_FDCAN_H
#define BSP_FDCAN_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported functions --------------------------------------------------------*/
void FDCAN1Config(void);

void FDCAN2Config(void);

void FDCAN3Config(void);

uint8_t FDCAN1_Send_Msg(const uint8_t* msg, uint8_t len, uint32_t id, uint32_t ide);

uint8_t FDCAN2_Send_Msg(const uint8_t* msg, uint8_t len, uint32_t id, uint32_t ide);

uint8_t FDCAN3_Send_Msg(const uint8_t* msg, uint8_t len, uint32_t id, uint32_t ide);

#endif //BSP_FDCAN_H

/***************************** (C) END OF FILE ******************************/

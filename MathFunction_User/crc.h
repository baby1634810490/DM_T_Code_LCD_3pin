//
// Created by Lenovo on 2025/1/7.
//

/**
  ******************************************************************************
  * @file    crc.h
  * @author  XJTU ROBOMASTER Team
  * @brief   crc校验
  ******************************************************************************
  * @attention
  *
  * Please add comments after adding or deleting functions to ensure code
  * specification.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CRC_H
#define CRC_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported functions --------------------------------------------------------*/
void crc16_update(uint16_t* currect_crc, const uint8_t* src, uint32_t len);

uint8_t is_crc_passing(uint8_t * rawData);

void crc16_vision_update(uint16_t* currectCrc, const uint8_t* src, uint32_t lengthInBytes);

uint8_t sumCheck(const uint8_t* src, uint16_t lengthInBytes);

#endif //CRC_H

/************************ (C) COPYRIGHT XJTU ROBOMASTER ********END OF FILE****/

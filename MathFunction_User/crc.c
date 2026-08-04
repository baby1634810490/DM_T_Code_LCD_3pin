//
// Created by Lenovo on 2025/1/7.
//

/**
  ******************************************************************************
  * @file    crc.c
  * @author  XJTU ROBOMASTER Team
  * @brief   crc校验函数，可以对陀螺仪数据和视觉数据进行校验
  *
  @verbatim
  ==============================================================================

  @endverbatim
  ******************************************************************************
  * @attention
  *
  * 这个文件包含crc校验，两个版本分别来自魏炳文、陈星宇
  *
  * Please add comments after adding or deleting functions to ensure code
  * specification.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "crc.h"
#include "string.h"

/* ----------------------- Function Implements ---------------------------- */
/**
  * @brief  CRC校验
  * @retval  None
  */
static uint16_t U2(uint8_t* p)
{
    uint16_t u;
    memcpy(&u, p, 2);
    return u;
}

void crc16_update(uint16_t* currect_crc, const uint8_t* src, uint32_t len)
{
    uint32_t crc = *currect_crc;
    uint32_t j;
    for (j = 0; j < len; ++j)
    {
        uint32_t i;
        uint32_t byte = src[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    }
    *currect_crc = crc;
}

uint8_t is_crc_passing(uint8_t* rawData)
{
    uint16_t crc = 0;

    /* checksum */
    crc16_update(&crc, rawData, 4);
    crc16_update(&crc, rawData + 6, rawData[2] | rawData[3] << 8);
    if (crc != U2(rawData + 4))
    {
        return DISABLE;
    }
    return ENABLE;
}

/**
  * @brief  CRC check
  * @retval None
  */
void crc16_vision_update(uint16_t* currectCrc, const uint8_t* src, uint32_t lengthInBytes)
{
    uint32_t crc = *currectCrc;
    uint32_t j;
    for (j = 0; j < lengthInBytes; ++j)
    {
        uint32_t i;
        uint32_t byte = src[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    }
    *currectCrc = crc;
}

/**
  * @brief  sum check
  * @retval sum
  */
uint8_t sumCheck(const uint8_t* src, uint16_t lengthInBytes)
{
    uint16_t i;
    uint8_t sum = 0;
    for (i = 0; i < lengthInBytes; i++)
    {
        sum += src[i];
    }
    return sum;
}

/************************ (C) COPYRIGHT XJTU ROBOMASTER ********END OF FILE****/

//
// Created by Lenovo on 2025/1/6.
//

/**
  ******************************************************************************
  * @file    bsp_usart.c
  * @author  Shuai Yang
  * @brief   bsp usart
  *          这个文件提供函数对串口通讯进行开启并在回调函数中释放信号量，包括
  *           + usart1 ENABLE
  *           + uart7 ENABLE
  *           + usart6 ENABLE
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

/* Includes ------------------------------------------------------------------*/
#include "bsp_usart.h"
#include "cmsis_os.h"

/* Exported variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart7;
extern osSemaphoreId_t remoteSemHandle;

/* ----------------------- Function Implements ---------------------------- */
/**
  * @brief  Enable UART7
  * @param  DMA_Memory0BaseAddr : Uart7DMAMemoryBaseAddress
  * @param  DMA_BufferSize : Amount of data elements (uint8_t or uint16_t) to be received.
  * @retval None
  */
void UART7ConfigEnable(uint8_t* DMA_Memory0BaseAddr, uint8_t DMA_BufferSize)
{
    //SrcAddress:UART7->DR;DstAddress:Usart6DMAMemoryBaseAddress
    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, (uint8_t*)DMA_Memory0BaseAddr, DMA_BufferSize);
    //关闭DMA接收中断，保证只有空闲中断
    __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
}

uint8_t testCount = 0;
/**
  * @brief  Reception Event Callback (Rx event notification called after use of advanced reception service).
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
    if (huart->Instance == UART7)
    {
        testCount++;
        //释放信号量，使程序进入remote任务
        osSemaphoreRelease(remoteSemHandle);
    }
}

/***************************** (C) END OF FILE ******************************/

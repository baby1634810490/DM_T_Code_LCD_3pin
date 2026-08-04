//
// Created by Lenovo on 2025/1/7.
//

/**
  ******************************************************************************
  * @file    task_los.c
  * @author  Shuai Yang
  * @brief   los task
  * @priority osPriorityLow
  *
  @verbatim
  ==============================================================================

  @endverbatim
  ******************************************************************************
  * @attention
  *
  * Don't forget the author
  * 本任务的功能是：
  * * los计数器初始化、计数器计数、判断不同信号是否丢失
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "driver_los.h"

/* ----------------------- Function Implements ---------------------------- */
void LosTask(void* argument)
{
    SignalTimeInit();
    for (;;)
    {
        chassisErrorStatus = LostSignalCount();
        SignalLossHandling(chassisErrorStatus);
        osDelay(LOST_OF_SIGNAL_TASK_MS);
    }
}

/***************************** (C) END OF FILE ******************************/

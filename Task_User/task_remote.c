//
// Created by Lenovo on 2025/1/6.
//

/**
  ******************************************************************************
  * @file    task_remote.c
  * @author  Shuai Yang
  * @brief   remote task
  * @priority osPriorityRealtime
  *
  @verbatim
  ==============================================================================

  @endverbatim
  ******************************************************************************
  * @attention
  *
  * Don't forget the author
  * 本任务的功能是：
  * * 开启串口空闲中断
  * * 接收来自串口一接收中断的信号量以开启任务
  * * 处理接收到的数组
  * * 解算得到遥控器拨杆和摇杆的状态，用于控制机器人
  * * los计数器清零
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "driver_remote.h"
#include "driver_los.h"

/* Exported variables ------------------------------------------------------ */
extern osSemaphoreId_t remoteSemHandle;
uint16_t testAA = 0;

/* ----------------------- Function Implements ---------------------------- */
void RemoteTask(void* argument)
{
    UART7ConfigEnable((uint8_t*)remoteData, 50);
    osSemaphoreAcquire(remoteSemHandle, 0);
    for (;;)
    {
        UART7ConfigEnable((uint8_t*)remoteData, 50);
        osStatus_t remoteStatus = osSemaphoreAcquire(remoteSemHandle, 18);
        if (remoteStatus == osOK)
        {
            testAA++;
            uint8_t dataFlag = CrsfDataUpdate((uint8_t*)remoteData, &CrsfData);
            if (dataFlag == ENABLE)
            {
                RemoteDataUpdate(CrsfData, &RobotRemote);
                //RobotCtlDataUpdate(RobotRemote, &RobotControl);
                LostOfSignalFeed(REMOTE_LOST_OF_SIGNAL);
            }
        }
    }
}

/***************************** (C) END OF FILE ******************************/

//
// Created by Lenovo on 2023/11/23.
//

/* Includes ---------------------------------------------------------------- */
#include "driver_los.h"

/* ----------------------- Internal Data ----------------------------------- */
/** @brief The time of the signal count
  */
uint8_t signalTime[SIGNAL_NUMBER] = {0};

/** @brief Each represents a state of the gimbal, one if there is an error, zero if there is no error
  */
error_status chassisErrorStatus;

/** @brief If the remote data is received? Yes, flag is ENABLE and no is DISABLE
  */
uint8_t remoteFlag = DISABLE;

/** @brief If the gyro data is right? Yes, flag is ENABLE and no is DISABLE
  */
uint8_t gyroFlag = DISABLE;

/* ----------------------- Function Implements ---------------------------- */
/**
  * @brief  Initialize the signal time
  * @retval None
  */
void SignalTimeInit(void)
{
    uint8_t i;
    for (i = 0; i < SIGNAL_NUMBER; i++)
    {
        signalTime[i] = 100;
    }
}

/**
  * @brief  Update the signal time to zero
  * @retval None
  */
void LostOfSignalFeed(uint8_t losType)
{
    signalTime[losType] = 0;
}

/**
  * @brief  Increase the signal count, if the count is more than time,
  *         the signal is considered lost
  * @retval None
  */
error_status LostSignalCount(void)
{
    uint8_t i;
    error_status systemErrorStatus = 0;

    for (i = 0; i < SIGNAL_NUMBER; i++)
    {
        signalTime[i]++;
    }

    if (signalTime[REMOTE_LOST_OF_SIGNAL] >= (REMOTE_LOST_OF_SIGNAL_MS / LOST_OF_SIGNAL_TASK_MS))
    {
        systemErrorStatus |= 0x01 << REMOTE_LOST_OF_SIGNAL;
        signalTime[REMOTE_LOST_OF_SIGNAL] = (REMOTE_LOST_OF_SIGNAL_MS / LOST_OF_SIGNAL_TASK_MS);
    }
    if (signalTime[GYRO_LOST_OF_SIGNAL] >= (GYRO_LOST_OF_SIGNAL_MS / LOST_OF_SIGNAL_TASK_MS))
    {
        systemErrorStatus |= 0x01 << GYRO_LOST_OF_SIGNAL;
        signalTime[GYRO_LOST_OF_SIGNAL] = (GYRO_LOST_OF_SIGNAL_MS / LOST_OF_SIGNAL_TASK_MS);
    }

    if (signalTime[YAWMOTOR_LOST_OF_SIGNAL] >= (MOTOR_LOST_OF_SIGNAL_MS / LOST_OF_SIGNAL_TASK_MS))
    {
        systemErrorStatus |= 0x01 << YAWMOTOR_LOST_OF_SIGNAL;
        /* 防止溢出 */
        signalTime[YAWMOTOR_LOST_OF_SIGNAL] = (MOTOR_LOST_OF_SIGNAL_MS / LOST_OF_SIGNAL_TASK_MS);
    }
    if (signalTime[PITCHMOTOR_LOST_OF_SIGNAL] >= (MOTOR_LOST_OF_SIGNAL_MS / LOST_OF_SIGNAL_TASK_MS))
    {
        systemErrorStatus |= 0x01 << PITCHMOTOR_LOST_OF_SIGNAL;
        signalTime[PITCHMOTOR_LOST_OF_SIGNAL] = (MOTOR_LOST_OF_SIGNAL_MS / LOST_OF_SIGNAL_TASK_MS);
    }
    if (signalTime[LEFTFRICTION_LOST_OF_SIGNAL] >= (MOTOR_LOST_OF_SIGNAL_MS / LOST_OF_SIGNAL_TASK_MS))
    {
        systemErrorStatus |= 0x01 << LEFTFRICTION_LOST_OF_SIGNAL;
        signalTime[LEFTFRICTION_LOST_OF_SIGNAL] = (MOTOR_LOST_OF_SIGNAL_MS / LOST_OF_SIGNAL_TASK_MS);
    }
    if (signalTime[RIGHTFRICTION_LOST_OF_SIGNAL] >= (MOTOR_LOST_OF_SIGNAL_MS / LOST_OF_SIGNAL_TASK_MS))
    {
        systemErrorStatus |= 0x01 << RIGHTFRICTION_LOST_OF_SIGNAL;
        signalTime[RIGHTFRICTION_LOST_OF_SIGNAL] = (MOTOR_LOST_OF_SIGNAL_MS / LOST_OF_SIGNAL_TASK_MS);
    }
    if (signalTime[HIGHFRICTION_LOST_OF_SIGNAL] >= (MOTOR_LOST_OF_SIGNAL_MS / LOST_OF_SIGNAL_TASK_MS))
    {
        systemErrorStatus |= 0x01 << HIGHFRICTION_LOST_OF_SIGNAL;
        signalTime[HIGHFRICTION_LOST_OF_SIGNAL] = (MOTOR_LOST_OF_SIGNAL_MS / LOST_OF_SIGNAL_TASK_MS);
    }

    return systemErrorStatus;
}

/**
  * @brief  Handle signal loss data flag
  * @retval None
  */
void SignalLossHandling(error_status status)
{
    if (status >> REMOTE_LOST_OF_SIGNAL & 0x01)
        remoteFlag = DISABLE;
    else
        remoteFlag = ENABLE;
    if (status >> GYRO_LOST_OF_SIGNAL & 0x01)
        gyroFlag = DISABLE;
}

/***************************** (C) END OF FILE ******************************/

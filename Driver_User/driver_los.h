//
// Created by Lenovo on 2023/11/23.
//

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DRIVER_LOS_H
#define DRIVER_LOS_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/
typedef uint16_t error_status;

/* Exported constants --------------------------------------------------------*/
#define SIGNAL_NUMBER                  16    /* 被检测信号的数量 */

#define REMOTE_LOST_OF_SIGNAL           0    /* 遥控器信号 */
#define GYRO_LOST_OF_SIGNAL             1    /* 陀螺仪信号 */

#define YAWMOTOR_LOST_OF_SIGNAL         9
#define PITCHMOTOR_LOST_OF_SIGNAL      14
#define LEFTFRICTION_LOST_OF_SIGNAL    13
#define RIGHTFRICTION_LOST_OF_SIGNAL   12
#define HIGHFRICTION_LOST_OF_SIGNAL    11

#define LOST_OF_SIGNAL_TASK_MS     50   /* 信号检测时间，单位ms */
#define REMOTE_LOST_OF_SIGNAL_MS   200  /* 遥控器信号检测时间，单位ms */
#define GYRO_LOST_OF_SIGNAL_MS     100  /* 陀螺仪信号检测时间，单位ms */
#define MOTOR_LOST_OF_SIGNAL_MS    100  /* 电机信号检测时间，单位ms */

/* Exported variables --------------------------------------------------------*/
extern error_status chassisErrorStatus;

extern uint8_t remoteFlag;

extern uint8_t gyroFlag;

/* Exported functions --------------------------------------------------------*/
void SignalTimeInit(void);

void LostOfSignalFeed(uint8_t losType);

error_status LostSignalCount(void);

void SignalLossHandling(error_status status);

#endif //DRIVER_LOS_H

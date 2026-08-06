//
// Created by Lenovo on 2025/4/18.
//

/**
  ******************************************************************************
  * @file    task_motor.c
  * @author  Shuai Yang
  * @brief   motor task
  * @priority osPriorityHigh
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
#include "cmsis_os.h"
#include "app_tsda_servo.h"
#include "bsp_fdcan.h"
#include "driver_motor.h"
#include "driver_power.h"
#include "main.h"

/* Exported variables ------------------------------------------------------ */
extern uint8_t remoteFlag;

/* ----------------------- Private Functions ------------------------------- */
/**
  * @brief TSDA 协议层使用的 FDCAN1 发送适配函数。
  * @param can_id 标准 CAN 帧 ID。
  * @param data   待发送的 TSDA 协议数据。
  * @param len    数据长度，TSDA 协议固定为 8 字节。
  * @param user   用户上下文，当前未使用。
  * @retval 0=发送成功，非0=发送失败。
  * @note 只负责把协议层发送接口转换为本工程的 FDCAN1 发送接口。
  */
static uint8_t TSDA_FDCAN1_SendAdapter(uint32_t can_id, const uint8_t* data, uint8_t len, void* user)
{
	(void)user;
	return FDCAN1_Send_Msg(data, len, can_id, FDCAN_STANDARD_ID);
}

/** @brief 读取3Pin上限位。硬件定义为PD7高电平有效。 */
static uint8_t TSDA_ReadUpperLimitAdapter(void* user)
{
	(void)user;
	return (HAL_GPIO_ReadPin(TSDA_UPPER_LIMIT_GPIO_Port,
	                         TSDA_UPPER_LIMIT_Pin) == GPIO_PIN_SET) ? 1U : 0U;
}

/** @brief 读取3Pin下限位。硬件定义为PB10高电平有效，仅用于状态观察。 */
static uint8_t TSDA_ReadLowerLimitAdapter(void* user)
{
	(void)user;
	return (HAL_GPIO_ReadPin(TSDA_LOWER_LIMIT_GPIO_Port,
	                         TSDA_LOWER_LIMIT_Pin) == GPIO_PIN_SET) ? 1U : 0U;
}

/** @brief 控制3Pin抱闸：PE15高电平释放，低电平闭合。 */
static void TSDA_WriteBrakeReleaseAdapter(uint8_t release, void* user)
{
	(void)user;
	HAL_GPIO_WritePin(TSDA_BRAKE_RELEASE_GPIO_Port,
	                  TSDA_BRAKE_RELEASE_Pin,
	                  (release != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* 板级层只描述本板 IO 接线，不包含TSDA协议或状态机策略。Chassis Profile会忽略这些回调。 */
static const TSDA_BoardIo tsdaBoardIo = {
	TSDA_ReadUpperLimitAdapter,
	TSDA_ReadLowerLimitAdapter,
	TSDA_WriteBrakeReleaseAdapter,
	NULL
};


/* ----------------------- Function Implements ---------------------------- */

static uint8_t tsdaAppStarted = DISABLE;

void MotorTask(void* argument)
{
	FDCAN1Config();
	FDCAN2Config();
	FDCAN3Config();
	PWM_Init();
	MotorInit();
	uint8_t safeFlag = DISABLE;
	uint16_t initCount = 0;

	for (;;)
	{
		if (initCount < 2000)
		{
			initCount++;
			BeepSet(50);
		}
		if (initCount >= 2000)
		{
			PowerOut5VTurnON();
			BeepSet(0);
			safeFlag = remoteFlag ? ENABLE : DISABLE;
			if (tsdaAppStarted == DISABLE)
			{
				/*
				 * 板级层只把统一Driver绑定到CAN1。Chassis的限位来自0x58，抱闸由
				 * 驱动器随使能自动联动，因此本阶段不向App注入任何GPIO回调。
				 */
				TSDA_AppInit(TSDA_FDCAN1_SendAdapter,
				             NULL,
				             &tsdaBoardIo,
				             osKernelGetTickCount());
				tsdaAppStarted = ENABLE;
			}
		}

		if (tsdaAppStarted == ENABLE)
			TSDA_AppUpdate(osKernelGetTickCount());

		MotorDataUpdate(safeFlag);
		MotorSettingUpdate();
		MotorCalculate(safeFlag);

		MotorCANSend();

		osDelay(1);
	}
}

/***************************** (C) END OF FILE ******************************/

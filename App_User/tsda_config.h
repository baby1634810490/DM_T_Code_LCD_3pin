/**
  ******************************************************************************
  * @file    tsda_config.h
  * @brief   TSDA 硬件变体的编译期只读能力配置。
  *
  * 使用方法：
  *   1. TSDA_HARDWARE_VARIANT=3：编译 3Pin 驱动器版本；
  *   2. TSDA_HARDWARE_VARIANT=4：编译 Chassis 驱动器版本。
  *
  * 设计约束：
  *   - 硬件编号只在本文件中翻译为“限位来源、抱闸所有权、等待时间”等能力；
  *   - App 状态机只读取能力，不直接判断 3Pin 或 Chassis；
  *   - Driver 不包含本文件，始终保留完整 TSDA 协议能力；
  *   - 配置对象为 const，运行时不可修改。
  ******************************************************************************
  */

#ifndef TSDA_CONFIG_H
#define TSDA_CONFIG_H

#include <stdint.h>

#define TSDA_VARIANT_3PIN       (3U)
#define TSDA_VARIANT_CHASSIS    (4U)

/* 当前 3Pin 工程默认构建 3Pin；移植或回归 Chassis 时由编译器宏覆盖该值。 */
#ifndef TSDA_HARDWARE_VARIANT
#define TSDA_HARDWARE_VARIANT   4
#endif

typedef enum
{
	TSDA_LIMIT_OBSERVER_BOARD_IO = 0U,      /*!< 限位由 MCU 板级 GPIO 提供。 */
	TSDA_LIMIT_OBSERVER_DRIVER_PORT58       /*!< 限位由驱动器寄存器 0x58 提供。 */
} TSDA_LimitObserver;

typedef enum
{
	TSDA_BRAKE_CONTROL_MCU = 0U,            /*!< MCU 输出 GPIO 控制抱闸释放/闭合。 */
	TSDA_BRAKE_CONTROL_SERVO                 /*!< 驱动器随使能自动控制抱闸。 */
} TSDA_BrakeControl;

typedef struct
{
	TSDA_LimitObserver limit_observer;      /*!< App 观察限位的统一策略。 */
	TSDA_BrakeControl brake_control;        /*!< App 执行抱闸动作的统一策略。 */
	uint16_t brake_release_settle_ms;       /*!< MCU 释放抱闸后的完整机械稳定时间。 */
	uint16_t brake_engage_delay_ms;         /*!< MCU 闭合抱闸后到驱动器失能的等待时间。 */
	int16_t homing_speed_rpm;               /*!< 向上寻限使用的负转速。 */
} TSDA_Config;

/*
 * 条件编译只负责把硬件编号翻译为能力策略。统一 App 状态机不会看到硬件编号。
 * 两种Profile采用同一状态推进：自动寻零、返回-100mm、进入DONE并开放速度规划。
 * 配置差异只表达真实硬件能力和参数，不再用于暂停或拆分业务流程。
 */
#if (TSDA_HARDWARE_VARIANT == TSDA_VARIANT_3PIN)
#define TSDA_CONFIG_INITIALIZER                                                \
	{                                                                          \
		TSDA_LIMIT_OBSERVER_BOARD_IO, TSDA_BRAKE_CONTROL_MCU,                  \
		2000U, 200U, -250                                                        \
	}
#elif (TSDA_HARDWARE_VARIANT == TSDA_VARIANT_CHASSIS)
#define TSDA_CONFIG_INITIALIZER                                                \
	{                                                                          \
		TSDA_LIMIT_OBSERVER_DRIVER_PORT58, TSDA_BRAKE_CONTROL_SERVO,          \
		0U, 0U, -250                                                           \
	}
#else
#error "Unsupported TSDA_HARDWARE_VARIANT: use 3 for 3Pin or 4 for Chassis"
#endif

/* 本对象仅供 App 实现读取，const 保证运行时不能切换硬件策略。 */
static const TSDA_Config tsda_config = TSDA_CONFIG_INITIALIZER;

#endif /* TSDA_CONFIG_H */

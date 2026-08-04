//
// Created by Lenovo on 2025/5/20.
//

/**
  ******************************************************************************
  * @file    task_led.c
  * @author  Shuai Yang
  * @brief   led task
  * @priority osPriorityLow
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
#include "driver_led.h"

/* ----------------------- Function Implements ---------------------------- */
void LEDTask()
{
	uint8_t enableColor = Red;

	for (;;)
	{
		if (TSDA_AppIsError() != 0U)
		{
			LED_Light_Up(Red);
			osDelay(100);
			LED_Light_Up(Black);
			osDelay(100);
			continue;
		}

		if (TSDA_AppIsServoEnabled() == 0U)
		{
			LED_Light_Up(Blue);
			osDelay(500);
			LED_Light_Up(Black);
			osDelay(500);
			continue;
		}

		LED_Light_Up(enableColor);
		osDelay(200);

		if (enableColor == Red)
		{
			enableColor = Green;
		}
		else if (enableColor == Green)
		{
			enableColor = Blue;
		}
		else
		{
			enableColor = Red;
		}
	}
}

/***************************** (C) END OF FILE ******************************/

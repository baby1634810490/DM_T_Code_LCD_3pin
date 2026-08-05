/**
  ******************************************************************************
  * @file    tsda_slide_motion.c
  * @brief   TSDA滑台速度规划、零速保持和软件行程边界实现。
  ******************************************************************************
  */

#include "tsda_slide_motion.h"

#include "position_velocity_planner.h"

#include <stddef.h>

/** @brief 取浮点绝对值，避免在控制路径中引入额外库接口。 */
static float TSDA_SlideAbsFloat(float value)
{
	return (value >= 0.0f) ? value : -value;
}

/** @brief 取有符号RPM绝对值；E4有效范围不会触发int32溢出。 */
static int32_t TSDA_SlideAbsRpm(int16_t rpm)
{
	return (rpm >= 0) ? (int32_t)rpm : -(int32_t)rpm;
}

/**
  * @brief 把用户坐标速度转换为驱动器RPM并四舍五入。
  * @note 用户速度向上为正，而实机确认驱动器负RPM向上，因此只在此处取反。
  */
static int16_t TSDA_SlideVelocityToMotorRpm(float velocity_mm_s)
{
	float rpm = -velocity_mm_s * TSDA_SLIDE_RPM_PER_MM_S;

	/* 协议寄存器为int16_t。正常协议上限200mm/s仅对应2400RPM，
	 * 这里仍保留类型边界，避免强制转换超出C语言可表示范围。 */
	if (rpm > 32767.0f)
		rpm = 32767.0f;
	else if (rpm < -32768.0f)
		rpm = -32768.0f;

	return (rpm >= 0.0f) ? (int16_t)(rpm + 0.5f) : (int16_t)(rpm - 0.5f);
}

void TSDA_SlideMotionReset(TSDA_SlideMotion* motion, uint8_t hold_active)
{
	if (motion == NULL)
		return;

	motion->planned_velocity_mm_s = 0.0f;
	motion->hold_active = (hold_active != 0U) ? 1U : 0U;
}

int16_t TSDA_SlideMotionStep(TSDA_SlideMotion* motion,
								 float target_height_mm,
								 float current_height_mm,
								 float max_speed_mm_s,
								 float acceleration_mm_s2,
								 int16_t actual_speed_rpm)
{
	PVP_Limit_t limit;
	float position_error;
	float abs_error;
	float next_velocity;
	const float stop_velocity_mm_s =
		(float)TSDA_SLIDE_STOP_SPEED_RPM / TSDA_SLIDE_RPM_PER_MM_S;

	if (motion == NULL)
		return 0;

	position_error = target_height_mm - current_height_mm;
	abs_error = TSDA_SlideAbsFloat(position_error);

	/* 保持态使用3mm退出门限。目标实时变化或受外力偏移不超过3mm时，
	 * 驱动器继续接收0RPM，不在边界附近反复启动。 */
	if (motion->hold_active != 0U)
	{
		if (abs_error <= TSDA_SLIDE_HOLD_EXIT_ERROR_MM)
		{
			motion->planned_velocity_mm_s = 0.0f;
			return 0;
		}
		motion->hold_active = 0U;
	}

	limit.max_vel = max_speed_mm_s;
	limit.accel = acceleration_mm_s2;
	limit.dt = TSDA_SLIDE_PLAN_PERIOD_S;

	/* PVP直接使用用户坐标。planned_velocity_mm_s是独立浮点状态，
	 * 不使用整数E4作为下一拍起点，因此低加速度可以逐周期累积。 */
	next_velocity = PVP_StepWithTolerance(target_height_mm,
										 current_height_mm,
										 motion->planned_velocity_mm_s,
										 &limit,
										 TSDA_SLIDE_HOLD_ENTER_ERROR_MM,
										 stop_velocity_mm_s);

	/* 软件行程边界限制的是继续越界的方向，反向离开边界始终允许。 */
	if (((current_height_mm >= TSDA_SLIDE_UPPER_LIMIT_MM) &&
	     (next_velocity > 0.0f)) ||
	    ((current_height_mm <= TSDA_SLIDE_LOWER_LIMIT_MM) &&
	     (next_velocity < 0.0f)))
	{
		next_velocity = 0.0f;
	}

	motion->planned_velocity_mm_s = next_velocity;

	/* 位置、规划速度和E4实际速度同时满足门限后锁存保持态。 */
	if ((abs_error <= TSDA_SLIDE_HOLD_ENTER_ERROR_MM) &&
	    (TSDA_SlideAbsFloat(next_velocity) <= stop_velocity_mm_s) &&
	    (TSDA_SlideAbsRpm(actual_speed_rpm) <= TSDA_SLIDE_STOP_SPEED_RPM))
	{
		motion->planned_velocity_mm_s = 0.0f;
		motion->hold_active = 1U;
		return 0;
	}

	return TSDA_SlideVelocityToMotorRpm(next_velocity);
}

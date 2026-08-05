#include <assert.h>

#include "tsda_slide_motion.h"

static float AbsFloat(float value)
{
	return (value >= 0.0f) ? value : -value;
}

int main(void)
{
	TSDA_SlideMotion motion;
	float previous_velocity;
	int16_t rpm;

	/* 初始位于-100mm保持点时必须持续0RPM。 */
	TSDA_SlideMotionReset(&motion, 1U);
	rpm = TSDA_SlideMotionStep(&motion, -100.0f, -100.0f, 20.0f, 80.0f, 0);
	assert(rpm == 0);
	assert(motion.hold_active == 1U);

	/* 2.5mm误差仍处于3mm保持回差，不应启动。 */
	rpm = TSDA_SlideMotionStep(&motion, -102.5f, -100.0f, 20.0f, 80.0f, 0);
	assert(rpm == 0);
	assert(motion.hold_active == 1U);

	/* 超过3mm后退出保持；向下用户速度为负，驱动器命令应为正RPM。 */
	rpm = TSDA_SlideMotionStep(&motion, -104.0f, -100.0f, 20.0f, 80.0f, 0);
	assert(rpm > 0);
	assert(motion.planned_velocity_mm_s < 0.0f);

	/* 低加速度5mm/s^2每拍只有0.18RPM，但浮点规划速度应持续累积并最终输出1RPM。 */
	TSDA_SlideMotionReset(&motion, 0U);
	rpm = TSDA_SlideMotionStep(&motion, -150.0f, -100.0f, 20.0f, 5.0f, 0);
	assert(rpm == 0);
	rpm = TSDA_SlideMotionStep(&motion, -150.0f, -100.0f, 20.0f, 5.0f, 0);
	assert(rpm == 0);
	rpm = TSDA_SlideMotionStep(&motion, -150.0f, -100.0f, 20.0f, 5.0f, 0);
	assert(rpm == 1);

	/* 实时反向目标不得重置规划速度，应先从当前负速度连续减速。 */
	previous_velocity = motion.planned_velocity_mm_s;
	(void)TSDA_SlideMotionStep(&motion, -50.0f, -100.0f, 20.0f, 5.0f, 0);
	assert(motion.planned_velocity_mm_s > previous_velocity);
	assert(AbsFloat(motion.planned_velocity_mm_s) > 0.0f);

	/* 进入2mm范围且规划/E4速度均接近0后锁存保持。 */
	TSDA_SlideMotionReset(&motion, 0U);
	rpm = TSDA_SlideMotionStep(&motion, -101.0f, -100.0f, 20.0f, 80.0f, 0);
	assert(rpm == 0);
	assert(motion.hold_active == 1U);

	/* 位于上边界时禁止继续向上，但允许向下离开边界。 */
	TSDA_SlideMotionReset(&motion, 0U);
	motion.planned_velocity_mm_s = 1.0f;
	rpm = TSDA_SlideMotionStep(&motion, 0.0f, 0.0f, 20.0f, 80.0f, 20);
	assert(rpm == 0);
	assert(motion.planned_velocity_mm_s == 0.0f);
	TSDA_SlideMotionReset(&motion, 0U);
	rpm = TSDA_SlideMotionStep(&motion, -10.0f, 0.0f, 20.0f, 80.0f, 0);
	assert(rpm > 0);

	/* 位于下边界时禁止继续向下，但允许向上离开边界。 */
	TSDA_SlideMotionReset(&motion, 0U);
	motion.planned_velocity_mm_s = -1.0f;
	rpm = TSDA_SlideMotionStep(&motion, -300.0f, -300.0f, 20.0f, 80.0f, 20);
	assert(rpm == 0);
	assert(motion.planned_velocity_mm_s == 0.0f);
	TSDA_SlideMotionReset(&motion, 0U);
	rpm = TSDA_SlideMotionStep(&motion, -290.0f, -300.0f, 20.0f, 80.0f, 0);
	assert(rpm < 0);

	return 0;
}

/**
  ******************************************************************************
  * @file    tsda_slide_motion.h
  * @brief   TSDA滑台速度规划适配层。
  *
  * 本模块位于通用位置速度规划器与TSDA App之间：
  *   - 输入、内部状态全部使用滑台用户坐标（向上为正、向下为负）；
  *   - 保存可连续累积的浮点规划速度，避免整数RPM截断小加速度；
  *   - 实现2mm进入保持、3mm退出保持的回差；
  *   - 最后一步才把mm/s换算为方向相反的驱动器RPM。
  *
  * 本模块不发送CAN、不读取寄存器，也不区分Chassis或3Pin。
  ******************************************************************************
  */

#ifndef TSDA_SLIDE_MOTION_H
#define TSDA_SLIDE_MOTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define TSDA_SLIDE_UPPER_LIMIT_MM          (0.0f)
#define TSDA_SLIDE_LOWER_LIMIT_MM          (-300.0f)
#define TSDA_SLIDE_HOLD_ENTER_ERROR_MM     (2.0f)
#define TSDA_SLIDE_HOLD_EXIT_ERROR_MM      (3.0f)
#define TSDA_SLIDE_STOP_SPEED_RPM          (2)
#define TSDA_SLIDE_PLAN_PERIOD_S           (0.003f)
#define TSDA_SLIDE_MM_PER_REV              (5.0f)
#define TSDA_SLIDE_RPM_PER_MM_S            (60.0f / TSDA_SLIDE_MM_PER_REV)

/** @brief 速度规划器需要跨周期保存的最小状态。 */
typedef struct
{
	float planned_velocity_mm_s; /*!< 用户坐标规划速度：向上为正、向下为负。 */
	uint8_t hold_active;         /*!< 1=处于零速保持，误差超过3mm才退出。 */
} TSDA_SlideMotion;

/** @brief 清零规划速度，并指定初始是否处于零速保持。 */
void TSDA_SlideMotionReset(TSDA_SlideMotion* motion, uint8_t hold_active);

/**
  * @brief 根据实时目标和约束生成本周期驱动器目标RPM。
  * @param motion               跨周期规划状态。
  * @param target_height_mm     目标高度，App已钳位到[-300,0]mm。
  * @param current_height_mm    E8/E9换算的真实浮点高度。
  * @param max_speed_mm_s       通信层保证合法的正最大速度。
  * @param acceleration_mm_s2   通信层保证合法的正加速度，加减速共用。
  * @param actual_speed_rpm     E4实际有符号RPM，用于确认零速保持。
  * @return 写入TSDA 0x10的有符号整数RPM；负RPM向上、正RPM向下。
  */
int16_t TSDA_SlideMotionStep(TSDA_SlideMotion* motion,
								 float target_height_mm,
								 float current_height_mm,
								 float max_speed_mm_s,
								 float acceleration_mm_s2,
								 int16_t actual_speed_rpm);

#ifdef __cplusplus
}
#endif

#endif /* TSDA_SLIDE_MOTION_H */

/**
 * @file    position_velocity_planner.h
 * @brief   位置-速度规划器对外接口定义
 * @details
 * 本文件提供一个适用于嵌入式控制周期调用的单步位置-速度规划函数。
 * 给定目标位置、当前位置、当前速度、最大速度、最大加速度和控制周期，
 * 函数输出下一控制周期应给定的速度指令。
 *
 * 该规划方法可自动形成：
 * - 长距离运动时的梯形速度曲线
 * - 短距离运动时的三角形速度曲线
 *
 * 核心思想：
 * - 根据剩余距离计算“当前还能刹住车”的最大允许速度
 * - 再根据最大加速度限制，生成下一拍速度指令
 *
 * @note
 * 所有输入量的单位必须保持一致：
 * - 位置：任意统一的位置单位，如 count、deg、rad、mm
 * - 速度：上述位置单位 / s
 * - 加速度：上述位置单位 / s^2
 * - 控制周期 dt：s
 *
 * @warning
 * 本模块输出的是“速度指令”，通常应由下层速度环或电机控制器进行跟踪。
 */

#ifndef POSITION_VELOCITY_PLANNER_H
#define POSITION_VELOCITY_PLANNER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @defgroup PositionVelocityPlanner 位置-速度规划模块
 * @brief    单步位置-速度在线规划算法
 * @details  根据目标位置、当前位置与当前速度，在线生成下一拍速度指令。
 * @{
 */

/**
 * @brief 规划器浮点数据类型
 * @details
 * 当前版本统一使用 float 进行计算，便于在资源受限的嵌入式平台上运行。
 */
typedef float pvp_float_t;

/**
 * @brief 规划器参数结构体
 * @details
 * 该结构体用于集中管理规划时所需的约束参数。
 */
typedef struct
{
    pvp_float_t max_vel; /**< 最大速度，单位：位置单位/s */
    pvp_float_t accel; /**< 最大加速度，单位：位置单位/s^2 */
    pvp_float_t dt; /**< 控制周期，单位：s */
} PVP_Limit_t;

/**
 * @brief 默认位置容差
 * @details
 * 当当前位置与目标位置的误差绝对值小于该值时，可认为已经接近目标位置。
 */
#define PVP_DEFAULT_POS_TOL   (1.0e-4f)

/**
 * @brief 默认速度容差
 * @details
 * 当当前速度绝对值小于该值时，可认为速度已接近 0。
 */
#define PVP_DEFAULT_VEL_TOL   (1.0e-4f)

/**
 * @brief 单步位置-速度规划函数
 * @details
 * 周期调用本函数，可根据当前位置与目标位置，自动生成满足以下要求的速度指令：
 * - 以不超过最大加速度的方式加速
 * - 速度不超过最大速度
 * - 接近目标点时自动减速
 * - 最终在目标位置附近减速到 0
 *
 * 算法核心为：
 * \f[
 * v_{brake} = \sqrt{2as}
 * \f]
 * 其中：
 * - \f$a\f$ 为最大加速度
 * - \f$s\f$ 为当前位置到目标位置的剩余距离
 * - \f$v_{brake}\f$ 为在剩余距离内可安全减速到 0 的最大允许速度
 *
 * @param[in] target_pos  目标位置
 * @param[in] current_pos 当前位置
 * @param[in] current_vel 当前速度
 * @param[in] limit       规划约束参数指针，不可为 NULL
 * @return 下一控制周期应输出的速度指令
 *
 * @retval 0.0f
 *         当输入参数非法，或已到达目标点附近且当前速度也足够小时返回 0
 *
 * @note
 * 本函数为“无状态函数”，不会在内部保存上一次速度。
 * 每次调用时应传入当前实际速度或当前控制器使用的速度。
 */
pvp_float_t PVP_Step(pvp_float_t target_pos,
                     pvp_float_t current_pos,
                     pvp_float_t current_vel,
                     const PVP_Limit_t* limit);

/**
 * @brief 带自定义容差的单步位置-速度规划函数
 * @details
 * 与 PVP_Step() 功能一致，但允许用户自定义位置容差与速度容差。
 *
 * @param[in] target_pos  目标位置
 * @param[in] current_pos 当前位置
 * @param[in] current_vel 当前速度
 * @param[in] limit       规划约束参数指针，不可为 NULL
 * @param[in] pos_tol     位置容差，单位：位置单位
 * @param[in] vel_tol     速度容差，单位：位置单位/s
 * @return 下一控制周期应输出的速度指令
 */
pvp_float_t PVP_StepWithTolerance(pvp_float_t target_pos,
                                  pvp_float_t current_pos,
                                  pvp_float_t current_vel,
                                  const PVP_Limit_t* limit,
                                  pvp_float_t pos_tol,
                                  pvp_float_t vel_tol);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* POSITION_VELOCITY_PLANNER_H */

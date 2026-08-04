/**
 * @file    position_velocity_planner.c
 * @brief   位置-速度规划器实现文件
 * @details
 * 本文件实现一个适用于嵌入式控制循环的在线位置-速度规划算法。
 * 其目标是在满足最大速度与最大加速度约束的前提下，生成下一拍速度命令，
 * 以实现从当前位置向目标位置平滑运动，并在终点附近减速到 0。
 */

#include "position_velocity_planner.h"

#include <math.h>
#include <stddef.h>

/**
 * @brief 取绝对值
 * @param[in] value 输入值
 * @return 输入值的绝对值
 */
static pvp_float_t PVP_Abs(pvp_float_t value)
{
    return (value >= 0.0f) ? value : -value;
}

/**
 * @brief 取较小值
 * @param[in] a 输入值 a
 * @param[in] b 输入值 b
 * @return a 与 b 中的较小值
 */
static pvp_float_t PVP_Min(pvp_float_t a, pvp_float_t b)
{
    return (a < b) ? a : b;
}

/**
 * @brief 获取符号
 * @param[in] value 输入值
 * @return
 * -  1.0f：正数
 * - -1.0f：负数
 * -  0.0f：零
 */
static pvp_float_t PVP_Sign(pvp_float_t value)
{
    if (value > 0.0f)
    {
        return 1.0f;
    }
    else if (value < 0.0f)
    {
        return -1.0f;
    }
    else
    {
        return 0.0f;
    }
}

pvp_float_t PVP_Step(pvp_float_t target_pos,
                     pvp_float_t current_pos,
                     pvp_float_t current_vel,
                     const PVP_Limit_t* limit)
{
    return PVP_StepWithTolerance(target_pos,
                                 current_pos,
                                 current_vel,
                                 limit,
                                 PVP_DEFAULT_POS_TOL,
                                 PVP_DEFAULT_VEL_TOL);
}

pvp_float_t PVP_StepWithTolerance(pvp_float_t target_pos,
                                  pvp_float_t current_pos,
                                  pvp_float_t current_vel,
                                  const PVP_Limit_t* limit,
                                  pvp_float_t pos_tol,
                                  pvp_float_t vel_tol)
{
    pvp_float_t pos_err;
    pvp_float_t dist;
    pvp_float_t dir;
    pvp_float_t brake_vel;
    pvp_float_t target_vel_mag;
    pvp_float_t target_vel;
    pvp_float_t dv_max;
    pvp_float_t dv;
    pvp_float_t next_vel;

    if ((limit == NULL) ||
        (limit->max_vel <= 0.0f) ||
        (limit->accel <= 0.0f) ||
        (limit->dt <= 0.0f) ||
        (pos_tol < 0.0f) ||
        (vel_tol < 0.0f))
    {
        return 0.0f;
    }

    /* 计算位置误差与运动方向 */
    pos_err = target_pos - current_pos;
    dist = PVP_Abs(pos_err);
    dir = PVP_Sign(pos_err);

    /* 已接近目标点且当前速度足够小，直接输出 0 */
    if ((dist < pos_tol) && (PVP_Abs(current_vel) < vel_tol))
    {
        return 0.0f;
    }

    /*
     * 根据剩余距离计算“当前还能刹住车”的最大允许速度。
     * 公式来源：v^2 = 2as
     */
    brake_vel = sqrtf(2.0f * limit->accel * dist);

    /* 当前目标速度幅值既不能超过最大速度，也不能超过刹车约束速度 */
    target_vel_mag = PVP_Min(limit->max_vel, brake_vel);

    /* 根据目标点相对当前位置的方向，生成带符号的目标速度 */
    target_vel = dir * target_vel_mag;

    /* 每个控制周期允许的最大速度变化量 */
    dv_max = limit->accel * limit->dt;

    /* 受最大加速度限制地逼近目标速度 */
    dv = target_vel - current_vel;

    if (dv > dv_max)
    {
        dv = dv_max;
    }
    else if (dv < -dv_max)
    {
        dv = -dv_max;
    }

    next_vel = current_vel + dv;

    /* 在目标点附近消除微小抖动 */
    if ((dist < pos_tol) && (PVP_Abs(next_vel) < dv_max))
    {
        next_vel = 0.0f;
    }

    return next_vel;
}

/**
 * @file motion_model.hpp
 * @brief ROS 无关的粒子运动预测抽象接口。
 * @author Antinomy
 * @date 2026-10-01
 */
#pragma once

#include <vector>

#include "mini_nav_core/localization/amcl/types.hpp"

namespace mini_nav_core::localization
{

/**
 * @brief 粒子运动预测接口；实现负责噪声采样，滤波器负责调用顺序。
 */
class MotionModel
{
public:
  /**
   * @brief 通过抽象基类安全释放具体运动模型。
   */
  virtual ~MotionModel() = default;

  /**
   * @brief 将 odom 增量施加到粒子集合，具体噪声与运动学由实现决定。
   *
   * 本接口不负责权重归一化或重采样，调用方控制滤波更新顺序。
   *
   * @param particles 原地更新的候选位姿集合。
   * @param previous_odom_pose 上一次 odom 位姿，米/弧度。
   * @param current_odom_pose 本次同一 odom 帧下的位姿，米/弧度。
   */
  virtual void UpdateParticles(
    std::vector<Particle> & particles,
    const Pose2D & previous_odom_pose,
    const Pose2D & current_odom_pose) = 0;
};

}  // namespace mini_nav_core::localization

/**
 * @file pose_utils.hpp
 * @brief 提供二维位姿的刚体复合运算。
 *
 * 本文件约定 Pose2D 的 x、y 为米，yaw 为弧度。复合操作将一个在父坐标系中
 * 表达的子位姿转换为同一父坐标系下的绝对位姿，供定位过程组合坐标变换使用。
 *
 * @author Antinomy
 * @date 2026-09-13
 */
#pragma once

#include <cmath>

#include "mini_nav_core/nav_types/pose_2d.hpp"

namespace mini_nav_core::nav_types
{

using Transform2D = Pose2D;

/**
 * @brief 将父位姿与其坐标系中表达的子位姿复合。
 *
 * 先以父位姿的 yaw 将子位姿的平移量旋转到父坐标系，再加上父位姿的位置；
 * 两个偏航角相加后归一化，避免结果在 ±π 边界外而破坏后续角度比较。
 *
 * @param parent_pose 父坐标系相对于目标参考系的位姿；位置单位为米，偏航单位为弧度。
 * @param child_pose 子坐标系相对于父坐标系的位姿；位置单位为米，偏航单位为弧度。
 * @return 子坐标系相对于目标参考系的复合位姿；yaw 位于 [-π, π]。
 */
inline Pose2D ComposePose2D(const Pose2D & parent_pose, const Pose2D & child_pose)
{
  /*
   * child_pose 的平移量仍沿父坐标系的轴表达，必须先按父偏航旋转：
   *
   *   target ◄── parent ──► child
   *
   * 若直接相加，父坐标系发生旋转时会把局部前进错误地当作目标坐标系的 x 方向位移。
   *
   * ┌          ┐   ┌          ┐   ┌             ┐ ┌         ┐
   * │ x_global │   │ x_parent │   │ cosΦ  -sinΦ │ │ x_child │
   * │          │ = │          │ + │             │ │         │
   * │ y_global │   │ y_parent │   │ sinΦ   cosΦ │ │ y_child │
   * └          ┘   └          ┘   └             ┘ └         ┘
   */
  const double cosine = std::cos(parent_pose.yaw);
  const double sine = std::sin(parent_pose.yaw);
  return Pose2D{
    parent_pose.x + cosine * child_pose.x - sine * child_pose.y,
    parent_pose.y + sine * child_pose.x + cosine * child_pose.y,
    NormalizeAngle(parent_pose.yaw + child_pose.yaw)};
}

}  // namespace mini_nav_core::nav_types

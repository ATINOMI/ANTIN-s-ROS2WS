/**
 * @file laser_model.hpp
 * @brief 激光观测似然接口及旧方法名兼容分发。
 * @author Antinomy
 * @date 2026-10-01
 */
#pragma once

#include <stdexcept>
#include <vector>

#include "mini_nav_core/localization/amcl/laser_scan_data.hpp"
#include "mini_nav_core/localization/amcl/localization_map.hpp"

namespace mini_nav_core::localization
{

/**
 * @brief 观测似然抽象接口；兼容旧 UpdateWeights 名称，不负责归一化。
 */
class LaserModel
{
public:
  /**
   * @brief 通过抽象基类安全释放具体激光模型。
   */
  virtual ~LaserModel() = default;

  /**
   * @brief 把观测似然乘入粒子权重，默认转发给旧名称兼容入口。
   *
   * 新模型覆盖本入口；旧模型可只覆盖 UpdateWeights。权重归一化由粒子滤波器完成。
   *
   * @param particles 原地更新的粒子权重集合。
   * @param scan 扫描量程为米，角度为弧度。
   * @param map 提供射线或障碍距离查询的定位地图。
   * @param base_to_laser_pose 激光帧在底盘帧中的位姿，米/弧度。
   * @throws std::logic_error 具体模型未覆盖任一观测入口。
   */
  virtual void ApplyMeasurementLikelihood(
    std::vector<Particle> & particles,
    const LaserScanData & scan,
    const LocalizationMap & map,
    const Pose2D & base_to_laser_pose) const
  {
    // Dispatch to legacy models that still override UpdateWeights().
    UpdateWeights(particles, scan, map, base_to_laser_pose);
  }

  /**
   * @brief 保留旧名称观测模型入口，默认报告未实现。
   *
   * @param particles 原地更新的粒子权重集合。
   * @param scan 扫描量程为米，角度为弧度。
   * @param map 提供射线或障碍距离查询的定位地图。
   * @param base_to_laser_pose 激光帧在底盘帧中的位姿，米/弧度。
   * @throws std::logic_error 基类默认实现总是抛出；具体模型须覆盖本方法或规范入口。
   */
  virtual void UpdateWeights(
    std::vector<Particle> & particles,
    const LaserScanData & scan,
    const LocalizationMap & map,
    const Pose2D & base_to_laser_pose) const
  {
    (void)particles;
    (void)scan;
    (void)map;
    (void)base_to_laser_pose;
    throw std::logic_error(
      "LaserModel must override ApplyMeasurementLikelihood or UpdateWeights");
  }
};

}  // namespace mini_nav_core::localization

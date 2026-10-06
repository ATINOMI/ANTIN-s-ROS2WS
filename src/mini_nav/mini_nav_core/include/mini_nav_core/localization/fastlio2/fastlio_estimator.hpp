#pragma once

#include <memory>
#include "mini_nav_core/localization/fastlio2/fastlio_types.hpp"

namespace mini_nav_core::fastlio2 {
/** 单实例 FAST-LIO2 计算；调用者串行提供测量，内部不创建 ROS 节点。 */
class FastlioEstimator {
  public:
    explicit FastlioEstimator(const FastlioConfig &config);
    ~FastlioEstimator();
    FastlioEstimator(const FastlioEstimator &) = delete;
    FastlioEstimator &operator=(const FastlioEstimator &) = delete;
    /** 在先验索引首次建立前设置地图；运行中的会话重置由调用者重建实例。 */
    void SetPriorMap(const PointCloudXYZI::ConstPtr &map);
    /** 初值表示先验地图中的 IMU 位姿，只用于首次先验索引初始化。 */
    void SetInitialPose(const Eigen::Matrix4d &pose);
    const FastlioResult &Process(const MeasureGroup &measurements);
    const FastlioResult &Result() const;
    void PointBodyToWorld(const PointType *input, PointType *output) const;
    void PointLidarToImu(const PointType *input, PointType *output) const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}

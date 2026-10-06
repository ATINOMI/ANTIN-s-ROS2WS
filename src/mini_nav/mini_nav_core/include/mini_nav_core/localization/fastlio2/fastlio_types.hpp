#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace mini_nav_core::fastlio2 {
using PointType = pcl::PointXYZINormal;
using PointCloudXYZI = pcl::PointCloud<PointType>;
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

/** 带时间的 IMU 样本；时间单位秒，加速度 m/s²，角速度 rad/s。 */
struct ImuSample {
    using Ptr = std::shared_ptr<const ImuSample>;
    struct Vector {
        double x{0.0}, y{0.0}, z{0.0};
    };
    double stamp{0.0};
    Vector linear_acceleration, angular_velocity;
};

/** 一帧点云及覆盖它的 IMU 样本；curvature 保留上游毫秒点时间约定。 */
struct MeasureGroup {
    double lidar_beg_time{0.0}, lidar_end_time{0.0};
    PointCloudXYZI::Ptr lidar{new PointCloudXYZI()};
    std::deque<ImuSample::Ptr> imu;
};

struct Pose6D {
    double offset_time{0.0};
    std::array<double, 3> acc{}, gyr{}, vel{}, pos{};
    std::array<double, 9> rot{};
};

struct LivoxPoint {
    float x, y, z;
    std::uint32_t offset_time;
    std::uint8_t reflectivity, line, tag;
};
struct LivoxScan {
    std::uint32_t point_num{0};
    std::vector<LivoxPoint> points;
};

struct FastlioConfig {
    int iterations{4};
    double filter_size_surf{0.5}, filter_size_map{0.5}, cube_length{200.0};
    double detection_range{300.0}, fov_degree{180.0};
    double gyr_cov{0.1}, acc_cov{0.1}, gyro_bias_cov{0.0001}, acc_bias_cov{0.0001};
    Eigen::Vector3d extrinsic_translation{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d extrinsic_rotation{Eigen::Matrix3d::Identity()};
    bool snapshot_input{false}, estimate_extrinsics{true}, prior_mode{false};
    bool capture_map{false}, runtime_log{false};
    std::string log_root;
};

/** 借用的当前帧结果；点云随下一次 Process 修改，调用者须在下一帧前消费。 */
struct FastlioResult {
    double stamp{0.0};
    bool valid{false}, match_available{false}, prior_initialized{false}, map_captured{false};
    Eigen::Vector3d pos{Eigen::Vector3d::Zero()}, vel{Eigen::Vector3d::Zero()}, bg{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond rot{Eigen::Quaterniond::Identity()}, offset_R_L_I{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d offset_T_L_I{Eigen::Vector3d::Zero()};
    Eigen::Matrix<double, 6, 6> covariance{Eigen::Matrix<double, 6, 6>::Zero()};
    int effective_points{0}, input_points{0};
    double mean_abs_residual{0.0};
    PointCloudXYZI::ConstPtr undistorted, downsampled, effective, prior_map, map_points;
};
}

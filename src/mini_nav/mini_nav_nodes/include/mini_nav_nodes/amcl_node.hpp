/**
 * @file amcl_node.hpp
 * @brief 自研 AMCL 生命周期、消息适配、质量心跳及动态 TF。
 * @author Antinomy
 * @date 2026-10-01
 */
#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "bondcpp/bond.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "message_filters/subscriber.h"
#include "nav2_msgs/msg/particle_cloud.hpp"
#include "nav2_msgs/srv/set_initial_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

#include "mini_nav_core/localization/particle_filter.hpp"

namespace mini_nav_nodes
{

/**
 * @brief 把 ROS 生命周期、地图、扫描和 TF 适配到独立粒子滤波核心。
 */
class AmclNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief 声明定位参数与质量心跳，延迟到 configure 阶段建立滤波资源。
   *
   * 部分 Nav2 风格参数仅为加载兼容而声明，不代表 beam-skip 或位姿持久化已实现。
   *
   * @param options ROS 节点选项及参数覆盖。
   * @throws std::invalid_argument 定位质量阈值不合法。
   */
  explicit AmclNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief 撤销 context 预关闭回调、释放 bond 并断开激光过滤器连接。
   */
  ~AmclNode() override;

protected:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief 读取参数并建立回调组、TF、滤波器、通信和服务。
   *
   * @param state 生命周期转换前状态；当前实现不使用此值。
   * @return 初始化成功为 SUCCESS；捕获到标准异常时为 FAILURE。
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief 激活定位与可视化发布器，并建立生命周期管理器 bond。
   *
   * @param state 生命周期转换前状态；当前实现不使用。
   * @return 完成激活后返回 SUCCESS。
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief 关闭滤波更新和生命周期发布器，并释放 bond。
   *
   * @param state 生命周期转换前状态；当前实现不使用。
   * @return SUCCESS。
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief 释放通信、TF 与滤波资源并清除地图、位姿和 TF 缓存状态。
   *
   * @param state 生命周期转换前状态；当前实现不使用。
   * @return SUCCESS。
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief 停止更新、释放 bond 并使定位质量和 TF 缓存失效。
   *
   * @param state 生命周期转换前状态；当前实现不使用。
   * @return SUCCESS。
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  friend class AmclNodeTfCacheTest;
  /**
   * @brief 校验并转换地图，重建定位距离场并使旧定位状态失效。
   *
   * first_map_only 启用后忽略后续地图；转换失败记录错误。
   *
   * @param message global_frame_id 下的未旋转占据图；未知保持未知，正占据值视作障碍。
   */
  void mapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr message);
  /**
   * @brief 在互斥保护下处理初始位姿话题。
   *
   * @param message 全局坐标系的初始位姿与 ROS 6×6 协方差。
   */
  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr message);
  /**
   * @brief 以扫描时刻 TF 驱动运动预测、激光加权、归一化与重采样。
   *
   * 未达运动阈值时只刷新缓存 map→odom 时间戳，避免低速停顿时 TF 过期。
   * 首帧跳过运动预测；成功估计后才更新 odom 基准和发布结果。
   *
   * @param message 激光观测；至少 3 个有限有效量程且时间新鲜才进入更新。
   */
  void laserCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr message);

  /**
   * @brief 在地图自由空间全局采样粒子，并使旧 TF 和里程计基准失效。
   *
   * @param request_header 服务请求标识；未使用。
   * @param request 空请求；未使用。
   * @param response 空响应；未使用。
   */
  void globalLocalizationCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);
  /**
   * @brief 设置强制更新标志，使下一帧扫描绕过运动阈值。
   *
   * @param request_header 服务请求标识；未使用。
   * @param request 空请求；未使用。
   * @param response 空响应；未使用。
   */
  void nomotionUpdateCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);
  /**
   * @brief 通过服务复用初始位姿校验与粒子初始化。
   *
   * @param request_header 请求标识；未使用。
   * @param request 包含初始位姿与协方差。
   * @param response 空响应；未使用。
   */
  void setInitialPoseCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::SetInitialPose::Request> request,
    std::shared_ptr<nav2_msgs::srv::SetInitialPose::Response> response);

  /**
   * @brief 读取已声明参数，校验数量、帧名和支持的运动模型。
   * @throws std::invalid_argument 必要字段为空或数量、阈值及运动模型不支持。
   */
  void initializeParameters();
  /**
   * @brief 建立 TF 缓冲、独立监听线程和动态变换广播器。
   */
  void initializeTransforms();
  /**
   * @brief 配置地图保留型 QoS、激光 SensorDataQoS 与生命周期发布器。
   *
   * MessageFilter 等待扫描时刻到 odom 的 TF；互斥回调组自动加入 executor，
   * 否则节点虽有订阅连接，定位回调也不会得到调度。
   */
  void initializeCommunications();
  /**
   * @brief 根据配置构建差速运动模型、激光模型与粒子滤波器。
   * @throws std::invalid_argument 激光模型类型不支持或核心模型参数非法。
   */
  void initializeFilter();
  /**
   * @brief 建立全局定位、静止强制更新与初始位姿服务。
   */
  void initializeServices();
  /**
   * @brief 提取 x、y、yaw 协方差并在已知自由空间初始化粒子。
   *
   * 成功后清空 odom 基准和 TF 缓存，强制下一帧更新，避免跨定位周期复用结果。
   *
   * @param message 全局坐标系初始位姿；错误帧、无地图或非自由位置被拒绝。
   */
  void handleInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped & message);
  /**
   * @brief 查询指定时刻 TF 并提取二维位姿。
   *
   * @param target_frame 目标参考帧。
   * @param source_frame 被查询帧。
   * @param stamp TF 查询时刻。
   * @param pose 输出 source_frame 在 target_frame 中的二维位姿。
   * @return TF 可用且提取位姿有限时为 true；失败时输出不可使用。
   */
  bool getTransformPose(
    const std::string & target_frame,
    const std::string & source_frame,
    const rclcpp::Time & stamp,
    mini_nav_core::localization::Pose2D & pose) const;
  /**
   * @brief 判断强制更新或相对上次滤波的平移/转角是否达到阈值。
   *
   * @param pose 当前 odom 系机器人位姿。
   * @return 需要更新为 true；角误差采用周期归一化。
   */
  bool shouldUpdate(const mini_nav_core::localization::Pose2D & pose) const;
  /**
   * @brief 把 ROS 扫描转换为核心数据，并按配置收窄有效量程。
   *
   * @param message 原始扫描；角度为弧度、量程为米。
   * @return 独立扫描副本；下限取较大值、上限取较小值。
   */
  mini_nav_core::localization::LaserScanData convertScan(
    const sensor_msgs::msg::LaserScan & message) const;
  /**
   * @brief 将核心位姿与 3×3 协方差嵌入 ROS 6×6 消息并发布。
   *
   * 发布器未激活时不发布；未表示的 z、roll、pitch 协方差保持默认值。
   *
   * @param estimate map 系定位估计。
   * @param stamp 扫描时刻。
   * @param odom_pose 保留的接口参数；当前实现未使用。
   */
  void publishEstimate(
    const mini_nav_core::localization::PoseEstimate & estimate,
    const rclcpp::Time & stamp,
    const mini_nav_core::localization::Pose2D & odom_pose);
  /**
   * @brief 发布当前粒子的全局位姿与权重，用于定位可视化。
   *
   * @param stamp 扫描时刻。
   */
  void publishParticleCloud(const rclcpp::Time & stamp);
  /**
   * @brief 根据定位距离场缓存热力图、分段色块及网格线。
   *
   * 高度略微错开避免共面闪烁；这些消息只用于显示，不参与规划膨胀。
   *
   * @param map 提供几何和坐标系的原始地图；定位地图必须已经建立。
   */
  void updateDistanceFieldVisualization(const nav_msgs::msg::OccupancyGrid & map);
  /**
   * @brief 在节点激活且距离场缓存就绪时发布三种可视化消息。
   */
  void publishDistanceFieldVisualization();
  /**
   * @brief 清空定位质量及动态 map→odom 缓存，禁止复用旧定位结果。
   */
  void invalidateMapToOdom();
  /**
   * @brief 根据同一机器人在 map 和 odom 中的位姿计算并缓存 map→odom。
   *
   * T_map_odom = T_map_base × inverse(T_odom_base)。
   * 不能直接相减平移，因为两个参考系的坐标轴可能发生旋转。
   *
   * @param estimate map 系定位估计。
   * @param odom_pose 同一时刻 odom 系机器人位姿。
   */
  void cacheMapToOdom(
    const mini_nav_core::localization::PoseEstimate & estimate,
    const mini_nav_core::localization::Pose2D & odom_pose);
  /**
   * @brief 为有效 TF 缓存填充帧名与前推的扫描时间戳。
   *
   * 只刷新时间戳，不改变缓存几何；前推量为 transform_tolerance 秒。
   *
   * @param scan_stamp 本次扫描时刻。
   * @param message 输出变换消息；缓存无效时不修改。
   * @return 缓存有效时 true；尚未估计或缓存失效时 false。
   */
  bool makeCachedMapToOdomTransform(
    const rclcpp::Time & scan_stamp,
    geometry_msgs::msg::TransformStamped & message) const;
  /**
   * @brief 在 TF 广播启用且缓存有效时重发动态 map→odom。
   *
   * @param scan_stamp 本次扫描时刻，用于刷新 TF 有效期。
   */
  void publishCachedMapToOdom(const rclcpp::Time & scan_stamp);

  /**
   * @brief 在递归互斥保护下发布定位有效性心跳，包括未激活阶段的 false。
   */
  void publishQuality();
  /**
   * @brief 检查节点活跃、可信主簇、协方差、扫描双时钟年龄与 TF 缓存。
   * @return 全部满足质量限制为 true；用于运动前提，不是绝对定位精度保证。
   */
  bool localizationQualityValid() const;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr quality_publisher_;
  rclcpp::TimerBase::SharedPtr quality_timer_;
  mini_nav_core::localization::PoseEstimate quality_estimate_;
  std::chrono::steady_clock::time_point quality_scan_received_{};
  rclcpp::Time quality_scan_stamp_{0, 0, RCL_ROS_TIME};
  double quality_max_scan_age_{0.8};
  double quality_min_mass_{0.6};
  double quality_max_position_variance_{0.25};
  double quality_max_yaw_variance_{0.35};
  std::string global_frame_id_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string map_topic_;
  std::string scan_topic_;
  std::string laser_model_type_;
  std::string robot_model_type_;
  bool tf_broadcast_{true};
  bool set_initial_pose_{false};
  bool first_map_only_{true};
  bool initial_pose_known_{false};
  bool have_odom_pose_{false};
  bool map_to_odom_valid_{false};
  bool distance_field_ready_{false};
  double update_min_d_{0.25};
  double update_min_a_{0.2};
  double transform_tolerance_{0.5};
  double laser_likelihood_max_dist_{2.0};
  double max_range_override_{100.0};
  double min_range_override_{-1.0};
  double alpha1_{0.2};
  double alpha2_{0.2};
  double alpha3_{0.2};
  double alpha4_{0.2};
  double alpha5_{0.2};
  double z_hit_{0.5};
  double z_short_{0.05};
  double z_max_{0.05};
  double z_rand_{0.5};
  double sigma_hit_{0.2};
  double lambda_short_{0.1};
  std::size_t max_beams_{60};
  std::size_t min_particles_{500};
  std::size_t max_particles_{2000};
  double pf_err_{0.05};
  double kld_normal_quantile_{2.33};
  double recovery_alpha_fast_{0.0};
  double recovery_alpha_slow_{0.0};
  std::size_t resample_interval_{1};
  std::size_t resample_count_{0};

  std::atomic_bool active_{false};
  std::atomic_bool map_received_{false};
  std::atomic_bool force_update_{false};
  mutable std::recursive_mutex mutex_;
  std::shared_ptr<bond::Bond> bond_;
  std::unique_ptr<rclcpp::PreShutdownCallbackHandle> bond_shutdown_callback_handle_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<message_filters::Subscriber<
    sensor_msgs::msg::LaserScan, rclcpp_lifecycle::LifecycleNode>> laser_subscription_;
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> laser_filter_;
  message_filters::Connection laser_connection_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_subscription_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav2_msgs::msg::ParticleCloud>::SharedPtr particle_cloud_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr distance_field_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr distance_tiles_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr distance_grid_publisher_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr global_localization_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr nomotion_update_service_;
  rclcpp::Service<nav2_msgs::srv::SetInitialPose>::SharedPtr set_initial_pose_service_;

  std::unique_ptr<mini_nav_core::localization::LocalizationMap> localization_map_;
  std::unique_ptr<mini_nav_core::localization::ParticleFilter> particle_filter_;
  mini_nav_core::localization::Pose2D last_odom_pose_;
  geometry_msgs::msg::Transform cached_map_to_odom_;
  nav_msgs::msg::OccupancyGrid distance_field_message_;
  visualization_msgs::msg::Marker distance_tiles_message_;
  visualization_msgs::msg::Marker distance_grid_message_;
};

}  // namespace mini_nav_nodes

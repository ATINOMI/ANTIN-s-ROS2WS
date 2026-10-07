/**
 * @file amcl_node.cpp
 * @brief 自研 AMCL 生命周期、消息适配、质量心跳及动态 TF。
 * @author Antinomy
 * @date 2026-10-01
 */
#include "mini_nav_nodes/amcl_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <utility>

#include "nav2_msgs/msg/particle.hpp"
#include "tf2/utils.h"
#include "mini_nav_core/localization/beam_model.hpp"
#include "mini_nav_core/localization/differential_motion_model.hpp"
#include "mini_nav_core/localization/likelihood_field_model.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/create_timer_ros.h"

namespace mini_nav_nodes
{

namespace
{
/**
 * @brief 移除参考帧名开头的全部斜杠。
 *
 * @param frame 输入帧名副本。
 * @return 不含前导斜杠的帧名；全斜杠输入变为空串。
 */
std::string StripLeadingSlash(std::string frame)
{
  while (!frame.empty() && frame.front() == '/') {
    frame.erase(frame.begin());
  }
  return frame;
}

/**
 * @brief 将平面偏航角转换为 ROS 四元数。
 *
 * @param yaw 偏航弧度。
 * @return roll、pitch 为零的四元数。
 */
geometry_msgs::msg::Quaternion QuaternionFromYaw(double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(quaternion);
}

}  // namespace

/**
 * @brief 声明定位参数与质量心跳，延迟到 configure 阶段建立滤波资源。
 *
 * 部分 Nav2 风格参数仅为加载兼容而声明，不代表 beam-skip 或位姿持久化已实现。
 *
 * @param options ROS 节点选项及参数覆盖。
 * @throws std::invalid_argument 定位质量阈值不合法。
 */
AmclNode::AmclNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("amcl", options)
{
  quality_publisher_ = rclcpp::create_publisher<std_msgs::msg::Bool>(
    *this,
    "/mini_nav/localization_valid", rclcpp::QoS(1).reliable());
  quality_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { publishQuality(); });
  quality_max_scan_age_ = declare_parameter<double>("quality.max_scan_age", 0.8);
  quality_min_mass_ = declare_parameter<double>("quality.min_hypothesis_mass", 0.6);
  quality_max_position_variance_ = declare_parameter<double>("quality.max_position_variance", 0.25);
  quality_max_yaw_variance_ = declare_parameter<double>("quality.max_yaw_variance", 0.35);
  if (!std::isfinite(quality_max_scan_age_) || quality_max_scan_age_ <= 0.0 ||
      !std::isfinite(quality_min_mass_) || quality_min_mass_ <= 0.0 || quality_min_mass_ > 1.0 ||
      !std::isfinite(quality_max_position_variance_) || quality_max_position_variance_ <= 0.0 ||
      !std::isfinite(quality_max_yaw_variance_) || quality_max_yaw_variance_ <= 0.0)
      throw std::invalid_argument("Invalid localization quality limits");
  declare_parameter<std::string>("global_frame_id", "map");
  declare_parameter<std::string>("odom_frame_id", "odom");
  declare_parameter<std::string>("base_frame_id", "base_footprint");
  declare_parameter<std::string>("map_topic", "map");
  declare_parameter<std::string>("scan_topic", "scan");
  declare_parameter<std::string>("laser_model_type", "likelihood_field");
  declare_parameter<bool>("tf_broadcast", true);
  declare_parameter<bool>("set_initial_pose", false);
  declare_parameter<bool>("first_map_only", true);
  declare_parameter<double>("update_min_d", 0.25);
  declare_parameter<double>("update_min_a", 0.2);
  declare_parameter<double>("transform_tolerance", 0.5);
  declare_parameter<double>("laser_likelihood_max_dist", 2.0);
  declare_parameter<double>("laser_max_range", 100.0);
  declare_parameter<double>("laser_min_range", -1.0);
  declare_parameter<double>("alpha1", 0.2);
  declare_parameter<double>("alpha2", 0.2);
  declare_parameter<double>("alpha3", 0.2);
  declare_parameter<double>("alpha4", 0.2);
  declare_parameter<double>("alpha5", 0.2);
  declare_parameter<double>("z_hit", 0.5);
  declare_parameter<double>("z_short", 0.05);
  declare_parameter<double>("z_max", 0.05);
  declare_parameter<double>("z_rand", 0.5);
  declare_parameter<double>("sigma_hit", 0.2);
  declare_parameter<double>("lambda_short", 0.1);
  declare_parameter<int>("max_beams", 60);
  declare_parameter<int>("min_particles", 500);
  declare_parameter<int>("max_particles", 2000);
  declare_parameter<double>("pf_err", 0.05);
  declare_parameter<double>("pf_z", 2.33);
  declare_parameter<double>("recovery_alpha_fast", 0.0);
  declare_parameter<double>("recovery_alpha_slow", 0.0);
  declare_parameter<int>("resample_interval", 1);
  // Keep the existing Nav2-style parameter file loadable while the initial
  // implementation does not yet expose every recovery/tuning knob.
  declare_parameter<double>("beam_skip_distance", 0.5);
  declare_parameter<double>("beam_skip_error_threshold", 0.9);
  declare_parameter<double>("beam_skip_threshold", 0.3);
  declare_parameter<bool>("do_beamskip", false);
  declare_parameter<std::string>("robot_model_type", "nav2_amcl::DifferentialMotionModel");
  declare_parameter<double>("save_pose_rate", 0.5);
  declare_parameter<bool>("always_reset_initial_pose", false);
  declare_parameter<double>("initial_pose.x", 0.0);
  declare_parameter<double>("initial_pose.y", 0.0);
  declare_parameter<double>("initial_pose.z", 0.0);
  declare_parameter<double>("initial_pose.yaw", 0.0);
  // bondcpp 的定时器必须在 ROS context 关闭前释放。
  bond_shutdown_callback_handle_ = std::make_unique<rclcpp::PreShutdownCallbackHandle>(
    get_node_base_interface()->get_context()->add_pre_shutdown_callback([this]() {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      bond_.reset();
    }));
}

/**
 * @brief 撤销 context 预关闭回调、释放 bond 并断开激光过滤器连接。
 */
AmclNode::~AmclNode()
{
  get_node_base_interface()->get_context()->remove_pre_shutdown_callback(
    *bond_shutdown_callback_handle_);
  bond_.reset();
  laser_connection_.disconnect();
}

/**
 * @brief 读取参数并建立回调组、TF、滤波器、通信和服务。
 *
 * @note 无名参数（state）：生命周期转换前状态；当前实现不使用此值。
 * @return 初始化成功为 SUCCESS；捕获到标准异常时为 FAILURE。
 */
AmclNode::CallbackReturn AmclNode::on_configure(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  try {
    initializeParameters();
    callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, true);
    initializeTransforms();
    initializeFilter();
    initializeCommunications();
    initializeServices();
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(get_logger(), "AMCL configuration failed: %s", exception.what());
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

/**
 * @brief 激活定位与可视化发布器，并建立生命周期管理器 bond。
 *
 * @note 无名参数（state）：生命周期转换前状态；当前实现不使用。
 * @return 完成激活后返回 SUCCESS。
 */
AmclNode::CallbackReturn AmclNode::on_activate(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  pose_publisher_->on_activate();
  particle_cloud_publisher_->on_activate();
  distance_field_publisher_->on_activate();
  distance_tiles_publisher_->on_activate();
  distance_grid_publisher_->on_activate();
  active_ = true;
  publishDistanceFieldVisualization();
  bond_ = std::make_shared<bond::Bond>("bond", get_name(), shared_from_this());
  bond_->setHeartbeatPeriod(0.1);
  bond_->setHeartbeatTimeout(4.0);
  bond_->start();
  return CallbackReturn::SUCCESS;
}

/**
 * @brief 关闭滤波更新和生命周期发布器，并释放 bond。
 *
 * @note 无名参数（state）：生命周期转换前状态；当前实现不使用。
 * @return SUCCESS。
 */
AmclNode::CallbackReturn AmclNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  active_ = false;
  if (pose_publisher_) {
    pose_publisher_->on_deactivate();
  }
  if (particle_cloud_publisher_) {
    particle_cloud_publisher_->on_deactivate();
  }
  if (distance_field_publisher_) {
    distance_field_publisher_->on_deactivate();
  }
  if (distance_tiles_publisher_) {
    distance_tiles_publisher_->on_deactivate();
  }
  if (distance_grid_publisher_) {
    distance_grid_publisher_->on_deactivate();
  }
  bond_.reset();
  return CallbackReturn::SUCCESS;
}

/**
 * @brief 释放通信、TF 与滤波资源并清除地图、位姿和 TF 缓存状态。
 *
 * @note 无名参数（state）：生命周期转换前状态；当前实现不使用。
 * @return SUCCESS。
 */
AmclNode::CallbackReturn AmclNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  active_ = false;
  bond_.reset();
  laser_connection_.disconnect();
  laser_filter_.reset();
  laser_subscription_.reset();
  map_subscription_.reset();
  initial_pose_subscription_.reset();
  global_localization_service_.reset();
  nomotion_update_service_.reset();
  set_initial_pose_service_.reset();
  pose_publisher_.reset();
  particle_cloud_publisher_.reset();
  distance_field_publisher_.reset();
  distance_tiles_publisher_.reset();
  distance_grid_publisher_.reset();
  tf_broadcaster_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  localization_map_.reset();
  particle_filter_.reset();
  map_received_ = false;
  distance_field_ready_ = false;
  initial_pose_known_ = false;
  have_odom_pose_ = false;
  invalidateMapToOdom();
  resample_count_ = 0;
  return CallbackReturn::SUCCESS;
}

/**
 * @brief 停止更新、释放 bond 并使定位质量和 TF 缓存失效。
 *
 * @note 无名参数（state）：生命周期转换前状态；当前实现不使用。
 * @return SUCCESS。
 */
AmclNode::CallbackReturn AmclNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  active_ = false;
  bond_.reset();
  invalidateMapToOdom();
  return CallbackReturn::SUCCESS;
}

/**
 * @brief 读取已声明参数，校验数量、帧名和支持的运动模型。
 * @throws std::invalid_argument 必要字段为空或数量、阈值及运动模型不支持。
 */
void AmclNode::initializeParameters()
{
  get_parameter("global_frame_id", global_frame_id_);
  get_parameter("odom_frame_id", odom_frame_id_);
  get_parameter("base_frame_id", base_frame_id_);
  get_parameter("map_topic", map_topic_);
  get_parameter("scan_topic", scan_topic_);
  get_parameter("laser_model_type", laser_model_type_);
  get_parameter("robot_model_type", robot_model_type_);
  get_parameter("tf_broadcast", tf_broadcast_);
  get_parameter("set_initial_pose", set_initial_pose_);
  get_parameter("first_map_only", first_map_only_);
  get_parameter("update_min_d", update_min_d_);
  get_parameter("update_min_a", update_min_a_);
  get_parameter("transform_tolerance", transform_tolerance_);
  get_parameter("laser_likelihood_max_dist", laser_likelihood_max_dist_);
  get_parameter("laser_max_range", max_range_override_);
  get_parameter("laser_min_range", min_range_override_);
  get_parameter("alpha1", alpha1_);
  get_parameter("alpha2", alpha2_);
  get_parameter("alpha3", alpha3_);
  get_parameter("alpha4", alpha4_);
  get_parameter("alpha5", alpha5_);
  get_parameter("z_hit", z_hit_);
  get_parameter("z_short", z_short_);
  get_parameter("z_max", z_max_);
  get_parameter("z_rand", z_rand_);
  get_parameter("sigma_hit", sigma_hit_);
  get_parameter("lambda_short", lambda_short_);

  int max_beams = 0;
  int min_particles = 0;
  int max_particles = 0;
  int resample_interval = 0;
  get_parameter("max_beams", max_beams);
  get_parameter("min_particles", min_particles);
  get_parameter("max_particles", max_particles);
  get_parameter("resample_interval", resample_interval);
  if (max_beams <= 0 || min_particles <= 0 || max_particles < min_particles ||
      resample_interval <= 0) {
    throw std::invalid_argument("AMCL particle and beam counts must be positive");
  }
  max_beams_ = static_cast<std::size_t>(max_beams);
  min_particles_ = static_cast<std::size_t>(min_particles);
  max_particles_ = static_cast<std::size_t>(max_particles);
  resample_interval_ = static_cast<std::size_t>(resample_interval);
  get_parameter("pf_err", pf_err_);
  get_parameter("pf_z", kld_normal_quantile_);
  get_parameter("recovery_alpha_fast", recovery_alpha_fast_);
  get_parameter("recovery_alpha_slow", recovery_alpha_slow_);

  if (global_frame_id_.empty() || odom_frame_id_.empty() || base_frame_id_.empty() ||
      map_topic_.empty() || scan_topic_.empty() || robot_model_type_.empty()) {
    throw std::invalid_argument("AMCL frame and topic parameters must not be empty");
  }
  if (robot_model_type_ != "nav2_amcl::DifferentialMotionModel") {
    throw std::invalid_argument("Only nav2_amcl::DifferentialMotionModel is supported");
  }
  if (!std::isfinite(update_min_d_) || update_min_d_ < 0.0 ||
      !std::isfinite(update_min_a_) || update_min_a_ < 0.0 ||
      !std::isfinite(transform_tolerance_) || transform_tolerance_ < 0.0 ||
      max_beams_ == 0U || min_particles_ == 0U || max_particles_ < min_particles_ ||
      resample_interval_ == 0U) {
    throw std::invalid_argument("AMCL numeric parameters are invalid");
  }
  global_frame_id_ = StripLeadingSlash(global_frame_id_);
  odom_frame_id_ = StripLeadingSlash(odom_frame_id_);
  base_frame_id_ = StripLeadingSlash(base_frame_id_);
}

/**
 * @brief 建立 TF 缓冲、独立监听线程和动态变换广播器。
 */
void AmclNode::initializeTransforms()
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface(), callback_group_);
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
}

/**
 * @brief 根据配置构建差速运动模型、激光模型与粒子滤波器。
 * @throws std::invalid_argument 激光模型类型不支持或核心模型参数非法。
 */
void AmclNode::initializeFilter()
{
  auto motion_model = std::make_unique<mini_nav_core::localization::DifferentialMotionModel>(
    alpha1_, alpha2_, alpha3_, alpha4_, alpha5_,
    mini_nav_core::localization::kDefaultRandomSeed);
  std::unique_ptr<mini_nav_core::localization::LaserModel> laser_model;
  if (laser_model_type_ == "beam") {
    laser_model = std::make_unique<mini_nav_core::localization::BeamModel>(
      z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_, max_beams_);
  } else if (laser_model_type_ == "likelihood_field") {
    laser_model = std::make_unique<mini_nav_core::localization::LikelihoodFieldModel>(
      z_hit_, z_rand_, sigma_hit_, max_beams_);
  } else {
    throw std::invalid_argument("Unsupported laser_model_type: " + laser_model_type_);
  }

  mini_nav_core::localization::ParticleFilterOptions options;
  options.min_particles = min_particles_;
  options.max_particles = max_particles_;
  options.pf_err = pf_err_;
  options.kld_normal_quantile = kld_normal_quantile_;
  options.recovery_alpha_fast = recovery_alpha_fast_;
  options.recovery_alpha_slow = recovery_alpha_slow_;
  particle_filter_ = std::make_unique<mini_nav_core::localization::ParticleFilter>(
    std::move(motion_model), std::move(laser_model), options,
    mini_nav_core::localization::kDefaultRandomSeed);
}

/**
 * @brief 配置地图保留型 QoS、激光 SensorDataQoS 与生命周期发布器。
 *
 * MessageFilter 等待扫描时刻到 odom 的 TF；互斥回调组自动加入 executor，
 * 否则节点虽有订阅连接，定位回调也不会得到调度。
 */
void AmclNode::initializeCommunications()
{
  const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, map_qos,
    std::bind(&AmclNode::mapCallback, this, std::placeholders::_1), options);
  initial_pose_subscription_ = create_subscription<
    geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", rclcpp::SystemDefaultsQoS(),
    std::bind(&AmclNode::initialPoseCallback, this, std::placeholders::_1), options);

  pose_publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "amcl_pose", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  particle_cloud_publisher_ = create_publisher<nav2_msgs::msg::ParticleCloud>(
    "particle_cloud", rclcpp::SensorDataQoS());
  const auto visualization_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  distance_field_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "obstacle_distance_field", visualization_qos);
  distance_tiles_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
    "obstacle_distance_tiles", visualization_qos);
  distance_grid_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
    "obstacle_distance_grid", visualization_qos);

  laser_subscription_ = std::make_unique<message_filters::Subscriber<
    sensor_msgs::msg::LaserScan, rclcpp_lifecycle::LifecycleNode>>(
    shared_from_this(), scan_topic_, rmw_qos_profile_sensor_data, options);
  const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(transform_tolerance_));
  laser_filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
    *laser_subscription_, *tf_buffer_, odom_frame_id_, 10,
    get_node_logging_interface(), get_node_clock_interface(), timeout);
  laser_connection_ = laser_filter_->registerCallback(
    std::bind(&AmclNode::laserCallback, this, std::placeholders::_1));
}

/**
 * @brief 建立全局定位、静止强制更新与初始位姿服务。
 */
void AmclNode::initializeServices()
{
  global_localization_service_ = create_service<std_srvs::srv::Empty>(
    "reinitialize_global_localization",
    std::bind(&AmclNode::globalLocalizationCallback, this,
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  nomotion_update_service_ = create_service<std_srvs::srv::Empty>(
    "request_nomotion_update",
    std::bind(&AmclNode::nomotionUpdateCallback, this,
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  set_initial_pose_service_ = create_service<nav2_msgs::srv::SetInitialPose>(
    "set_initial_pose",
    std::bind(&AmclNode::setInitialPoseCallback, this,
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

/**
 * @brief 校验并转换地图，重建定位距离场并使旧定位状态失效。
 *
 * first_map_only 启用后忽略后续地图；转换失败记录错误。
 *
 * @param message global_frame_id 下的未旋转占据图；未知保持未知，正占据值视作障碍。
 */
void AmclNode::mapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr message)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (first_map_only_ && map_received_) {
    return;
  }
  if (StripLeadingSlash(message->header.frame_id) != global_frame_id_) {
    RCLCPP_WARN(get_logger(), "Ignoring map in frame '%s'; expected '%s'",
      message->header.frame_id.c_str(), global_frame_id_.c_str());
    return;
  }
  const double map_yaw = tf2::getYaw(message->info.origin.orientation);
  if (!std::isfinite(map_yaw) ||
      std::abs(mini_nav_core::localization::AngularDistance(map_yaw, 0.0)) > 1.0e-6) {
    RCLCPP_ERROR(get_logger(), "Ignoring maps with a rotated origin; only yaw=0 is supported");
    return;
  }
  const auto width = message->info.width;
  const auto height = message->info.height;
  const auto expected_size = static_cast<std::size_t>(width) * height;
  if (width == 0U || height == 0U || message->info.resolution <= 0.0F ||
      !std::isfinite(message->info.resolution) || message->data.size() != expected_size) {
    RCLCPP_ERROR(get_logger(), "Ignoring invalid map geometry or data size");
    return;
  }

  try {
    mini_nav_core::Costmap2D costmap(
      width, height, message->info.resolution,
      message->info.origin.position.x, message->info.origin.position.y,
      mini_nav_core::localization::LocalizationMap::kUnknownCost);
    for (unsigned int y = 0; y < height; ++y) {
      for (unsigned int x = 0; x < width; ++x) {
        const int8_t occupancy = message->data[static_cast<std::size_t>(y) * width + x];
        const unsigned char cost = occupancy < 0 ?
          mini_nav_core::localization::LocalizationMap::kUnknownCost :
          (occupancy > 0 ? mini_nav_core::localization::LocalizationMap::kLethalObstacle : 0);
        costmap.SetCost(x, y, cost);
      }
    }
    auto new_map = std::make_unique<mini_nav_core::localization::LocalizationMap>(
      costmap, laser_likelihood_max_dist_);
    localization_map_ = std::move(new_map);
    updateDistanceFieldVisualization(*message);
    map_received_ = true;
    initial_pose_known_ = false;
    have_odom_pose_ = false;
    invalidateMapToOdom();
    force_update_ = false;
    publishDistanceFieldVisualization();
    RCLCPP_INFO(get_logger(), "Loaded localization map: %u x %u at %.3f m/cell",
      width, height, message->info.resolution);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(get_logger(), "Failed to construct localization map: %s", exception.what());
  }
}

/**
 * @brief 根据定位距离场缓存热力图、分段色块及网格线。
 *
 * 高度略微错开避免共面闪烁；这些消息只用于显示，不参与规划膨胀。
 *
 * @param map 提供几何和坐标系的原始地图；定位地图必须已经建立。
 */
void AmclNode::updateDistanceFieldVisualization(const nav_msgs::msg::OccupancyGrid & map)
{
  distance_field_message_.header = map.header;
  distance_field_message_.info = map.info;
  // 把热力图抬离原始地图，避免两个栅格层共面闪烁。
  distance_field_message_.info.origin.position.z += 0.02;
  distance_field_message_.data.resize(map.data.size());
  const double max_distance = localization_map_->GetMaxObstacleDistance();

  auto & tiles = distance_tiles_message_;
  tiles.header = map.header;
  tiles.ns = "obstacle_distance_tiles";
  tiles.id = 0;
  tiles.type = visualization_msgs::msg::Marker::CUBE_LIST;
  tiles.action = visualization_msgs::msg::Marker::ADD;
  tiles.pose.orientation.w = 1.0;
  tiles.scale.x = map.info.resolution;
  tiles.scale.y = map.info.resolution;
  tiles.scale.z = 0.005;
  tiles.color.a = 1.0F;
  tiles.points.clear();
  tiles.colors.clear();
  tiles.points.reserve(map.data.size());
  tiles.colors.reserve(map.data.size());
  const double left = map.info.origin.position.x;
  const double bottom = map.info.origin.position.y;
  const double tile_z = map.info.origin.position.z + 0.02;
  // 深灰、红、琥珀、青、紫、浅灰：每个距离段只使用一种固定颜色。
  constexpr std::array<std::array<float, 3>, 6> colors{{
    {{0.188F, 0.204F, 0.247F}},
    {{0.773F, 0.231F, 0.196F}},
    {{0.937F, 0.749F, 0.224F}},
    {{0.322F, 0.706F, 0.784F}},
    {{0.525F, 0.392F, 0.702F}},
    {{0.851F, 0.871F, 0.890F}},
  }};
  for (unsigned int y = 0; y < map.info.height; ++y) {
    for (unsigned int x = 0; x < map.info.width; ++x) {
      const double distance = localization_map_->GetObstacleDistanceAtCell({x, y});
      // 100 表示障碍物格，0 表示距离已达到观测模型的截断上限。
      const double proximity = 1.0 - distance / max_distance;
      distance_field_message_.data[static_cast<std::size_t>(y) * map.info.width + x] =
        static_cast<int8_t>(std::lround(100.0 * std::clamp(proximity, 0.0, 1.0)));

      geometry_msgs::msg::Point point;
      point.x = left + (x + 0.5) * map.info.resolution;
      point.y = bottom + (y + 0.5) * map.info.resolution;
      point.z = tile_z;
      tiles.points.push_back(point);
      const double fraction = distance / max_distance;
      const std::size_t band = fraction <= 0.0 ? 0U :
        fraction < 0.125 ? 1U :
        fraction < 0.25 ? 2U :
        fraction < 0.5 ? 3U :
        fraction < 0.75 ? 4U : 5U;
      auto & color = tiles.colors.emplace_back();
      color.r = colors[band][0];
      color.g = colors[band][1];
      color.b = colors[band][2];
      color.a = 1.0F;
    }
  }

  auto & grid = distance_grid_message_;
  grid.header = map.header;
  grid.ns = "obstacle_distance_grid";
  grid.id = 0;
  grid.type = visualization_msgs::msg::Marker::LINE_LIST;
  grid.action = visualization_msgs::msg::Marker::ADD;
  grid.pose.orientation.w = 1.0;
  grid.scale.x = std::min(0.006, static_cast<double>(map.info.resolution) * 0.08);
  grid.color.r = 0.2F;
  grid.color.g = 0.2F;
  grid.color.b = 0.2F;
  grid.color.a = 0.65F;
  grid.points.clear();
  grid.points.reserve(2U * (map.info.width + map.info.height + 2U));
  const double right = left + map.info.width * map.info.resolution;
  const double top = bottom + map.info.height * map.info.resolution;
  const double z = map.info.origin.position.z + 0.03;
  for (unsigned int x = 0; x <= map.info.width; ++x) {
    geometry_msgs::msg::Point start;
    start.x = left + x * map.info.resolution;
    start.y = bottom;
    start.z = z;
    auto end = start;
    end.y = top;
    grid.points.push_back(start);
    grid.points.push_back(end);
  }
  for (unsigned int y = 0; y <= map.info.height; ++y) {
    geometry_msgs::msg::Point start;
    start.x = left;
    start.y = bottom + y * map.info.resolution;
    start.z = z;
    auto end = start;
    end.x = right;
    grid.points.push_back(start);
    grid.points.push_back(end);
  }
  distance_field_ready_ = true;
}

/**
 * @brief 在节点激活且距离场缓存就绪时发布三种可视化消息。
 */
void AmclNode::publishDistanceFieldVisualization()
{
  if (!active_ || !distance_field_ready_) {
    return;
  }
  distance_field_publisher_->publish(distance_field_message_);
  distance_tiles_publisher_->publish(distance_tiles_message_);
  distance_grid_publisher_->publish(distance_grid_message_);
}

/**
 * @brief 在互斥保护下处理初始位姿话题。
 *
 * @param message 全局坐标系的初始位姿与 ROS 6×6 协方差。
 */
void AmclNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr message)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  handleInitialPose(*message);
}

/**
 * @brief 提取 x、y、yaw 协方差并在已知自由空间初始化粒子。
 *
 * 成功后清空 odom 基准和 TF 缓存，强制下一帧更新，避免跨定位周期复用结果。
 *
 * @param message 全局坐标系初始位姿；错误帧、无地图或非自由位置被拒绝。
 */
void AmclNode::handleInitialPose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & message)
{
  if (!map_received_ || !localization_map_) {
    RCLCPP_WARN(get_logger(), "Ignoring initial pose before a map is available");
    return;
  }
  if (StripLeadingSlash(message.header.frame_id) != global_frame_id_) {
    RCLCPP_WARN(get_logger(), "Ignoring initial pose in frame '%s'; expected '%s'",
      message.header.frame_id.c_str(), global_frame_id_.c_str());
    return;
  }

  mini_nav_core::localization::Pose2D pose;
  pose.x = message.pose.pose.position.x;
  pose.y = message.pose.pose.position.y;
  pose.yaw = tf2::getYaw(message.pose.pose.orientation);
  mini_nav_core::localization::Covariance3 covariance;
  covariance.At(0, 0) = message.pose.covariance[0];
  covariance.At(0, 1) = message.pose.covariance[1];
  covariance.At(0, 2) = message.pose.covariance[5];
  covariance.At(1, 0) = message.pose.covariance[6];
  covariance.At(1, 1) = message.pose.covariance[7];
  covariance.At(1, 2) = message.pose.covariance[11];
  covariance.At(2, 0) = message.pose.covariance[30];
  covariance.At(2, 1) = message.pose.covariance[31];
  covariance.At(2, 2) = message.pose.covariance[35];

  try {
    mini_nav_core::localization::GridCell cell;
    if (!localization_map_->TryGetCellFromWorld(pose.x, pose.y, cell) ||
        !localization_map_->IsKnownFree(cell)) {
      RCLCPP_WARN(get_logger(), "Ignoring initial pose outside free map space");
      return;
    }
    particle_filter_->InitializeLocalized(pose, covariance);
    initial_pose_known_ = true;
    have_odom_pose_ = false;
    invalidateMapToOdom();
    force_update_ = true;
    RCLCPP_INFO(get_logger(), "Initialized AMCL at %.3f %.3f %.3f", pose.x, pose.y, pose.yaw);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize AMCL: %s", exception.what());
  }
}

/**
 * @brief 以扫描时刻 TF 驱动运动预测、激光加权、归一化与重采样。
 *
 * 未达运动阈值时只刷新缓存 map→odom 时间戳，避免低速停顿时 TF 过期。
 * 首帧跳过运动预测；成功估计后才更新 odom 基准和发布结果。
 *
 * @param message 激光观测；至少 3 个有限有效量程且时间新鲜才进入更新。
 */
void AmclNode::laserCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr message)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!active_ || !map_received_ || !localization_map_ || !initial_pose_known_) {
    return;
  }

  mini_nav_core::localization::Pose2D odom_pose;
  mini_nav_core::localization::Pose2D base_to_laser_pose;
  if (!getTransformPose(odom_frame_id_, base_frame_id_, message->header.stamp, odom_pose) ||
      !getTransformPose(base_frame_id_, StripLeadingSlash(message->header.frame_id),
      message->header.stamp, base_to_laser_pose)) {
    RCLCPP_DEBUG(get_logger(), "Skipping scan because the required TF is unavailable");
    return;
  }

  const double age = (now() - rclcpp::Time(message->header.stamp, get_clock()->get_clock_type())).seconds();
  const auto useful = std::count_if(message->ranges.begin(), message->ranges.end(), [&](float range) {
    return std::isfinite(range) && range >= message->range_min && range <= message->range_max;
  });
  if (!std::isfinite(age) || age < -0.1 || age > quality_max_scan_age_ || useful < 3 ||
      !std::isfinite(message->angle_increment) || message->angle_increment == 0.0) {
    quality_estimate_.valid = false; force_update_ = true; publishQuality(); return;
  }
  quality_scan_received_ = std::chrono::steady_clock::now();
  quality_scan_stamp_ = rclcpp::Time(message->header.stamp, get_clock()->get_clock_type());
  const bool first_update = !have_odom_pose_;
  if (!first_update && !shouldUpdate(odom_pose)) {
    publishCachedMapToOdom(message->header.stamp);
    return;
  }
  try {
    if (!first_update) {
      particle_filter_->MotionUpdate(last_odom_pose_, odom_pose);
    }
    particle_filter_->SensorUpdate(
      convertScan(*message), *localization_map_, base_to_laser_pose);
    quality_estimate_.valid = false;
    if (!particle_filter_->NormalizeWeights()) {
      RCLCPP_WARN(get_logger(), "AMCL received a scan with unusable particle weights");
      return;
    }
    ++resample_count_;
    if (resample_count_ % resample_interval_ == 0U && !particle_filter_->Resample(*localization_map_)) {
      RCLCPP_WARN(get_logger(), "AMCL particle resampling failed");
      return;
    }
    const auto estimate = particle_filter_->Estimate();
    if (!estimate.valid) {
      return;
    }
    quality_estimate_ = estimate;
    last_odom_pose_ = odom_pose;
    have_odom_pose_ = true;
    force_update_ = false;
    publishEstimate(estimate, message->header.stamp, odom_pose);
    publishParticleCloud(message->header.stamp);
    cacheMapToOdom(estimate, odom_pose);
    publishCachedMapToOdom(message->header.stamp);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(get_logger(), "AMCL update failed: %s", exception.what());
  }
}

/**
 * @brief 在地图自由空间全局采样粒子，并使旧 TF 和里程计基准失效。
 *
 * @note 无名参数（request_header）：服务请求标识；未使用。
 * @note 无名参数（request）：空请求；未使用。
 * @note 无名参数（response）：空响应；未使用。
 */
void AmclNode::globalLocalizationCallback(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<std_srvs::srv::Empty::Request>,
  std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!localization_map_ || !particle_filter_) {
    RCLCPP_WARN(get_logger(), "Cannot globally initialize before a map is available");
    return;
  }
  try {
    particle_filter_->InitializeGlobal(*localization_map_);
    initial_pose_known_ = true;
    have_odom_pose_ = false;
    invalidateMapToOdom();
    force_update_ = true;
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(get_logger(), "Global localization failed: %s", exception.what());
  }
}

/**
 * @brief 设置强制更新标志，使下一帧扫描绕过运动阈值。
 *
 * @note 无名参数（request_header）：服务请求标识；未使用。
 * @note 无名参数（request）：空请求；未使用。
 * @note 无名参数（response）：空响应；未使用。
 */
void AmclNode::nomotionUpdateCallback(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<std_srvs::srv::Empty::Request>,
  std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  force_update_ = true;
}

/**
 * @brief 通过服务复用初始位姿校验与粒子初始化。
 *
 * @note 无名参数（request_header）：请求标识；未使用。
 * @param request 包含初始位姿与协方差。
 * @note 无名参数（response）：空响应；未使用。
 */
void AmclNode::setInitialPoseCallback(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<nav2_msgs::srv::SetInitialPose::Request> request,
  std::shared_ptr<nav2_msgs::srv::SetInitialPose::Response>)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  handleInitialPose(request->pose);
}

/**
 * @brief 查询指定时刻 TF 并提取二维位姿。
 *
 * @param target_frame 目标参考帧。
 * @param source_frame 被查询帧。
 * @param stamp TF 查询时刻。
 * @param pose 输出 source_frame 在 target_frame 中的二维位姿。
 * @return TF 可用且提取位姿有限时为 true；失败时输出不可使用。
 */
bool AmclNode::getTransformPose(
  const std::string & target_frame,
  const std::string & source_frame,
  const rclcpp::Time & stamp,
  mini_nav_core::localization::Pose2D & pose) const
{
  try {
    const auto transform = tf_buffer_->lookupTransform(
      target_frame, source_frame, stamp, rclcpp::Duration::from_seconds(0.1));
    pose.x = transform.transform.translation.x;
    pose.y = transform.transform.translation.y;
    pose.yaw = tf2::getYaw(transform.transform.rotation);
    return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw);
  } catch (const tf2::TransformException &) {
    return false;
  }
}

/**
 * @brief 判断强制更新或相对上次滤波的平移/转角是否达到阈值。
 *
 * @param pose 当前 odom 系机器人位姿。
 * @return 需要更新为 true；角误差采用周期归一化。
 */
bool AmclNode::shouldUpdate(const mini_nav_core::localization::Pose2D & pose) const
{
  if (force_update_) {
    return true;
  }
  return std::hypot(pose.x - last_odom_pose_.x, pose.y - last_odom_pose_.y) >= update_min_d_ ||
         std::abs(mini_nav_core::localization::AngularDistance(pose.yaw, last_odom_pose_.yaw)) >=
         update_min_a_;
}

/**
 * @brief 把 ROS 扫描转换为核心数据，并按配置收窄有效量程。
 *
 * @param message 原始扫描；角度为弧度、量程为米。
 * @return 独立扫描副本；下限取较大值、上限取较小值。
 */
mini_nav_core::localization::LaserScanData AmclNode::convertScan(
  const sensor_msgs::msg::LaserScan & message) const
{
  mini_nav_core::localization::LaserScanData scan;
  scan.ranges.assign(message.ranges.begin(), message.ranges.end());
  scan.angle_min = message.angle_min;
  scan.angle_increment = message.angle_increment;
  scan.range_min = message.range_min;
  scan.range_max = message.range_max;
  if (min_range_override_ >= 0.0) {
    scan.range_min = std::max(scan.range_min, min_range_override_);
  }
  if (max_range_override_ > 0.0) {
    scan.range_max = std::min(scan.range_max, max_range_override_);
  }
  return scan;
}

/**
 * @brief 将核心位姿与 3×3 协方差嵌入 ROS 6×6 消息并发布。
 *
 * 发布器未激活时不发布；未表示的 z、roll、pitch 协方差保持默认值。
 *
 * @param estimate map 系定位估计。
 * @param stamp 扫描时刻。
 * @note 无名参数（odom_pose）：保留的接口参数；当前实现未使用。
 */
void AmclNode::publishEstimate(
  const mini_nav_core::localization::PoseEstimate & estimate,
  const rclcpp::Time & stamp,
  const mini_nav_core::localization::Pose2D &)
{
  if (!pose_publisher_ || !pose_publisher_->is_activated()) {
    return;
  }
  geometry_msgs::msg::PoseWithCovarianceStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = global_frame_id_;
  message.pose.pose.position.x = estimate.pose.x;
  message.pose.pose.position.y = estimate.pose.y;
  message.pose.pose.orientation = QuaternionFromYaw(estimate.pose.yaw);
  message.pose.covariance[0] = estimate.covariance.At(0, 0);
  message.pose.covariance[1] = estimate.covariance.At(0, 1);
  message.pose.covariance[5] = estimate.covariance.At(0, 2);
  message.pose.covariance[6] = estimate.covariance.At(1, 0);
  message.pose.covariance[7] = estimate.covariance.At(1, 1);
  message.pose.covariance[11] = estimate.covariance.At(1, 2);
  message.pose.covariance[30] = estimate.covariance.At(2, 0);
  message.pose.covariance[31] = estimate.covariance.At(2, 1);
  message.pose.covariance[35] = estimate.covariance.At(2, 2);
  pose_publisher_->publish(message);
}

/**
 * @brief 发布当前粒子的全局位姿与权重，用于定位可视化。
 *
 * @param stamp 扫描时刻。
 */
void AmclNode::publishParticleCloud(const rclcpp::Time & stamp)
{
  if (!particle_cloud_publisher_ || !particle_cloud_publisher_->is_activated()) {
    return;
  }
  nav2_msgs::msg::ParticleCloud message;
  message.header.stamp = stamp;
  message.header.frame_id = global_frame_id_;
  const auto & particles = particle_filter_->GetParticles();
  message.particles.reserve(particles.size());
  for (const auto & particle : particles) {
    nav2_msgs::msg::Particle output;
    output.pose.position.x = particle.pose.x;
    output.pose.position.y = particle.pose.y;
    output.pose.orientation = QuaternionFromYaw(particle.pose.yaw);
    output.weight = particle.weight;
    message.particles.push_back(output);
  }
  particle_cloud_publisher_->publish(message);
}

/**
 * @brief 清空定位质量及动态 map→odom 缓存，禁止复用旧定位结果。
 */
void AmclNode::invalidateMapToOdom()
{
  quality_estimate_ = {};
  quality_scan_received_ = {};
  map_to_odom_valid_ = false;
  cached_map_to_odom_ = geometry_msgs::msg::Transform();
}

/**
 * @brief 根据同一机器人在 map 和 odom 中的位姿计算并缓存 map→odom。
 *
 * T_map_odom = T_map_base × inverse(T_odom_base)。
 * 不能直接相减平移，因为两个参考系的坐标轴可能发生旋转。
 *
 * @param estimate map 系定位估计。
 * @param odom_pose 同一时刻 odom 系机器人位姿。
 */
void AmclNode::cacheMapToOdom(
  const mini_nav_core::localization::PoseEstimate & estimate,
  const mini_nav_core::localization::Pose2D & odom_pose)
{
  tf2::Transform map_to_base;
  tf2::Quaternion map_rotation;
  map_rotation.setRPY(0.0, 0.0, estimate.pose.yaw);
  map_to_base.setOrigin(tf2::Vector3(estimate.pose.x, estimate.pose.y, 0.0));
  map_to_base.setRotation(map_rotation);

  tf2::Transform odom_to_base;
  tf2::Quaternion odom_rotation;
  odom_rotation.setRPY(0.0, 0.0, odom_pose.yaw);
  odom_to_base.setOrigin(tf2::Vector3(odom_pose.x, odom_pose.y, 0.0));
  odom_to_base.setRotation(odom_rotation);

  const tf2::Transform map_to_odom = map_to_base * odom_to_base.inverse();
  cached_map_to_odom_ = tf2::toMsg(map_to_odom);
  map_to_odom_valid_ = true;
}

/**
 * @brief 为有效 TF 缓存填充帧名与前推的扫描时间戳。
 *
 * 只刷新时间戳，不改变缓存几何；前推量为 transform_tolerance 秒。
 *
 * @param scan_stamp 本次扫描时刻。
 * @param message 输出变换消息；缓存无效时不修改。
 * @return 缓存有效时 true；尚未估计或缓存失效时 false。
 */
bool AmclNode::makeCachedMapToOdomTransform(
  const rclcpp::Time & scan_stamp,
  geometry_msgs::msg::TransformStamped & message) const
{
  if (!map_to_odom_valid_) {
    return false;
  }
  message.header.frame_id = global_frame_id_;
  message.child_frame_id = odom_frame_id_;
  message.transform = cached_map_to_odom_;
  message.header.stamp = scan_stamp + rclcpp::Duration::from_seconds(transform_tolerance_);
  return true;
}

/**
 * @brief 在 TF 广播启用且缓存有效时重发动态 map→odom。
 *
 * @param scan_stamp 本次扫描时刻，用于刷新 TF 有效期。
 */
void AmclNode::publishCachedMapToOdom(const rclcpp::Time & scan_stamp)
{
  if (!tf_broadcast_ || !tf_broadcaster_) {
    return;
  }
  geometry_msgs::msg::TransformStamped message;
  if (makeCachedMapToOdomTransform(scan_stamp, message)) {
    tf_broadcaster_->sendTransform(message);
  }
}

}  // namespace mini_nav_nodes


/**
 * @brief 检查节点活跃、可信主簇、协方差、扫描双时钟年龄与 TF 缓存。
 * @return 全部满足质量限制为 true；用于运动前提，不是绝对定位精度保证。
 */
bool mini_nav_nodes::AmclNode::localizationQualityValid() const
{
    const double age = (now() - quality_scan_stamp_).seconds();
    bool valid = active_ && map_to_odom_valid_ && initial_pose_known_ && quality_estimate_.valid &&
        quality_scan_received_ != std::chrono::steady_clock::time_point{} &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - quality_scan_received_).count() <= quality_max_scan_age_ &&
        std::isfinite(age) && age >= -0.1 && age <= quality_max_scan_age_ &&
        quality_estimate_.hypothesis_mass >= quality_min_mass_;
    for (int i = 0; i < 3; ++i) {
        const double variance = quality_estimate_.covariance.At(i, i);
        valid = valid && std::isfinite(variance) && variance >= 0.0 &&
          variance <= (i == 2 ? quality_max_yaw_variance_ : quality_max_position_variance_);
    }
    return valid;
}

/**
 * @brief 在递归互斥保护下发布定位有效性心跳，包括未激活阶段的 false。
 */
void mini_nav_nodes::AmclNode::publishQuality()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std_msgs::msg::Bool message;
    message.data = localizationQualityValid();
    quality_publisher_->publish(message);
}

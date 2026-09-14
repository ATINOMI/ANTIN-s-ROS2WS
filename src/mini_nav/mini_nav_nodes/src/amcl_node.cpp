#include "mini_nav_nodes/amcl_node.hpp"

#include <algorithm>
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
std::string StripLeadingSlash(std::string frame)
{
  while (!frame.empty() && frame.front() == '/') {
    frame.erase(frame.begin());
  }
  return frame;
}

geometry_msgs::msg::Quaternion QuaternionFromYaw(double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(quaternion);
}

}  // namespace

AmclNode::AmclNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("amcl", options)
{
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
}

AmclNode::~AmclNode()
{
  laser_connection_.disconnect();
}

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

AmclNode::CallbackReturn AmclNode::on_activate(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  pose_publisher_->on_activate();
  particle_cloud_publisher_->on_activate();
  active_ = true;
  return CallbackReturn::SUCCESS;
}

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
  return CallbackReturn::SUCCESS;
}

AmclNode::CallbackReturn AmclNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  active_ = false;
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
  tf_broadcaster_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  localization_map_.reset();
  particle_filter_.reset();
  map_received_ = false;
  initial_pose_known_ = false;
  have_odom_pose_ = false;
  invalidateMapToOdom();
  resample_count_ = 0;
  return CallbackReturn::SUCCESS;
}

AmclNode::CallbackReturn AmclNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  active_ = false;
  invalidateMapToOdom();
  return CallbackReturn::SUCCESS;
}

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

void AmclNode::initializeTransforms()
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface(), callback_group_);
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
}

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
    map_received_ = true;
    initial_pose_known_ = false;
    have_odom_pose_ = false;
    invalidateMapToOdom();
    force_update_ = false;
    RCLCPP_INFO(get_logger(), "Loaded localization map: %u x %u at %.3f m/cell",
      width, height, message->info.resolution);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(get_logger(), "Failed to construct localization map: %s", exception.what());
  }
}

void AmclNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr message)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  handleInitialPose(*message);
}

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
    if (!particle_filter_->NormalizeWeights()) {
      RCLCPP_WARN(get_logger(), "AMCL received a scan with unusable particle weights");
      return;
    }
    ++resample_count_;
    if (resample_count_ % resample_interval_ == 0U && !particle_filter_->Resample()) {
      RCLCPP_WARN(get_logger(), "AMCL particle resampling failed");
      return;
    }
    const auto estimate = particle_filter_->Estimate();
    if (!estimate.valid) {
      return;
    }
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

void AmclNode::nomotionUpdateCallback(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<std_srvs::srv::Empty::Request>,
  std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  force_update_ = true;
}

void AmclNode::setInitialPoseCallback(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<nav2_msgs::srv::SetInitialPose::Request> request,
  std::shared_ptr<nav2_msgs::srv::SetInitialPose::Response>)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  handleInitialPose(request->pose);
}

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

bool AmclNode::shouldUpdate(const mini_nav_core::localization::Pose2D & pose) const
{
  if (force_update_) {
    return true;
  }
  return std::hypot(pose.x - last_odom_pose_.x, pose.y - last_odom_pose_.y) >= update_min_d_ ||
         std::abs(mini_nav_core::localization::AngularDistance(pose.yaw, last_odom_pose_.yaw)) >=
         update_min_a_;
}

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

void AmclNode::invalidateMapToOdom()
{
  map_to_odom_valid_ = false;
  cached_map_to_odom_ = geometry_msgs::msg::Transform();
}

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

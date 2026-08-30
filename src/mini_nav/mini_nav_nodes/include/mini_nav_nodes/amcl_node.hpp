#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "message_filters/subscriber.h"
#include "nav2_msgs/msg/particle_cloud.hpp"
#include "nav2_msgs/srv/set_initial_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "mini_nav_core/localization/particle_filter.hpp"

namespace mini_nav_nodes
{

class AmclNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit AmclNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~AmclNode() override;

protected:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr message);
  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr message);
  void laserCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr message);

  void globalLocalizationCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);
  void nomotionUpdateCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);
  void setInitialPoseCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::SetInitialPose::Request> request,
    std::shared_ptr<nav2_msgs::srv::SetInitialPose::Response> response);

  void initializeParameters();
  void initializeTransforms();
  void initializeCommunications();
  void initializeFilter();
  void initializeServices();
  void handleInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped & message);
  bool getTransformPose(
    const std::string & target_frame,
    const std::string & source_frame,
    const rclcpp::Time & stamp,
    mini_nav_core::localization::Pose2D & pose) const;
  bool shouldUpdate(const mini_nav_core::localization::Pose2D & pose) const;
  mini_nav_core::localization::LaserScanData convertScan(
    const sensor_msgs::msg::LaserScan & message) const;
  void publishEstimate(
    const mini_nav_core::localization::PoseEstimate & estimate,
    const rclcpp::Time & stamp,
    const mini_nav_core::localization::Pose2D & odom_pose);
  void publishParticleCloud(const rclcpp::Time & stamp);
  void publishMapToOdom(
    const mini_nav_core::localization::PoseEstimate & estimate,
    const mini_nav_core::localization::Pose2D & odom_pose,
    const rclcpp::Time & stamp);

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
  double pf_z_{2.33};
  double recovery_alpha_fast_{0.0};
  double recovery_alpha_slow_{0.0};
  std::size_t resample_interval_{1};
  std::size_t resample_count_{0};

  std::atomic_bool active_{false};
  std::atomic_bool map_received_{false};
  std::atomic_bool force_update_{false};
  mutable std::recursive_mutex mutex_;

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
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr global_localization_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr nomotion_update_service_;
  rclcpp::Service<nav2_msgs::srv::SetInitialPose>::SharedPtr set_initial_pose_service_;

  std::unique_ptr<mini_nav_core::localization::LocalizationMap> localization_map_;
  std::unique_ptr<mini_nav_core::localization::ParticleFilter> particle_filter_;
  mini_nav_core::localization::Pose2D last_odom_pose_;
  mini_nav_core::localization::Pose2D laser_pose_in_base_;
};

}  // namespace mini_nav_nodes

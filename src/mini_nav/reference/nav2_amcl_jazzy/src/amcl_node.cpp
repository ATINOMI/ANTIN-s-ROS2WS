/*
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* Author: Brian Gerkey */

#include "nav2_amcl/amcl_node.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "message_filters/subscriber.h"
#include "nav2_amcl/angleutils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_amcl/pf/pf.hpp"
#include "nav2_util/string_utils.hpp"
#include "nav2_amcl/sensors/laser/laser.hpp"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

#include "nav2_amcl/portable_utils.hpp"
#include "nav2_util/validate_messages.hpp"

using namespace std::placeholders;
using rcl_interfaces::msg::ParameterType;
using namespace std::chrono_literals;

namespace nav2_amcl
{
using nav2_util::geometry_utils::orientationAroundZAxis;

AmclNode::AmclNode(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("amcl", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating");

  /*
  *init_pose_是一个长度为3的数组，用于存储机器人的初始位姿信息，包括x坐标、y坐标和朝向角度（yaw）。
  *init_cov_是一个长度为3的数组，用于存储机器人的初始位姿的不确定性信息，包括x坐标、y坐标和朝向角度（yaw）的协方差。
  *这些数组在AMCL算法中用于初始化粒子滤波器的状态和不确定性，以便在机器人定位过程中进行估计和更新
  */
  init_pose_[0] = 0.0; 
  init_pose_[1] = 0.0;
  init_pose_[2] = 0.0;
  init_cov_[0] = 0.0;
  init_cov_[1] = 0.0;
  init_cov_[2] = 0.0;

  // 添加参数，使用 add_parameter 函数声明参数，并设置默认值、描述和约束条件
  add_parameter(
    "alpha1", rclcpp::ParameterValue(0.2),
    "This is the alpha1 parameter", "These are additional constraints for alpha1");

  add_parameter(
    "alpha2", rclcpp::ParameterValue(0.2),
    "This is the alpha2 parameter", "These are additional constraints for alpha2");

  add_parameter(
    "alpha3", rclcpp::ParameterValue(0.2),
    "This is the alpha3 parameter", "These are additional constraints for alpha3");

  add_parameter(
    "alpha4", rclcpp::ParameterValue(0.2),
    "This is the alpha4 parameter", "These are additional constraints for alpha4");

  add_parameter(
    "alpha5", rclcpp::ParameterValue(0.2),
    "This is the alpha5 parameter", "These are additional constraints for alpha5");

  add_parameter(
    "base_frame_id", rclcpp::ParameterValue(std::string("base_footprint")),
    "Which frame to use for the robot base");

  add_parameter("beam_skip_distance", rclcpp::ParameterValue(0.5));
  add_parameter("beam_skip_error_threshold", rclcpp::ParameterValue(0.9));
  add_parameter("beam_skip_threshold", rclcpp::ParameterValue(0.3));
  add_parameter("do_beamskip", rclcpp::ParameterValue(false));

  add_parameter(
    "global_frame_id", rclcpp::ParameterValue(std::string("map")),
    "The name of the coordinate frame published by the localization system");

  add_parameter(
    "lambda_short", rclcpp::ParameterValue(0.1),
    "Exponential decay parameter for z_short part of model");

  add_parameter(
    "laser_likelihood_max_dist", rclcpp::ParameterValue(2.0),
    "Maximum distance to do obstacle inflation on map, for use in likelihood_field model");

  add_parameter(
    "laser_max_range", rclcpp::ParameterValue(100.0),
    "Maximum scan range to be considered",
    "-1.0 will cause the laser's reported maximum range to be used");

  add_parameter(
    "laser_min_range", rclcpp::ParameterValue(-1.0),
    "Minimum scan range to be considered",
    "-1.0 will cause the laser's reported minimum range to be used");

  add_parameter(
    "laser_model_type", rclcpp::ParameterValue(std::string("likelihood_field")),
    "Which model to use, either beam, likelihood_field, or likelihood_field_prob",
    "Same as likelihood_field but incorporates the beamskip feature, if enabled");

  add_parameter(
    "set_initial_pose", rclcpp::ParameterValue(false),
    "Causes AMCL to set initial pose from the initial_pose* parameters instead of "
    "waiting for the initial_pose message");

  add_parameter(
    "initial_pose.x", rclcpp::ParameterValue(0.0),
    "X coordinate of the initial robot pose in the map frame");

  add_parameter(
    "initial_pose.y", rclcpp::ParameterValue(0.0),
    "Y coordinate of the initial robot pose in the map frame");

  add_parameter(
    "initial_pose.z", rclcpp::ParameterValue(0.0),
    "Z coordinate of the initial robot pose in the map frame");

  add_parameter(
    "initial_pose.yaw", rclcpp::ParameterValue(0.0),
    "Yaw of the initial robot pose in the map frame");

  add_parameter(
    "max_beams", rclcpp::ParameterValue(60),
    "How many evenly-spaced beams in each scan to be used when updating the filter");

  add_parameter(
    "max_particles", rclcpp::ParameterValue(2000),
    "Maximum allowed number of particles");

  add_parameter(
    "min_particles", rclcpp::ParameterValue(500),
    "Minimum allowed number of particles");

  add_parameter(
    "odom_frame_id", rclcpp::ParameterValue(std::string("odom")),
    "Which frame to use for odometry");

  add_parameter("pf_err", rclcpp::ParameterValue(0.05));
  add_parameter("pf_z", rclcpp::ParameterValue(0.99));

  add_parameter(
    "recovery_alpha_fast", rclcpp::ParameterValue(0.0),
    "Exponential decay rate for the fast average weight filter, used in deciding when to recover "
    "by adding random poses",
    "A good value might be 0.1");

  add_parameter(
    "recovery_alpha_slow", rclcpp::ParameterValue(0.0),
    "Exponential decay rate for the slow average weight filter, used in deciding when to recover "
    "by adding random poses",
    "A good value might be 0.001");

  add_parameter(
    "resample_interval", rclcpp::ParameterValue(1),
    "Number of filter updates required before resampling");

  add_parameter("robot_model_type", rclcpp::ParameterValue("nav2_amcl::DifferentialMotionModel"));

  add_parameter(
    "save_pose_rate", rclcpp::ParameterValue(0.5),
    "Maximum rate (Hz) at which to store the last estimated pose and covariance to the parameter "
    "server, in the variables ~initial_pose_* and ~initial_cov_*. This saved pose will be used "
    "on subsequent runs to initialize the filter",
    "-1.0 to disable");

  add_parameter("sigma_hit", rclcpp::ParameterValue(0.2));

  add_parameter(
    "tf_broadcast", rclcpp::ParameterValue(true),
    "Set this to false to prevent amcl from publishing the transform between the global frame and "
    "the odometry frame");

  add_parameter(
    "transform_tolerance", rclcpp::ParameterValue(1.0),
    "Time with which to post-date the transform that is published, to indicate that this transform "
    "is valid into the future");

  add_parameter(
    "update_min_a", rclcpp::ParameterValue(0.2),
    "Rotational movement required before performing a filter update");

  add_parameter(
    "update_min_d", rclcpp::ParameterValue(0.25),
    "Translational movement required before performing a filter update");

  add_parameter("z_hit", rclcpp::ParameterValue(0.5));
  add_parameter("z_max", rclcpp::ParameterValue(0.05));
  add_parameter("z_rand", rclcpp::ParameterValue(0.5));
  add_parameter("z_short", rclcpp::ParameterValue(0.05));

  add_parameter(
    "always_reset_initial_pose", rclcpp::ParameterValue(false),
    "Requires that AMCL is provided an initial pose either via topic or initial_pose* parameter "
    "(with parameter set_initial_pose: true) when reset. Otherwise, by default AMCL will use the"
    "last known pose to initialize");

  add_parameter(
    "scan_topic", rclcpp::ParameterValue("scan"),
    "Topic to subscribe to in order to receive the laser scan for localization");

  add_parameter(
    "map_topic", rclcpp::ParameterValue("map"),
    "Topic to subscribe to in order to receive the map to localize on");

  add_parameter(
    "first_map_only", rclcpp::ParameterValue(false),
    "Set this to true, when you want to load a new map published from the map_server");

  add_parameter(
    "freespace_downsampling", rclcpp::ParameterValue(
      false),
    "Downsample the free space used by the Pose Generator. Use it with large maps to save memory");
}

AmclNode::~AmclNode()
{
}
/**
 * @brief Configure the AMCL node
 * 
 * This function is called when the node is configured.
 * It initializes the necessary components for the AMCL node.
 */
nav2_util::CallbackReturn
AmclNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");// 记录日志，表示正在配置 AMCL 节点
  /*
  *callback_group_是一个回调组，用于管理节点的回调函数。
  *initParameters()函数用于初始化节点的参数。
  *initTransforms()函数用于初始化坐标变换相关的组件。
  *initParticleFilter()函数用于初始化粒子滤波器。
  *initLaserScan()函数用于初始化激光扫描相关的组件。
  *initMessageFilters()函数用于初始化消息过滤器。
  *initPubSub()函数用于初始化发布和订阅相关的组件。
  *initServices()函数用于初始化服务相关的组件。
  *initOdometry()函数用于初始化里程计相关的组件。
  *executor_是一个单线程执行器，用于执行节点的回调函数。
  *executor_thread_是一个独立的线程，用于运行执行器。
  */
  callback_group_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  initParameters();
  initTransforms();
  initParticleFilter();
  initLaserScan();
  initMessageFilters();
  initPubSub();
  initServices();
  initOdometry();
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_callback_group(callback_group_, get_node_base_interface());
  executor_thread_ = std::make_unique<nav2_util::NodeThread>(executor_);
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
/**
 * @brief Activate the AMCL node
 * This function is called when the node is activated.
 * It activates the necessary components for the AMCL node.
 * 
 * @param state The current state of the node.
 * @return CallbackReturn indicating the success or failure of the activation.
 */
AmclNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // Lifecycle publishers must be explicitly activated
  // 激活生命周期发布者，确保它们可以发布消息
  pose_pub_->on_activate();
  particle_cloud_pub_->on_activate();

  //重置 first_pose_sent_ 标志，表示初始位姿尚未发送
  first_pose_sent_ = false;

  // Keep track of whether we're in the active state. We won't
  // process incoming callbacks until we are
  // 设置 active_ 标志为 true，表示节点处于活动状态，可以处理传入的回调函数
  active_ = true;

  /*
  * 如果设置了初始位姿，则发送初始位姿消息
  * 如果没有设置初始位姿，但在非活动状态下接收到了初始位姿消息，则处理该消息
  */
  if (set_initial_pose_) {
    auto msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

    /*
    * 填充初始位姿消息的头部信息
    * 设置时间戳为当前时间
    * 设置坐标系为全局坐标系
    * 填充位姿信息，包括位置和朝向
    * 调用 initialPoseReceived() 函数处理初始位姿消息
    */
    msg->header.stamp = now();
    msg->header.frame_id = global_frame_id_;
    msg->pose.pose.position.x = initial_pose_x_;
    msg->pose.pose.position.y = initial_pose_y_;
    msg->pose.pose.position.z = initial_pose_z_;
    msg->pose.pose.orientation = orientationAroundZAxis(initial_pose_yaw_);

    initialPoseReceived(msg);
  } else if (init_pose_received_on_inactive) {
    handleInitialPose(last_published_pose_);
  }

  // 添加动态参数回调函数，用于处理参数变化事件
  auto node = shared_from_this();
  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &AmclNode::dynamicParametersCallback,
      this, std::placeholders::_1));

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
/**
 * @brief 关闭 AMCL 节点
 * 这个函数在节点关闭时被调用。它执行必要的清理操作，释放资源，并确保节点正确关闭。
 * @param state 节点的当前状态。
 * @return CallbackReturn 表示关闭操作的成功或失败。
 */
AmclNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{

  RCLCPP_INFO(get_logger(), "Deactivating");

  active_ = false;

  // 生命周期发布者必须显式地停用，以确保它们不再发布消息
  pose_pub_->on_deactivate();
  particle_cloud_pub_->on_deactivate();

  // 关闭动态参数回调函数，确保在节点停用后不再处理参数变化事件
  remove_on_set_parameters_callback(dyn_params_handler_.get());
  dyn_params_handler_.reset();

  // 摧毁 bond 连接，确保节点在停用时正确断开与其他组件的连接
  destroyBond();

  // 返回 CallbackReturn::SUCCESS，表示停用操作成功完成
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
/**
 * @brief 这个函数在节点清理时被调用。
 * 它执行必要的清理操作，释放资源，并确保节点正确清理。
 * @param state 节点的当前状态。
 * @return CallbackReturn 表示清理操作的成功或失败。
 */
AmclNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // 停止执行器线程，确保在清理过程中不再处理回调函数
  executor_thread_.reset();

  // 重置所有资源和组件，确保节点在清理后不再占用任何资源
  /*
  * global_loc_srv_: 用于处理全局定位服务的服务对象。
  * initial_guess_srv_: 用于处理初始猜测服务的服务对象。
  * nomotion_update_srv_: 用于处理无运动更新服务的服务对象。
  * initial_pose_sub_: 用于订阅初始位姿消息的订阅对象
  * laser_scan_connection_: 用于管理激光扫描消息连接的对象。
  * tf_listener_: 用于监听和处理坐标变换的对象。
  * laser_scan_filter_: 用于过滤激光扫描消息的对象。
  * laser_scan_sub_: 用于订阅激光扫描消息的订阅对象。
  *
  */
  global_loc_srv_.reset();
  initial_guess_srv_.reset();
  nomotion_update_srv_.reset();
  initial_pose_sub_.reset();
  laser_scan_connection_.disconnect();
  tf_listener_.reset();  // listener_可能访问tf_buffer_，所以它应该先被重置
  laser_scan_filter_.reset();
  laser_scan_sub_.reset();

  // Map
  map_sub_.reset();  //  map_sub_ 可能访问map_，所以它应该先被重置
  // 释放地图资源，确保在清理过程中不再占用地图相关的内存
  if (map_ != NULL) {
    map_free(map_);
    map_ = nullptr;
  }
  // 重置地图相关的参数，确保在清理后不再使用旧的地图数据
  first_map_received_ = false;
  free_space_indices.resize(0);

  /*
  * tf_broadcaster_: 用于广播坐标变换的对象。
  * tf_buffer_: 用于存储和管理坐标变换的对象。
  */
  tf_broadcaster_.reset();
  tf_buffer_.reset();

  /* 
  * pose_pub_: 用于发布位姿消息的发布者对象。 
  * particle_cloud_pub_: 用于发布粒子云消息的发布者对象。
  */
  pose_pub_.reset();
  particle_cloud_pub_.reset();

  // motion_model_: 用于处理运动模型的对象。重置它以释放相关资源。
  motion_model_.reset();

  // pf_free() 函数用于释放粒子滤波器相关的资源，确保在清理过程中不再占用内存
  // pf_ 是一个指向粒子滤波器结构体的指针，表示当前的粒子滤波器实例
  pf_free(pf_);
  pf_ = nullptr;

  /* lasers_ 是一个存储激光传感器对象的容器
  * lasers_update_ 是一个存储激光传感器更新对象的容器
  * frame_to_laser_ 是一个存储激光传感器坐标系到机器人坐标系的映射关系的容器
  * force_update_ 是一个标志，表示是否强制更新激光传感器数据
  */
  lasers_.clear();
  lasers_update_.clear();
  frame_to_laser_.clear();
  force_update_ = true;

  // 如果设置了初始位姿，则将最后发布的位姿设置为参数服务器上的初始位姿参数
  if (set_initial_pose_) {
    set_parameter(
      rclcpp::Parameter(
        "initial_pose.x",
        rclcpp::ParameterValue(last_published_pose_.pose.pose.position.x)));
    set_parameter(
      rclcpp::Parameter(
        "initial_pose.y",
        rclcpp::ParameterValue(last_published_pose_.pose.pose.position.y)));
    set_parameter(
      rclcpp::Parameter(
        "initial_pose.z",
        rclcpp::ParameterValue(last_published_pose_.pose.pose.position.z)));
    set_parameter(
      rclcpp::Parameter(
        "initial_pose.yaw",
        rclcpp::ParameterValue(tf2::getYaw(last_published_pose_.pose.pose.orientation))));
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
/**
 * @brief 当节点关闭时调用的回调函数，用于执行必要的清理操作和资源释放。
 * @param state 节点的当前状态
 * @return CallbackReturn 表示关闭操作的成功或失败
 * 
 */
AmclNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}


/**
 * @brief 检查经过的时间是否超过了指定的间隔
 * 
 * @param check_interval 要检查的时间间隔
 * @param last_time 上次检查的时间
 * @return true 经过的时间超过了指定间隔
 * @return false 经过的时间未超过指定间隔
 */
bool
AmclNode::checkElapsedTime(std::chrono::seconds check_interval, rclcpp::Time last_time)
{
  rclcpp::Duration elapsed_time = now() - last_time;
  if (elapsed_time.nanoseconds() * 1e-9 > check_interval.count()) {
    return true;
  }
  return false;
}


#if NEW_UNIFORM_SAMPLING
std::vector<AmclNode::Point2D> AmclNode::free_space_indices;
#endif

/**
 * @brief 获取里程计位姿
 * 
 * @param odom_pose 里程计位姿
 * @param x 机器人在地图坐标系中的X坐标
 * @param y 机器人在地图坐标系中的Y坐标
 * @param yaw 机器人在地图坐标系中的偏航角
 * @param sensor_timestamp 传感器时间戳
 * @param frame_id 坐标系ID
 * @return true 获取成功
 * @return false 获取失败
 */
bool
AmclNode::getOdomPose(
  geometry_msgs::msg::PoseStamped & odom_pose,
  double & x, double & y, double & yaw,
  const rclcpp::Time & sensor_timestamp, const std::string & frame_id)
{
  // ident: 用于存储一个位姿消息，表示机器人在地图坐标系中的位姿
  geometry_msgs::msg::PoseStamped ident;
  // 填充 ident 的头部信息，包括坐标系ID和时间戳
  ident.header.frame_id = nav2_util::strip_leading_slash(frame_id);
  ident.header.stamp = sensor_timestamp;
  // 将 ident 的位姿设置为单位变换，即机器人在地图坐标系中的位姿为原点，朝向为0
  tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);

  // 使用 tf_buffer_ 将 ident 从地图坐标系转换到里程计坐标系，并将结果存储在 odom_pose 中
  try {
    // 使用tramsform函数将ident从frame_id坐标系转换到odom_frame_id_坐标系，并将结果存储在odom_pose中
    tf_buffer_->transform(ident, odom_pose, odom_frame_id_);
  } 
  // 如果转换过程中发生异常，捕获异常并记录错误信息，同时增加扫描错误计数器
  catch (tf2::TransformException & e) {
    ++scan_error_count_;
    if (scan_error_count_ % 20 == 0) {
      RCLCPP_ERROR(
        get_logger(), "(%d) consecutive laser scan transforms failed: (%s)", scan_error_count_,
        e.what());
    }
    return false;
  }

  // 重置扫描错误计数器，因为我们得到了一个有效的变换
  scan_error_count_ = 0;  
  // 将 odom_pose 的位置信息和朝向信息提取出来，分别存储在 x、y 和 yaw 中
  x = odom_pose.pose.position.x;
  y = odom_pose.pose.position.y;
  yaw = tf2::getYaw(odom_pose.pose.orientation);

  return true;
}
/**
 * @brief 均匀分布的位姿生成器,用于在地图中生成随机的位姿
 * 
 * @param arg 地图指针
 * @return pf_vector_t 生成的位姿
 */
pf_vector_t
AmclNode::uniformPoseGenerator(void * arg)
{
  // 将传入的参数 arg 转换为 map_t 类型的指针 map，用于访问地图信息
  map_t * map = reinterpret_cast<map_t *>(arg);

  // 如果启用了新的均匀采样方法，则从 free_space_indices 中随机选择一个索引，获取对应的自由空间点，并生成一个随机的位姿
#if NEW_UNIFORM_SAMPLING
// 生成一个随机索引，范围在 0 到 free_space_indices 的大小之间
  unsigned int rand_index = drand48() * free_space_indices.size();
// 获取对应的自由空间点
  AmclNode::Point2D free_point = free_space_indices[rand_index];
// 将自由空间点的坐标转换为地图坐标系下的位姿，并生成一个随机的朝向角度
  pf_vector_t p;
/*
* p.v[0]: 表示位姿的 x 坐标，通过 MAP_WXGX(map, free_point.x) 将自由空间点的 x 坐标转换为地图坐标系下的 x 坐标。
* p.v[1]: 表示位姿的 y 坐标，通过 MAP_WYGY(map, free_point.y) 将自由空间点的 y 坐标转换为地图坐标系下的 y 坐标。  
* p.v[2]: 表示位姿的朝向角度（yaw），通过 drand48() * 2 * M_PI - M_PI 生成一个随机的角度，范围在 -π 到 π 之间。
*
*/
  p.v[0] = MAP_WXGX(map, free_point.x);
  p.v[1] = MAP_WYGY(map, free_point.y);
  p.v[2] = drand48() * 2 * M_PI - M_PI;
#else
  double min_x, max_x, min_y, max_y;

  min_x = (map->size_x * map->scale) / 2.0 - map->origin_x;
  max_x = (map->size_x * map->scale) / 2.0 + map->origin_x;
  min_y = (map->size_y * map->scale) / 2.0 - map->origin_y;
  max_y = (map->size_y * map->scale) / 2.0 + map->origin_y;

  pf_vector_t p;

  RCLCPP_DEBUG(get_logger(), "Generating new uniform sample");
  for (;; ) {
    p.v[0] = min_x + drand48() * (max_x - min_x);
    p.v[1] = min_y + drand48() * (max_y - min_y);
    p.v[2] = drand48() * 2 * M_PI - M_PI;
    // Check that it's a free cell
    int i, j;
    i = MAP_GXWX(map, p.v[0]);
    j = MAP_GYWY(map, p.v[1]);
    if (MAP_VALID(map, i, j) && (map->cells[MAP_INDEX(map, i, j)].occ_state == -1)) {
      break;
    }
  }
#endif
  return p;
}
/**
 * @brief 全局定位回调函数,用于处理全局定位请求
 * 
 * @param request_header 请求头
 * @param req 请求对象
 * @param res 响应对象
 */
void
AmclNode::globalLocalizationCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request>/*req*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>/*res*/)
{
  // 使用互斥锁保护全局定位操作，确保线程安全
  std::lock_guard<std::recursive_mutex> cfl(mutex_);

  RCLCPP_INFO(get_logger(), "Initializing with uniform distribution");

  // 调用 pf_init_model 函数初始化粒子滤波器，使用均匀分布的位姿生成器
  /*
  * pf_ 是一个指向粒子滤波器结构体的指针，表示当前的粒子滤波器实例。
  *
  * (pf_init_model_fn_t)AmclNode::uniformPoseGenerator 是一个函数指针，
  * 指向 AmclNode 类中的 uniformPoseGenerator 函数，用于生成均匀分布的位姿。
  * 
  * reinterpret_cast<void *>(map_) 将 map_ 指针转换为 void* 类型，
  * 以便传递给 uniformPoseGenerator 函数。
  */
  pf_init_model(
    pf_, (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
    reinterpret_cast<void *>(map_));
  RCLCPP_INFO(get_logger(), "Global initialisation done!");
  initial_pose_is_known_ = true;
  pf_init_ = false;
}

/**
 * @brief 初始位姿接收服务回调函数
 * 
 * @param request_header 请求头
 * @param req 请求对象
 * @param res 响应对象
 */
void
AmclNode::initialPoseReceivedSrv(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<nav2_msgs::srv::SetInitialPose::Request> req,
  std::shared_ptr<nav2_msgs::srv::SetInitialPose::Response>/*res*/)
{
  // 调用initialPoseReceived函数处理接收到的初始位姿消息
  initialPoseReceived(std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(req->pose));
}

// force nomotion updates (amcl updating without requiring motion)
/**
 * @brief 无运动更新回调函数
 * @param request_header 请求头
 * @param req 请求对象
 * @param res 响应对象
 * 
 */
void
AmclNode::nomotionUpdateCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request>/*req*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>/*res*/)
{
  // 打印日志，表示请求了无运动更新
  RCLCPP_INFO(get_logger(), "Requesting no-motion update");
  // 设置 force_update_ 标志为 true，表示强制进行无运动更新
  force_update_ = true;
}

/**
 * @brief 初始位姿接收函数,用于处理接收到的初始位姿消息
 *    
 * 
 * @param msg 初始位姿消息
 */
void
AmclNode::initialPoseReceived(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  // 使用互斥锁保护初始位姿接收操作，确保线程安全
  std::lock_guard<std::recursive_mutex> cfl(mutex_);

  RCLCPP_INFO(get_logger(), "initialPoseReceived");

  // 检查接收到的初始位姿消息是否有效，如果无效则记录错误日志并返回
  if (!nav2_util::validateMsg(*msg)) {
    RCLCPP_ERROR(get_logger(), "Received initialpose message is malformed. Rejecting.");
    return;
  }
  // 检查接收到的初始位姿消息的坐标系是否与全局坐标系一致，如果不一致则记录警告日志并返回
  if (nav2_util::strip_leading_slash(msg->header.frame_id) != global_frame_id_) {
    RCLCPP_WARN(
      get_logger(),
      "Ignoring initial pose in frame \"%s\"; initial poses must be in the global frame, \"%s\"",
      nav2_util::strip_leading_slash(msg->header.frame_id).c_str(),
      global_frame_id_.c_str());
    return;
  }
  // 检查接收到的初始位姿消息的位置信息是否超出了地图的范围，如果超出则记录错误日志并返回
  if (first_map_received_ && (abs(msg->pose.pose.position.x) > map_->size_x ||
    abs(msg->pose.pose.position.y) > map_->size_y))
  {
    RCLCPP_ERROR(
      get_logger(), "Received initialpose from message is out of the size of map. Rejecting.");
    return;
  }

  // Overriding last published pose to initial pose
  // 将接收到的初始位姿消息保存为最后发布的位姿
  last_published_pose_ = *msg;

  // 如果 AMCL 节点尚未处于活动状态，则记录警告日志并返回，表示无法处理初始位姿请求
  if (!active_) {
    init_pose_received_on_inactive = true;
    RCLCPP_WARN(
      get_logger(), "Received initial pose request, "
      "but AMCL is not yet in the active state");
    return;
  }
  // 调用 handleInitialPose 函数处理接收到的初始位姿消息
  handleInitialPose(*msg);
}

/**
 * @brief 处理初始位姿函数
 * @param msg 初始位姿消息
 */
void
AmclNode::handleInitialPose(geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  // 使用互斥锁保护初始位姿处理操作，确保线程安全
  std::lock_guard<std::recursive_mutex> cfl(mutex_);
  // 由于 AMCL 需要将初始位姿从全局坐标系转换到里程计坐标系，因此需要获取当前的时间戳，并使用 tf2 库进行坐标变换
  geometry_msgs::msg::TransformStamped tx_odom;
  // 获取当前时间戳，并将其转换为 tf2::TimePoint 类型，以便在坐标变换中使用
  try {
    rclcpp::Time rclcpp_time = now();
    // 将 rclcpp::Time 类型的时间戳转换为 tf2::TimePoint 类型，以便在坐标变换中使用
    tf2::TimePoint tf2_time(std::chrono::nanoseconds(rclcpp_time.nanoseconds()));

    // 使用 tf_buffer_ 查找从全局坐标系到里程计坐标系的变换，并将结果存储在 tx_odom 中
    tx_odom = tf_buffer_->lookupTransform(
      base_frame_id_, tf2_ros::fromMsg(msg.header.stamp),
      base_frame_id_, tf2_time, odom_frame_id_);
  } catch (tf2::TransformException & e) {
    /*
    * 如果在指定的时间内无法获取到坐标变换，则记录警告日志，并将 tx_odom 设置为单位变换
    * 这意味着初始位姿将不会被转换，可能会导致定位不准确，但至少不会导致程序崩溃
    */
    if (sent_first_transform_) {
      RCLCPP_WARN(get_logger(), "Failed to transform initial pose in time (%s)", e.what());
    }
    tf2::impl::Converter<false, true>::convert(tf2::Transform::getIdentity(), tx_odom.transform);
  }

  // 将 geometry_msgs::msg::Transform 转换为 tf2::Transform，以便进行后续的位姿计算
  tf2::Transform tx_odom_tf2;
  tf2::impl::Converter<true, false>::convert(tx_odom.transform, tx_odom_tf2);

  tf2::Transform pose_old;
  tf2::impl::Converter<true, false>::convert(msg.pose.pose, pose_old);

  tf2::Transform pose_new = pose_old * tx_odom_tf2;

  // Transform into the global frame

  RCLCPP_INFO(
    get_logger(), "Setting pose (%.6f): %.3f %.3f %.3f",
    now().nanoseconds() * 1e-9,
    pose_new.getOrigin().x(),
    pose_new.getOrigin().y(),
    tf2::getYaw(pose_new.getRotation()));

  // Re-initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = pose_new.getOrigin().x();
  pf_init_pose_mean.v[1] = pose_new.getOrigin().y();
  pf_init_pose_mean.v[2] = tf2::getYaw(pose_new.getRotation());

  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  // Copy in the covariance, converting from 6-D to 3-D
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      pf_init_pose_cov.m[i][j] = msg.pose.covariance[6 * i + j];
    }
  }

  pf_init_pose_cov.m[2][2] = msg.pose.covariance[6 * 5 + 5];

  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
  pf_init_ = false;
  init_pose_received_on_inactive = false;
  initial_pose_is_known_ = true;
}

void
AmclNode::laserReceived(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan)
{
  std::lock_guard<std::recursive_mutex> cfl(mutex_);

  // Since the sensor data is continually being published by the simulator or robot,
  // we don't want our callbacks to fire until we're in the active state
  if (!active_) {return;}
  if (!first_map_received_) {
    if (checkElapsedTime(2s, last_time_printed_msg_)) {
      RCLCPP_WARN(get_logger(), "Waiting for map....");
      last_time_printed_msg_ = now();
    }
    return;
  }

  std::string laser_scan_frame_id = nav2_util::strip_leading_slash(laser_scan->header.frame_id);
  last_laser_received_ts_ = now();
  int laser_index = -1;
  geometry_msgs::msg::PoseStamped laser_pose;

  // Do we have the base->base_laser Tx yet?
  if (frame_to_laser_.find(laser_scan_frame_id) == frame_to_laser_.end()) {
    if (!addNewScanner(laser_index, laser_scan, laser_scan_frame_id, laser_pose)) {
      return;  // could not find transform
    }
  } else {
    // we have the laser pose, retrieve laser index
    laser_index = frame_to_laser_[laser_scan->header.frame_id];
  }

  // Where was the robot when this scan was taken?
  pf_vector_t pose;
  if (!getOdomPose(
      latest_odom_pose_, pose.v[0], pose.v[1], pose.v[2],
      laser_scan->header.stamp, base_frame_id_))
  {
    RCLCPP_ERROR(get_logger(), "Couldn't determine robot's pose associated with laser scan");
    return;
  }

  pf_vector_t delta = pf_vector_zero();
  bool force_publication = false;
  if (!pf_init_) {
    // Pose at last filter update
    pf_odom_pose_ = pose;
    pf_init_ = true;

    for (unsigned int i = 0; i < lasers_update_.size(); i++) {
      lasers_update_[i] = true;
    }

    force_publication = true;
    resample_count_ = 0;
  } else {
    // Set the laser update flags
    if (shouldUpdateFilter(pose, delta)) {
      for (unsigned int i = 0; i < lasers_update_.size(); i++) {
        lasers_update_[i] = true;
      }
    }
    if (lasers_update_[laser_index]) {
      motion_model_->odometryUpdate(pf_, pose, delta);
    }
    force_update_ = false;
  }

  bool resampled = false;

  // If the robot has moved, update the filter
  if (lasers_update_[laser_index]) {
    updateFilter(laser_index, laser_scan, pose);

    // Resample the particles
    if (!(++resample_count_ % resample_interval_)) {
      pf_update_resample(pf_, reinterpret_cast<void *>(map_));
      resampled = true;
    }

    pf_sample_set_t * set = pf_->sets + pf_->current_set;
    RCLCPP_DEBUG(get_logger(), "Num samples: %d\n", set->sample_count);

    if (!force_update_) {
      publishParticleCloud(set);
    }
  }
  if (resampled || force_publication || !first_pose_sent_) {
    amcl_hyp_t max_weight_hyps;
    std::vector<amcl_hyp_t> hyps;
    int max_weight_hyp = -1;
    if (getMaxWeightHyp(hyps, max_weight_hyps, max_weight_hyp)) {
      publishAmclPose(laser_scan, hyps, max_weight_hyp);
      calculateMaptoOdomTransform(laser_scan, hyps, max_weight_hyp);

      if (tf_broadcast_ == true) {
        // We want to send a transform that is good up until a
        // tolerance time so that odom can be used
        auto stamp = tf2_ros::fromMsg(laser_scan->header.stamp);
        tf2::TimePoint transform_expiration = stamp + transform_tolerance_;
        sendMapToOdomTransform(transform_expiration);
        sent_first_transform_ = true;
      }
    } else {
      RCLCPP_ERROR(get_logger(), "No pose!");
    }
  } else if (latest_tf_valid_) {
    if (tf_broadcast_ == true) {
      // Nothing changed, so we'll just republish the last transform, to keep
      // everybody happy.
      tf2::TimePoint transform_expiration = tf2_ros::fromMsg(laser_scan->header.stamp) +
        transform_tolerance_;
      sendMapToOdomTransform(transform_expiration);
    }
  }
}

bool AmclNode::addNewScanner(
  int & laser_index,
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
  const std::string & laser_scan_frame_id,
  geometry_msgs::msg::PoseStamped & laser_pose)
{
  lasers_.push_back(createLaserObject());
  lasers_update_.push_back(true);
  laser_index = frame_to_laser_.size();

  geometry_msgs::msg::PoseStamped ident;
  ident.header.frame_id = laser_scan_frame_id;
  ident.header.stamp = rclcpp::Time();
  tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);
  try {
    tf_buffer_->transform(ident, laser_pose, base_frame_id_, transform_tolerance_);
  } catch (tf2::TransformException & e) {
    RCLCPP_ERROR(
      get_logger(), "Couldn't transform from %s to %s, "
      "even though the message notifier is in use: (%s)",
      laser_scan->header.frame_id.c_str(),
      base_frame_id_.c_str(), e.what());
    return false;
  }

  pf_vector_t laser_pose_v;
  laser_pose_v.v[0] = laser_pose.pose.position.x;
  laser_pose_v.v[1] = laser_pose.pose.position.y;
  // laser mounting angle gets computed later -> set to 0 here!
  laser_pose_v.v[2] = 0;
  lasers_[laser_index]->SetLaserPose(laser_pose_v);
  frame_to_laser_[laser_scan->header.frame_id] = laser_index;
  return true;
}

bool AmclNode::shouldUpdateFilter(const pf_vector_t pose, pf_vector_t & delta)
{
  delta.v[0] = pose.v[0] - pf_odom_pose_.v[0];
  delta.v[1] = pose.v[1] - pf_odom_pose_.v[1];
  delta.v[2] = angleutils::angle_diff(pose.v[2], pf_odom_pose_.v[2]);

  // See if we should update the filter
  bool update = fabs(delta.v[0]) > d_thresh_ ||
    fabs(delta.v[1]) > d_thresh_ ||
    fabs(delta.v[2]) > a_thresh_;
  update = update || force_update_;
  return update;
}

bool AmclNode::updateFilter(
  const int & laser_index,
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
  const pf_vector_t & pose)
{
  nav2_amcl::LaserData ldata;
  ldata.laser = lasers_[laser_index].get();
  ldata.range_count = laser_scan->ranges.size();
  // To account for lasers that are mounted upside-down, we determine the
  // min, max, and increment angles of the laser in the base frame.
  //
  // Construct min and max angles of laser, in the base_link frame.
  // Here we set the roll pich yaw of the lasers.  We assume roll and pich are zero.
  geometry_msgs::msg::QuaternionStamped min_q, inc_q;
  min_q.header.stamp = laser_scan->header.stamp;
  min_q.header.frame_id = nav2_util::strip_leading_slash(laser_scan->header.frame_id);
  min_q.quaternion = orientationAroundZAxis(laser_scan->angle_min);

  inc_q.header = min_q.header;
  inc_q.quaternion = orientationAroundZAxis(laser_scan->angle_min + laser_scan->angle_increment);
  try {
    tf_buffer_->transform(min_q, min_q, base_frame_id_);
    tf_buffer_->transform(inc_q, inc_q, base_frame_id_);
  } catch (tf2::TransformException & e) {
    RCLCPP_WARN(
      get_logger(), "Unable to transform min/max laser angles into base frame: %s",
      e.what());
    return false;
  }
  double angle_min = tf2::getYaw(min_q.quaternion);
  double angle_increment = tf2::getYaw(inc_q.quaternion) - angle_min;

  // wrapping angle to [-pi .. pi]
  angle_increment = fmod(angle_increment + 5 * M_PI, 2 * M_PI) - M_PI;

  RCLCPP_DEBUG(
    get_logger(), "Laser %d angles in base frame: min: %.3f inc: %.3f", laser_index, angle_min,
    angle_increment);

  // Check the validity of range_max, must > 0.0
  if (laser_scan->range_max <= 0.0) {
    RCLCPP_WARN(
      get_logger(), "wrong range_max of laser_scan data: %f. The message could be malformed."
      " Ignore this message and stop updating.",
      laser_scan->range_max);
    return false;
  }

  // Apply range min/max thresholds, if the user supplied them
  if (laser_max_range_ > 0.0) {
    ldata.range_max = std::min(laser_scan->range_max, static_cast<float>(laser_max_range_));
  } else {
    ldata.range_max = laser_scan->range_max;
  }
  double range_min;
  if (laser_min_range_ > 0.0) {
    range_min = std::max(laser_scan->range_min, static_cast<float>(laser_min_range_));
  } else {
    range_min = laser_scan->range_min;
  }

  // The LaserData destructor will free this memory
  ldata.ranges = new double[ldata.range_count][2];
  for (int i = 0; i < ldata.range_count; i++) {
    // amcl doesn't (yet) have a concept of min range.  So we'll map short
    // readings to max range.
    if (laser_scan->ranges[i] <= range_min) {
      ldata.ranges[i][0] = ldata.range_max;
    } else {
      ldata.ranges[i][0] = laser_scan->ranges[i];
    }
    // Compute bearing
    ldata.ranges[i][1] = angle_min +
      (i * angle_increment);
  }
  lasers_[laser_index]->sensorUpdate(pf_, reinterpret_cast<nav2_amcl::LaserData *>(&ldata));
  lasers_update_[laser_index] = false;
  pf_odom_pose_ = pose;
  return true;
}

void
AmclNode::publishParticleCloud(const pf_sample_set_t * set)
{
  // If initial pose is not known, AMCL does not know the current pose
  if (!initial_pose_is_known_) {return;}
  auto cloud_with_weights_msg = std::make_unique<nav2_msgs::msg::ParticleCloud>();
  cloud_with_weights_msg->header.stamp = this->now();
  cloud_with_weights_msg->header.frame_id = global_frame_id_;
  cloud_with_weights_msg->particles.resize(set->sample_count);

  for (int i = 0; i < set->sample_count; i++) {
    cloud_with_weights_msg->particles[i].pose.position.x = set->samples[i].pose.v[0];
    cloud_with_weights_msg->particles[i].pose.position.y = set->samples[i].pose.v[1];
    cloud_with_weights_msg->particles[i].pose.position.z = 0;
    cloud_with_weights_msg->particles[i].pose.orientation = orientationAroundZAxis(
      set->samples[i].pose.v[2]);
    cloud_with_weights_msg->particles[i].weight = set->samples[i].weight;
  }

  particle_cloud_pub_->publish(std::move(cloud_with_weights_msg));
}

bool
AmclNode::getMaxWeightHyp(
  std::vector<amcl_hyp_t> & hyps, amcl_hyp_t & max_weight_hyps,
  int & max_weight_hyp)
{
  // Read out the current hypotheses
  double max_weight = 0.0;
  hyps.resize(pf_->sets[pf_->current_set].cluster_count);
  for (int hyp_count = 0;
    hyp_count < pf_->sets[pf_->current_set].cluster_count; hyp_count++)
  {
    double weight;
    pf_vector_t pose_mean;
    pf_matrix_t pose_cov;
    if (!pf_get_cluster_stats(pf_, hyp_count, &weight, &pose_mean, &pose_cov)) {
      RCLCPP_ERROR(get_logger(), "Couldn't get stats on cluster %d", hyp_count);
      return false;
    }

    hyps[hyp_count].weight = weight;
    hyps[hyp_count].pf_pose_mean = pose_mean;
    hyps[hyp_count].pf_pose_cov = pose_cov;

    if (hyps[hyp_count].weight > max_weight) {
      max_weight = hyps[hyp_count].weight;
      max_weight_hyp = hyp_count;
    }
  }

  if (max_weight > 0.0) {
    RCLCPP_DEBUG(
      get_logger(), "Max weight pose: %.3f %.3f %.3f",
      hyps[max_weight_hyp].pf_pose_mean.v[0],
      hyps[max_weight_hyp].pf_pose_mean.v[1],
      hyps[max_weight_hyp].pf_pose_mean.v[2]);

    max_weight_hyps = hyps[max_weight_hyp];
    return true;
  }
  return false;
}

void
AmclNode::publishAmclPose(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
  const std::vector<amcl_hyp_t> & hyps, const int & max_weight_hyp)
{
  // If initial pose is not known, AMCL does not know the current pose
  if (!initial_pose_is_known_) {
    if (checkElapsedTime(2s, last_time_printed_msg_)) {
      RCLCPP_WARN(
        get_logger(), "AMCL cannot publish a pose or update the transform. "
        "Please set the initial pose...");
      last_time_printed_msg_ = now();
    }
    return;
  }

  auto p = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
  // Fill in the header
  p->header.frame_id = global_frame_id_;
  p->header.stamp = laser_scan->header.stamp;
  // Copy in the pose
  p->pose.pose.position.x = hyps[max_weight_hyp].pf_pose_mean.v[0];
  p->pose.pose.position.y = hyps[max_weight_hyp].pf_pose_mean.v[1];
  p->pose.pose.orientation = orientationAroundZAxis(hyps[max_weight_hyp].pf_pose_mean.v[2]);
  // Copy in the covariance, converting from 3-D to 6-D
  pf_sample_set_t * set = pf_->sets + pf_->current_set;
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      // Report the overall filter covariance, rather than the
      // covariance for the highest-weight cluster
      // p->covariance[6*i+j] = hyps[max_weight_hyp].pf_pose_cov.m[i][j];
      p->pose.covariance[6 * i + j] = set->cov.m[i][j];
    }
  }
  p->pose.covariance[6 * 5 + 5] = set->cov.m[2][2];
  float temp = 0.0;
  for (auto covariance_value : p->pose.covariance) {
    temp += covariance_value;
  }
  temp += p->pose.pose.position.x + p->pose.pose.position.y;
  if (!std::isnan(temp)) {
    RCLCPP_DEBUG(get_logger(), "Publishing pose");
    last_published_pose_ = *p;
    first_pose_sent_ = true;
    pose_pub_->publish(std::move(p));
  } else {
    RCLCPP_WARN(
      get_logger(), "AMCL covariance or pose is NaN, likely due to an invalid "
      "configuration or faulty sensor measurements! Pose is not available!");
  }

  RCLCPP_DEBUG(
    get_logger(), "New pose: %6.3f %6.3f %6.3f",
    hyps[max_weight_hyp].pf_pose_mean.v[0],
    hyps[max_weight_hyp].pf_pose_mean.v[1],
    hyps[max_weight_hyp].pf_pose_mean.v[2]);
}

void
AmclNode::calculateMaptoOdomTransform(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
  const std::vector<amcl_hyp_t> & hyps, const int & max_weight_hyp)
{
  // subtracting base to odom from map to base and send map to odom instead
  geometry_msgs::msg::PoseStamped odom_to_map;
  try {
    tf2::Quaternion q;
    q.setRPY(0, 0, hyps[max_weight_hyp].pf_pose_mean.v[2]);
    tf2::Transform tmp_tf(q, tf2::Vector3(
        hyps[max_weight_hyp].pf_pose_mean.v[0],
        hyps[max_weight_hyp].pf_pose_mean.v[1],
        0.0));

    geometry_msgs::msg::PoseStamped tmp_tf_stamped;
    tmp_tf_stamped.header.frame_id = base_frame_id_;
    tmp_tf_stamped.header.stamp = laser_scan->header.stamp;
    tf2::toMsg(tmp_tf.inverse(), tmp_tf_stamped.pose);

    tf_buffer_->transform(tmp_tf_stamped, odom_to_map, odom_frame_id_);
  } catch (tf2::TransformException & e) {
    RCLCPP_DEBUG(get_logger(), "Failed to subtract base to odom transform: (%s)", e.what());
    return;
  }

  tf2::impl::Converter<true, false>::convert(odom_to_map.pose, latest_tf_);
  latest_tf_valid_ = true;
}

void
AmclNode::sendMapToOdomTransform(const tf2::TimePoint & transform_expiration)
{
  // AMCL will update transform only when it has knowledge about robot's initial position
  if (!initial_pose_is_known_) {return;}
  geometry_msgs::msg::TransformStamped tmp_tf_stamped;
  tmp_tf_stamped.header.frame_id = global_frame_id_;
  tmp_tf_stamped.header.stamp = tf2_ros::toMsg(transform_expiration);
  tmp_tf_stamped.child_frame_id = odom_frame_id_;
  tf2::impl::Converter<false, true>::convert(latest_tf_.inverse(), tmp_tf_stamped.transform);
  tf_broadcaster_->sendTransform(tmp_tf_stamped);
}

std::unique_ptr<nav2_amcl::Laser>
AmclNode::createLaserObject()
{
  RCLCPP_INFO(get_logger(), "createLaserObject");

  if (sensor_model_type_ == "beam") {
    return std::make_unique<nav2_amcl::BeamModel>(
      z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_,
      0.0, max_beams_, map_);
  }

  if (sensor_model_type_ == "likelihood_field_prob") {
    return std::make_unique<nav2_amcl::LikelihoodFieldModelProb>(
      z_hit_, z_rand_, sigma_hit_,
      laser_likelihood_max_dist_, do_beamskip_, beam_skip_distance_, beam_skip_threshold_,
      beam_skip_error_threshold_, max_beams_, map_);
  }

  return std::make_unique<nav2_amcl::LikelihoodFieldModel>(
    z_hit_, z_rand_, sigma_hit_,
    laser_likelihood_max_dist_, max_beams_, map_);
}

void
AmclNode::initParameters()
{
  double save_pose_rate;
  double tmp_tol;

  get_parameter("alpha1", alpha1_);
  get_parameter("alpha2", alpha2_);
  get_parameter("alpha3", alpha3_);
  get_parameter("alpha4", alpha4_);
  get_parameter("alpha5", alpha5_);
  get_parameter("base_frame_id", base_frame_id_);
  get_parameter("beam_skip_distance", beam_skip_distance_);
  get_parameter("beam_skip_error_threshold", beam_skip_error_threshold_);
  get_parameter("beam_skip_threshold", beam_skip_threshold_);
  get_parameter("do_beamskip", do_beamskip_);
  get_parameter("global_frame_id", global_frame_id_);
  get_parameter("lambda_short", lambda_short_);
  get_parameter("laser_likelihood_max_dist", laser_likelihood_max_dist_);
  get_parameter("laser_max_range", laser_max_range_);
  get_parameter("laser_min_range", laser_min_range_);
  get_parameter("laser_model_type", sensor_model_type_);
  get_parameter("set_initial_pose", set_initial_pose_);
  get_parameter("initial_pose.x", initial_pose_x_);
  get_parameter("initial_pose.y", initial_pose_y_);
  get_parameter("initial_pose.z", initial_pose_z_);
  get_parameter("initial_pose.yaw", initial_pose_yaw_);
  get_parameter("max_beams", max_beams_);
  get_parameter("max_particles", max_particles_);
  get_parameter("min_particles", min_particles_);
  get_parameter("odom_frame_id", odom_frame_id_);
  get_parameter("pf_err", pf_err_);
  get_parameter("pf_z", pf_z_);
  get_parameter("recovery_alpha_fast", alpha_fast_);
  get_parameter("recovery_alpha_slow", alpha_slow_);
  get_parameter("resample_interval", resample_interval_);
  get_parameter("robot_model_type", robot_model_type_);
  get_parameter("save_pose_rate", save_pose_rate);
  get_parameter("sigma_hit", sigma_hit_);
  get_parameter("tf_broadcast", tf_broadcast_);
  get_parameter("transform_tolerance", tmp_tol);
  get_parameter("update_min_a", a_thresh_);
  get_parameter("update_min_d", d_thresh_);
  get_parameter("z_hit", z_hit_);
  get_parameter("z_max", z_max_);
  get_parameter("z_rand", z_rand_);
  get_parameter("z_short", z_short_);
  get_parameter("first_map_only", first_map_only_);
  get_parameter("always_reset_initial_pose", always_reset_initial_pose_);
  get_parameter("scan_topic", scan_topic_);
  get_parameter("map_topic", map_topic_);
  get_parameter("freespace_downsampling", freespace_downsampling_);

  save_pose_period_ = tf2::durationFromSec(1.0 / save_pose_rate);
  transform_tolerance_ = tf2::durationFromSec(tmp_tol);

  odom_frame_id_ = nav2_util::strip_leading_slash(odom_frame_id_);
  base_frame_id_ = nav2_util::strip_leading_slash(base_frame_id_);
  global_frame_id_ = nav2_util::strip_leading_slash(global_frame_id_);

  last_time_printed_msg_ = now();

  // Semantic checks
  if (laser_likelihood_max_dist_ < 0) {
    RCLCPP_WARN(
      get_logger(), "You've set laser_likelihood_max_dist to be negative,"
      " this isn't allowed so it will be set to default value 2.0.");
    laser_likelihood_max_dist_ = 2.0;
  }
  if (max_particles_ < 0) {
    RCLCPP_WARN(
      get_logger(), "You've set max_particles to be negative,"
      " this isn't allowed so it will be set to default value 2000.");
    max_particles_ = 2000;
  }

  if (min_particles_ < 0) {
    RCLCPP_WARN(
      get_logger(), "You've set min_particles to be negative,"
      " this isn't allowed so it will be set to default value 500.");
    min_particles_ = 500;
  }

  if (min_particles_ > max_particles_) {
    RCLCPP_WARN(
      get_logger(), "You've set min_particles to be greater than max particles,"
      " this isn't allowed so max_particles will be set to min_particles.");
    max_particles_ = min_particles_;
  }

  if (resample_interval_ <= 0) {
    RCLCPP_WARN(
      get_logger(), "You've set resample_interval to be zero or negative,"
      " this isn't allowed so it will be set to default value to 1.");
    resample_interval_ = 1;
  }

  if (always_reset_initial_pose_) {
    initial_pose_is_known_ = false;
  }
}

/**
  * @brief Callback executed when a parameter change is detected
  * @param event ParameterEvent message
  */
rcl_interfaces::msg::SetParametersResult
AmclNode::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::recursive_mutex> cfl(mutex_);
  rcl_interfaces::msg::SetParametersResult result;
  double save_pose_rate;
  double tmp_tol;

  int max_particles = max_particles_;
  int min_particles = min_particles_;

  bool reinit_pf = false;
  bool reinit_odom = false;
  bool reinit_laser = false;
  bool reinit_map = false;

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == "alpha1") {
        alpha1_ = parameter.as_double();
        // alpha restricted to be non-negative
        if (alpha1_ < 0.0) {
          RCLCPP_WARN(
            get_logger(), "You've set alpha1 to be negative,"
            " this isn't allowed, so the alpha1 will be set to be zero.");
          alpha1_ = 0.0;
        }
        reinit_odom = true;
      } else if (param_name == "alpha2") {
        alpha2_ = parameter.as_double();
        // alpha restricted to be non-negative
        if (alpha2_ < 0.0) {
          RCLCPP_WARN(
            get_logger(), "You've set alpha2 to be negative,"
            " this isn't allowed, so the alpha2 will be set to be zero.");
          alpha2_ = 0.0;
        }
        reinit_odom = true;
      } else if (param_name == "alpha3") {
        alpha3_ = parameter.as_double();
        // alpha restricted to be non-negative
        if (alpha3_ < 0.0) {
          RCLCPP_WARN(
            get_logger(), "You've set alpha3 to be negative,"
            " this isn't allowed, so the alpha3 will be set to be zero.");
          alpha3_ = 0.0;
        }
        reinit_odom = true;
      } else if (param_name == "alpha4") {
        alpha4_ = parameter.as_double();
        // alpha restricted to be non-negative
        if (alpha4_ < 0.0) {
          RCLCPP_WARN(
            get_logger(), "You've set alpha4 to be negative,"
            " this isn't allowed, so the alpha4 will be set to be zero.");
          alpha4_ = 0.0;
        }
        reinit_odom = true;
      } else if (param_name == "alpha5") {
        alpha5_ = parameter.as_double();
        // alpha restricted to be non-negative
        if (alpha5_ < 0.0) {
          RCLCPP_WARN(
            get_logger(), "You've set alpha5 to be negative,"
            " this isn't allowed, so the alpha5 will be set to be zero.");
          alpha5_ = 0.0;
        }
        reinit_odom = true;
      } else if (param_name == "beam_skip_distance") {
        beam_skip_distance_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "beam_skip_error_threshold") {
        beam_skip_error_threshold_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "beam_skip_threshold") {
        beam_skip_threshold_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "lambda_short") {
        lambda_short_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "laser_likelihood_max_dist") {
        laser_likelihood_max_dist_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "laser_max_range") {
        laser_max_range_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "laser_min_range") {
        laser_min_range_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "pf_err") {
        pf_err_ = parameter.as_double();
        reinit_pf = true;
      } else if (param_name == "pf_z") {
        pf_z_ = parameter.as_double();
        reinit_pf = true;
      } else if (param_name == "recovery_alpha_fast") {
        alpha_fast_ = parameter.as_double();
        reinit_pf = true;
      } else if (param_name == "recovery_alpha_slow") {
        alpha_slow_ = parameter.as_double();
        reinit_pf = true;
      } else if (param_name == "save_pose_rate") {
        save_pose_rate = parameter.as_double();
        save_pose_period_ = tf2::durationFromSec(1.0 / save_pose_rate);
      } else if (param_name == "sigma_hit") {
        sigma_hit_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "transform_tolerance") {
        tmp_tol = parameter.as_double();
        transform_tolerance_ = tf2::durationFromSec(tmp_tol);
        reinit_laser = true;
      } else if (param_name == "update_min_a") {
        a_thresh_ = parameter.as_double();
      } else if (param_name == "update_min_d") {
        d_thresh_ = parameter.as_double();
      } else if (param_name == "z_hit") {
        z_hit_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "z_max") {
        z_max_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "z_rand") {
        z_rand_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "z_short") {
        z_short_ = parameter.as_double();
        reinit_laser = true;
      }
    } else if (param_type == ParameterType::PARAMETER_STRING) {
      if (param_name == "base_frame_id") {
        base_frame_id_ = parameter.as_string();
      } else if (param_name == "global_frame_id") {
        global_frame_id_ = parameter.as_string();
      } else if (param_name == "map_topic") {
        map_topic_ = parameter.as_string();
        reinit_map = true;
      } else if (param_name == "laser_model_type") {
        sensor_model_type_ = parameter.as_string();
        reinit_laser = true;
      } else if (param_name == "odom_frame_id") {
        odom_frame_id_ = parameter.as_string();
        reinit_laser = true;
      } else if (param_name == "scan_topic") {
        scan_topic_ = parameter.as_string();
        reinit_laser = true;
      } else if (param_name == "robot_model_type") {
        robot_model_type_ = parameter.as_string();
        reinit_odom = true;
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == "do_beamskip") {
        do_beamskip_ = parameter.as_bool();
        reinit_laser = true;
      } else if (param_name == "tf_broadcast") {
        tf_broadcast_ = parameter.as_bool();
      } else if (param_name == "set_initial_pose") {
        set_initial_pose_ = parameter.as_bool();
      } else if (param_name == "first_map_only") {
        first_map_only_ = parameter.as_bool();
      }
    } else if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (param_name == "max_beams") {
        max_beams_ = parameter.as_int();
        reinit_laser = true;
      } else if (param_name == "max_particles") {
        max_particles_ = parameter.as_int();
        reinit_pf = true;
      } else if (param_name == "min_particles") {
        min_particles_ = parameter.as_int();
        reinit_pf = true;
      } else if (param_name == "resample_interval") {
        resample_interval_ = parameter.as_int();
      }
    }
  }

  // Checking if the minimum particles is greater than max_particles_
  if (min_particles_ > max_particles_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "You've set min_particles to be greater than max particles,"
      " this isn't allowed.");
    // sticking to the old values
    max_particles_ = max_particles;
    min_particles_ = min_particles;
    result.successful = false;
    return result;
  }

  // Re-initialize the particle filter
  if (reinit_pf) {
    if (pf_ != NULL) {
      pf_free(pf_);
      pf_ = NULL;
    }
    initParticleFilter();
  }

  // Re-initialize the odometry
  if (reinit_odom) {
    motion_model_.reset();
    initOdometry();
  }

  // Re-initialize the lasers and it's filters
  if (reinit_laser) {
    lasers_.clear();
    lasers_update_.clear();
    frame_to_laser_.clear();
    laser_scan_connection_.disconnect();
    laser_scan_filter_.reset();
    laser_scan_sub_.reset();

    initMessageFilters();
  }

  // Re-initialize the map
  if (reinit_map) {
    map_sub_.reset();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&AmclNode::mapReceived, this, std::placeholders::_1));
  }

  result.successful = true;
  return result;
}

void
AmclNode::mapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  RCLCPP_DEBUG(get_logger(), "AmclNode: A new map was received.");
  if (!nav2_util::validateMsg(*msg)) {
    RCLCPP_ERROR(get_logger(), "Received map message is malformed. Rejecting.");
    return;
  }
  if (first_map_only_ && first_map_received_) {
    return;
  }
  handleMapMessage(*msg);
  first_map_received_ = true;
}

void
AmclNode::handleMapMessage(const nav_msgs::msg::OccupancyGrid & msg)
{
  std::lock_guard<std::recursive_mutex> cfl(mutex_);

  RCLCPP_INFO(
    get_logger(), "Received a %d X %d map @ %.3f m/pix",
    msg.info.width,
    msg.info.height,
    msg.info.resolution);
  if (msg.header.frame_id != global_frame_id_) {
    RCLCPP_WARN(
      get_logger(), "Frame_id of map received:'%s' doesn't match global_frame_id:'%s'. This could"
      " cause issues with reading published topics",
      msg.header.frame_id.c_str(),
      global_frame_id_.c_str());
  }
  freeMapDependentMemory();
  map_ = convertMap(msg);

#if NEW_UNIFORM_SAMPLING
  createFreeSpaceVector();
#endif
}

void
AmclNode::createFreeSpaceVector()
{
  int delta = freespace_downsampling_ ? 2 : 1;
  // Index of free space
  free_space_indices.resize(0);
  for (int i = 0; i < map_->size_x; i += delta) {
    for (int j = 0; j < map_->size_y; j += delta) {
      if (map_->cells[MAP_INDEX(map_, i, j)].occ_state == -1) {
        AmclNode::Point2D point = {i, j};
        free_space_indices.push_back(point);
      }
    }
  }
}

void
AmclNode::freeMapDependentMemory()
{
  if (map_ != NULL) {
    map_free(map_);
    map_ = NULL;
  }

  // Clear queued laser objects because they hold pointers to the existing
  // map, #5202.
  lasers_.clear();
  lasers_update_.clear();
  frame_to_laser_.clear();
}

// Convert an OccupancyGrid map message into the internal representation. This function
// allocates a map_t and returns it.
map_t *
AmclNode::convertMap(const nav_msgs::msg::OccupancyGrid & map_msg)
{
  map_t * map = map_alloc();

  map->size_x = map_msg.info.width;
  map->size_y = map_msg.info.height;
  map->scale = map_msg.info.resolution;
  map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
  map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;

  map->cells =
    reinterpret_cast<map_cell_t *>(malloc(sizeof(map_cell_t) * map->size_x * map->size_y));

  // Convert to player format
  for (int i = 0; i < map->size_x * map->size_y; i++) {
    if (map_msg.data[i] == 0) {
      map->cells[i].occ_state = -1;
    } else if (map_msg.data[i] == 100) {
      map->cells[i].occ_state = +1;
    } else {
      map->cells[i].occ_state = 0;
    }
  }

  return map;
}

void
AmclNode::initTransforms()
{
  RCLCPP_INFO(get_logger(), "initTransforms");

  // Initialize transform listener and broadcaster
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(),
    get_node_timers_interface(),
    callback_group_);
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());

  sent_first_transform_ = false;
  latest_tf_valid_ = false;
  latest_tf_ = tf2::Transform::getIdentity();
}

void
AmclNode::initMessageFilters()
{
  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group_;
  laser_scan_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::LaserScan,
      rclcpp_lifecycle::LifecycleNode>>(
    shared_from_this(), scan_topic_, rmw_qos_profile_sensor_data, sub_opt);

  laser_scan_filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
    *laser_scan_sub_, *tf_buffer_, odom_frame_id_, 10,
    get_node_logging_interface(),
    get_node_clock_interface(),
    transform_tolerance_);


  laser_scan_connection_ = laser_scan_filter_->registerCallback(
    std::bind(
      &AmclNode::laserReceived,
      this, std::placeholders::_1));
}

void
AmclNode::initPubSub()
{
  RCLCPP_INFO(get_logger(), "initPubSub");

  particle_cloud_pub_ = create_publisher<nav2_msgs::msg::ParticleCloud>(
    "particle_cloud",
    rclcpp::SensorDataQoS());

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "amcl_pose",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", rclcpp::SystemDefaultsQoS(),
    std::bind(&AmclNode::initialPoseReceived, this, std::placeholders::_1));

  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&AmclNode::mapReceived, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Subscribed to map topic.");
}

void
AmclNode::initServices()
{
  global_loc_srv_ = create_service<std_srvs::srv::Empty>(
    "reinitialize_global_localization",
    std::bind(&AmclNode::globalLocalizationCallback, this, _1, _2, _3));

  initial_guess_srv_ = create_service<nav2_msgs::srv::SetInitialPose>(
    "set_initial_pose",
    std::bind(&AmclNode::initialPoseReceivedSrv, this, _1, _2, _3));

  nomotion_update_srv_ = create_service<std_srvs::srv::Empty>(
    "request_nomotion_update",
    std::bind(&AmclNode::nomotionUpdateCallback, this, _1, _2, _3));
}

void
AmclNode::initOdometry()
{
  // TODO(mjeronimo): We should handle persistance of the last known pose of the robot. We could
  // then read that pose here and initialize using that.

  // When pausing and resuming, remember the last robot pose so we don't start at 0:0 again
  init_pose_[0] = last_published_pose_.pose.pose.position.x;
  init_pose_[1] = last_published_pose_.pose.pose.position.y;
  init_pose_[2] = tf2::getYaw(last_published_pose_.pose.pose.orientation);

  if (!initial_pose_is_known_) {
    init_cov_[0] = 0.5 * 0.5;
    init_cov_[1] = 0.5 * 0.5;
    init_cov_[2] = (M_PI / 12.0) * (M_PI / 12.0);
  } else {
    init_cov_[0] = last_published_pose_.pose.covariance[0];
    init_cov_[1] = last_published_pose_.pose.covariance[7];
    init_cov_[2] = last_published_pose_.pose.covariance[35];
  }

  motion_model_ = plugin_loader_.createSharedInstance(robot_model_type_);
  motion_model_->initialize(alpha1_, alpha2_, alpha3_, alpha4_, alpha5_);

  latest_odom_pose_ = geometry_msgs::msg::PoseStamped();
}

void
AmclNode::initParticleFilter()
{
  // Create the particle filter
  pf_ = pf_alloc(
    min_particles_, max_particles_, alpha_slow_, alpha_fast_,
    (pf_init_model_fn_t)AmclNode::uniformPoseGenerator);
  pf_->pop_err = pf_err_;
  pf_->pop_z = pf_z_;

  // Initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = init_pose_[0];
  pf_init_pose_mean.v[1] = init_pose_[1];
  pf_init_pose_mean.v[2] = init_pose_[2];

  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = init_cov_[0];
  pf_init_pose_cov.m[1][1] = init_cov_[1];
  pf_init_pose_cov.m[2][2] = init_cov_[2];

  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);

  pf_init_ = false;
  resample_count_ = 0;
  memset(&pf_odom_pose_, 0, sizeof(pf_odom_pose_));
}

void
AmclNode::initLaserScan()
{
  scan_error_count_ = 0;
  last_laser_received_ts_ = rclcpp::Time(0);
}

}  // namespace nav2_amcl

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_amcl::AmclNode)

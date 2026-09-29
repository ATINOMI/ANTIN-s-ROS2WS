#include "mini_nav_nodes/local_costmap_node.hpp"
#include "mini_nav_nodes/costmap_display.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mini_nav_nodes
{
    namespace
    {
        unsigned int GridCells(double meters, double resolution)
        {
            if (!std::isfinite(meters) || !std::isfinite(resolution) ||
                meters <= 0.0 || resolution <= 0.0) {
                throw std::invalid_argument("Local costmap dimensions and resolution must be positive");
            }
            const double cells = std::round(meters / resolution);
            if (cells < 1.0 || cells > 1000.0) {
                throw std::invalid_argument("Local costmap dimensions must contain 1 to 1000 cells");
            }
            return static_cast<unsigned int>(cells);
        }

        double PositiveParameter(rclcpp::Node & node, const char * name, double fallback)
        {
            const double value = node.declare_parameter<double>(name, fallback);
            if (!std::isfinite(value) || value <= 0.0) {
                throw std::invalid_argument(std::string(name) + " must be positive and finite");
            }
            return value;
        }
    }

    LocalCostmapNode::LocalCostmapNode()
      : Node("local_costmap")
    {
        const double width = PositiveParameter(*this, "local_costmap.width", 4.0);
        const double height = PositiveParameter(*this, "local_costmap.height", 4.0);
        const double resolution = PositiveParameter(*this, "local_costmap.resolution", 0.05);
        mini_nav_core::InflationParameters inflation;
        inflation.robot_radius = PositiveParameter(*this, "local_costmap.robot_radius", 0.24);
        inflation.safety_margin = declare_parameter<double>("local_costmap.safety_margin", 0.05);
        inflation.inflation_radius = PositiveParameter(*this, "local_costmap.inflation_radius", 0.45);
        inflation.cost_scaling_factor = PositiveParameter(*this, "local_costmap.cost_scaling_factor", 10.0);
        inflation.inflate_around_unknown =
          declare_parameter<bool>("local_costmap.inflate_around_unknown", false);
        if (!std::isfinite(inflation.safety_margin) || inflation.safety_margin < 0.0 ||
            inflation.inflation_radius < inflation.robot_radius + inflation.safety_margin) {
            throw std::invalid_argument("Local costmap safety radius is invalid");
        }
        grid_ = std::make_unique<mini_nav_core::RollingObstacleGrid>(
          GridCells(width, resolution), GridCells(height, resolution), resolution, inflation);

        odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
        base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_footprint");
        const std::string scan_topic = declare_parameter<std::string>("scan_topic", "/scan");
        max_scan_age_ = PositiveParameter(*this, "local_costmap.max_scan_age", 1.0);
        observation_persistence_ = PositiveParameter(
          *this, "local_costmap.observation_persistence", 2.0);
        obstacle_max_range_ = PositiveParameter(*this, "local_costmap.obstacle_max_range", 2.5);
        raytrace_max_range_ = PositiveParameter(*this, "local_costmap.raytrace_max_range", 3.0);
        const double publish_frequency = PositiveParameter(*this, "local_costmap.publish_frequency", 5.0);
        if (odom_frame_id_.empty() || base_frame_id_.empty() || scan_topic.empty() ||
            odom_frame_id_ == base_frame_id_ ||
            raytrace_max_range_ < obstacle_max_range_) {
            throw std::invalid_argument("Local costmap frame, scan topic, or sensor range is invalid");
        }

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);
        map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
          "/mini_nav/local_costmap", rclcpp::QoS(1).reliable());
        valid_publisher_ = create_publisher<std_msgs::msg::Bool>(
          "/mini_nav/local_costmap_valid", rclcpp::QoS(1).reliable());
        scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
          scan_topic, rclcpp::SensorDataQoS(),
          [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) { scanCallback(scan); });
        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_frequency),
          [this]() { publishMap(); });
    }

    void LocalCostmapNode::invalidate(const char * reason)
    {
        if (valid_) {
            RCLCPP_WARN(get_logger(), "Local costmap invalid: %s", reason);
        }
        valid_ = false;
        last_scan_stamp_.reset();
        grid_->Reset();
    }

    void LocalCostmapNode::scanCallback(
      const sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
    {
        const rclcpp::Time stamp(scan->header.stamp, get_clock()->get_clock_type());
        const double age = (now() - stamp).seconds();
        if (scan->header.frame_id.empty() || scan->ranges.empty() ||
            !std::isfinite(scan->angle_min) || !std::isfinite(scan->angle_increment) ||
            scan->angle_increment == 0.0 || !std::isfinite(scan->range_min) ||
            !std::isfinite(scan->range_max) || scan->range_min < 0.0 ||
            scan->range_max <= scan->range_min || !std::isfinite(age) ||
            age > max_scan_age_ || age < -max_scan_age_ ||
            (last_scan_stamp_ && stamp <= *last_scan_stamp_)) {
            invalidate("scan metadata or timestamp is unusable");
            return;
        }

        try {
            // 两次 TF 查询都使用扫描时刻；机器人中心和每束激光不能混用当前 TF。
            const auto base_tf = tf_buffer_->lookupTransform(
              odom_frame_id_, base_frame_id_, stamp, rclcpp::Duration::from_seconds(0.05));
            const auto sensor_tf = tf_buffer_->lookupTransform(
              odom_frame_id_, scan->header.frame_id, stamp,
              rclcpp::Duration::from_seconds(0.05));
            tf2::Transform sensor_to_odom;
            tf2::fromMsg(sensor_tf.transform, sensor_to_odom);
            const tf2::Vector3 sensor = sensor_to_odom.getOrigin();
            if (!std::isfinite(base_tf.transform.translation.x) ||
                !std::isfinite(base_tf.transform.translation.y) ||
                !std::isfinite(sensor.x()) || !std::isfinite(sensor.y())) {
                invalidate("scan TF contains non-finite position");
                return;
            }
            grid_->CenterOn(base_tf.transform.translation.x, base_tf.transform.translation.y);

            std::vector<std::pair<double, double>> hits;
            const double scan_time = stamp.seconds();
            unsigned int traced = 0;
            for (std::size_t index = 0; index < scan->ranges.size(); ++index) {
                const double range = scan->ranges[index];
                const bool no_return = std::isinf(range) && range > 0.0;
                if (!no_return && (!std::isfinite(range) || range < scan->range_min ||
                                   range > scan->range_max)) {
                    continue;
                }
                const double distance = no_return ?
                    std::min(raytrace_max_range_, static_cast<double>(scan->range_max)) :
                    std::min(raytrace_max_range_, range);
                const double angle = scan->angle_min + index * scan->angle_increment;
                if (!std::isfinite(angle) || !std::isfinite(distance) || distance <= 0.0) {
                    continue;
                }
                const tf2::Vector3 endpoint = sensor_to_odom * tf2::Vector3(
                  distance * std::cos(angle), distance * std::sin(angle), 0.0);
                if (grid_->IntegrateRay(
                      sensor.x(), sensor.y(), endpoint.x(), endpoint.y(), scan_time)) {
                    ++traced;
                    if (!no_return && range <= obstacle_max_range_) {
                        hits.emplace_back(endpoint.x(), endpoint.y());
                    }
                }
            }
            if (traced == 0) {
                invalidate("scan has no usable rays");
                return;
            }
            // 全部射线清除后再标记命中点，避免后一束射线擦掉前一束的障碍。
            for (const auto & hit : hits) {
                grid_->MarkObstacle(hit.first, hit.second, scan_time);
            }
            last_scan_stamp_ = stamp;
            valid_ = true;
        } catch (const tf2::TransformException & error) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
              "Local costmap scan TF unavailable: %s", error.what());
            invalidate("scan TF unavailable");
        }
    }

    void LocalCostmapNode::publishMap()
    {
        if (valid_ && last_scan_stamp_ &&
            ((now() - *last_scan_stamp_).seconds() > max_scan_age_ ||
             (now() - *last_scan_stamp_).seconds() < -max_scan_age_)) {
            invalidate("scan timeout");
        }
        grid_->Expire(now().seconds(), observation_persistence_);
        const auto & raw = grid_->Raw();
        const mini_nav_core::Costmap2D inflated = valid_ ? grid_->Inflated() : raw;
        nav_msgs::msg::OccupancyGrid message;
        message.header.stamp = now();
        message.header.frame_id = odom_frame_id_;
        message.info.map_load_time = message.header.stamp;
        message.info.resolution = static_cast<float>(inflated.GetResolution());
        message.info.width = inflated.GetSizeInCellsX();
        message.info.height = inflated.GetSizeInCellsY();
        message.info.origin.position.x = inflated.GetOriginX();
        message.info.origin.position.y = inflated.GetOriginY();
        message.info.origin.orientation.w = 1.0;
        message.data.reserve(inflated.GetCellCount());
        for (std::size_t index = 0; index < inflated.GetCellCount(); ++index) {
            const auto cost = inflated.GetCost(index);
            message.data.push_back(CostToOccupancyValue(cost));
        }
        map_publisher_->publish(message);
        std_msgs::msg::Bool status;
        status.data = valid_;
        valid_publisher_->publish(status);
    }
}

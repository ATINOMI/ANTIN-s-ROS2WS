/**
 * @file local_costmap_node.cpp
 * @brief 激光观测到 odom 系滚动局部安全图的适配。
 * @author Antinomy
 * @date 2026-10-01
 */
#include "mini_nav_nodes/local_costmap_node.hpp"
#include "mini_nav_nodes/costmap_display.hpp"

#include <algorithm>
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "mini_nav_nodes/cloud_validation.hpp"
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
        /**
         * @brief 按分辨率四舍五入计算局部窗口格数并限制容量。
         *
         * @param meters 窗口单轴长度，有限正数，米。
         * @param resolution 格边长，有限正数，米。
         * @return 1..1000 范围的格数。
         * @throws std::invalid_argument 几何非法或格数超出限制。
         */
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

        /**
         * @brief 声明并读取有限正浮点参数。
         *
         * @param node 声明参数的节点。
         * @param name 参数名称。
         * @param fallback 未覆盖时的默认值。
         * @return 通过校验的参数值。
         * @throws std::invalid_argument 数值非有限或不大于零。
         */
        double PositiveParameter(rclcpp::Node & node, const char * name, double fallback)
        {
            const double value = node.declare_parameter<double>(name, fallback);
            if (!std::isfinite(value) || value <= 0.0) {
                throw std::invalid_argument(std::string(name) + " must be positive and finite");
            }
            return value;
        }
    }

    /**
     * @brief 建立 odom 系滚动局部图、传感器订阅和发布定时器。
     * @throws std::invalid_argument 图尺寸、膨胀参数、帧名或量程限制不合法。
     */
    LocalCostmapNode::LocalCostmapNode()
      : Node("local_costmap")
    {
        const double width = PositiveParameter(*this, "local_costmap.width", 4.0);
        const double height = PositiveParameter(*this, "local_costmap.height", 4.0);
        const double resolution = PositiveParameter(*this, "local_costmap.resolution", 0.05);
        mini_nav_core::InflationParameters inflation;
        inflation.robot_radius = PositiveParameter(*this, "local_costmap.robot_radius", 0.24);
        inflation.safety_margin = declare_parameter<double>("local_costmap.safety_margin", 0.02);
        inflation.inscribed_radius = declare_parameter<double>(
            "local_costmap.inscribed_radius", 0.22549849949589046);
        inflation.inflation_radius = PositiveParameter(*this, "local_costmap.inflation_radius", 0.70);
        inflation.cost_scaling_factor = PositiveParameter(*this, "local_costmap.cost_scaling_factor", 3.0);
        inflation.inflate_around_unknown =
          declare_parameter<bool>("local_costmap.inflate_around_unknown", false);
        if (!std::isfinite(inflation.safety_margin) || inflation.safety_margin < 0.0 ||
            inflation.inflation_radius < inflation.robot_radius + inflation.safety_margin) {
            throw std::invalid_argument("Local costmap safety radius is invalid");
        }
        grid_ = std::make_unique<mini_nav_core::RollingObstacleGrid>(
          GridCells(width, resolution), GridCells(height, resolution), resolution, inflation);
        safety_inflation_ = inflation;
        safety_inflation_.inscribed_radius = 0.0;
        observation_uncertainty_ = PositiveParameter(*this, "local_costmap.observation_uncertainty", 0.03);

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
        safety_map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
          "/mini_nav/local_safety_costmap", rclcpp::QoS(1).reliable());
        collision_publisher_ = create_publisher<msg::CollisionMap>(
          "/mini_nav/local_collision_map", rclcpp::QoS(1).reliable());
        valid_publisher_ = create_publisher<std_msgs::msg::Bool>(
          "/mini_nav/local_costmap_valid", rclcpp::QoS(1).reliable());
        scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
          scan_topic, rclcpp::SensorDataQoS(),
          [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) { scanCallback(scan); });
        const auto cloud_topic = declare_parameter<std::string>("collision_cloud_topic", "");
        require_cloud_ = !cloud_topic.empty();
        if (require_cloud_) {
            cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
                cloud_topic, rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
                    latest_cloud_ = msg; cloud_received_ = std::chrono::steady_clock::now();
                });
        }
        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_frequency),
          [this]() { publishMap(); });
    }

    /**
     * @brief 使全部观测失效，清除扫描时间并把局部图恢复为未知。
     *
     * @param reason 失效原因，用于有效到无效转换时的警告日志。
     */
    void LocalCostmapNode::invalidate(const char * reason)
    {
        if (valid_) {
            RCLCPP_WARN(get_logger(), "Local costmap invalid: %s", reason);
        }
        valid_ = false;
        last_scan_stamp_.reset();
        grid_->Reset();
        integrated_cloud_stamp_.reset();
    }

    /**
     * @brief 按扫描时刻 TF 清除射线并统一标记命中障碍。
     *
     * 正无穷量程按无返回处理，只清除可见空间而不标记障碍。
     * 一帧先完成全部清除再标记命中，避免束间相互擦除；传感器和机器人使用同一时刻 TF。
     *
     * @param scan 激光扫描；非法时间、元数据、TF 或无可用射线使局部图失效。
     */
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

    /**
     * @brief 过期观测后发布局部图与独立的感知有效性标志。
     *
     * 扫描超时先重置为未知；有效时生成膨胀副本。消息发布时间不是观测时间，
     * 消费者必须同时检查 local_costmap_valid 的新鲜度。
     */
    void LocalCostmapNode::publishMap()
    {
        if (valid_ && last_scan_stamp_ &&
            ((now() - *last_scan_stamp_).seconds() > max_scan_age_ ||
             (now() - *last_scan_stamp_).seconds() < -max_scan_age_)) {
            invalidate("scan timeout");
        }
        grid_->Expire(now().seconds(), observation_persistence_);
        bool cloud_valid = !require_cloud_;
        if (require_cloud_ && latest_cloud_) {
            const auto & cloud = *latest_cloud_;
            const auto stamp = rclcpp::Time(cloud.header.stamp, get_clock()->get_clock_type());
            const double age = (now() - stamp).seconds();
            cloud_valid = age >= -0.1 && age <= max_scan_age_ &&
                std::chrono::duration<double>(std::chrono::steady_clock::now() - cloud_received_).count() <= max_scan_age_ &&
                !cloud.header.frame_id.empty() && ValidCollisionCloud(cloud);
            if (cloud_valid) {
                try {
                    const auto tf = tf_buffer_->lookupTransform(odom_frame_id_, cloud.header.frame_id, stamp,
                        rclcpp::Duration::from_seconds(0.05));
                    tf2::Transform sensor_to_odom;
                    tf2::fromMsg(tf.transform, sensor_to_odom);
                    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
                    for (; x != x.end(); ++x, ++y, ++z) {
                        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) continue;
                        const auto point = sensor_to_odom * tf2::Vector3(*x, *y, *z);
                        grid_->MarkObstacle(point.x(), point.y(), stamp.seconds(), false);
                    }
                    integrated_cloud_stamp_ = stamp;
                } catch (const std::exception &) { cloud_valid = false; }
            }
        }
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
        // 从原始观测单独膨胀真实车体，不能再次膨胀已生成的显示图。
        const auto safety = valid_ ? mini_nav_core::InflateCostmap(raw, safety_inflation_) : raw;
        for (std::size_t index = 0; index < safety.GetCellCount(); ++index) {
            message.data[index] = CostToOccupancyValue(safety.GetCost(index));
        }
        safety_map_publisher_->publish(message);
        msg::CollisionMap collision;
        collision.grid = message;
        // 原子快照携带观测时间，定时重发不能把旧感知变成新感知。
        if (last_scan_stamp_) collision.grid.header.stamp = *last_scan_stamp_;
        if (require_cloud_ && integrated_cloud_stamp_ && last_scan_stamp_ &&
            *integrated_cloud_stamp_ < *last_scan_stamp_)
            collision.grid.header.stamp = *integrated_cloud_stamp_;
        const auto coverage = grid_->CollisionGrid();
        for (std::size_t i = 0; i < coverage.GetCellCount(); ++i)
            collision.grid.data[i] = CostToOccupancyValue(coverage.GetCost(i));
        for (const auto & point : grid_->ObstaclePoints()) {
            geometry_msgs::msg::Point value;
            value.x = point.x; value.y = point.y;
            collision.points.push_back(value);
        }
        collision.observation_uncertainty = observation_uncertainty_;
        collision.clearance_radius = safety_inflation_.robot_radius + safety_inflation_.safety_margin;
        collision.valid = valid_ && cloud_valid;
        collision_publisher_->publish(collision);
        std_msgs::msg::Bool status;
        status.data = valid_ && cloud_valid;
        valid_publisher_->publish(status);
    }
}

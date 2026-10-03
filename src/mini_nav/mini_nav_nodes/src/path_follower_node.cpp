/**
 * @file path_follower_node.cpp
 * @brief FollowPath 执行、输入新鲜度检查和候选速度输出。
 * @author Antinomy
 * @date 2026-10-01
 */
#include "mini_nav_nodes/path_follower_node.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mini_nav_nodes
{
    namespace
    {
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

        /**
         * @brief 验证轴对齐占据图并转为核心安全图。
         *
         * 本转换用于二值安全检查而非保留软代价规划。
         *
         * @param message -1..100 的占据数据，最多 100 万格。
         * @param frame 要求的参考帧。
         * @return 独立安全图；未知→255、100→254、99→253，其余可通行值→0。
         * @throws std::invalid_argument 几何、帧、四元数或数据范围非法。
         */
        std::unique_ptr<mini_nav_core::Costmap2D> ConvertSafetyMap(
            const nav_msgs::msg::OccupancyGrid & message, const std::string & frame)
        {
            const auto & info = message.info;
            const auto & orientation = info.origin.orientation;
            const std::size_t cells = static_cast<std::size_t>(info.width) * info.height;
            if (message.header.frame_id != frame || info.width == 0 || info.height == 0 ||
                cells > 1000000 || cells != message.data.size() ||
                !std::isfinite(info.resolution) || info.resolution <= 0.0 ||
                !std::isfinite(info.origin.position.x) ||
                !std::isfinite(info.origin.position.y) ||
                !std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
                !std::isfinite(orientation.z) || !std::isfinite(orientation.w) ||
                std::abs(orientation.x) > 1.0e-6 ||
                std::abs(orientation.y) > 1.0e-6 ||
                std::abs(orientation.z) > 1.0e-6 ||
                std::abs(std::abs(orientation.w) - 1.0) > 1.0e-6) {
                throw std::invalid_argument("Safety map metadata is invalid");
            }
            auto grid = std::make_unique<mini_nav_core::Costmap2D>(
                info.width, info.height, info.resolution,
                info.origin.position.x, info.origin.position.y, 255);
            for (unsigned int y = 0; y < info.height; ++y) {
                for (unsigned int x = 0; x < info.width; ++x) {
                    const int value = message.data[static_cast<std::size_t>(y) * info.width + x];
                    if (value < -1 || value > 100) {
                        throw std::invalid_argument("Safety map cell is outside OccupancyGrid range");
                    }
                    const unsigned char cost = value < 0 ? 255 :
                        value == 100 ? 254 : value >= 99 ? 253 : 0;
                    grid->SetCost(x, y, cost);
                }
            }
            return grid;
        }

        /**
         * @brief 从 TF 提取有限二维位姿。
         *
         * @param transform 源帧在目标帧中的变换。
         * @param pose 输出二维位姿；失败时不能使用。
         * @return 平移、四元数模长及偏航有效时 true。
         */
        bool TransformPose(
            const geometry_msgs::msg::TransformStamped & transform,
            mini_nav_core::localization::Pose2D & pose)
        {
            const auto & translation = transform.transform.translation;
            const auto & rotation = transform.transform.rotation;
            const double norm = std::hypot(
                std::hypot(rotation.x, rotation.y),
                std::hypot(rotation.z, rotation.w));
            if (!std::isfinite(translation.x) || !std::isfinite(translation.y) ||
                !std::isfinite(norm) || norm < 1.0e-6) {
                return false;
            }
            pose = {translation.x, translation.y, tf2::getYaw(rotation)};
            return std::isfinite(pose.yaw);
        }

        /**
         * @brief 把核心跟踪枚举转为监控使用的稳定状态码。
         *
         * @param status 核心跟踪状态。
         * @return 静态字符串；未识别枚举返回 invalid_state。
         */
        const char * TrackingStatusName(mini_nav_core::TrackingStatus status)
        {
            using mini_nav_core::TrackingStatus;
            switch (status) {
                case TrackingStatus::kNoPath: return "no_path";
                case TrackingStatus::kTracking: return "tracking";
                case TrackingStatus::kGoalReached: return "goal_reached";
                case TrackingStatus::kOffPath: return "off_path";
                case TrackingStatus::kCollisionRisk: return "collision_risk";
                case TrackingStatus::kProgressTimeout: return "progress_timeout";
                case TrackingStatus::kInvalidPose: return "invalid_pose";
            }
            return "invalid_state";
        }
    }

    /**
     * @brief 建立路径跟踪 Action、双安全图输入与周期控制。
     *
     * action_mode 禁用话题路径入口；命令发布话题可配置，主入口使用 cmd_vel_raw。
     * 对传感器/TF 丢失的反应时间取配置期限的最大值，用于制动距离预测。
     * @throws std::invalid_argument 帧名、话题、速度或数值限制非法。
     */
    PathFollowerNode::PathFollowerNode() : Node("path_follower")
    {
        map_frame_id_ = declare_parameter<std::string>("map_frame_id", "map");
        odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
        base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_footprint");
        const std::string path_topic = declare_parameter<std::string>(
            "path_topic", "/mini_nav/global_path");
        const std::string static_map_topic = declare_parameter<std::string>(
            "static_map_topic", "/mini_nav/planning_costmap");
        const std::string local_map_topic = declare_parameter<std::string>(
            "local_map_topic", "/mini_nav/local_costmap");
        const std::string local_valid_topic = declare_parameter<std::string>(
            "local_valid_topic", "/mini_nav/local_costmap_valid");
        const double control_frequency = PositiveParameter(*this, "controller.frequency", 10.0);
        max_path_age_ = PositiveParameter(*this, "controller.max_path_age", 3.0);
        max_static_map_age_ = PositiveParameter(*this, "controller.max_static_map_age", 3.0);
        max_local_map_age_ = PositiveParameter(*this, "controller.max_local_map_age", 0.8);
        max_tf_age_ = PositiveParameter(*this, "controller.max_tf_age", 1.0);
        max_tf_future_ = PositiveParameter(*this, "controller.max_tf_future", 0.7);
        clock_stall_timeout_ = PositiveParameter(*this, "controller.clock_stall_timeout", 1.0);
        mini_nav_core::PathTrackerParameters parameters;
        parameters.max_linear_speed = PositiveParameter(*this, "controller.max_linear_speed", 0.10);
        parameters.max_angular_speed = PositiveParameter(*this, "controller.max_angular_speed", 0.40);
        parameters.lookahead_distance = PositiveParameter(*this, "controller.lookahead_distance", 0.35);
        parameters.goal_position_tolerance = PositiveParameter(
            *this, "controller.goal_position_tolerance", 0.12);
        parameters.goal_yaw_tolerance = PositiveParameter(
            *this, "controller.goal_yaw_tolerance", 0.15);
        parameters.rotate_in_place_angle = PositiveParameter(
            *this, "controller.rotate_in_place_angle", 0.35);
        parameters.max_path_deviation = PositiveParameter(
            *this, "controller.max_path_deviation", 0.40);
        parameters.progress_distance = PositiveParameter(
            *this, "controller.progress_distance", 0.10);
        parameters.progress_yaw = PositiveParameter(*this, "controller.progress_yaw", 0.30);
        parameters.progress_timeout = PositiveParameter(*this, "controller.progress_timeout", 10.0);
        parameters.max_linear_acceleration = PositiveParameter(*this, "controller.max_linear_acceleration", 0.30);
        parameters.max_linear_deceleration = PositiveParameter(*this, "controller.max_linear_deceleration", 0.50);
        parameters.max_angular_acceleration = PositiveParameter(*this, "controller.max_angular_acceleration", 1.0);
        parameters.command_reaction_time = PositiveParameter(*this, "controller.command_reaction_time", 1.0);
        // Predict at least through the slowest input-loss deadline, plus braking distance.
        parameters.command_reaction_time = std::max({parameters.command_reaction_time,
            max_tf_age_, max_local_map_age_, clock_stall_timeout_});
        if (map_frame_id_.empty() || odom_frame_id_.empty() || base_frame_id_.empty() ||
            map_frame_id_ == odom_frame_id_ || odom_frame_id_ == base_frame_id_ ||
            path_topic.empty() || static_map_topic.empty() || local_map_topic.empty() ||
            local_valid_topic.empty() || control_frequency > 50.0 ||
            parameters.max_linear_speed > 0.15 || parameters.max_angular_speed > 0.60 ||
            parameters.goal_yaw_tolerance >= 3.14159265358979323846) {
            throw std::invalid_argument("Path follower frames, topics, or limits are invalid");
        }
        tracker_ = std::make_unique<mini_nav_core::PathTracker>(parameters);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

        const auto latched = rclcpp::QoS(1).reliable().transient_local();
        const auto live = rclcpp::QoS(1).reliable();
        action_mode_ = declare_parameter<bool>("action_mode", false);
        require_quality_ = declare_parameter<bool>("require_localization_quality", false);
        if (!action_mode_) {
        path_subscription_ = create_subscription<nav_msgs::msg::Path>(
            path_topic, latched, [this](nav_msgs::msg::Path::ConstSharedPtr message) {
                pathCallback(message);
            });
        }
        quality_subscription_ = create_subscription<std_msgs::msg::Bool>(
            "/mini_nav/localization_valid", live, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
                quality_valid_ = msg->data;
                quality_received_ = std::chrono::steady_clock::now();
            });
        // 前端重启或重新配准使旧路径失效，要求新的 FollowPath 目标。
        epoch_subscription_ = create_subscription<std_msgs::msg::String>(
            "/mini_nav/localization_epoch", latched,
            [this](std_msgs::msg::String::ConstSharedPtr msg) {
                if (!localization_epoch_.empty() && localization_epoch_ != msg->data) {
                    quality_valid_ = false;
                    if (follow_goal_) finishFollow(FollowPath::Result::UNKNOWN, "localization_epoch_changed");
                    else { tracker_->ClearPath(); publishStop("localization_epoch_changed"); }
                }
                localization_epoch_ = msg->data;
            });
        follow_server_ = rclcpp_action::create_server<FollowPath>(this, "/follow_path",
            [](const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowPath::Goal> goal) {
                if (goal->path.poses.empty() || (!goal->controller_id.empty() && goal->controller_id != "PathTracker") ||
                    !goal->goal_checker_id.empty() || !goal->progress_checker_id.empty())
                    return rclcpp_action::GoalResponse::REJECT;
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            }, [this](auto handle) {
                if (handle == follow_goal_) publishStop("canceling");
                return rclcpp_action::CancelResponse::ACCEPT;
            }, [this](auto handle) {
                if (follow_goal_) finishFollow(FollowPath::Result::UNKNOWN, "preempted");
                follow_goal_ = handle;
                pathCallback(std::make_shared<nav_msgs::msg::Path>(handle->get_goal()->path));
                follow_endpoint_ = handle->get_goal()->path.poses.back();
                if (!tracker_->HasPath()) finishFollow(FollowPath::Result::INVALID_PATH, "invalid_path");
            });
        static_map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            static_map_topic, latched,
            [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {
                staticMapCallback(message);
            });
        local_map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            local_map_topic, live,
            [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {
                localMapCallback(message);
            });
        local_valid_subscription_ = create_subscription<std_msgs::msg::Bool>(
            local_valid_topic, live,
            [this](std_msgs::msg::Bool::ConstSharedPtr message) {
                localValidCallback(message);
            });
        command_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(
            declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel"), live);
        status_publisher_ = create_publisher<std_msgs::msg::String>(
            "/mini_nav/controller_status", latched);
        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / control_frequency),
            [this]() { controlTick(); });
        // SIGINT 时先发送零速度，再让 ROS context 和通信对象退出。
        shutdown_callback_handle_ = std::make_unique<rclcpp::PreShutdownCallbackHandle>(
            get_node_base_interface()->get_context()->add_pre_shutdown_callback([this]() {
                geometry_msgs::msg::TwistStamped stop;
                stop.header.stamp = now();
                stop.header.frame_id = base_frame_id_;
                command_publisher_->publish(stop);
            }));
    }

    /**
     * @brief 撤销 context 预关闭回调，防止已销毁对象被回调访问。
     */
    PathFollowerNode::~PathFollowerNode()
    {
        get_node_base_interface()->get_context()->remove_pre_shutdown_callback(
            *shutdown_callback_handle_);
    }

    /**
     * @brief 校验 map 系路径与末点四元数，转换后提交核心跟踪器。
     *
     * 非法帧或数据导致清除路径与零速；Action 路径持续由目标身份管理，
     * 话题模式另外检查路径时间戳和接收时间。
     *
     * @param message 路径消息；空路径清除跟踪状态并停车。
     */
    void PathFollowerNode::pathCallback(const nav_msgs::msg::Path::ConstSharedPtr message)
    {
        if (message->poses.empty()) {
            tracker_->ClearPath();
            path_stamp_.reset();
            publishStop("no_path");
            return;
        }
        try {
            if (message->header.frame_id != map_frame_id_) {
                throw std::invalid_argument("Path frame does not match map frame");
            }
            std::vector<mini_nav_core::PathPoint> points;
            points.reserve(message->poses.size());
            for (const auto & pose : message->poses) {
                if (!pose.header.frame_id.empty() &&
                    pose.header.frame_id != map_frame_id_) {
                    throw std::invalid_argument("Path pose frame does not match map frame");
                }
                points.push_back({pose.pose.position.x, pose.pose.position.y});
            }
            const auto & q = message->poses.back().pose.orientation;
            const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
            if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1.0e-3) {
                throw std::invalid_argument("Path goal orientation is invalid");
            }
            tracker_->SetPath(points, tf2::getYaw(q));
            path_stamp_ = rclcpp::Time(message->header.stamp, get_clock()->get_clock_type());
            path_received_ = std::chrono::steady_clock::now();
        } catch (const std::exception & error) {
            RCLCPP_WARN(get_logger(), "Rejecting unsafe path: %s", error.what());
            tracker_->ClearPath();
            path_stamp_.reset();
            publishStop("invalid_path");
        }
    }

    /**
     * @brief 转换并保存 map 系膨胀安全图及双时钟新鲜度记录。
     *
     * @param message 全局规划图 OccupancyGrid；转换失败清空缓存并停车。
     */
    void PathFollowerNode::staticMapCallback(
        const nav_msgs::msg::OccupancyGrid::ConstSharedPtr message)
    {
        try {
            static_map_ = ConvertSafetyMap(*message, map_frame_id_);
            static_map_stamp_ = rclcpp::Time(message->header.stamp, get_clock()->get_clock_type());
            static_map_received_ = std::chrono::steady_clock::now();
        } catch (const std::exception & error) {
            RCLCPP_WARN(get_logger(), "Rejecting static safety map: %s", error.what());
            static_map_.reset();
            static_map_stamp_.reset();
            publishStop("invalid_static_map");
        }
    }

    /**
     * @brief 转换并保存 odom 系局部安全图及双时钟新鲜度记录。
     *
     * @param message 滚动局部图 OccupancyGrid；转换失败清空缓存并停车。
     */
    void PathFollowerNode::localMapCallback(
        const nav_msgs::msg::OccupancyGrid::ConstSharedPtr message)
    {
        try {
            local_map_ = ConvertSafetyMap(*message, odom_frame_id_);
            local_map_stamp_ = rclcpp::Time(message->header.stamp, get_clock()->get_clock_type());
            local_map_received_ = std::chrono::steady_clock::now();
        } catch (const std::exception & error) {
            RCLCPP_WARN(get_logger(), "Rejecting local safety map: %s", error.what());
            local_map_.reset();
            local_map_stamp_.reset();
            publishStop("invalid_local_map");
        }
    }

    /**
     * @brief 记录局部感知有效性心跳；失效时暂停进展计时并停车。
     *
     * @param message 局部图节点发布的有效性标志。
     */
    void PathFollowerNode::localValidCallback(const std_msgs::msg::Bool::ConstSharedPtr message)
    {
        local_valid_ = message->data;
        local_valid_received_ = std::chrono::steady_clock::now();
        if (!local_valid_ && tracker_->HasPath()) {
            tracker_->PauseProgress();
            publishStop("local_sensor_invalid");
        }
    }

    /**
     * @brief 同时按 ROS 消息时间与稳态接收时间判断输入新鲜度。
     *
     * 两种检查共同防止迟到消息或仿真时钟停滞掩盖输入断流。
     *
     * @param stamp ROS 消息时间戳。
     * @param received 稳态时钟接收时刻。
     * @param max_age 最大年龄，秒。
     * @param current 当前 ROS 时间，与 stamp 同源。
     * @param steady_now 当前稳态时间。
     * @return 消息年龄位于 [-0.1, max_age] 且接收年龄位于 [0, max_age] 时为 true。
     */
    bool PathFollowerNode::fresh(
        const rclcpp::Time & stamp, const SteadyTime & received,
        double max_age, const rclcpp::Time & current,
        const SteadyTime & steady_now) const
    {
        const double message_age = (current - stamp).seconds();
        const double receive_age = std::chrono::duration<double>(steady_now - received).count();
        return std::isfinite(message_age) && message_age >= -0.1 &&
            message_age <= max_age &&
            receive_age >= 0.0 && receive_age <= max_age;
    }

    /**
     * @brief 逐项检查时钟、目标、定位、双图和 TF，再执行一次核心跟踪。
     *
     * 任一前提失败都发送零速；到达或进展超时结束 FollowPath。
     * TF 时间检查允许 AMCL transform_tolerance 引入的有限未来时间戳。
     */
    void PathFollowerNode::controlTick()
    {
        const auto steady_now = std::chrono::steady_clock::now();
        const auto current = now();
        if (follow_goal_ && follow_goal_->is_canceling()) {
            auto result = std::make_shared<FollowPath::Result>();
            follow_goal_->canceled(result);
            follow_goal_.reset();
            tracker_->ClearPath();
            publishStop("canceled");
            return;
        }
        if (last_clock_time_ && current < *last_clock_time_) {
            last_clock_time_ = current;
            last_clock_advance_ = steady_now;
            tracker_->PauseProgress();
            publishStop("clock_jump");
            return;
        }
        if (!last_clock_time_ || current > *last_clock_time_) {
            last_clock_advance_ = steady_now;
        }
        last_clock_time_ = current;
        if (current.nanoseconds() == 0 || !last_clock_advance_ ||
            std::chrono::duration<double>(steady_now - *last_clock_advance_).count() >
                clock_stall_timeout_) {
            tracker_->PauseProgress();
            publishStop("clock_stale");
            return;
        }
        if (action_mode_ && !follow_goal_) { publishStop("no_path"); return; }
        if (require_quality_ && (!quality_valid_ || quality_received_ == SteadyTime{} ||
            std::chrono::duration<double>(steady_now - quality_received_).count() > 0.8)) {
            tracker_->PauseProgress(); publishStop("localization_invalid"); return;
        }
        if (!tracker_->HasPath()) {
            publishStop("no_path");
            return;
        }
        if (!follow_goal_ && (!path_stamp_ || !path_received_ ||
            !fresh(*path_stamp_, *path_received_, max_path_age_, current, steady_now))) {
            tracker_->PauseProgress();
            publishStop("path_stale");
            return;
        }
        if (!static_map_ || !static_map_stamp_ || !static_map_received_ ||
            !fresh(*static_map_stamp_, *static_map_received_,
                   max_static_map_age_, current, steady_now)) {
            tracker_->PauseProgress();
            publishStop("static_map_stale");
            return;
        }
        if (!local_valid_ || !local_valid_received_ ||
            std::chrono::duration<double>(steady_now - *local_valid_received_).count() >
                max_local_map_age_ ||
            !local_map_ || !local_map_stamp_ || !local_map_received_ ||
            !fresh(*local_map_stamp_, *local_map_received_,
                   max_local_map_age_, current, steady_now)) {
            tracker_->PauseProgress();
            publishStop("local_sensor_invalid");
            return;
        }

        try {
            const auto zero = rclcpp::Time(0, 0, get_clock()->get_clock_type());
            const auto map_tf = tf_buffer_->lookupTransform(map_frame_id_, base_frame_id_, zero);
            const auto odom_tf = tf_buffer_->lookupTransform(odom_frame_id_, base_frame_id_, zero);
            const auto odom_from_map_tf = tf_buffer_->lookupTransform(
                odom_frame_id_, map_frame_id_, zero);
            const auto tf_fresh = [&](const geometry_msgs::msg::TransformStamped & transform) {
                const double age = (current - rclcpp::Time(
                    transform.header.stamp, get_clock()->get_clock_type())).seconds();
                // AMCL 将 map->odom 时间戳按 transform_tolerance 前推。
                return std::isfinite(age) && age >= -max_tf_future_ &&
                    age <= max_tf_age_;
            };
            mini_nav_core::localization::Pose2D map_pose;
            mini_nav_core::localization::Pose2D odom_pose;
            mini_nav_core::localization::Pose2D odom_from_map;
            if (!tf_fresh(map_tf) || !tf_fresh(odom_tf) ||
                !tf_fresh(odom_from_map_tf) ||
                !TransformPose(map_tf, map_pose) ||
                !TransformPose(odom_tf, odom_pose) ||
                !TransformPose(odom_from_map_tf, odom_from_map)) {
                tracker_->PauseProgress();
                publishStop("tf_stale");
                return;
            }
            const double steady_seconds = std::chrono::duration<double>(
                steady_now.time_since_epoch()).count();
            const auto command = tracker_->Step(
                map_pose, odom_pose, odom_from_map,
                *static_map_, *local_map_, steady_seconds);
            publishCommand(command.linear_x, command.angular_z,
                           TrackingStatusName(command.status));
            if (follow_goal_) {
                auto feedback = std::make_shared<FollowPath::Feedback>();
                feedback->distance_to_goal = std::hypot(map_pose.x - follow_endpoint_.pose.position.x,
                    map_pose.y - follow_endpoint_.pose.position.y);
                feedback->speed = command.linear_x;
                follow_goal_->publish_feedback(feedback);
                if (command.status == mini_nav_core::TrackingStatus::kGoalReached)
                    finishFollow(FollowPath::Result::NONE, "goal_reached");
                else if (command.status == mini_nav_core::TrackingStatus::kProgressTimeout)
                    finishFollow(FollowPath::Result::FAILED_TO_MAKE_PROGRESS, "progress_timeout");
            }
        } catch (const tf2::TransformException &) {
            tracker_->PauseProgress();
            publishStop("tf_unavailable");
        } catch (const std::exception & error) {
            tracker_->PauseProgress();
            RCLCPP_ERROR(get_logger(), "Path follower control failed: %s", error.what());
            publishStop("control_error");
        }
    }

    /**
     * @brief 发布带底盘帧与当前时间戳的平面速度及控制状态。
     *
     * 本函数不再限幅；调用者须先完成安全检查。
     *
     * @param linear 前向速度，m/s。
     * @param angular 偏航角速度，rad/s。
     * @param status 控制状态码；只有状态变化时记录日志。
     */
    void PathFollowerNode::publishCommand(
        double linear, double angular, const std::string & status)
    {
        geometry_msgs::msg::TwistStamped command;
        command.header.stamp = now();
        command.header.frame_id = base_frame_id_;
        command.twist.linear.x = linear;
        command.twist.angular.z = angular;
        command_publisher_->publish(command);
        std_msgs::msg::String status_message;
        status_message.data = status;
        status_publisher_->publish(status_message);
        if (status != last_status_) {
            if (status == "tracking" || status == "goal_reached" || status == "no_path") {
                RCLCPP_INFO(get_logger(), "Path follower: %s", status.c_str());
            } else {
                RCLCPP_WARN(get_logger(), "Path follower stopped: %s", status.c_str());
            }
            last_status_ = status;
        }
    }

    /**
     * @brief 发布零线速度、零角速度并说明停车状态。
     *
     * @param status 停车原因状态码。
     */
    void PathFollowerNode::publishStop(const std::string & status)
    {
        publishCommand(0.0, 0.0, status);
    }
}

/**
 * @brief 先停车，再按错误码或取消状态结束当前 FollowPath 并清除路径。
 *
 * @param code FollowPath 结果错误码；NONE 表示成功。
 * @param reason 结果消息和停车状态。
 */
void mini_nav_nodes::PathFollowerNode::finishFollow(uint16_t code, const std::string & reason)
{
    if (!follow_goal_) return;
    publishStop(reason);
    auto result = std::make_shared<FollowPath::Result>();
    result->error_code = code;
    result->error_msg = reason;
    if (follow_goal_->is_canceling()) follow_goal_->canceled(result);
    else if (code == FollowPath::Result::NONE) follow_goal_->succeed(result);
    else follow_goal_->abort(result);
    follow_goal_.reset();
    tracker_->ClearPath();
}

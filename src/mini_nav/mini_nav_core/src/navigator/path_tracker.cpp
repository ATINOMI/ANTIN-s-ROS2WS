/**
 * @file path_tracker.cpp
 * @brief 低速路径跟踪、双安全图连续碰撞检查及进展期限。
 * @author Antinomy
 * @date 2026-10-01
 */
#include "mini_nav_core/navigator/path_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace mini_nav_core
{
    namespace
    {
        constexpr double kPi = 3.14159265358979323846;
        // 输入已是车体膨胀图；用极小正半径满足连续扫掠接口的正半径约束。
        constexpr double kCenterlineRadius = 1.0e-6;

        /**
         * @brief 按 2π 周期求余，把偏航角归一化至 [-π, π]。
         *
         * @param angle 输入弧度。
         * @return 归一化弧度；非有限输入由标准求余函数产生非有限结果。
         */
        double NormalizeAngle(double angle)
        {
            return std::remainder(angle, 2.0 * kPi);
        }

        /**
         * @brief 检查二维位姿各分量是否有限。
         *
         * @param pose 待检查的米/弧度位姿。
         * @return x、y、yaw 均有限为 true。
         */
        bool FinitePose(const localization::Pose2D & pose)
        {
            return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw);
        }

        /**
         * @brief 以二维刚体变换旋转并平移路径点。
         *
         * @param transform 源坐标系在目标坐标系中的位姿。
         * @param point 源坐标系路径点，米。
         * @return 目标坐标系路径点，米。
         */
        PathPoint TransformPoint(const localization::Pose2D & transform, const PathPoint & point)
        {
            const double c = std::cos(transform.yaw);
            const double s = std::sin(transform.yaw);
            return {transform.x + c * point.x - s * point.y,
                    transform.y + s * point.x + c * point.y};
        }

        /**
         * @brief 在 map 与 odom 两张膨胀图上检查候选速度的连续运动。
         *
         * 预测时长取 min(1 秒, 前视点行程时间) 与制动时间的较大者，
         * 因此制动要求可使预测超出前视点；每段最多 0.05 秒，并补偿圆弧到弦的偏离。
         *
         * @param command 已限幅的非负前向速度和偏航角速度。
         * @param map_pose map 系当前机器人位姿。
         * @param odom_pose odom 系当前机器人位姿。
         * @param static_map map 系膨胀安全图。
         * @param local_map odom 系膨胀安全图。
         * @param carrot_distance 到前视点的直线距离，米。
         * @param braking_distance 反应行程加制动距离，米。
         * @return 当前位置与全部预测段均安全为 true；非法预测期限或冲突为 false。
         */
        bool IsVelocitySweepClear(
            const VelocityCommand & command,
            const localization::Pose2D & map_pose,
            const localization::Pose2D & odom_pose,
            const Costmap2D & static_map,
            const Costmap2D & local_map,
            double carrot_distance, double braking_distance)
        {
            PathPoint map_previous{map_pose.x, map_pose.y};
            PathPoint odom_previous{odom_pose.x, odom_pose.y};
            if (!IsCircularSweepClear(static_map, map_previous, map_previous, kCenterlineRadius) ||
                !IsCircularSweepClear(local_map, odom_previous, odom_previous, kCenterlineRadius)) {
                return false;
            }
            // 两张图已按外接圆膨胀，原地转向只需验证当前中心，不能被远处前视点挡住。
            if (command.linear_x == 0.0) return true;

            // 参考官方 RPP：检查候选速度的短期运动，范围不超过前视目标距离。
            constexpr double prediction_time = 1.0;
            constexpr double max_time_step = 0.05;
            const double horizon = std::max(std::min(prediction_time, carrot_distance / command.linear_x),
                braking_distance / command.linear_x);
            if (!std::isfinite(horizon) || horizon > 60.0) return false;
            if (horizon <= 0.0) return true;
            const int steps = static_cast<int>(std::ceil(horizon / max_time_step));
            const double dt = horizon / steps;
            const double half_turn = command.angular_z * dt * 0.5;
            const double advance = command.linear_x * dt *
                (half_turn == 0.0 ? 1.0 : std::sin(half_turn) / half_turn);
            // 连续圆弧到其弦的偏离上界，计入扫掠半径以免漏掉弦外的碰撞。
            const double sweep_radius = kCenterlineRadius +
                std::abs(command.linear_x * command.angular_z) * dt * dt / 8.0;
            PathPoint relative{0.0, 0.0};
            double relative_yaw = 0.0;
            for (int index = 0; index < steps; ++index) {
                relative.x += advance * std::cos(relative_yaw + half_turn);
                relative.y += advance * std::sin(relative_yaw + half_turn);
                relative_yaw += command.angular_z * dt;
                const PathPoint map_next = TransformPoint(map_pose, relative);
                const PathPoint odom_next = TransformPoint(odom_pose, relative);
                if (!IsCircularSweepClear(static_map, map_previous, map_next, sweep_radius) ||
                    !IsCircularSweepClear(local_map, odom_previous, odom_next, sweep_radius)) {
                    return false;
                }
                map_previous = map_next;
                odom_previous = odom_next;
            }
            return true;
        }
    }

    /**
     * @brief 创建 ROS 无关的路径跟踪器并校验速度、容差及时间参数。
     *
     * @param parameters 跟踪参数；距离为米、角度为弧度、时间为秒。
     * @throws std::invalid_argument 参数非有限、正值限制不满足，或加速度/反应时间为负。
     */
    PathTracker::PathTracker(PathTrackerParameters parameters) : parameters_(parameters)
    {
        const auto positive = [](double value) { return std::isfinite(value) && value > 0.0; };
        if (!positive(parameters_.max_linear_speed) ||
            !positive(parameters_.max_angular_speed) ||
            !positive(parameters_.lookahead_distance) ||
            !positive(parameters_.goal_position_tolerance) ||
            !std::isfinite(parameters_.goal_position_hysteresis) ||
            parameters_.goal_position_hysteresis < 0.0 ||
            parameters_.goal_position_hysteresis >= parameters_.goal_position_tolerance ||
            !positive(parameters_.goal_yaw_tolerance) ||
            !positive(parameters_.rotate_in_place_angle) ||
            !positive(parameters_.max_path_deviation) ||
            !positive(parameters_.progress_distance) ||
            !positive(parameters_.progress_yaw) ||
            !positive(parameters_.progress_timeout) ||
            !std::isfinite(parameters_.max_linear_acceleration) || parameters_.max_linear_acceleration < 0.0 ||
            !positive(parameters_.max_linear_deceleration) || !positive(parameters_.max_angular_acceleration) ||
            !std::isfinite(parameters_.command_reaction_time) || parameters_.command_reaction_time < 0.0) {
            throw std::invalid_argument("Path tracker parameters must be positive and finite");
        }
    }

    /**
     * @brief 设置 map 系路径与终点朝向；空路径等价于清除。
     *
     * 点位置与朝向在 1e-6 容差内相同的重复路径不重置进展计时。
     * 新路径解除到达/超时终态并清空速度历史；朝向归一化至 [-π, π]。
     *
     * @param path 连续路径点，单位为米。
     * @param goal_yaw 末点目标偏航，单位为弧度。
     * @throws std::invalid_argument 非空路径含非有限位置或目标偏航。
     */
    void PathTracker::SetPath(const std::vector<PathPoint> & path, double goal_yaw)
    {
        if (path.empty()) {
            ClearPath();
            return;
        }
        if (!std::isfinite(goal_yaw)) {
            throw std::invalid_argument("Path goal yaw must be finite");
        }
        for (const auto & point : path) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
                throw std::invalid_argument("Path points must be finite");
            }
        }
        bool same = path.size() == path_.size() &&
            std::abs(NormalizeAngle(goal_yaw - goal_yaw_)) <= 1.0e-6;
        if (same) {
            for (std::size_t index = 0; index < path.size(); ++index) {
                if (std::hypot(path[index].x - path_[index].x,
                               path[index].y - path_[index].y) > 1.0e-6) {
                    same = false;
                    break;
                }
            }
        }
        if (same) {
            return;
        }
        path_ = path;
        goal_yaw_ = NormalizeAngle(goal_yaw);
        aligning_goal_yaw_ = false;
        terminal_status_ = TrackingStatus::kTracking;
        PauseProgress();
    }

    /**
     * @brief 移除路径，恢复无路径状态，并清除进展和候选速度历史。
     */
    void PathTracker::ClearPath()
    {
        path_.clear();
        aligning_goal_yaw_ = false;
        terminal_status_ = TrackingStatus::kNoPath;
        PauseProgress();
    }

    /**
     * @brief 清除进展计时与速度历史，保留路径及已有终态。
     *
     * 感知或 TF 失效期间调用，避免把被动停车累计为跟踪器的无进展超时。
     * 任务节点另有总期限，暂停此计时不会延长整项导航任务。
     */
    void PathTracker::PauseProgress()
    {
        progress_started_ = false;
        have_command_ = false;
        previous_command_ = {};
    }

    /**
     * @brief 查询是否保存了路径点。
     * @return 路径非空时为 true；不代表输入新鲜、运动安全或仍处于跟踪状态。
     */
    bool PathTracker::HasPath() const
    {
        return !path_.empty();
    }

    /**
     * @brief 从机器人到折线的最近投影处沿路径弧长寻找前视点。
     *
     * 调用前路径必须非空；单点路径直接返回该点。
     *
     * @param robot map 系机器人中心，单位为米。
     * @param distance_to_path 输出到最近线段的欧氏距离，单位为米。
     * @return 前视点；剩余路径不足前视距离时为末点。
     */
    PathPoint PathTracker::Lookahead(const PathPoint & robot, double & distance_to_path) const
    {
        if (path_.size() == 1) {
            distance_to_path = std::hypot(robot.x - path_.front().x, robot.y - path_.front().y);
            return path_.front();
        }
        double nearest_squared = std::numeric_limits<double>::infinity();
        std::size_t nearest_segment = 0;
        double nearest_fraction = 0.0;
        for (std::size_t index = 0; index + 1 < path_.size(); ++index) {
            const double dx = path_[index + 1].x - path_[index].x;
            const double dy = path_[index + 1].y - path_[index].y;
            const double length_squared = dx * dx + dy * dy;
            const double fraction = length_squared == 0.0 ? 0.0 :
                std::clamp(((robot.x - path_[index].x) * dx +
                            (robot.y - path_[index].y) * dy) / length_squared, 0.0, 1.0);
            const double px = path_[index].x + fraction * dx;
            const double py = path_[index].y + fraction * dy;
            const double squared = std::pow(robot.x - px, 2) + std::pow(robot.y - py, 2);
            if (squared < nearest_squared) {
                nearest_squared = squared;
                nearest_segment = index;
                nearest_fraction = fraction;
            }
        }
        distance_to_path = std::sqrt(nearest_squared);
        /* 前视距离沿折线弧长累计，不是绕机器人画圆；这样折弯后的目标仍保持路径顺序。 */
        double remaining = parameters_.lookahead_distance;
        for (std::size_t index = nearest_segment; index + 1 < path_.size(); ++index) {
            const double dx = path_[index + 1].x - path_[index].x;
            const double dy = path_[index + 1].y - path_[index].y;
            const double length = std::hypot(dx, dy);
            const double start_fraction = index == nearest_segment ? nearest_fraction : 0.0;
            const double available = (1.0 - start_fraction) * length;
            if (remaining <= available && length > 0.0) {
                const double fraction = start_fraction + remaining / length;
                return {path_[index].x + fraction * dx,
                        path_[index].y + fraction * dy};
            }
            remaining -= available;
        }
        return path_.back();
    }

    /**
     * @brief 用 odom 位移或转角判断候选运动是否持续取得进展。
     *
     * 零速命令暂停计时；平移或旋转任一达到阈值即更新进展基准。
     *
     * @param command 已完成碰撞检查的候选速度。
     * @param odom_pose odom 系当前机器人位姿。
     * @param steady_seconds 单调时钟秒数，须与前次调用使用同一时基。
     * @return 原命令；无进展超时或时间倒退时返回零速并锁存超时终态。
     */
    VelocityCommand PathTracker::CheckProgress(
        const VelocityCommand & command,
        const localization::Pose2D & odom_pose,
        double steady_seconds)
    {
        if (command.linear_x == 0.0 && command.angular_z == 0.0) {
            PauseProgress();
            return command;
        }
        if (!progress_started_) {
            progress_started_ = true;
            progress_pose_ = odom_pose;
            progress_time_ = steady_seconds;
            return command;
        }
        if (std::hypot(odom_pose.x - progress_pose_.x, odom_pose.y - progress_pose_.y) >=
                parameters_.progress_distance ||
            std::abs(NormalizeAngle(odom_pose.yaw - progress_pose_.yaw)) >=
                parameters_.progress_yaw) {
            progress_pose_ = odom_pose;
            progress_time_ = steady_seconds;
            return command;
        }
        if (steady_seconds < progress_time_ ||
            steady_seconds - progress_time_ > parameters_.progress_timeout) {
            terminal_status_ = TrackingStatus::kProgressTimeout;
            return {0.0, 0.0, terminal_status_};
        }
        return command;
    }

    /**
     * @brief 计算一次限速、进展检查与连续碰撞检查后的跟踪命令。
     *
     * 位置满足容差后只校正末点朝向；同时满足两种容差才锁存到达。
     * 先限制实际候选速度再检查其扫掠；安全停车直接置零，不受减速度限制。
     *
     * @param map_pose map 系机器人位姿，米/弧度。
     * @param odom_pose odom 系机器人位姿；用于局部图碰撞及进展判断。
     * @param odom_from_map map 到 odom 的二维变换；当前实现仅检查其有限性。
     * @param static_safety_map map 系已按车体外接圆膨胀的全局安全图。
     * @param local_safety_map odom 系已膨胀的局部安全图；未知格也禁止进入。
     * @param steady_seconds 单调时钟秒数，用于进展期限与加速度限制。
     * @return 线速度 m/s、角速度 rad/s 及状态；失效、碰撞、到达或超时返回零速。
     */
    VelocityCommand PathTracker::Step(
        const localization::Pose2D & map_pose,
        const localization::Pose2D & odom_pose,
        const localization::Pose2D & odom_from_map,
        const Costmap2D & static_safety_map,
        const Costmap2D & local_safety_map,
        double steady_seconds)
    {
        if (!HasPath()) {
            return {0.0, 0.0, TrackingStatus::kNoPath};
        }
        if (terminal_status_ == TrackingStatus::kGoalReached ||
            terminal_status_ == TrackingStatus::kProgressTimeout) {
            return {0.0, 0.0, terminal_status_};
        }
        if (!FinitePose(map_pose) || !FinitePose(odom_pose) ||
            !FinitePose(odom_from_map) || !std::isfinite(steady_seconds)) {
            PauseProgress();
            return {0.0, 0.0, TrackingStatus::kInvalidPose};
        }

        const PathPoint map_robot{map_pose.x, map_pose.y};
        const PathPoint goal = path_.back();
        const double goal_distance = std::hypot(goal.x - map_pose.x, goal.y - map_pose.y);
        const double goal_yaw_error = NormalizeAngle(goal_yaw_ - map_pose.yaw);
        if (goal_distance <= parameters_.goal_position_tolerance &&
            std::abs(goal_yaw_error) <= parameters_.goal_yaw_tolerance) {
            terminal_status_ = TrackingStatus::kGoalReached;
            PauseProgress();
            return {0.0, 0.0, terminal_status_};
        }

        // 内圈进入转向、外圈退出，避免定位噪声让终点航向与路径航向反复切换。
        if (goal_distance > parameters_.goal_position_tolerance) {
            aligning_goal_yaw_ = false;
        } else if (goal_distance <= parameters_.goal_position_tolerance -
                parameters_.goal_position_hysteresis) {
            aligning_goal_yaw_ = true;
        }
        double distance_to_path = 0.0;
        const PathPoint target = aligning_goal_yaw_ ?
            /**
             * @brief 从机器人到折线的最近投影处沿路径弧长寻找前视点。
             *
             * 调用前路径必须非空；单点路径直接返回该点。
             *
             * @param robot map 系机器人中心，单位为米。
             * @param distance_to_path 输出到最近线段的欧氏距离，单位为米。
             * @return 前视点；剩余路径不足前视距离时为末点。
             */
            map_robot : Lookahead(map_robot, distance_to_path);
        if (distance_to_path > parameters_.max_path_deviation) {
            PauseProgress();
            return {0.0, 0.0, TrackingStatus::kOffPath};
        }
        VelocityCommand command{0.0, 0.0, TrackingStatus::kTracking};
        if (aligning_goal_yaw_) {
            command.angular_z = std::clamp(1.5 * goal_yaw_error,
                -parameters_.max_angular_speed, parameters_.max_angular_speed);
        } else {
            const double heading = std::atan2(target.y - map_pose.y, target.x - map_pose.x);
            const double heading_error = NormalizeAngle(heading - map_pose.yaw);
            command.angular_z = std::clamp(1.5 * heading_error,
                -parameters_.max_angular_speed, parameters_.max_angular_speed);
            command.linear_x = std::abs(heading_error) >= parameters_.rotate_in_place_angle ? 0.0 :
                std::min(parameters_.max_linear_speed, 0.5 * goal_distance) *
                std::max(0.0, std::cos(heading_error));
        }
        /* 限制候选速度之后才计算 v×反应时间+v²/(2×减速度)。若按未限幅速度检查，
         * 检查轨迹与实际输出可能不同；碰撞停车则直接置零，不等待平滑减速。 */
        double braking_distance = 0.0;
        if (parameters_.max_linear_acceleration > 0.0) {
            const double dt = have_command_ ? std::clamp(steady_seconds - command_time_, 0.0, 0.2) : 0.1;
            command.linear_x = std::clamp(command.linear_x,
                std::max(0.0, previous_command_.linear_x - parameters_.max_linear_deceleration * dt),
                previous_command_.linear_x + parameters_.max_linear_acceleration * dt);
            command.angular_z = std::clamp(command.angular_z,
                previous_command_.angular_z - parameters_.max_angular_acceleration * dt,
                previous_command_.angular_z + parameters_.max_angular_acceleration * dt);
            braking_distance = command.linear_x * parameters_.command_reaction_time +
                command.linear_x * command.linear_x / (2.0 * parameters_.max_linear_deceleration);
        }
        if (!IsVelocitySweepClear(command, map_pose, odom_pose,
                                  static_safety_map, local_safety_map,
                                  std::hypot(target.x - map_pose.x, target.y - map_pose.y), braking_distance)) {
            PauseProgress();
            return {0.0, 0.0, TrackingStatus::kCollisionRisk};
        }
        const auto checked = CheckProgress(command, odom_pose, steady_seconds);
        previous_command_ = checked; command_time_ = steady_seconds; have_command_ = true;
        return checked;
    }
}

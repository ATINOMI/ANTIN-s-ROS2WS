/**
 * @file path_tracker.hpp
 * @brief 低速路径跟踪、双原始几何连续碰撞检查及进展期限。
 * @author Antinomy
 * @date 2026-10-01
 */
#pragma once

#include <vector>

#include "mini_nav_core/nav_types/transform_2d.hpp"
#include "mini_nav_core/collision_checker/collision_checker.hpp"

namespace mini_nav_core
{
    /**
     * @brief 一次跟踪输出的状态；到达和无进展超时会锁存，其他失效可重新检查。
     */
    enum class TrackingStatus
    {
        kNoPath,
        kTracking,
        kGoalReached,
        kOffPath,
        kCollisionRisk,
        kProgressTimeout,
        kInvalidPose,
        kAvoidingObstacle
    };

    /**
     * @brief 底盘平面候选速度及状态，linear_x 为 m/s，angular_z 为 rad/s。
     */
    struct VelocityCommand
    {
        double linear_x{0.0};
        double angular_z{0.0};
        TrackingStatus status{TrackingStatus::kNoPath};
    };

    /**
     * @brief 跟踪阈值与运动限制；距离为米、角度为弧度、时间为秒。
     */
    struct PathTrackerParameters
    {
        double max_linear_speed{0.10};
        double max_angular_speed{0.40};
        double lookahead_distance{0.35};
        double goal_position_tolerance{0.12};
        // 进入终点转向前额外靠近的距离；零保持原有切换行为，不放宽到达容差。
        double goal_position_hysteresis{0.0};
        double goal_yaw_tolerance{0.15};
        double rotate_in_place_angle{0.35};
        double max_path_deviation{0.40};
        double progress_distance{0.10};
        double progress_yaw{0.30};
        double progress_timeout{10.0};
        // Zero disables slew limits for legacy algorithm callers. Nodes explicitly enable them.
        double max_linear_acceleration{0.0};
        double max_linear_deceleration{0.5};
        double max_angular_acceleration{1.0};
        double command_reaction_time{0.35};
    };

    /** 硬安全检查和评分结果；clearance 是通过扩张检查得到的额外余量下界。 */
    struct CandidateEvaluation {
        VelocityCommand command;
        bool safe{false};
        double clearance{0.0};
        double score{0.0};
        CollisionConflict conflict;
    };

    /** 一次控制周期的实际被检查候选，顺序首项为名义速度或锁存的失败速度。 */
    struct TrackingDiagnostics {
        std::vector<CandidateEvaluation> candidates;
    };

    /** ROS 无关的低速路径跟踪、原始几何碰撞检查和进展判断。 */
    class PathTracker
    {
    public:
        /**
         * @brief 创建 ROS 无关的路径跟踪器并校验速度、容差及时间参数。
         *
         * @param parameters 跟踪参数；距离为米、角度为弧度、时间为秒。
         * @throws std::invalid_argument 参数非有限、正值限制不满足，或加速度/反应时间为负。
         */
        explicit PathTracker(PathTrackerParameters parameters = {});

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
        void SetPath(const std::vector<PathPoint> & path, double goal_yaw);
        /**
         * @brief 移除路径，恢复无路径状态，并清除进展和候选速度历史。
         */
        void ClearPath();
        /**
         * @brief 清除进展计时与速度历史，保留路径及已有终态。
         *
         * 感知或 TF 失效期间调用，避免把被动停车累计为跟踪器的无进展超时。
         * 任务节点另有总期限，暂停此计时不会延长整项导航任务。
         */
        void PauseProgress();
        /**
         * @brief 查询是否保存了路径点。
         * @return 路径非空时为 true；不代表输入新鲜、运动安全或仍处于跟踪状态。
         */
        bool HasPath() const;

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
        VelocityCommand Step(
            const nav_types::Pose2D & map_pose,
            const nav_types::Pose2D & odom_pose,
            const nav_types::Pose2D & odom_from_map,
            const Costmap2D & static_safety_map,
            const Costmap2D & local_safety_map,
            double steady_seconds);

        /** 原始静态格和连续动态端点的生产入口，所有输出均按完整圆盘检查。 */
        VelocityCommand Step(
            const nav_types::Pose2D & map_pose,
            const nav_types::Pose2D & odom_pose,
            const nav_types::Pose2D & odom_from_map,
            const CollisionGeometry & static_geometry,
            const CollisionGeometry & local_geometry,
            double steady_seconds, TrackingDiagnostics * diagnostics = nullptr);

    private:
        /**
         * @brief 从机器人到折线的最近投影处沿路径弧长寻找前视点。
         *
         * 调用前路径必须非空；单点路径直接返回该点。
         *
         * @param robot map 系机器人中心，单位为米。
         * @param distance_to_path 输出到最近线段的欧氏距离，单位为米。
         * @return 前视点；剩余路径不足前视距离时为末点。
         */
        PathPoint Lookahead(const PathPoint & robot, double & distance_to_path, double lookahead_distance, std::vector<PathPoint> * prefix = nullptr) const;
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
        VelocityCommand CheckProgress(
            const VelocityCommand & command,
            const nav_types::Pose2D & odom_pose,
            double steady_seconds);

        PathTrackerParameters parameters_;
        std::vector<PathPoint> path_;
        double goal_yaw_{0.0};
        bool aligning_goal_yaw_{false};
        TrackingStatus terminal_status_{TrackingStatus::kNoPath};
        bool progress_started_{false};
        nav_types::Pose2D progress_pose_;
        double progress_time_{0.0};
        double command_time_{0.0};
        bool have_command_{false};
        VelocityCommand previous_command_;
        bool blocked_{false};
        VelocityCommand blocked_command_;
        double blocked_time_{0.0};
        double recovery_started_{-1.0};
    };
}

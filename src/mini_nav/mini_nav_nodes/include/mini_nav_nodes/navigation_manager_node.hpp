/**
 * @file navigation_manager_node.hpp
 * @brief 单目标导航任务、重规划、期限和速度许可管理。
 * @author Antinomy
 * @date 2026-10-01
 */
#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace mini_nav_nodes
{
// Owns task identity and deadlines; planner/controller callbacks must match generation_.
/**
 * @brief 持有当前任务身份与不可被重规划重置的期限，统筹规划和跟踪。
 */
class NavigationManagerNode : public rclcpp::Node
{
public:
    /**
     * @brief 建立单目标 NavigateToPose 状态机、子 Action 客户端与速度许可心跳。
     *
     * 使用稳态时钟维护任务、受阻和服务响应期限；新目标抢占旧任务。
     * @throws std::invalid_argument 超时或重规划间隔不是有限正数。
     */
    NavigationManagerNode();
private:
    using Navigate = nav2_msgs::action::NavigateToPose;
    using Plan = nav2_msgs::action::ComputePathToPose;
    using Follow = nav2_msgs::action::FollowPath;
    using Task = rclcpp_action::ServerGoalHandle<Navigate>;
    using Clock = std::chrono::steady_clock;
    /**
     * @brief 接管已接受的目标，终止旧目标并初始化本次任务期限。
     *
     * 任务代数递增用于拒绝旧异步回调；开始规划前先撤销运动许可。
     *
     * @param handle 已被 Action 服务接受的目标句柄。
     */
    void begin(const std::shared_ptr<Task> & handle);
    /**
     * @brief 撤销运动许可、取消子目标并结束当前导航任务。
     *
     * 递增 generation_ 使延迟回调失效；终止后必须提交新目标才能继续。
     *
     * @param reason 结果消息及对外状态码。
     * @param success true 请求成功结束；取消中的句柄优先按取消结束。
     */
    void finish(const std::string & reason, bool success = false);
    /**
     * @brief 撤销速度许可并异步取消已获得句柄的规划与跟踪目标。
     *
     * 取消请求不会同步等待服务器；清空本地句柄和请求状态。
     */
    void stopChildren();
    /**
     * @brief 向规划服务器提交当前目标，并把有效结果转交 FollowPath。
     *
     * 规划期间撤销许可；只有当前任务代数匹配的回调可以推进状态。
     * 跟踪结果还核对 goal_id，避免同一任务旧重规划结果结束新的跟踪。
     */
    void plan();
    /**
     * @brief 维护任务心跳、反馈、期限和受阻时的限频重规划。
     *
     * 定位/感知或控制器断流立即撤销许可；持续受阻及整体任务期限不会被重规划重置。
     * 反馈距离是到实际规划末点的直线距离，预计剩余时间只是按 0.15 m/s 换算。
     */
    void tick();
    /**
     * @brief 保存状态码并发布可靠、保留最新值的导航状态。
     *
     * @param value 机器可读状态码，供 RViz 与外部监控解释。
     */
    void status(const std::string & value);
    /**
     * @brief 更新并发布任务运动许可。
     *
     * 许可必须定期刷新；单条 true 消息不能永久授权运动。
     *
     * @param enabled true 许可看门狗转发新鲜命令；false 请求停车。
     */
    void lease(bool enabled);
    /**
     * @brief 检查导航启用、地图、质量和局部感知心跳及子服务器可用性。
     * @return 所有条件满足且两种心跳在 800 ms 内时为 true；不替代跟踪器的 TF/碰撞检查。
     */
    bool ready() const;
    bool enabled_{true}, quality_valid_{false}, local_valid_{false};
    Clock::time_point quality_received_{}, local_received_{};
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr quality_subscription_, local_subscription_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
    rclcpp_action::Server<Navigate>::SharedPtr server_;
    rclcpp_action::Client<Navigate>::SharedPtr topic_client_;
    rclcpp_action::Client<Plan>::SharedPtr planner_;
    rclcpp_action::Client<Follow>::SharedPtr controller_;
    rclcpp_action::ClientGoalHandle<Plan>::SharedPtr plan_goal_;
    rclcpp_action::ClientGoalHandle<Follow>::SharedPtr follow_goal_;
    std::shared_ptr<Task> task_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr reset_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr controller_subscription_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lease_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    uint64_t generation_{0};
    bool request_pending_{false};
    bool motion_enabled_{false};
    bool can_follow_{false};
    int16_t replans_{0};
    std::string state_{"idle"};
    std::string controller_status_;
    Clock::time_point started_{}, last_plan_{}, request_started_{}, controller_received_{};
    std::optional<Clock::time_point> blocked_since_;
    geometry_msgs::msg::PoseStamped endpoint_;
    geometry_msgs::msg::PoseStamped progress_pose_;
    Clock::time_point progress_time_{};
    bool have_progress_pose_{false};
    double blocked_timeout_{20.0}, task_timeout_{180.0}, replan_interval_{1.0};
};
}

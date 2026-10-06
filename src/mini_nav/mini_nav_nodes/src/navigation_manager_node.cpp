/**
 * @file navigation_manager_node.cpp
 * @brief 单目标导航任务、重规划、期限和速度许可管理。
 * @author Antinomy
 * @date 2026-10-01
 */
#include "mini_nav_nodes/navigation_manager_node.hpp"
#include <cmath>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <limits>
#include <stdexcept>
#include "tf2/utils.h"

namespace mini_nav_nodes
{
namespace {
/**
 * @brief 声明并读取有限正数任务参数。
 *
 * @param node 目标节点。
 * @param name 参数名。
 * @param fallback 默认值。
 * @return 有限正参数值。
 * @throws std::invalid_argument 参数非法。
 */
double positive(rclcpp::Node & node, const char * name, double fallback)
{
    const auto value = node.declare_parameter<double>(name, fallback);
    if (!std::isfinite(value) || value <= 0.0) throw std::invalid_argument(name);
    return value;
}
/**
 * @brief 校验 map 系目标位置和单位四元数。
 *
 * @param pose 待接受的导航目标。
 * @return 帧为 map、位置有限且四元数模长误差不超过 1e-3 时 true。
 */
bool validGoal(const geometry_msgs::msg::PoseStamped & pose)
{
    const auto & p = pose.pose.position;
    const auto & q = pose.pose.orientation;
    const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
    return pose.header.frame_id == "map" && std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
        std::isfinite(norm) && std::abs(norm - 1.0) <= 1e-3;
}
}
/**
 * @brief 建立单目标 NavigateToPose 状态机、子 Action 客户端与速度许可心跳。
 *
 * 使用稳态时钟维护任务、受阻和服务响应期限；新目标抢占旧任务。
 * @throws std::invalid_argument 超时或重规划间隔不是有限正数。
 */
NavigationManagerNode::NavigationManagerNode() : Node("navigation_manager")
{
    enabled_ = declare_parameter<bool>("enabled", true);
    blocked_timeout_ = positive(*this, "blocked_timeout", 20.0);
    task_timeout_ = positive(*this, "task_timeout", 180.0);
    replan_interval_ = positive(*this, "replan_interval", 1.0);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);
    lease_publisher_ = create_publisher<std_msgs::msg::Bool>("/mini_nav/task_active", rclcpp::QoS(1).reliable());
    status_publisher_ = create_publisher<std_msgs::msg::String>(
        "/mini_nav/navigation_status", rclcpp::QoS(1).reliable().transient_local());
    planner_ = rclcpp_action::create_client<Plan>(this, "/compute_path_to_pose");
    controller_ = rclcpp_action::create_client<Follow>(this, "/follow_path");
    server_ = rclcpp_action::create_server<Navigate>(this, "/navigate_to_pose",
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Navigate::Goal> goal) {
            if (!validGoal(goal->pose) || !goal->behavior_tree.empty() || !ready()) {
                RCLCPP_WARN(get_logger(), "Rejecting navigation goal: invalid request or navigation not ready");
                return rclcpp_action::GoalResponse::REJECT;
            }
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }, [this](auto handle) {
            if (task_ == handle) lease(false);
            return rclcpp_action::CancelResponse::ACCEPT;
        }, [this](auto handle) { begin(handle); });
    topic_client_ = rclcpp_action::create_client<Navigate>(this, "/navigate_to_pose");
    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", rclcpp::QoS(10).reliable(), [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
            if (!validGoal(*msg)) { RCLCPP_WARN(get_logger(), "Rejected invalid RViz goal"); return; }
            Navigate::Goal goal; goal.pose = *msg;
            topic_client_->async_send_goal(goal);
        });
    reset_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", rclcpp::QoS(10), [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr) { if (task_) finish("localization_reset"); });
    controller_subscription_ = create_subscription<std_msgs::msg::String>(
        "/mini_nav/controller_status", rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::String::ConstSharedPtr msg) {
            controller_status_ = msg->data; controller_received_ = Clock::now();
        });
    quality_subscription_ = create_subscription<std_msgs::msg::Bool>(
        "/mini_nav/localization_valid", rclcpp::QoS(1).reliable(), [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
            quality_valid_ = msg->data; quality_received_ = Clock::now();
            if (!quality_valid_ && task_) lease(false);
        });
    // 定位会话改变时撤销旧任务，避免质量恢复后继续旧目标。
    epoch_subscription_ = create_subscription<std_msgs::msg::String>(
        "/mini_nav/localization_epoch", rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::String::ConstSharedPtr msg) {
            if (!localization_epoch_.empty() && localization_epoch_ != msg->data) {
                quality_valid_ = false;
                if (task_) finish("localization_epoch_changed"); else lease(false);
            }
            localization_epoch_ = msg->data;
        });
    local_subscription_ = create_subscription<std_msgs::msg::Bool>(
        "/mini_nav/local_costmap_valid", rclcpp::QoS(1).reliable(), [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
            local_valid_ = msg->data; local_received_ = Clock::now();
            if (!local_valid_ && task_) lease(false);
        });
    map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(1).reliable().transient_local(), [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
            if (map_ && task_ && (map_->data != msg->data || map_->header.frame_id != msg->header.frame_id ||
                map_->info.width != msg->info.width || map_->info.height != msg->info.height ||
                map_->info.resolution != msg->info.resolution || map_->info.origin != msg->info.origin)) finish("map_changed");
            map_ = msg;
        });
    enable_service_ = create_service<std_srvs::srv::SetBool>("/mini_nav/set_navigation_enabled",
        [this](std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
            enabled_ = request->data;
            if (!enabled_) { if (task_) finish("navigation_disabled"); else { stopChildren(); status("navigation_disabled"); } }
            else if (!task_) status("idle");
            response->success = true;
            response->message = enabled_ ? "Enabled; send a new goal after localization is ready" : "Disabled; task terminated and velocity lease revoked";
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { tick(); });
}
/**
 * @brief 更新并发布任务运动许可。
 *
 * 许可必须定期刷新；单条 true 消息不能永久授权运动。
 *
 * @param enabled true 许可看门狗转发新鲜命令；false 请求停车。
 */
void NavigationManagerNode::lease(bool enabled)
{
    motion_enabled_ = enabled;
    std_msgs::msg::Bool msg; msg.data = enabled; lease_publisher_->publish(msg);
}
/**
 * @brief 保存状态码并发布可靠、保留最新值的导航状态。
 *
 * @param value 机器可读状态码，供 RViz 与外部监控解释。
 */
void NavigationManagerNode::status(const std::string & value)
{
    state_ = value;
    std_msgs::msg::String msg; msg.data = value; status_publisher_->publish(msg);
}
/**
 * @brief 撤销速度许可并异步取消已获得句柄的规划与跟踪目标。
 *
 * 取消请求不会同步等待服务器；清空本地句柄和请求状态。
 */
void NavigationManagerNode::stopChildren()
{
    lease(false); can_follow_ = false;
    if (plan_goal_) planner_->async_cancel_goal(plan_goal_);
    if (follow_goal_) controller_->async_cancel_goal(follow_goal_);
    plan_goal_.reset(); follow_goal_.reset(); request_pending_ = false;
}
/**
 * @brief 撤销运动许可、取消子目标并结束当前导航任务。
 *
 * 递增 generation_ 使延迟回调失效；终止后必须提交新目标才能继续。
 *
 * @param reason 结果消息及对外状态码。
 * @param success true 请求成功结束；取消中的句柄优先按取消结束。
 */
void NavigationManagerNode::finish(const std::string & reason, bool success)
{
    stopChildren();
    ++generation_;
    if (task_) {
        auto result = std::make_shared<Navigate::Result>();
        result->error_msg = reason;
        result->error_code = success ? 0 : 1;
        if (task_->is_canceling()) task_->canceled(result);
        else if (success) task_->succeed(result);
        else task_->abort(result);
        task_.reset();
    }
    status(reason);
}
/**
 * @brief 接管已接受的目标，终止旧目标并初始化本次任务期限。
 *
 * 任务代数递增用于拒绝旧异步回调；开始规划前先撤销运动许可。
 *
 * @param handle 已被 Action 服务接受的目标句柄。
 */
void NavigationManagerNode::begin(const std::shared_ptr<Task> & handle)
{
    if (task_) finish("preempted");
    task_ = handle;
    ++generation_;
    started_ = Clock::now(); last_plan_ = Clock::time_point{};
    blocked_since_.reset(); have_progress_pose_ = false; replans_ = 0;
    controller_status_.clear(); endpoint_ = handle->get_goal()->pose;
    status("planning"); lease(false);
    plan();
}
/**
 * @brief 向规划服务器提交当前目标，并把有效结果转交 FollowPath。
 *
 * 规划期间撤销许可；只有当前任务代数匹配的回调可以推进状态。
 * 跟踪结果还核对 goal_id，避免同一任务旧重规划结果结束新的跟踪。
 */
void NavigationManagerNode::plan()
{
    if (!task_ || request_pending_ || !planner_->action_server_is_ready() ||
        !controller_->action_server_is_ready()) return;
    lease(false); can_follow_ = false;
    // 内部换路先解除旧句柄身份，再取消；迟到的取消结果不能结束新跟踪。
    auto previous_follow = follow_goal_;
    follow_goal_.reset();
    if (previous_follow) controller_->async_cancel_goal(previous_follow);
    /* 捕获任务代数而非只检查 task_ 非空：旧任务的延迟响应可能在新任务开始后到达。
     * 旧句柄仍要取消，但不能把其路径、状态或结果提交给新的导航任务。 */
    const auto token = generation_;
    Plan::Goal goal; goal.goal = task_->get_goal()->pose; goal.use_start = false;
    goal.planner_id = replans_ >= 2 ? "AStarDynamic" : "AStar";
    request_pending_ = true; request_started_ = last_plan_ = Clock::now();
    rclcpp_action::Client<Plan>::SendGoalOptions options;
    options.goal_response_callback = [this, token](auto handle) {
        if (token != generation_) { if (handle) planner_->async_cancel_goal(handle); return; }
        plan_goal_ = handle;
        if (!handle) { request_pending_ = false; status("planning_failed"); }
    };
    options.result_callback = [this, token](const auto & wrapped) {
        if (token != generation_ || !task_) return;
        plan_goal_.reset();
        if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || wrapped.result->path.poses.empty()) {
            request_pending_ = false;
            status(wrapped.result && wrapped.result->error_code == Plan::Result::START_OCCUPIED ?
                   "start_in_collision" : "planning_failed");
            lease(false); return;
        }
        endpoint_ = wrapped.result->path.poses.back();
        Follow::Goal follow; follow.path = wrapped.result->path;
        rclcpp_action::Client<Follow>::SendGoalOptions follow_options;
        follow_options.goal_response_callback = [this, token](auto handle) {
            if (token != generation_) { if (handle) controller_->async_cancel_goal(handle); return; }
            follow_goal_ = handle; can_follow_ = static_cast<bool>(handle); request_pending_ = false;
            if (!handle) { lease(false); status("controller_rejected"); }
            else { controller_status_.clear(); controller_received_ = Clock::now(); lease(ready()); status("tracking"); }
        };
        follow_options.result_callback = [this, token](const auto & result) {
            if (token != generation_ || !task_) return;
            // Ignore results from an older replan's preempted controller goal.
            if (!follow_goal_ || result.goal_id != follow_goal_->get_goal_id()) return;
            follow_goal_.reset(); can_follow_ = false; lease(false);
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) finish("goal_reached", true);
            else if (result.code == rclcpp_action::ResultCode::CANCELED) finish("controller_canceled");
            /**
             * @brief 保存状态码并发布可靠、保留最新值的导航状态。
             *
             * @param value 机器可读状态码，供 RViz 与外部监控解释。
             */
            else status("controller_failed");
        };
        request_started_ = Clock::now();
        controller_->async_send_goal(follow, follow_options);
    };
    planner_->async_send_goal(goal, options);
}
/**
 * @brief 维护任务心跳、反馈、期限和受阻时的限频重规划。
 *
 * 定位/感知或控制器断流立即撤销许可；持续受阻及整体任务期限不会被重规划重置。
 * 反馈距离是到实际规划末点的直线距离，预计剩余时间只是按 0.15 m/s 换算。
 */
void NavigationManagerNode::tick()
{
    const auto current = Clock::now();
    if (!task_) {
        lease(false);
        if (state_ == "idle" || state_ == "not_ready") status(ready() ? "idle" : "not_ready");
        /**
         * @brief 保存状态码并发布可靠、保留最新值的导航状态。
         *
         * @param value 机器可读状态码，供 RViz 与外部监控解释。
         */
        else status(state_);
        return;
    }
    if (task_->is_canceling()) { finish("canceled"); return; }
    const double elapsed = std::chrono::duration<double>(current - started_).count();
    if (elapsed > task_timeout_) { finish("task_timeout"); return; }
    if (request_pending_ && std::chrono::duration<double>(current - request_started_).count() > 3.0) {
        finish("server_timeout"); return;
    }
    const bool healthy = ready();
    bool blocked = !follow_goal_ || !can_follow_ || controller_status_ != "tracking" || !healthy;
    if (!healthy) { lease(false); status("waiting_for_localization_or_sensor"); }
    else if (follow_goal_ && can_follow_ &&
             (controller_status_ == "tracking" || controller_status_ == "avoiding_obstacle")) lease(true);
    if (motion_enabled_ && current - controller_received_ > std::chrono::milliseconds(800)) {
        lease(false); blocked = true; status("controller_unavailable");
    }
    try {
        auto tf = tf_buffer_->lookupTransform("map", "base_footprint", rclcpp::Time(0, 0, get_clock()->get_clock_type()));
        auto feedback = std::make_shared<Navigate::Feedback>();
        feedback->current_pose.header = tf.header;
        feedback->current_pose.pose.position.x = tf.transform.translation.x;
        feedback->current_pose.pose.position.y = tf.transform.translation.y;
        feedback->current_pose.pose.orientation = tf.transform.rotation;
        feedback->navigation_time = rclcpp::Duration::from_seconds(elapsed);
        feedback->number_of_recoveries = replans_;
        feedback->distance_remaining = std::hypot(endpoint_.pose.position.x - tf.transform.translation.x,
            endpoint_.pose.position.y - tf.transform.translation.y);
        // A straight-line lower bound, not a promised arrival time.
        feedback->estimated_time_remaining = rclcpp::Duration::from_seconds(feedback->distance_remaining / 0.15);
        task_->publish_feedback(feedback);
        const auto & pose = feedback->current_pose;
        if (!have_progress_pose_ || std::hypot(pose.pose.position.x - progress_pose_.pose.position.x,
            pose.pose.position.y - progress_pose_.pose.position.y) >= 0.10 ||
            std::abs(std::remainder(tf2::getYaw(pose.pose.orientation) - tf2::getYaw(progress_pose_.pose.orientation),
                2.0 * 3.14159265358979323846)) >= 0.30) {
            progress_pose_ = pose; progress_time_ = current; have_progress_pose_ = true;
        } else if (std::chrono::duration<double>(current - progress_time_).count() > blocked_timeout_) {
            finish("no_progress"); return;
        }
    } catch (const tf2::TransformException &) { blocked = true; lease(false); status("tf_unavailable"); }
    if (blocked) {
        if (!blocked_since_) blocked_since_ = current;
        if (std::chrono::duration<double>(current - *blocked_since_).count() > blocked_timeout_) {
            finish("blocked_timeout"); return;
        }
        if (healthy && !request_pending_ && std::chrono::duration<double>(current - last_plan_).count() >= replan_interval_) {
            lease(false); status("replanning");
            if (replans_ < std::numeric_limits<int16_t>::max()) ++replans_;
            plan();
        }
    } else { blocked_since_.reset(); status("tracking"); }
    lease(motion_enabled_); status(state_);
}
}

/**
 * @brief 检查导航启用、地图、质量和局部感知心跳及子服务器可用性。
 * @return 所有条件满足且两种心跳在 800 ms 内时为 true；不替代跟踪器的 TF/碰撞检查。
 */
bool mini_nav_nodes::NavigationManagerNode::ready() const
{
    const auto current = Clock::now();
    return enabled_ && map_ && !map_->data.empty() && quality_valid_ && local_valid_ &&
        quality_received_ != Clock::time_point{} && local_received_ != Clock::time_point{} &&
        current - quality_received_ <= std::chrono::milliseconds(800) &&
        current - local_received_ <= std::chrono::milliseconds(800) &&
        planner_->action_server_is_ready() && controller_->action_server_is_ready();
}

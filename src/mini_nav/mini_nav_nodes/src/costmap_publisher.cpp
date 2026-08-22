/* Includes ----------------------------------------------------------------*/
#include "costmap_publisher.hpp"

/* Node construction -------------------------------------------------------*/
mini_nav_nodes::CostmapPublisherNode::CostmapPublisherNode(int size_x, 
                                 int size_y, 
                                 double resolution, 
                                 double origin_x, 
                                 double origin_y, 
                                 unsigned char default_value, 
                                 unsigned int publish_period_ms)
                                : Node("costmap_publisher"),
                                  size_x_(declarePositiveIntParameter("map.size_x", size_x)),
                                  size_y_(declarePositiveIntParameter("map.size_y", size_y)),
                                  resolution_(declarePositiveDoubleParameter("map.resolution", resolution)),
                                  origin_x_(declare_parameter<double>("map.origin_x", origin_x)),
                                  origin_y_(declare_parameter<double>("map.origin_y", origin_y)),
                                  default_value_(declareByteParameter("map.default_value", default_value)),
                                  publish_period_ms_(
                                    declarePositiveIntParameter("publish_period_ms", publish_period_ms)),
                                  frame_id_(declare_parameter<std::string>("frame_id", "map"))

  {

    // mini_nav_core 保持 ROS 无关；ROS 消息的转换只在本节点中完成。    
    costmap_ = std::make_unique<mini_nav_core::Costmap2D>(size_x_, size_y_, 
                                                          resolution_,
                                                          origin_x_, origin_y_, 
                                                          default_value_);
                                                          
    // 地图属于“后加入的订阅者也应立即获得”的静态数据：
    // KeepLast(1) 只保存最新一张图，reliable 保证可靠传输，
    // transient_local 让 RViz 在节点已经发布后启动时也能收到最新地图。
    const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/mini_nav/map", map_qos);
    path_publisher_ = create_publisher<nav_msgs::msg::Path>("/mini_nav/global_path", map_qos);
    axes_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/mini_nav/map_axes", map_qos);

    initial_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", rclcpp::QoS(10),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
        initialPoseCallback(message);
      });
    goal_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", rclcpp::QoS(10),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        goalPoseCallback(message);
      });

    // 将来 costmap_ 接入传感器更新后，这个定时器无需改变。
    timer_ = create_wall_timer(
      std::chrono::milliseconds(publish_period_ms_),
      std::bind(&CostmapPublisherNode::publishMap, this));

    RCLCPP_INFO(
      get_logger(), "Publishing %u x %u map on /mini_nav/map in frame %s",
      size_x_, size_y_, frame_id_.c_str());                                                          
  }      

/* Parameter validation ----------------------------------------------------*/
unsigned int mini_nav_nodes::CostmapPublisherNode::declarePositiveIntParameter(const std::string & name, int default_value)
{
    const auto value = declare_parameter<int>(name, default_value);
    if (value <= 0) {
        throw std::invalid_argument(name + " must be greater than zero");
    }
    return static_cast<unsigned int>(value);
}

double mini_nav_nodes::CostmapPublisherNode::declarePositiveDoubleParameter(const std::string & name, double default_value)
{
    const auto value = declare_parameter<double>(name, default_value);
    if (value <= 0.0) {
        throw std::invalid_argument(name + " must be greater than zero");
    }
    return value;
}

unsigned char mini_nav_nodes::CostmapPublisherNode::declareByteParameter(const std::string & name, int default_value)
{
    const auto value = declare_parameter<int>(name, default_value);
    if (value < 0 || value > 255) {
        throw std::invalid_argument(name + " must be in [0, 255]");
    }
    return static_cast<unsigned char>(value);
}

/* Costmap access -----------------------------------------------------------*/
mini_nav_core::Costmap2D & mini_nav_nodes::CostmapPublisherNode::GetCostmap()
{
    return *costmap_;
}

/* Path planning and publication ------------------------------------------*/
bool mini_nav_nodes::CostmapPublisherNode::PlanAndPublish(
  const mini_nav_core::MapLocation & start,
  const mini_nav_core::MapLocation & goal)
{
    mini_nav_core::AStarPlanner planner;
    const std::vector<mini_nav_core::MapLocation> grid_path = planner.Plan(*costmap_, start, goal);

    if (grid_path.empty()) {
        RCLCPP_WARN(get_logger(), "A* could not find a path from (%u, %u) to (%u, %u)",
                    start.x, start.y, goal.x, goal.y);
        return false;
    }

    global_path_ = nav_msgs::msg::Path();
    global_path_.header.stamp = now();
    global_path_.header.frame_id = frame_id_;

    for (const auto & cell : grid_path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = global_path_.header;
        costmap_->MapToWorld(cell.x, cell.y, pose.pose.position.x, pose.pose.position.y);
        // 将路径抬高到地图平面上方，避免 RViz 深度测试将线遮住。
        pose.pose.position.z = 0.05;
        pose.pose.orientation.w = 1.0;
        global_path_.poses.push_back(pose);
    }

    path_publisher_->publish(global_path_);
    RCLCPP_INFO(get_logger(), "A* published %zu poses on /mini_nav/global_path", global_path_.poses.size());
    return true;
}

/* RViz interactive planning ---------------------------------------------*/
void mini_nav_nodes::CostmapPublisherNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
{
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap_->WorldToMap(message->pose.pose.position.x, message->pose.pose.position.y, mx, my)) {
        RCLCPP_WARN(get_logger(), "Selected start is outside the map");
        return;
    }

    start_cell_ = mini_nav_core::MapLocation{mx, my};
    RCLCPP_INFO(get_logger(), "RViz start set to cell (%u, %u)", mx, my);
    planIfReady();
}

void mini_nav_nodes::CostmapPublisherNode::goalPoseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr message)
{
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap_->WorldToMap(message->pose.position.x, message->pose.position.y, mx, my)) {
        RCLCPP_WARN(get_logger(), "Selected goal is outside the map");
        return;
    }

    goal_cell_ = mini_nav_core::MapLocation{mx, my};
    RCLCPP_INFO(get_logger(), "RViz goal set to cell (%u, %u)", mx, my);
    planIfReady();
}

void mini_nav_nodes::CostmapPublisherNode::planIfReady()
{
    if (!start_cell_.has_value() || !goal_cell_.has_value()) {
        RCLCPP_INFO(get_logger(), "Select the other point in RViz to start A* planning");
        return;
    }

    PlanAndPublish(*start_cell_, *goal_cell_);
}

/* Map publication ---------------------------------------------------------*/
void mini_nav_nodes::CostmapPublisherNode::publishMap()
{
    nav_msgs::msg::OccupancyGrid message;
    // header 描述消息所属坐标系及发布时间；RViz 会据此放置地图。
    message.header.stamp    = now();
    message.header.frame_id = frame_id_;
    // 对静态地图，map_load_time 表示这张地图的生成时间。
    message.info.map_load_time = message.header.stamp;
    message.info.resolution    = static_cast<float>(costmap_->GetResolution());
    message.info.width         = costmap_->GetSizeInCellsX();
    message.info.height        = costmap_->GetSizeInCellsY();

    message.info.origin.position.x    = costmap_->GetOriginX();
    message.info.origin.position.y    = costmap_->GetOriginY();
    // OccupancyGrid 的 origin 是 Pose，二维无旋转时四元数必须设为单位四元数。
    message.info.origin.orientation.w = 1.0;
    // costmap_ 的 unsigned char 代价转换为 ROS 约定的 int8 占据概率。
    message.data.resize(message.info.width * message.info.height);

    for (unsigned int my = 0; my < message.info.height; ++my) {
      for (unsigned int mx = 0; mx < message.info.width; ++mx) {
        const auto cost = costmap_->GetCost(mx, my);
        const auto index = my * message.info.width + mx;
        // 普通代价按 0~254 线性压缩至 0~100，254 及以上封顶为 100。
        message.data[index] = cost == kUnknownCost ? -1 :
          static_cast<int8_t>(std::min(100U, static_cast<unsigned int>(cost) * 100U / kLethalObstacle));
      }
    }

    // QoS 在创建 publisher 时已固定；这里只负责发送新消息。
    map_publisher_->publish(message);
    if (!global_path_.poses.empty()) {
        global_path_.header.stamp = message.header.stamp;
        for (auto & pose : global_path_.poses) {
            pose.header = global_path_.header;
        }
        path_publisher_->publish(global_path_);
    }
    publishCoordinateAxes();    
}

/* Coordinate-axis visualization -------------------------------------------*/
void mini_nav_nodes::CostmapPublisherNode::publishCoordinateAxes()
{
    visualization_msgs::msg::MarkerArray markers;
    const auto stamp = now();
    const double origin_x = costmap_->GetOriginX();
    const double origin_y = costmap_->GetOriginY();
    const double map_width = costmap_->GetSizeInCellsX() * costmap_->GetResolution();
    const double map_height = costmap_->GetSizeInCellsY() * costmap_->GetResolution();
    const double axis_length = (map_width < map_height ? map_width : map_height) * 0.20;
    const float shaft_diameter = static_cast<float>(axis_length * 0.10);
    const float head_diameter = static_cast<float>(axis_length * 0.20);
    const float label_height = static_cast<float>(axis_length / 3.0);

    auto make_arrow = [&](int id, const char * label, double end_x, double end_y,
                          float red, float green, float blue) {
        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = frame_id_;
        arrow.header.stamp = stamp;
        arrow.ns = "map_axes";
        arrow.id = id;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        // 箭头尺寸随地图尺度变化，保持与地图相同的视觉比例。
        arrow.scale.x = shaft_diameter;
        arrow.scale.y = head_diameter;
        arrow.color.r = red;
        arrow.color.g = green;
        arrow.color.b = blue;
        arrow.color.a = 1.0F;
        geometry_msgs::msg::Point start;
        start.x = origin_x;
        start.y = origin_y;
        geometry_msgs::msg::Point end;
        end.x = origin_x + end_x;
        end.y = origin_y + end_y;
        arrow.points = {start, end};
        markers.markers.push_back(arrow);

        visualization_msgs::msg::Marker text;
        text.header = arrow.header;
        text.ns = "map_axes_labels";
        text.id = id;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.pose.position = end;
        text.pose.position.z = head_diameter;
        text.pose.orientation.w = 1.0;
        text.scale.z = label_height;
        text.color = arrow.color;
        text.text = label;
        markers.markers.push_back(text);
    };

    make_arrow(0, "+X", axis_length, 0.0, 1.0F, 0.0F, 0.0F);
    make_arrow(1, "+Y", 0.0, axis_length, 0.0F, 1.0F, 0.0F);

    visualization_msgs::msg::Marker origin_label;
    origin_label.header.frame_id = frame_id_;
    origin_label.header.stamp = stamp;
    origin_label.ns = "map_axes_labels";
    origin_label.id = 2;
    origin_label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    origin_label.action = visualization_msgs::msg::Marker::ADD;
    origin_label.pose.position.x = origin_x;
    origin_label.pose.position.y = origin_y;
    origin_label.pose.position.z = head_diameter;
    origin_label.pose.orientation.w = 1.0;
    origin_label.scale.z = label_height;
    origin_label.color.r = 1.0F;
    origin_label.color.g = 1.0F;
    origin_label.color.b = 1.0F;
    origin_label.color.a = 1.0F;
    origin_label.text = "O";
    markers.markers.push_back(origin_label);

    axes_publisher_->publish(markers);
}

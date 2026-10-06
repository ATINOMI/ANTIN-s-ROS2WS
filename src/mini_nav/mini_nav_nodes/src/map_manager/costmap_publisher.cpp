/**
 * @file costmap_publisher.cpp
 * @brief 静态地图与扫描融合、自研规划 Action 和 RViz 输出。
 * @author Antinomy
 * @date 2026-10-01
 */
/* Includes ----------------------------------------------------------------*/
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "mini_nav_nodes/map_manager/cloud_validation.hpp"
#include "mini_nav_nodes/map_manager/costmap_publisher.hpp"
#include "mini_nav_nodes/map_manager/costmap_display.hpp"

#include <cctype>
#include <cmath>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include <filesystem>
#include <fstream>
#include <functional>
#include <sstream>
#include <vector>

#include "tf2/exceptions.h"

/* Node construction -------------------------------------------------------*/
/**
 * @brief 建立地图输入、规划 Action 与路径及地图可视化发布。
 *
 * map_file 非空直接加载 YAML/PGM，否则订阅地图。主导航入口关闭话题目标，
 * 由任务节点统一持有目标身份；核心算法不依赖 ROS。
 *
 * @param size_x 未收到地图前的默认 x 格数。
 * @param size_y 默认 y 格数。
 * @param resolution 默认格边长，米。
 * @param origin_x 默认原点 x，米。
 * @param origin_y 默认原点 y，米。
 * @param default_value 默认代价，0..255。
 * @param publish_period_ms 地图及已有路径重新发布的周期，毫秒。
 * @throws std::invalid_argument 参数或地图几何非法；地图文件不可读时抛出 std::runtime_error。
 */
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
                                  cost_travel_multiplier_(
                                    declare_parameter<double>("planning.cost_travel_multiplier", 2.0)),
                                  goal_tolerance_(
                                    declare_parameter<double>("planning.goal_tolerance", 0.5)),
                                  goal_position_tolerance_(
                                    declarePositiveDoubleParameter("planning.goal_position_tolerance", 0.12)),
                                  map_topic_(declare_parameter<std::string>("map_topic", "/map")),
                                  map_file_(declare_parameter<std::string>("map_file", "")),
                                  frame_id_(declare_parameter<std::string>("frame_id", "map")),
                                  base_frame_id_(declare_parameter<std::string>("base_frame_id", "base_footprint")),
                                  odom_frame_id_(declare_parameter<std::string>("odom_frame_id", "odom")),
                                  max_pose_age_(declarePositiveDoubleParameter("planning.max_pose_age", 1.0)),
                                  use_initial_pose_as_start_(
                                    declare_parameter<bool>("planning.use_initial_pose_as_start", false))

  {
    if (map_topic_.empty()) {
      throw std::invalid_argument("map_topic must not be empty");
    }
    if (frame_id_.empty()) {
      throw std::invalid_argument("frame_id must not be empty");
    }
    if (base_frame_id_.empty() || base_frame_id_ == frame_id_) {
      throw std::invalid_argument("base_frame_id must be nonempty and differ from frame_id");
    }
    if (odom_frame_id_.empty() || odom_frame_id_ == frame_id_ || odom_frame_id_ == base_frame_id_) {
      throw std::invalid_argument("odom_frame_id must differ from frame_id and base_frame_id");
    }
    inflation_parameters_.robot_radius =
      declarePositiveDoubleParameter("planning.robot_radius", 0.24);
    inflation_parameters_.safety_margin =
      declare_parameter<double>("planning.safety_margin", 0.02);
    inflation_parameters_.inscribed_radius =
      declare_parameter<double>("planning.inscribed_radius", 0.22549849949589046);
    inflation_parameters_.inflation_radius =
      declarePositiveDoubleParameter("planning.inflation_radius", 0.70);
    inflation_parameters_.cost_scaling_factor =
      declarePositiveDoubleParameter("planning.cost_scaling_factor", 3.0);
    inflation_parameters_.inflate_around_unknown =
      declare_parameter<bool>("planning.inflate_around_unknown", false);
    if (!std::isfinite(inflation_parameters_.safety_margin) ||
        inflation_parameters_.safety_margin < 0.0 ||
        !std::isfinite(inflation_parameters_.inscribed_radius) ||
        inflation_parameters_.inscribed_radius < 0.0 ||
        inflation_parameters_.inflation_radius < inflation_parameters_.inscribed_radius ||
        inflation_parameters_.inflation_radius <
          inflation_parameters_.robot_radius + inflation_parameters_.safety_margin ||
        !std::isfinite(cost_travel_multiplier_) || cost_travel_multiplier_ < 0.0) {
      throw std::invalid_argument("Invalid planning safety margin, inflation radius, or traversal multiplier");
    }

    if (!std::isfinite(goal_tolerance_) || goal_tolerance_ < 0.0) {
      throw std::invalid_argument("planning.goal_tolerance must be finite and nonnegative");
    }

    // mini_nav_core 保持 ROS 无关；ROS 消息的转换只在本节点中完成。    
    costmap_ = std::make_unique<mini_nav_core::Costmap2D>(size_x_, size_y_, 
                                                          resolution_,
                                                          origin_x_, origin_y_, 
                                                          default_value_);
    // TF 监听器使用独立线程接收变换，目标回调中短暂等待 TF 不会阻塞其更新。
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);
                                                          
    // 地图属于“后加入的订阅者也应立即获得”的静态数据：
    // KeepLast(1) 只保存最新一张图，reliable 保证可靠传输，
    // transient_local 让 RViz 在节点已经发布后启动时也能收到最新地图。
    // map_qos是一个rclcpp::QoS对象，配置了发布/订阅的质量服务参数。
    const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    // 如果指定了 map_file，则不订阅 /map；否则订阅 /map,
    if (map_file_.empty()) {
        // 订阅 /map，接收 map_server 发布的 OccupancyGrid 消息。
        map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, map_qos,
        // 使用 std::bind 绑定成员函数 mapCallback用于处理接收到的 OccupancyGrid 消息。
        std::bind(&CostmapPublisherNode::mapCallback, this, std::placeholders::_1));
    }
    // 发布 /mini_nav/map，供 RViz 显示；发布 /mini_nav/global_path，供 RViz 显示 A* 路径。
    map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/mini_nav/map", map_qos);
    planning_costmap_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/mini_nav/planning_costmap", map_qos);
    safety_costmap_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/mini_nav/planning_safety_costmap", map_qos);
    collision_publisher_ = create_publisher<msg::CollisionMap>("/mini_nav/static_collision_map", map_qos);
    observation_uncertainty_ = declarePositiveDoubleParameter("planning.observation_uncertainty", 0.03);
    localization_uncertainty_ = declare_parameter<double>("planning.localization_uncertainty", 0.03);
    if (!std::isfinite(localization_uncertainty_) || localization_uncertainty_ < 0.0)
        throw std::invalid_argument("planning.localization_uncertainty must be finite and nonnegative");
    dynamic_policy_ = declare_parameter<std::string>("planning.dynamic_policy", "immediate");
    if (dynamic_policy_ != "immediate" && dynamic_policy_ != "static_then_stable")
        throw std::invalid_argument("Unknown dynamic obstacle policy");
    raw_path_publisher_ = create_publisher<nav_msgs::msg::Path>("/mini_nav/raw_path", map_qos);
    path_publisher_ = create_publisher<nav_msgs::msg::Path>("/mini_nav/global_path", map_qos);
    axes_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/mini_nav/map_axes", map_qos);

    // 定位入口中 /initialpose 使旧路径失效；独立演示可显式启用手选起点。
    initial_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", rclcpp::QoS(10),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
        initialPoseCallback(message);
      });
    // 订阅 /goal_pose，监听目标点
    if (declare_parameter<bool>("enable_topic_goals", true)) {
    goal_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", rclcpp::QoS(10),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        goalPoseCallback(message);
      });
    }
    epoch_subscription_ = create_subscription<std_msgs::msg::String>(
        "/mini_nav/localization_epoch", rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::String::ConstSharedPtr msg) {
            if (!localization_epoch_.empty() && localization_epoch_ != msg->data) {
                clearPath(); waitForNewLocalizationTf();
                latest_cloud_.reset();
                stable_observations_.clear();
                stable_scan_stamp_ = 0.0;
            }
            localization_epoch_ = msg->data;
        });
    const auto cloud_topic = declare_parameter<std::string>("collision_cloud_topic", "");
    require_cloud_ = !cloud_topic.empty();
    if (require_cloud_) {
        cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic, rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
                latest_cloud_ = msg; cloud_received_ = std::chrono::steady_clock::now();
            });
    }
    fuse_local_obstacles_ = declare_parameter<bool>("fuse_local_obstacles", false);
    obstacle_max_range_ = declarePositiveDoubleParameter("planning.obstacle_max_range", 2.5);
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      declare_parameter<std::string>("scan_topic", "/scan"), rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr message) {
        latest_scan_ = message; scan_received_ = std::chrono::steady_clock::now();
      });
    local_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/mini_nav/local_costmap", rclcpp::QoS(1).reliable(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {
        local_obstacles_ = message;
        local_received_ = std::chrono::steady_clock::now();
      });
    plan_server_ = rclcpp_action::create_server<ComputePath>(this, "/compute_path_to_pose",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const ComputePath::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<rclcpp_action::ServerGoalHandle<ComputePath>>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      }, [this](auto handle) { computePath(handle); });

    // 将来 costmap_ 接入传感器更新后，这个定时器无需改变。
    timer_ = create_wall_timer(
      std::chrono::milliseconds(publish_period_ms_),
      std::bind(&CostmapPublisherNode::publishMap, this));

    // 如果指定了 map_file，则从文件加载地图；
    if (!map_file_.empty()) 
    {
      loadMapFromYaml(map_file_);
      RCLCPP_INFO(
        get_logger(), "Publishing external map from %s on /mini_nav/map",
        map_file_.c_str());
    } 
    else 
    {
      RCLCPP_INFO(
        get_logger(), "Waiting for OccupancyGrid on %s in frame %s",
        map_topic_.c_str(), frame_id_.c_str());
    }   
  }      

/* Parameter validation ----------------------------------------------------*/
/**
 * @brief 声明一个必须大于零的整数参数。
 *
 * @param name ROS 参数名。
 * @param default_value 参数未覆盖时采用的默认值。
 * @return 校验后的 unsigned int 参数值。
 * @throws std::invalid_argument 参数不大于零。
 */
unsigned int mini_nav_nodes::CostmapPublisherNode::declarePositiveIntParameter(const std::string & name, int default_value)
{
    const auto value = declare_parameter<int>(name, default_value);
    if (value <= 0) {
        throw std::invalid_argument(name + " must be greater than zero");
    }
    return static_cast<unsigned int>(value);
}

/**
 * @brief 声明一个有限正浮点参数。
 *
 * @param name ROS 参数名。
 * @param default_value 参数未覆盖时采用的默认值。
 * @return 校验后的浮点参数值。
 * @throws std::invalid_argument 参数非有限或不大于零。
 */
double mini_nav_nodes::CostmapPublisherNode::declarePositiveDoubleParameter(const std::string & name, double default_value)
{
    const auto value = declare_parameter<double>(name, default_value);
    if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument(name + " must be finite and greater than zero");
    }
    return value;
}

/**
 * @brief 声明一个范围为 0..255 的代价参数。
 *
 * @param name ROS 参数名。
 * @param default_value 参数未覆盖时采用的默认值。
 * @return 转换为 unsigned char 的值。
 * @throws std::invalid_argument 参数超出字节范围。
 */
unsigned char mini_nav_nodes::CostmapPublisherNode::declareByteParameter(const std::string & name, int default_value)
{
    const auto value = declare_parameter<int>(name, default_value);
    if (value < 0 || value > 255) {
        throw std::invalid_argument(name + " must be in [0, 255]");
    }
    return static_cast<unsigned char>(value);
}

/* Map input ----------------------------------------------------------------*/

/**
 * @brief 把地图话题输入交给统一校验和转换入口。
 *
 * @param message map_server 占据图。
 */
void mini_nav_nodes::CostmapPublisherNode::mapCallback(
  const nav_msgs::msg::OccupancyGrid::ConstSharedPtr message)
{
    loadMapMessage(*message);// 调用 loadMapMessage 加载地图
}

/**
 * @brief 校验地图帧、尺寸和原点旋转，构建原始图与膨胀图后替换缓存。
 *
 * 旧路径及演示选点随地图替换清除；TF 起点模式等待新的定位变换。
 * 转换失败记录日志，不把未完成构建的地图提交给规划器。
 *
 * @param message 占据图：0 空闲、负值未知、正值障碍。
 */
void mini_nav_nodes::CostmapPublisherNode::loadMapMessage(
  const nav_msgs::msg::OccupancyGrid & message)
{
    // 检查地图的 frame_id 是否与期望的 frame_id 一致。
    if (message.header.frame_id != frame_id_) {
        RCLCPP_WARN(
          get_logger(), "Ignoring map in frame '%s'; expected '%s'",
          message.header.frame_id.c_str(), frame_id_.c_str());
        return;
    }

    // 检查地图的尺寸和分辨率是否有效。
    const auto width = message.info.width;
    const auto height = message.info.height;
    const auto expected_cells = static_cast<std::size_t>(width) * height;
    if (width == 0 || height == 0 || message.info.resolution <= 0.0F ||
        !std::isfinite(message.info.resolution) || message.data.size() != expected_cells) {
        RCLCPP_ERROR(
          get_logger(), "Ignoring invalid map geometry or data size: %u x %u, resolution %.3f, data %zu",
          width, height, message.info.resolution, message.data.size());
        return;
    }

    // 检查地图原点是否旋转（Costmap2D 只支持 yaw=0）。
    const auto & orientation = message.info.origin.orientation;
    // 计算 yaw 角，检查是否旋转。计算方法是使用四元数转换为欧拉角，如果 yaw 不为 0，则忽略该地图，因为 Costmap2D 只支持 yaw=0。
    const double yaw = std::atan2(
      2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
      1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
    if (!std::isfinite(yaw) || std::abs(yaw) > 1.0e-6) {
        RCLCPP_ERROR(
          get_logger(), "Ignoring rotated map origin; Costmap2D currently supports yaw=0 only");
        return;
    }

    try {
        // 先构造完整的新地图，再替换旧地图，避免规划过程中看到半张地图。
        auto new_costmap = std::make_unique<mini_nav_core::Costmap2D>(
          width, height, message.info.resolution,
          message.info.origin.position.x, message.info.origin.position.y,
          kUnknownCost);

        // 将 OccupancyGrid 数据转换为 Costmap2D 的成本值。e.info.origin.orientation;
    // 计算 yaw 角，检查是否旋转。计
        for (unsigned int my = 0; my < height; ++my) 
        {
            for (unsigned int mx = 0; mx < width; ++mx) 
            {
                // 将 OccupancyGrid 数据转换为 Costmap2D 的成本值。
                const auto index = static_cast<std::size_t>(my) * width + mx;
                const int8_t occupancy = message.data[index];
                // 假如 occupancy 为 0，则 cost 为 0；如果 occupancy 为负数，则 cost 为 kUnknownCost；否则 cost 为 kLethalObstacle。
                const unsigned char cost = occupancy == 0 ? 0 :
                  (occupancy < 0 ? kUnknownCost : kLethalObstacle);
                // 将成本值设置到 costmap_ 中。
                new_costmap->SetCost(mx, my, cost);
            }
        }

        // 在提交新地图前完成膨胀，确保 A* 总能使用与原始地图对应的规划图。
        auto new_planning_costmap = std::make_unique<mini_nav_core::Costmap2D>(
          mini_nav_core::InflateCostmap(*new_costmap, inflation_parameters_));
        auto safety_parameters = inflation_parameters_;
        safety_parameters.inscribed_radius = 0.0;
        auto new_safety_costmap = std::make_unique<mini_nav_core::Costmap2D>(
          mini_nav_core::InflateCostmap(*new_costmap, safety_parameters));

        // 替换旧地图。std::move()是 C++11 引入的右值引用转换，将 new_costmap 的所有权转移给 costmap_。
        costmap_ = std::move(new_costmap);
        planning_costmap_ = std::move(new_planning_costmap);
        safety_costmap_ = std::move(new_safety_costmap);
        stable_observations_.clear();
        stable_scan_stamp_ = 0.0;
        actual_start_.reset();
        map_received_ = true;
        // 地图变化后，旧路径的碰撞判断不再可信。
        demo_start_cell_.reset();
        demo_goal_cell_.reset();
        demo_goal_orientation_.reset();
        clearPath();
        if (!use_initial_pose_as_start_) {
            waitForNewLocalizationTf();
        }

        // 发布地图,路径,坐标轴
        publishMap(); // 发布地图
        RCLCPP_INFO(
          get_logger(), "Loaded %u x %u map at %.3f m/cell with origin (%.3f, %.3f)",
          width, height, message.info.resolution,
          message.info.origin.position.x, message.info.origin.position.y); 
    } 
    // 捕获异常并记录错误信息。
    catch (const std::exception & exception) 
    {
        RCLCPP_ERROR(get_logger(), "Failed to load map into Costmap2D: %s", exception.what());
    }
}

/**
 * @brief 解析 trinary 地图 YAML 与 P2/P5 PGM，并转为占据图。
 *
 * 图像行方向翻转为地图 y 递增方向；仅支持无原点旋转的 trinary 模式。
 *
 * @param yaml_file 地图元数据路径；相对图像路径以 YAML 所在目录解析。
 * @throws std::runtime_error 文件不可读、元数据或图像格式不支持；数字解析可能抛出标准转换异常。
 */
void mini_nav_nodes::CostmapPublisherNode::loadMapFromYaml(const std::string & yaml_file)
{
    // 定义 lambda 函数 trim 和 unquote，用于去除字符串首尾空白和引号。
    const auto trim = [](std::string value) 
    {
        // 去除字符串首尾空白。
        const auto first = value.find_first_not_of(" \t\r\n");

        // 如果字符串为空，则返回空字符串。
        if (first == std::string::npos) return std::string();
    
        // 去除字符串末尾空白。
        const auto last = value.find_last_not_of(" \t\r\n");

        // 返回去除首尾空白后的字符串。
        return value.substr(first, last - first + 1);
    };
    // 去除字符串首尾引号。
    const auto unquote = [&trim](std::string value) 
    {
        // 先用trim函数去除字符串首尾空白。
        value = trim(value);
        // 如果字符串以引号开头和结尾，则去除引号。
        if (value.size() >= 2 &&
            ((value.front() == '"' && value.back() == '"') ||
             (value.front() == '\'' && value.back() == '\''))) 
        {
            return value.substr(1, value.size() - 2);  //substr函数用于提取子字符串，第一个参数是起始位置，第二个参数是长度。
        }
        return value;
    };

    // 打开 YAML 文件。
    std::ifstream yaml_stream(yaml_file);
    if (!yaml_stream) {
        throw std::runtime_error("Could not open map YAML file: " + yaml_file);
    }

    /*
    * image_name YAML 文件中的图像文件名，
    * mode YAML 文件中的模式,"trinary"表示三值模式，即地图中的每个像素只有三种状态：空闲、占用、未知。
    * origin_text YAML 文件中的原点文本，格式为 [x, y, yaw] 格式。
    */
    std::string image_name;
    std::string mode = "trinary";
    std::string origin_text;

    /*
    * 解析 YAML 文件中的参数。
    * 如果 YAML 文件中没有指定这些参数，则使用默认值。
    * 
    * resolution 是地图的分辨率，即每个像素代表的实际距离。
    * occupied_threshold 是占用阈值，即像素值大于该阈值的像素被视为占用。
    * free_threshold 是空闲阈值，即像素值小于该阈值的像素被视为空闲。
    * negate 是取反标志，如果为 1，则像素值取反。
    */
    double resolution = 0.0;
    double occupied_threshold = 0.0;
    double free_threshold = 0.0;
    int negate = 0;
    bool has_origin = false;
    bool has_resolution = false;
    bool has_occupied_threshold = false;
    bool has_free_threshold = false;

    // 逐行解析 YAML 文件。
    std::string line;
    // 使用 std::getline 逐行读取 YAML 文件。
    while (std::getline(yaml_stream, line)) 
    {
        // 去除注释和空白。
        const auto comment = line.find('#');
        if (comment != std::string::npos) 
        {
            line.erase(comment);
        }
        // 去除字符串首尾空白。
        line = trim(line);
        // 如果字符串为空，则跳过。
        if (line.empty()) 
        {
            continue;
        }

        // 查找键值对分隔符。
        const auto separator = line.find(':');
        // 如果找不到分隔符，则跳过。
        if (separator == std::string::npos) 
        {
            continue;
        }
        // 提取键和值。
        const auto key = trim(line.substr(0, separator));
        const auto value = unquote(line.substr(separator + 1));
        if (key == "image") {
            image_name = value;
        } else if (key == "mode") {
            mode = value;
        } else if (key == "resolution") {
            resolution = std::stod(value);
            has_resolution = true;
        } else if (key == "origin") {
            origin_text = value;
            has_origin = true;
        } else if (key == "negate") {
            negate = std::stoi(value);
        } else if (key == "occupied_thresh") {
            occupied_threshold = std::stod(value);
            has_occupied_threshold = true;
        } else if (key == "free_thresh") {
            free_threshold = std::stod(value);
            has_free_threshold = true;
        }
    }

    // 检查 YAML 文件中的参数是否完整。
    if (image_name.empty() || mode != "trinary" || !has_resolution || !has_origin ||
        !has_occupied_threshold || !has_free_threshold) {
        throw std::runtime_error(
          "Map YAML must provide image, trinary mode, resolution, origin and occupancy thresholds");
    }
    // 检查分辨率、占用阈值和空闲阈值是否有效。
    if (!std::isfinite(resolution) || resolution <= 0.0 ||
        !std::isfinite(occupied_threshold) || !std::isfinite(free_threshold) ||
        free_threshold < 0.0 || occupied_threshold > 1.0 || free_threshold >= occupied_threshold) {
        throw std::runtime_error("Map YAML contains invalid resolution or occupancy thresholds");
    }

    // 解析原点文本。将 [x, y, yaw] 格式的字符串转换为三个 double 值。
    for (char & character : origin_text) {
        if (character == '[' || character == ']' || character == ',') {
            character = ' ';
        }
    }
    // 使用 istringstream 解析三个 double 值。
    // std::istringstream 是一个输入字符串流，可以将字符串作为输入流来读取数据。
    std::istringstream origin_stream(origin_text);
    double origin_x = 0.0;
    double origin_y = 0.0;
    double origin_yaw = 0.0;
    // 解析三个 double 值，如果解析失败或者值不合法，则抛出异常。
    if (!(origin_stream >> origin_x >> origin_y >> origin_yaw) ||
        !std::isfinite(origin_x) || !std::isfinite(origin_y) ||
        !std::isfinite(origin_yaw) || std::abs(origin_yaw) > 1.0e-6) {
        throw std::runtime_error("Map YAML origin must contain finite x, y and yaw=0 values");
    }

    // 解析图像文件路径。如果图像文件是相对路径，则相对于 YAML 文件所在目录。
    const auto yaml_path = std::filesystem::path(yaml_file);
    // std::filesystem::path 是 C++17 引入的文件系统库，可以方便地操作文件路径。
    auto image_path = std::filesystem::path(image_name);
    // 如果图像文件是相对路径，则相对于 YAML 文件所在目录。
    if (image_path.is_relative()) {
        image_path = yaml_path.parent_path() / image_path;
    }

    // 打开图像文件。使用 std::ifstream 以二进制模式打开图像文件。
    std::ifstream image_stream(image_path, std::ios::binary);
    if (!image_stream) {
        throw std::runtime_error("Could not open map image file: " + image_path.string());
    }

    // 定义一个 lambda 函数 read_header_line，用于读取 PGM 文件的头部信息。
    const auto read_header_line = [&image_stream, &trim]() {
        // 逐行读取 PGM 文件的头部信息，去除注释和空白。
        std::string header_line;
        // std::getline 从输入流中读取一行文本，直到遇到换行符为止。
        while (std::getline(image_stream, header_line)) {
            // 查找注释符号 #，如果找到则去除注释部分。
            const auto comment = header_line.find('#');
            // 如果找到注释符号，则去除注释部分。
            if (comment != std::string::npos) {
                header_line.erase(comment);
            }
            // 去除字符串首尾空白。
            header_line = trim(header_line);
            // 如果字符串不为空，则返回该行。
            if (!header_line.empty()) {
                return header_line;
            }
        }
        // 如果读取到文件末尾仍未找到有效的头部信息，则抛出异常。
        throw std::runtime_error("Unexpected end of PGM header");
    };

    /*
    *  读取 PGM 文件的头部信息，检查文件格式是否为二进制 P5 格式。
    *  P5 是 PGM 文件的一种格式，表示灰度图像的二进制数据。
    *  格式如下：
    *  P5
    *  # 注释行（可选）
    *  宽度 高度
    *  最大灰度值
    *  二进制像素数据
    * 
    *  如果不是 P5 格式，则抛出异常。
    */
    if (read_header_line() != "P5") {
        throw std::runtime_error("Only binary P5 PGM maps are supported");
    }

    // 读取 PGM 文件的宽度和高度，并检查是否有效。
    const auto dimensions = read_header_line();
    // 使用 istringstream 将 dimensions 字符串转换为输入流。
    std::istringstream dimension_stream(dimensions);
    // 读取宽度和高度，如果读取失败或者值不合法，则抛出异常。
    unsigned int width = 0;
    unsigned int height = 0;
    if (!(dimension_stream >> width >> height) || width == 0 || height == 0) {
        throw std::runtime_error("Invalid PGM dimensions");
    }

    // 读取 PGM 文件的最大灰度值，并检查是否有效。
    const auto max_value_text = read_header_line();
    // 使用 stoul 将 max_value_text 字符串转换为无符号长整数，并检查是否在 [1, 255] 范围内。
    const unsigned int max_value = static_cast<unsigned int>(std::stoul(max_value_text));
    // 如果 max_value 不在 [1, 255] 范围内，则抛出异常。
    if (max_value == 0 || max_value > 255) {
        throw std::runtime_error("PGM max value must be in [1, 255]");
    }

    // 读取 PGM 文件的像素数据，并检查是否完整。
    const auto pixel_count = static_cast<std::size_t>(width) * height;
    // 创建一个 vector 来存储像素数据，大小为 pixel_count。
    std::vector<unsigned char> pixels(pixel_count);
    // 使用 read 方法从输入流中读取像素数据，reinterpret_cast 将 unsigned char* 转换为 char*。
    image_stream.read(reinterpret_cast<char *>(pixels.data()),
                     static_cast<std::streamsize>(pixels.size()));
    // 如果读取的像素数据长度不等于 pixel_count，则抛出异常。
    if (image_stream.gcount() != static_cast<std::streamsize>(pixels.size())) {
        throw std::runtime_error("PGM image data is shorter than its declared dimensions");
    }

    // 将 PGM 文件的像素数据转换为 OccupancyGrid 消息，并加载到 costmap_ 中。
    nav_msgs::msg::OccupancyGrid message;
    message.header.stamp = now();
    message.header.frame_id = frame_id_;
    message.info.map_load_time = message.header.stamp;
    message.info.resolution = static_cast<float>(resolution);
    message.info.width = width;
    message.info.height = height;
    message.info.origin.position.x = origin_x;
    message.info.origin.position.y = origin_y;
    message.info.origin.orientation.w = 1.0;
    message.data.resize(pixel_count);

    // 将 PGM 文件的像素数据转换为 OccupancyGrid 消息的数据。
    for (unsigned int image_y = 0; image_y < height; ++image_y) {
        const unsigned int map_y = height - 1 - image_y;
        for (unsigned int mx = 0; mx < width; ++mx) {
            const auto index = static_cast<std::size_t>(image_y) * width + mx;
            const double pixel = static_cast<double>(pixels[index]) * 255.0 / max_value;
            double occupancy = (255.0 - pixel) / 255.0;
            if (negate != 0) {
                occupancy = 1.0 - occupancy;
            }

            const auto map_index = static_cast<std::size_t>(map_y) * width + mx;
            message.data[map_index] = occupancy > occupied_threshold ? 100 :
              (occupancy < free_threshold ? 0 : -1);
        }
    }

    loadMapMessage(message);
    if (!map_received_) {
        throw std::runtime_error("Loaded map could not be converted to Costmap2D");
    }
}

/* Costmap access -----------------------------------------------------------*/
/**
 * @brief 获取节点维护的原始代价图。
 *
 * 直接写入不会自动同步膨胀图；调用方须维持规划图与原图一致。
 * @return 可修改的内部地图引用。
 */
mini_nav_core::Costmap2D & mini_nav_nodes::CostmapPublisherNode::GetCostmap()
{
    return *costmap_;
}

/* Path planning and publication ------------------------------------------*/
/**
 * @brief 调用 A* 与安全后处理，并发布原始和最终路径。
 *
 * 中间点朝向沿路径方向；最终路径末点可与原目标不同。
 *
 * @param start 起点栅格。
 * @param goal 原目标栅格；不可达时允许容差替代。
 * @param goal_orientation 可选目标朝向，须为有效四元数；保留到实际末点。
 * @return 成功产生并发布路径为 true；地图、目标或安全检查失败清空旧路径并返回 false。
 */
bool mini_nav_nodes::CostmapPublisherNode::PlanAndPublish(
  const mini_nav_core::MapLocation & start,
  const mini_nav_core::MapLocation & goal,
  const std::optional<geometry_msgs::msg::Quaternion> & goal_orientation)
{
    // 检查是否已经接收到有效的地图，如果没有则无法进行路径规划。
    if (!map_received_ || !planning_costmap_) {
        clearPath();
        RCLCPP_WARN(get_logger(), "Cannot plan before a valid map is received");
        return false;
    }

    geometry_msgs::msg::Quaternion terminal_orientation;
    terminal_orientation.w = 1.0;
    if (goal_orientation.has_value()) {
        const auto & q = *goal_orientation;
        const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
        if (!std::isfinite(norm) || norm <= 1.0e-6) {
            clearPath();
            RCLCPP_WARN(get_logger(), "Ignoring goal with invalid orientation");
            return false;
        }
        terminal_orientation.x = q.x / norm;
        terminal_orientation.y = q.y / norm;
        terminal_orientation.z = q.z / norm;
        terminal_orientation.w = q.w / norm;
    }

    if (!rebuildPlanningMap()) {
        clearPath();
        return false;
    }
    const auto & obstacle_map = *collision_source_;
    start_unsafe_ = false;
    mini_nav_core::PathPoint start_point{};
    costmap_->MapToWorld(start.x, start.y, start_point.x, start_point.y);
    if (actual_start_) start_point = *actual_start_;
    const double body_radius = inflation_parameters_.robot_radius + inflation_parameters_.safety_margin;
    const double radius = body_radius + localization_uncertainty_;
    mini_nav_core::PathPoint points_translation{};
    double points_yaw = 0.0;
    if (collision_points_frame_ == odom_frame_id_) {
        try {
            const auto tf = tf_buffer_->lookupTransform(odom_frame_id_, frame_id_,
                rclcpp::Time(0, 0, get_clock()->get_clock_type()), rclcpp::Duration::from_seconds(.05));
            const auto & q = tf.transform.rotation;
            const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
            const double age = (now() - rclcpp::Time(tf.header.stamp, get_clock()->get_clock_type())).seconds();
            if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3 ||
                !std::isfinite(tf.transform.translation.x) || !std::isfinite(tf.transform.translation.y) ||
                (rclcpp::Time(tf.header.stamp).nanoseconds() != 0 &&
                 (!std::isfinite(age) || age < -.7 || age > max_pose_age_))) {
                clearPath(); return false;
            }
            points_translation = {tf.transform.translation.x, tf.transform.translation.y};
            points_yaw = tf2::getYaw(q);
        } catch (const tf2::TransformException &) { clearPath(); return false; }
    }
    const auto make_geometry = [&](const std::vector<mini_nav_core::PathPoint> & points,
                                   double terminal_margin = 0.0) {
        mini_nav_core::CollisionGeometry result{
            obstacle_map, points, radius + terminal_margin, observation_uncertainty_, true};
        if (collision_points_frame_ == odom_frame_id_) {
            result.point_radius = body_radius + terminal_margin;
            result.points_from_grid_translation = points_translation;
            result.points_from_grid_yaw = points_yaw;
        }
        return result;
    };
    const auto current_geometry = make_geometry(collision_points_);
    if (!current_geometry.IsClear(start_point, start_point)) {
        start_unsafe_ = true;
        clearPath();
        RCLCPP_WARN(get_logger(), "start_in_collision at (%.4f, %.4f)", start_point.x, start_point.y);
        return false;
    }
    std::vector<mini_nav_core::PathPoint> planning_points;
    if (dynamic_policy_ == "immediate") planning_points = collision_points_;
    else if (dynamic_replan_) {
        for (std::size_t index = 0; index < dynamic_points_.size(); ++index) {
            const auto & point = dynamic_points_[index];
            unsigned int mx, my;
            if (!costmap_->WorldToMap(point.x, point.y, mx, my)) continue;
            const auto it = stable_observations_.find(static_cast<std::size_t>(my) * costmap_->GetSizeInCellsX() + mx);
            if (it != stable_observations_.end() && it->second.count >= 3 &&
                it->second.last - it->second.first >= .3) planning_points.push_back(collision_points_[index]);
        }
    }
    const auto geometry = make_geometry(planning_points);
    // 覆盖整个允许停车区域，避免终点中心安全但容差内的实际停车点不安全。
    const auto terminal_geometry = make_geometry(collision_points_, goal_position_tolerance_);
    // A* 在膨胀规划图上搜索；原始图仍用于定位和地图显示。
    mini_nav_core::AStarPlanner planner(cost_travel_multiplier_);
    const std::vector<mini_nav_core::MapLocation> grid_path =
      planner.Plan(*planning_costmap_, geometry, start_point, start, goal, goal_tolerance_,
          &terminal_geometry, &current_geometry);

    if (grid_path.empty()) {
        clearPath();
        RCLCPP_WARN(get_logger(), "A* could not find a safe path from (%u, %u) to (%u, %u)",
                    start.x, start.y, goal.x, goal.y);
        return false;
    }

    const auto & selected_goal = grid_path.back();
    if (selected_goal.x != goal.x || selected_goal.y != goal.y) {
        const double offset = std::hypot(
          static_cast<double>(selected_goal.x) - goal.x,
          static_cast<double>(selected_goal.y) - goal.y) * costmap_->GetResolution();
        RCLCPP_INFO(get_logger(),
          "Goal (%u, %u) unreachable; using nearest reachable cell (%u, %u), offset %.3f m",
          goal.x, goal.y, selected_goal.x, selected_goal.y, offset);
    }

    // 核心层检查捷径穿越格、连续车体扫掠和积分软代价；失败则保留原始安全路径。
    auto final_points = mini_nav_core::SimplifyAndSmoothPath(
      obstacle_map, *planning_costmap_, grid_path,
      radius, cost_travel_multiplier_, true, &geometry);
    if (final_points.empty()) {
        clearPath();
        RCLCPP_WARN(get_logger(), "A* path failed continuous body-clearance validation");
        return false;
    }
    if (!current_geometry.IsClear(start_point, final_points.front())) {
        start_unsafe_ = true;
        clearPath();
        return false;
    }
    if (std::hypot(start_point.x - final_points.front().x, start_point.y - final_points.front().y) > 1e-6)
        final_points.insert(final_points.begin(), start_point);
    std::vector<mini_nav_core::PathPoint> raw_points;
    raw_points.reserve(grid_path.size());
    for (const auto & cell : grid_path) {
        mini_nav_core::PathPoint point{};
        costmap_->MapToWorld(cell.x, cell.y, point.x, point.y);
        raw_points.push_back(point);
    }
    const auto stamp = now();
    const auto make_path = [&](const std::vector<mini_nav_core::PathPoint> & points, double height) {
        nav_msgs::msg::Path path;
        path.header.stamp = stamp;
        path.header.frame_id = frame_id_;
        for (std::size_t index = 0; index < points.size(); ++index) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = points[index].x;
            pose.pose.position.y = points[index].y;
            // 两条线略微错开高度，避免 RViz 深度测试遮住比较结果。
            pose.pose.position.z = height;
            if (index + 1 < points.size()) {
                const double yaw = std::atan2(
                  points[index + 1].y - points[index].y,
                  points[index + 1].x - points[index].x);
                pose.pose.orientation.z = std::sin(yaw * 0.5);
                pose.pose.orientation.w = std::cos(yaw * 0.5);
            } else if (goal_orientation.has_value()) {
                pose.pose.orientation = terminal_orientation;
            } else if (!path.poses.empty()) {
                pose.pose.orientation = path.poses.back().pose.orientation;
            } else {
                pose.pose.orientation.w = 1.0;
            }
            path.poses.push_back(pose);
        }
        return path;
    };
    raw_path_ = make_path(raw_points, 0.06);
    global_path_ = make_path(final_points, 0.09);

    raw_path_publisher_->publish(raw_path_);
    path_publisher_->publish(global_path_);
    RCLCPP_INFO(get_logger(), "A* published %zu raw and %zu final poses",
                raw_path_.poses.size(), global_path_.poses.size());
    return true;
}

/* RViz interactive planning ---------------------------------------------*/
// 定位入口用 /initialpose 使旧路径失效；独立演示保留无 TF 的选点交互。
/**
 * @brief 收到重定位请求时清除旧路径；演示模式改为更新手选起点。
 *
 * @param message 全局坐标系初始位姿消息。
 */
void mini_nav_nodes::CostmapPublisherNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
{
    clearPath();
    stable_observations_.clear();
    stable_scan_stamp_ = 0.0;
    actual_start_.reset();
    if (use_initial_pose_as_start_) {
        demo_start_cell_.reset();
        if (!map_received_ || message->header.frame_id != frame_id_) {
            RCLCPP_WARN(get_logger(), "Ignoring demo start before a map or in the wrong frame");
            return;
        }
        unsigned int mx = 0;
        unsigned int my = 0;
        if (!costmap_->WorldToMap(message->pose.pose.position.x, message->pose.pose.position.y, mx, my) ||
            planning_costmap_->GetCost(mx, my) >= mini_nav_core::kInscribedInflatedObstacle) {
            RCLCPP_WARN(get_logger(), "Demo start is outside the safe planning area");
            return;
        }
        demo_start_cell_ = mini_nav_core::MapLocation{mx, my};
        if (demo_goal_cell_) {
            PlanAndPublish(*demo_start_cell_, *demo_goal_cell_, demo_goal_orientation_);
        }
        return;
    }
    // 旧 map -> odom 在 TF 缓存中会继续存在；下一次规划须等定位节点发布新结果。
    waitForNewLocalizationTf();
    RCLCPP_INFO(get_logger(), "Localization reset requested; waiting for a new map-to-odom TF");
}

/**
 * @brief 保存当前 map→odom 时间戳，要求下次规划等待更新结果。
 *
 * 旧 TF 仍可能留在缓存中；记录时间戳屏障避免刚重定位就使用旧起点。
 */
void mini_nav_nodes::CostmapPublisherNode::waitForNewLocalizationTf()
{
    try {
        const auto transform = tf_buffer_->lookupTransform(
          frame_id_, odom_frame_id_, rclcpp::Time(0, 0, get_clock()->get_clock_type()));
        localization_reset_tf_stamp_ = rclcpp::Time(
          transform.header.stamp, get_clock()->get_clock_type());
    } catch (const tf2::TransformException &) {
        localization_reset_tf_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    }
}

// 订阅 /goal_pose 话题，接收 RViz 中设置的终点。
/**
 * @brief 处理话题目标，从 TF 当前位置或演示手选起点规划。
 *
 * 先清空旧路径；越界目标拒绝，硬安全区目标交给容差选点逻辑。
 *
 * @param message 与地图帧一致的目标位姿。
 */
void mini_nav_nodes::CostmapPublisherNode::goalPoseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr message)
{
    clearPath();
    if (use_initial_pose_as_start_) {
        demo_goal_cell_.reset();
        demo_goal_orientation_.reset();
    }
    if (!map_received_) {
        RCLCPP_WARN(get_logger(), "Ignoring goal pose until a valid map is received");
        return;
    }
    if (message->header.frame_id != frame_id_) {
        RCLCPP_WARN(
          get_logger(), "Ignoring goal pose in frame '%s'; expected '%s'",
          message->header.frame_id.c_str(), frame_id_.c_str());
        return;
    }

    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap_->WorldToMap(message->pose.position.x, message->pose.position.y, mx, my)) {
        RCLCPP_WARN(get_logger(), "Selected goal is outside the map");
        return;
    }
    // 目标位于硬安全区时交给规划器，在目标容差内寻找可达安全格。

    if (use_initial_pose_as_start_) {
        demo_goal_cell_ = mini_nav_core::MapLocation{mx, my};
        demo_goal_orientation_ = message->pose.orientation;
        if (demo_start_cell_) {
            PlanAndPublish(*demo_start_cell_, *demo_goal_cell_, demo_goal_orientation_);
        } else {
            RCLCPP_INFO(get_logger(), "Select a demo start pose in RViz to plan");
        }
        return;
    }

    mini_nav_core::MapLocation start{0, 0};
    if (!lookupCurrentCell(start, 0.1)) {
        return;
    }
    RCLCPP_INFO(get_logger(), "Planning from current cell (%u, %u) to goal (%u, %u)",
      start.x, start.y, mx, my);
    PlanAndPublish(start, {mx, my}, message->pose.orientation);
}

/**
 * @brief 检查重定位后的新 TF、机器人位姿年龄与起点硬安全区。
 *
 * @param cell 成功时输出当前栅格；失败时不提交输出。
 * @param timeout_seconds TF 查询允许等待时长，秒。
 * @return 可用新鲜机器人 TF 落在安全图内时为 true，否则 false。
 */
bool mini_nav_nodes::CostmapPublisherNode::lookupCurrentCell(
  mini_nav_core::MapLocation & cell, double timeout_seconds)
{
    try {
        if (localization_reset_tf_stamp_.has_value()) {
            const auto localization_tf = tf_buffer_->lookupTransform(
              frame_id_, odom_frame_id_, rclcpp::Time(0, 0, get_clock()->get_clock_type()));
            const rclcpp::Time localization_stamp(
              localization_tf.header.stamp, get_clock()->get_clock_type());
            if (localization_stamp <= *localization_reset_tf_stamp_) {
                RCLCPP_WARN(get_logger(), "Waiting for localization TF after a map or initial pose update");
                return false;
            }
            localization_reset_tf_stamp_.reset();
        }
        const auto transform = tf_buffer_->lookupTransform(
          frame_id_, base_frame_id_,
          rclcpp::Time(0, 0, get_clock()->get_clock_type()),
          rclcpp::Duration::from_seconds(timeout_seconds));
        const rclcpp::Time stamp(transform.header.stamp, get_clock()->get_clock_type());
        const double age = (now() - stamp).seconds();
        if (!std::isfinite(age) || std::abs(age) > max_pose_age_) {
            RCLCPP_WARN(get_logger(), "Robot TF is stale or has an invalid timestamp (age %.3f s)", age);
            return false;
        }
        const double x = transform.transform.translation.x;
        const double y = transform.transform.translation.y;
        unsigned int mx = 0;
        unsigned int my = 0;
        if (!std::isfinite(x) || !std::isfinite(y) || !costmap_->WorldToMap(x, y, mx, my)) {
            RCLCPP_WARN(get_logger(), "Current robot position is outside the map or invalid");
            return false;
        }
        actual_start_ = mini_nav_core::PathPoint{x, y};
        cell = {mx, my};
        return true;
    } catch (const tf2::TransformException & exception) {
        RCLCPP_WARN(get_logger(), "Cannot use current robot TF: %s", exception.what());
        return false;
    }
}

/**
 * @brief 清空并发布两条空路径，使订阅者立即撤销旧路径显示或跟踪。
 */
void mini_nav_nodes::CostmapPublisherNode::clearPath()
{
    raw_path_ = nav_msgs::msg::Path();
    raw_path_.header.stamp = now();
    raw_path_.header.frame_id = frame_id_;
    global_path_ = nav_msgs::msg::Path();
    global_path_.header.stamp = raw_path_.header.stamp;
    global_path_.header.frame_id = frame_id_;
    raw_path_publisher_->publish(raw_path_);
    path_publisher_->publish(global_path_);
}

/* Map publication ---------------------------------------------------------*/

// 将 costmap_ 转换为 OccupancyGrid 消息，并发布到 /mini_nav/map 话题。
/**
 * @brief 按行转换并发布原始图、膨胀图与已有路径。
 *
 * 原始图 0 为空闲、100 为占据、-1 为未知；膨胀图另用 99 表示硬安全区。
 * 重发路径会刷新时间戳；TF 起点模式若当前位置不可用则清除路径。
 */
void mini_nav_nodes::CostmapPublisherNode::publishMap()
{
    if (!map_received_) {
        return;
    }

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
    msg::CollisionMap collision;
    collision.grid = message;
    const bool collision_valid = rebuildPlanningMap();
    for (std::size_t i = 0; i < collision.grid.data.size(); ++i) {
        const auto cost = collision_source_->GetCost(i);
        collision.grid.data[i] = cost == kUnknownCost ? -1 : cost >= kLethalObstacle ? 100 : 0;
    }
    // 与规划起点检查共享全部连续端点；稳定观测门限只控制全局绕行路线。
    collision.points_frame_id = collision_points_frame_;
    for (const auto & point : collision_points_) {
        geometry_msgs::msg::Point endpoint;
        endpoint.x = point.x;
        endpoint.y = point.y;
        collision.points.push_back(endpoint);
    }
    collision.observation_uncertainty = observation_uncertainty_;
    collision.valid = collision_valid;
    if (fuse_local_obstacles_ && latest_scan_) collision.grid.header.stamp = latest_scan_->header.stamp;
    if (require_cloud_ && latest_cloud_ &&
        rclcpp::Time(latest_cloud_->header.stamp) < rclcpp::Time(collision.grid.header.stamp))
        collision.grid.header.stamp = latest_cloud_->header.stamp;
    collision.clearance_radius = inflation_parameters_.robot_radius + inflation_parameters_.safety_margin +
        localization_uncertainty_;
    collision_publisher_->publish(collision);
    auto planning_message = message;
    for (unsigned int my = 0; my < planning_message.info.height; ++my) {
      for (unsigned int mx = 0; mx < planning_message.info.width; ++mx) {
        const auto cost = planning_costmap_->GetCost(mx, my);
        const auto index = static_cast<std::size_t>(my) * planning_message.info.width + mx;
        planning_message.data[index] = mini_nav_nodes::CostToOccupancyValue(cost);
      }
    }
    planning_costmap_publisher_->publish(planning_message);
    auto safety_message = message;
    for (std::size_t index = 0; index < safety_message.data.size(); ++index) {
        safety_message.data[index] = mini_nav_nodes::CostToOccupancyValue(safety_costmap_->GetCost(index));
    }
    safety_costmap_publisher_->publish(safety_message);
    if (!global_path_.poses.empty()) {
        mini_nav_core::MapLocation current{0, 0};
        if (!use_initial_pose_as_start_ && !lookupCurrentCell(current, 0.0)) {
            clearPath();
            publishCoordinateAxes();
            return;
        }
        global_path_.header.stamp = message.header.stamp;
        for (auto & pose : global_path_.poses) {
            pose.header = global_path_.header;
        }
        raw_path_.header.stamp = message.header.stamp;
        for (auto & pose : raw_path_.poses) {
            pose.header = raw_path_.header;
        }
        raw_path_publisher_->publish(raw_path_);
        path_publisher_->publish(global_path_);
    }
    publishCoordinateAxes();    
}

/* Coordinate-axis visualization -------------------------------------------*/
// 在 RViz 中显示地图坐标轴，便于观察地图的方向。
/**
 * @brief 发布地图原点以及 +X、+Y 方向的 RViz 标记。
 *
 * 箭头和标签按地图宽高缩放，仅用于坐标方向识别。
 */
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


/**
 * @brief 复制原始图，按扫描时刻 TF 投影激光端点后生成新膨胀图。
 *
 * 融合开启时局部图只提供新鲜度前提；实际障碍来自原始扫描端点，
 * 只在 map 系栅格化一次，避免把 odom 格面积二次投影造成虚假增厚。
 * 本次副本不保留上次动态障碍，也不修改定位所用静态地图。
 * @return 所需扫描、局部输入与 TF 有效且重建完成为 true，否则 false。
 */
bool mini_nav_nodes::CostmapPublisherNode::rebuildPlanningMap()
{
    dynamic_points_.clear();
    collision_points_.clear();
    collision_points_frame_ = frame_id_;
    fused_costmap_ = std::make_unique<mini_nav_core::Costmap2D>(*costmap_);
    collision_source_ = std::make_unique<mini_nav_core::Costmap2D>(*costmap_);
    if (fuse_local_obstacles_) {
        if (!latest_scan_ || !local_obstacles_ ||
            std::chrono::duration<double>(std::chrono::steady_clock::now() - scan_received_).count() > 0.8 ||
            std::chrono::duration<double>(std::chrono::steady_clock::now() - local_received_).count() > 0.8) return false;
        const auto & scan = *latest_scan_;
        const double age = (now() - rclcpp::Time(scan.header.stamp, get_clock()->get_clock_type())).seconds();
        if (!std::isfinite(age) || age < -0.1 || age > 0.8 || scan.ranges.empty() || scan.ranges.size() > 10000 ||
            !std::isfinite(scan.angle_min) || !std::isfinite(scan.angle_increment) || scan.angle_increment == 0.0 ||
            !std::isfinite(scan.range_min) || !std::isfinite(scan.range_max) || scan.range_min < 0.0 ||
            scan.range_max <= scan.range_min || scan.header.frame_id.empty()) return false;
        try {
            const auto tf = tf_buffer_->lookupTransform(frame_id_, scan.header.frame_id,
                rclcpp::Time(scan.header.stamp, get_clock()->get_clock_type()), rclcpp::Duration::from_seconds(0.05));
            const auto & q = tf.transform.rotation;
            const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
            if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3 ||
                !std::isfinite(tf.transform.translation.x) || !std::isfinite(tf.transform.translation.y)) return false;
            // 碰撞端点保留在 odom，静态先验的 map 配准误差不叠加到同源相对观测。
            collision_points_frame_ = scan.header.frame_id == frame_id_ ? frame_id_ : odom_frame_id_;
            const auto point_tf = tf_buffer_->lookupTransform(collision_points_frame_, scan.header.frame_id,
                rclcpp::Time(scan.header.stamp, get_clock()->get_clock_type()), rclcpp::Duration::from_seconds(.05));
            // Rasterize original laser endpoints once in map coordinates. Re-rasterizing an odom
            // occupancy cell adds another cell footprint and can falsely engulf the robot during turns.
            for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
                const double range = scan.ranges[i];
                if (!std::isfinite(range) || range < scan.range_min || range >= scan.range_max ||
                    range > obstacle_max_range_) continue;
                const double angle = scan.angle_min + i * scan.angle_increment;
                geometry_msgs::msg::PoseStamped source, target;
                source.pose.position.x = range * std::cos(angle);
                source.pose.position.y = range * std::sin(angle);
                source.pose.orientation.w = 1.0;
                tf2::doTransform(source, target, tf);
                unsigned int mx, my;
                if (fused_costmap_->WorldToMap(target.pose.position.x, target.pose.position.y, mx, my) &&
                    fused_costmap_->GetCost(mx, my) != kUnknownCost) {
                    dynamic_points_.push_back({target.pose.position.x, target.pose.position.y});
                    geometry_msgs::msg::PoseStamped observed;
                    tf2::doTransform(source, observed, point_tf);
                    collision_points_.push_back({observed.pose.position.x, observed.pose.position.y});
                    fused_costmap_->SetCost(mx, my, kLethalObstacle);
                }
            }
        } catch (const tf2::TransformException &) { return false; }
    }
    if (require_cloud_) {
        if (!latest_cloud_ || std::chrono::duration<double>(
            std::chrono::steady_clock::now() - cloud_received_).count() > 0.8) return false;
        const auto & cloud = *latest_cloud_;
        const auto stamp = rclcpp::Time(cloud.header.stamp, get_clock()->get_clock_type());
        const double age = (now() - stamp).seconds();
        if (age < -0.1 || age > 0.8 || cloud.header.frame_id.empty() ||
            !ValidCollisionCloud(cloud)) return false;
        try {
            const auto tf = tf_buffer_->lookupTransform(frame_id_, cloud.header.frame_id, stamp,
                rclcpp::Duration::from_seconds(0.05));
            sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
            for (; x != x.end(); ++x, ++y, ++z) {
                if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) continue;
                geometry_msgs::msg::PoseStamped source, target;
                source.pose.position.x = *x; source.pose.position.y = *y; source.pose.position.z = *z;
                source.pose.orientation.w = 1.0;
                tf2::doTransform(source, target, tf);
                unsigned int mx, my;
                if (fused_costmap_->WorldToMap(target.pose.position.x, target.pose.position.y, mx, my) &&
                    fused_costmap_->GetCost(mx, my) != kUnknownCost) {
                    collision_source_->SetCost(mx, my, kLethalObstacle);
                    fused_costmap_->SetCost(mx, my, kLethalObstacle);
                }
            }
        } catch (const std::exception &) { return false; }
    }
    if (latest_scan_) {
        const double stamp = rclcpp::Time(latest_scan_->header.stamp).seconds();
        if (stamp < stable_scan_stamp_) stable_observations_.clear();
        if (stamp != stable_scan_stamp_) {
            std::unordered_map<std::size_t, bool> seen;
            for (const auto & point : dynamic_points_) {
                unsigned int mx, my;
                if (!costmap_->WorldToMap(point.x, point.y, mx, my)) continue;
                const auto index = static_cast<std::size_t>(my) * costmap_->GetSizeInCellsX() + mx;
                if (seen[index]) continue;
                seen[index] = true;
                auto & observation = stable_observations_[index];
                if (observation.count == 0 || stamp - observation.last > 1.5)
                    observation = StableObservation{stamp, stamp, 1};
                else { observation.last = stamp; ++observation.count; }
            }
            for (auto it = stable_observations_.begin(); it != stable_observations_.end();) {
                if (stamp - it->second.last > 1.5) it = stable_observations_.erase(it);
                else ++it;
            }
            stable_scan_stamp_ = stamp;
        }
    }
    planning_costmap_ = std::make_unique<mini_nav_core::Costmap2D>(
      mini_nav_core::InflateCostmap(*fused_costmap_, inflation_parameters_));
    auto safety_parameters = inflation_parameters_;
    safety_parameters.inscribed_radius = 0.0;
    safety_costmap_ = std::make_unique<mini_nav_core::Costmap2D>(
      mini_nav_core::InflateCostmap(*fused_costmap_, safety_parameters));
    return true;
}

/**
 * @brief 处理 ComputePathToPose 请求，选择起点并返回规划结果和耗时。
 *
 * 仅支持空 planner_id 或 AStar；显式起点仍须在原始地图内并通过安全规划。
 * 当前实现同步规划，以 succeed/abort 返回；取消回调接受请求但这里未单独检查取消态。
 *
 * @param handle 规划 Action 目标句柄。
 */
void mini_nav_nodes::CostmapPublisherNode::computePath(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<ComputePath>> handle)
{
    auto result = std::make_shared<ComputePath::Result>();
    const auto started = std::chrono::steady_clock::now();
    const auto goal = handle->get_goal();
    result->error_code = ComputePath::Result::NO_VALID_PATH;
    if (!goal->planner_id.empty() && goal->planner_id != "AStar" && goal->planner_id != "AStarDynamic") {
        result->error_code = ComputePath::Result::INVALID_PLANNER;
    } else if (!map_received_) {
        result->error_msg = "map_unavailable";
    } else if (goal->goal.header.frame_id != frame_id_ ||
               (goal->use_start && goal->start.header.frame_id != frame_id_)) {
        result->error_code = ComputePath::Result::TF_ERROR;
    } else if (!rebuildPlanningMap()) {
        result->error_msg = "fresh_obstacles_unavailable";
    } else {
        actual_start_.reset();
        dynamic_replan_ = goal->planner_id == "AStarDynamic";
        // Check the start against current observations, never a cached dynamic obstacle map.
        mini_nav_core::MapLocation start{}, end{};
        bool valid_start = false;
        if (goal->use_start) {
            actual_start_ = mini_nav_core::PathPoint{goal->start.pose.position.x, goal->start.pose.position.y};
            valid_start = costmap_->WorldToMap(goal->start.pose.position.x, goal->start.pose.position.y,
              start.x, start.y);
        } else { valid_start = lookupCurrentCell(start, 0.0); }
        if (!valid_start) result->error_code = ComputePath::Result::TF_ERROR;
        else if (!costmap_->WorldToMap(goal->goal.pose.position.x, goal->goal.pose.position.y, end.x, end.y)) {
            result->error_code = ComputePath::Result::GOAL_OUTSIDE_MAP;
        } else if (PlanAndPublish(start, end, goal->goal.pose.orientation)) {
            result->path = global_path_;
            result->error_code = ComputePath::Result::NONE;
        } else if (start_unsafe_) {
            result->error_code = ComputePath::Result::START_OCCUPIED;
            result->error_msg = "start_in_collision";
        }
    }
    result->planning_time = rclcpp::Duration::from_seconds(
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count());
    if (result->error_code == ComputePath::Result::NONE) handle->succeed(result);
    else { if (result->error_msg.empty()) result->error_msg = "No safe path from current pose"; handle->abort(result); }
}

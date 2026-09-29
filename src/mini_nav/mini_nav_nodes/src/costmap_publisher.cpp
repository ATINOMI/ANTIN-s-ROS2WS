/* Includes ----------------------------------------------------------------*/
#include "costmap_publisher.hpp"
#include "mini_nav_nodes/costmap_display.hpp"

#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <sstream>
#include <vector>

#include "tf2/exceptions.h"

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
                                  cost_travel_multiplier_(
                                    declare_parameter<double>("planning.cost_travel_multiplier", 2.0)),
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
      declare_parameter<double>("planning.safety_margin", 0.05);
    inflation_parameters_.inflation_radius =
      declarePositiveDoubleParameter("planning.inflation_radius", 0.45);
    inflation_parameters_.cost_scaling_factor =
      declarePositiveDoubleParameter("planning.cost_scaling_factor", 10.0);
    inflation_parameters_.inflate_around_unknown =
      declare_parameter<bool>("planning.inflate_around_unknown", false);
    if (!std::isfinite(inflation_parameters_.safety_margin) ||
        inflation_parameters_.safety_margin < 0.0 ||
        inflation_parameters_.inflation_radius <
          inflation_parameters_.robot_radius + inflation_parameters_.safety_margin ||
        !std::isfinite(cost_travel_multiplier_) || cost_travel_multiplier_ < 0.0) {
      throw std::invalid_argument("Invalid planning safety margin, inflation radius, or traversal multiplier");
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
    goal_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", rclcpp::QoS(10),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        goalPoseCallback(message);
      });

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
 * @brief  声明一个正整数参数，如果参数值小于等于0则抛出异常。
 * 
 * @param name  参数名
 * @param default_value  默认值
 * @return unsigned int 
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
 * @brief  声明一个正浮点数参数，如果参数值小于等于0则抛出异常。
 * 
 * @param name  参数名
 * @param default_value  默认值
 * @return double 
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
 * @brief  声明一个字节参数，如果参数值不在[0, 255]则抛出异常。
 * 
 * @param name 
 * @param default_value 
 * @return unsigned char 
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
 * @brief  订阅 /map 话题，接收 map_server 发布的 OccupancyGrid 消息。
 * 
 * @param message 
 */
void mini_nav_nodes::CostmapPublisherNode::mapCallback(
  const nav_msgs::msg::OccupancyGrid::ConstSharedPtr message)
{
    loadMapMessage(*message);// 调用 loadMapMessage 加载地图
}

/**
 * @brief  加载地图消息到 costmap_ 中。
 * 
 * @param message  地图消息
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

        // 替换旧地图。std::move()是 C++11 引入的右值引用转换，将 new_costmap 的所有权转移给 costmap_。
        costmap_ = std::move(new_costmap);
        planning_costmap_ = std::move(new_planning_costmap);
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
 * @brief  从 YAML 文件加载地图。
 *         该函数解析 YAML 文件，读取图像文件，
 *         并将图像转换为 Costmap2D。
 * @param yaml_file  YAML 文件路径
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
mini_nav_core::Costmap2D & mini_nav_nodes::CostmapPublisherNode::GetCostmap()
{
    return *costmap_;
}

/* Path planning and publication ------------------------------------------*/
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

    // A* 在膨胀规划图上搜索；原始图仍用于定位和地图显示。
    mini_nav_core::AStarPlanner planner(cost_travel_multiplier_);
    const std::vector<mini_nav_core::MapLocation> grid_path =
      planner.Plan(*planning_costmap_, *costmap_,
                   inflation_parameters_.robot_radius + inflation_parameters_.safety_margin,
                   start, goal, inflation_parameters_.inflate_around_unknown);

    if (grid_path.empty()) {
        clearPath();
        RCLCPP_WARN(get_logger(), "A* could not find a safe path from (%u, %u) to (%u, %u)",
                    start.x, start.y, goal.x, goal.y);
        return false;
    }

    // 核心层检查捷径穿越格、连续车体扫掠和积分软代价；失败则保留原始安全路径。
    const auto final_points = mini_nav_core::SimplifyAndSmoothPath(
      *costmap_, *planning_costmap_, grid_path,
      inflation_parameters_.robot_radius + inflation_parameters_.safety_margin,
      cost_travel_multiplier_, inflation_parameters_.inflate_around_unknown);
    if (final_points.empty()) {
        clearPath();
        RCLCPP_WARN(get_logger(), "A* path failed continuous body-clearance validation");
        return false;
    }
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
void mini_nav_nodes::CostmapPublisherNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
{
    clearPath();
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
    if (planning_costmap_->GetCost(mx, my) >= mini_nav_core::kInscribedInflatedObstacle) {
        RCLCPP_WARN(get_logger(), "Selected goal cell (%u, %u) is inside the planning safety zone", mx, my);
        return;
    }

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
        if (planning_costmap_->GetCost(mx, my) >= mini_nav_core::kInscribedInflatedObstacle) {
            RCLCPP_WARN(get_logger(), "Current robot cell (%u, %u) is inside the planning safety zone", mx, my);
            return false;
        }
        cell = {mx, my};
        return true;
    } catch (const tf2::TransformException & exception) {
        RCLCPP_WARN(get_logger(), "Cannot use current robot TF: %s", exception.what());
        return false;
    }
}

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
    auto planning_message = message;
    for (unsigned int my = 0; my < planning_message.info.height; ++my) {
      for (unsigned int mx = 0; mx < planning_message.info.width; ++mx) {
        const auto cost = planning_costmap_->GetCost(mx, my);
        const auto index = static_cast<std::size_t>(my) * planning_message.info.width + mx;
        planning_message.data[index] = mini_nav_nodes::CostToOccupancyValue(cost);
      }
    }
    planning_costmap_publisher_->publish(planning_message);
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

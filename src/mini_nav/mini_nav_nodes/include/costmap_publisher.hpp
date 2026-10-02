/**
 * @file costmap_publisher.hpp
 * @brief 静态地图与扫描融合、自研规划 Action 和 RViz 输出。
 * @author Antinomy
 * @date 2026-10-01
 */
#pragma once

/* Includes ----------------------------------------------------------------*/
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "mini_nav_core/navigator/astar_navigator.hpp"
#include "mini_nav_core/navigator/path_postprocessor.hpp"
#include "mini_nav_core/map/costmap_2d.hpp"
#include "mini_nav_core/map/inflation_layer.hpp"

/* Namespace ---------------------------------------------------------------*/
namespace mini_nav_nodes
{
    /* Class definition --------------------------------------------------------*/
    /**
     * @brief 将 /map 的 OccupancyGrid 转换为 Costmap2D，并运行 A*。
     *
     * 地图的权威来源是 map_server 发布的 /map；本节点保留一份
     * mini_nav_core 使用的 ROS 无关代价地图，并发布规划结果供可视化。
     */
    class CostmapPublisherNode : public rclcpp::Node
    {
        friend class CostmapPublisherNodeTest;

        /* Public API -----------------------------------------------------------*/

        public:
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
            CostmapPublisherNode(int size_x, 
                                 int size_y, 
                                 double resolution, 
                                 double origin_x, 
                                 double origin_y, 
                                 unsigned char default_value, 
                                 unsigned int publish_period_ms);

            /**
             * @brief 获取节点维护的原始代价图。
             *
             * 直接写入不会自动同步膨胀图；调用方须维持规划图与原图一致。
             * @return 可修改的内部地图引用。
             */
            mini_nav_core::Costmap2D & GetCostmap();

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
            bool PlanAndPublish(
              const mini_nav_core::MapLocation & start,
              const mini_nav_core::MapLocation & goal,
              const std::optional<geometry_msgs::msg::Quaternion> & goal_orientation = std::nullopt);


        /* Private members ------------------------------------------------------*/

        private:
            const unsigned int size_x_;
            const unsigned int size_y_;
            const double resolution_;
            const double origin_x_;
            const double origin_y_;
            const unsigned char default_value_;
            const unsigned int publish_period_ms_;
            bool add_demo_obstacles_;
            mini_nav_core::InflationParameters inflation_parameters_;
            double cost_travel_multiplier_;
            double goal_tolerance_;

            /* Cost constants ----------------------------------------------------*/

            /// Nav2 常用约定：254 表示致命障碍物；转换后会变为 OccupancyGrid 的 100。
            static constexpr unsigned char kLethalObstacle = 254;
            /// OccupancyGrid 约定：-1 表示未知；这里用 255 作为内部未知代价。
            static constexpr unsigned char kUnknownCost = 255;

            /* ROS entities and map state ----------------------------------------*/

            /// ROS 无关的地图数据模型；将来由传感器回调或地图加载器更新。 
            std::unique_ptr<mini_nav_core::Costmap2D> costmap_;
            /// 原始地图生成的膨胀规划图；不会写回定位所用的 /map。
            std::unique_ptr<mini_nav_core::Costmap2D> planning_costmap_;
            /// 根据 map -> base_footprint 等 TF 获取当前车位，不从 /initialpose 缓存起点。
            std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
            /// map_server 发布的静态地图输入。
            rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
            /// /mini_nav/map 的发布器。
            rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
            /// /mini_nav/planning_costmap 的发布器，供 RViz 核对安全区。
            rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr planning_costmap_publisher_;
            /// /mini_nav/raw_path 的原始八邻域 A* 路径发布器。
            rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_publisher_;
            /// /mini_nav/global_path 的最终路径发布器。
            rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
            /// 缓存的原始路径，与最终路径一起清除和重新发布。
            nav_msgs::msg::Path raw_path_;
            /// 缓存的最终路径；由定时器持续发布，保证 RViz 后启动也能显示。
            nav_msgs::msg::Path global_path_;
            /// /mini_nav/map_axes 的 RViz 坐标轴标记发布器。
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr axes_publisher_;
            /// 定位入口中使旧路径失效；独立 A* 演示中可作为手选起点。
            rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_subscription_;
            /// RViz 2D Goal Pose 发送的终点订阅器。
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscription_;
            /// 周期发布定时器，避免在回调中阻塞等待。
            rclcpp::TimerBase::SharedPtr timer_;
            /// 输入地图的话题名称，默认为 map_server 的 /map。
            const std::string map_topic_;
            /// 可选的外部 YAML 地图文件；非空时由本节点直接加载。
            const std::string map_file_;
            /// 发布消息使用的坐标系名称，默认是 map。
            const std::string frame_id_;
            /// 实时起点使用的机器人基座坐标系。
            const std::string base_frame_id_;
            /// 定位节点发布的里程计坐标系，用于辨别初始位姿后的新定位结果。
            const std::string odom_frame_id_;
            /// 最后一次有效机器人 TF 可以距离当前时刻的最大秒数。
            const double max_pose_age_;
            /// 独立 A* 演示显式开启的手选起点模式；定位入口保持关闭。
            const bool use_initial_pose_as_start_;
            /// 收到 /initialpose 时的 map -> odom 时间戳；下一次规划须等待更新。
            std::optional<rclcpp::Time> localization_reset_tf_stamp_;
            /// 仅供无机器人 TF 的独立 A* 演示使用。
            std::optional<mini_nav_core::MapLocation> demo_start_cell_;
            std::optional<mini_nav_core::MapLocation> demo_goal_cell_;
            std::optional<geometry_msgs::msg::Quaternion> demo_goal_orientation_;
            /// 只有收到并成功转换地图后才允许规划和发布。
            bool map_received_ = false;
            using ComputePath = nav2_msgs::action::ComputePathToPose;
            rclcpp_action::Server<ComputePath>::SharedPtr plan_server_;
            rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_subscription_;
            nav_msgs::msg::OccupancyGrid::ConstSharedPtr local_obstacles_;
            std::chrono::steady_clock::time_point local_received_{};
            std::unique_ptr<mini_nav_core::Costmap2D> fused_costmap_;
            bool fuse_local_obstacles_{false};
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
            sensor_msgs::msg::LaserScan::ConstSharedPtr latest_scan_;
            std::chrono::steady_clock::time_point scan_received_{};
            double obstacle_max_range_{2.5};
            /**
             * @brief 复制原始图，按扫描时刻 TF 投影激光端点后生成新膨胀图。
             *
             * 融合开启时局部图只提供新鲜度前提；实际障碍来自原始扫描端点，
             * 只在 map 系栅格化一次，避免把 odom 格面积二次投影造成虚假增厚。
             * 本次副本不保留上次动态障碍，也不修改定位所用静态地图。
             * @return 所需扫描、局部输入与 TF 有效且重建完成为 true，否则 false。
             */
            bool rebuildPlanningMap();
            /**
             * @brief 处理 ComputePathToPose 请求，选择起点并返回规划结果和耗时。
             *
             * 仅支持空 planner_id 或 AStar；显式起点仍须在原始地图内并通过安全规划。
             * 当前实现同步规划，以 succeed/abort 返回；取消回调接受请求但这里未单独检查取消态。
             *
             * @param handle 规划 Action 目标句柄。
             */
            void computePath(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ComputePath>> handle);


            /* Private API ------------------------------------------------------*/
            
            /**
             * @brief 声明一个必须大于零的整型参数。
             * @param name 参数名称，例如 map.size_x。
             * @param default_value 参数未设置时使用的默认值。
             * @return 已验证并转换为 unsigned int 的参数值。
             * @throws std::invalid_argument 参数小于或等于零时抛出。
             */            
            unsigned int declarePositiveIntParameter(const std::string & name, int default_value);

            /**
             * @brief 声明一个必须大于零的浮点参数。
             * @param name 参数名称，例如 map.resolution。
             * @param default_value 参数未设置时使用的默认值。
             * @return 已验证的参数值。
             * @throws std::invalid_argument 参数小于或等于零时抛出。
             */            
            double declarePositiveDoubleParameter(const std::string & name, double default_value);

            /**
             * @brief 声明一个取值范围为 [0, 255] 的代价参数。
             * @param name 参数名称。
             * @param default_value 参数未设置时使用的默认值。
             * @return 转换为 unsigned char 的代价值。
             * @throws std::invalid_argument 参数超出一个字节的可表示范围时抛出。
             */
            unsigned char declareByteParameter(const std::string & name, int default_value);

            /**
             * @brief 把地图话题输入交给统一校验和转换入口。
             *
             * @param message map_server 占据图。
             */
            void mapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr message);

            /**
             * @brief 解析 trinary 地图 YAML 与 P2/P5 PGM，并转为占据图。
             *
             * 图像行方向翻转为地图 y 递增方向；仅支持无原点旋转的 trinary 模式。
             *
             * @param yaml_file 地图元数据路径；相对图像路径以 YAML 所在目录解析。
             * @throws std::runtime_error 文件不可读、元数据或图像格式不支持；数字解析可能抛出标准转换异常。
             */
            void loadMapFromYaml(const std::string & yaml_file);

            /**
             * @brief 校验地图帧、尺寸和原点旋转，构建原始图与膨胀图后替换缓存。
             *
             * 旧路径及演示选点随地图替换清除；TF 起点模式等待新的定位变换。
             * 转换失败记录日志，不把未完成构建的地图提交给规划器。
             *
             * @param message 占据图：0 空闲、负值未知、正值障碍。
             */
            void loadMapMessage(const nav_msgs::msg::OccupancyGrid & message);



            /**
             * @brief 按行转换并发布原始图、膨胀图与已有路径。
             *
             * 原始图 0 为空闲、100 为占据、-1 为未知；膨胀图另用 99 表示硬安全区。
             * 重发路径会刷新时间戳；TF 起点模式若当前位置不可用则清除路径。
             */
            void publishMap();

            /**
             * @brief 发布地图原点以及 +X、+Y 方向的 RViz 标记。
             *
             * 箭头和标签按地图宽高缩放，仅用于坐标方向识别。
             */
            void publishCoordinateAxes();

            /* RViz interactive planning -------------------------------------------*/
            /**
             * @brief 收到重定位请求时清除旧路径；演示模式改为更新手选起点。
             *
             * @param message 全局坐标系初始位姿消息。
             */
            void initialPoseCallback(
              const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message);
            /**
             * @brief 处理话题目标，从 TF 当前位置或演示手选起点规划。
             *
             * 先清空旧路径；越界目标拒绝，硬安全区目标交给容差选点逻辑。
             *
             * @param message 与地图帧一致的目标位姿。
             */
            void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr message);
            /**
             * @brief 检查重定位后的新 TF、机器人位姿年龄与起点硬安全区。
             *
             * @param cell 成功时输出当前栅格；失败时不提交输出。
             * @param timeout_seconds TF 查询允许等待时长，秒。
             * @return 可用新鲜机器人 TF 落在安全图内时为 true，否则 false。
             */
            bool lookupCurrentCell(mini_nav_core::MapLocation & cell, double timeout_seconds);
            /**
             * @brief 保存当前 map→odom 时间戳，要求下次规划等待更新结果。
             *
             * 旧 TF 仍可能留在缓存中；记录时间戳屏障避免刚重定位就使用旧起点。
             */
            void waitForNewLocalizationTf();
            /**
             * @brief 清空并发布两条空路径，使订阅者立即撤销旧路径显示或跟踪。
             */
            void clearPath();


    };
}

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
#include "rclcpp/rclcpp.hpp"
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

        /* Public API -----------------------------------------------------------*/

        public:
            CostmapPublisherNode(int size_x, 
                                 int size_y, 
                                 double resolution, 
                                 double origin_x, 
                                 double origin_y, 
                                 unsigned char default_value, 
                                 unsigned int publish_period_ms);

            /**
             * @brief 获取节点维护的代价地图，用于构建或更新地图内容。
             * @return 可修改的 Costmap2D 引用。
             */
            mini_nav_core::Costmap2D & GetCostmap();

            /** 对指定栅格执行 A*，可选保留终点朝向，并发布 RViz Path。 */
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

            /** 将 map_server 的 OccupancyGrid 原子式转换为内部 Costmap2D。 */
            void mapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr message);

            /** 从标准 trinary PGM 地图 YAML 加载并发布一张静态地图。 */
            void loadMapFromYaml(const std::string & yaml_file);

            /** 将一张已经构造好的 OccupancyGrid 更新到内部代价地图。 */
            void loadMapMessage(const nav_msgs::msg::OccupancyGrid & message);



            /**
             * @brief 将 Costmap2D 转换为 OccupancyGrid 并发布。
             *
             * OccupancyGrid 的 data 同样是按行存储，索引公式为 my * width + mx，
             * 因此可以按相同的双层循环逐栅格拷贝。ROS 约定：0 为空闲、100 为占据、-1 为未知。
             */
            void publishMap();

            /**  发布 map 坐标系原点以及 +X、+Y 方向的 RViz 标记。 */
            void publishCoordinateAxes();

            /* RViz interactive planning -------------------------------------------*/
            void initialPoseCallback(
              const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message);
            void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr message);
            bool lookupCurrentCell(mini_nav_core::MapLocation & cell, double timeout_seconds);
            void waitForNewLocalizationTf();
            void clearPath();


    };
}

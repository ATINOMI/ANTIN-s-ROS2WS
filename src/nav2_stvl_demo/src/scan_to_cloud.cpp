/**
 * @file scan_to_cloud.cpp
 * @brief 将二维激光 LaserScan 转换为 STVL 使用的 PointCloud2。
 *
 * 当前仿真模型主要提供 /scan 话题，而 STVL 的典型输入是三维点云。
 * 注意：当前适配器生成的点云 z 坐标全部为 0，不等同于真实三维传感器。
 */

#include <cmath>
#include <limits>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class ScanToCloud : public rclcpp::Node
{
public:
  /**
   * @brief 创建节点，并建立 /scan 到 /stvl/points 的通信关系。
   */
  ScanToCloud()
  : Node("scan_to_cloud")
  {
    // STVL 配置文件会从这个话题读取 PointCloud2 数据。
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/stvl/points", 10);
    // 订阅 Gazebo 中 TurtleBot3 激光雷达发布的二维扫描数据。
    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
        // 沿用激光消息的时间戳和坐标系，便于 TF 进行坐标变换。
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header = scan->header;
        cloud.height = 1;
        cloud.width = 0;
        cloud.is_dense = false;

        // 创建 x、y、z 字段，并按扫描点数量分配空间。
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(scan->ranges.size());

        // 通过迭代器逐点写入 PointCloud2 的坐标字段。
        sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");

        for (std::size_t i = 0; i < scan->ranges.size(); ++i, ++x, ++y, ++z) {
          const float range = scan->ranges[i];
          const float angle = scan->angle_min + static_cast<float>(i) * scan->angle_increment;
          // 过滤无穷大、NaN 和超出雷达有效量程的扫描值。
          if (std::isfinite(range) && range >= scan->range_min && range <= scan->range_max) {
            // 极坐标转笛卡尔坐标；当前适配器将所有点放在 z=0 平面。
            *x = range * std::cos(angle);
            *y = range * std::sin(angle);
            *z = 0.0F;
          } else {
            // 无效点使用 NaN 标记，避免被当作真实障碍物。
            *x = *y = *z = std::numeric_limits<float>::quiet_NaN();
          }
        }
        // 将转换后的点云发布给 STVL。
        publisher_->publish(cloud);
      });
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  // 初始化 ROS 2，运行节点回调，退出时释放 ROS 2 资源。
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanToCloud>());
  rclcpp::shutdown();
  return 0;
}

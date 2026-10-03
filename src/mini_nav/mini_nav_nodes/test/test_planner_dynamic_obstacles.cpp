/**
 * @file test_planner_dynamic_obstacles.cpp
 * @brief 验证 planner_dynamic_obstacles 模块行为及边界的测试。
 * @author Antinomy
 * @date 2026-10-01
 */
#include <cmath>
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "mini_nav_nodes/cloud_validation.hpp"
#include <gtest/gtest.h>
#include "costmap_publisher.hpp"

namespace mini_nav_nodes
{
class CostmapPublisherNodeTest : public ::testing::Test
{
protected:
    /**
     * @brief 在测试套件启动前初始化 ROS context。
     */
    static void SetUpTestSuite() { int argc = 0; rclcpp::init(argc, nullptr); }
    /**
     * @brief 在套件结束时关闭 ROS context。
     */
    static void TearDownTestSuite() { rclcpp::shutdown(); }
    /**
     * @brief 建立转弯碰撞回归场景的地图、扫描和新鲜度输入。
     *
     * @param node 被测规划节点。
     * @param age 模拟扫描年龄，秒。
     */
    static void SetScan(CostmapPublisherNode & node, double age = 0.0)
    {
        node.costmap_ = std::make_unique<mini_nav_core::Costmap2D>(112, 103, 0.05, -0.961, -2.072, 0);
        node.planning_costmap_ = std::make_unique<mini_nav_core::Costmap2D>(*node.costmap_);
        node.map_received_ = true;
        node.fuse_local_obstacles_ = true;
        auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
        scan->header.frame_id = "map";
        scan->header.stamp = node.now() - rclcpp::Duration::from_seconds(age);
        scan->angle_min = std::atan2(0.345, 0.859);
        scan->angle_increment = 0.01;
        scan->range_min = 0.01;
        scan->range_max = 3.0;
        scan->ranges = {static_cast<float>(std::hypot(0.859, 0.345))};
        node.latest_scan_ = scan;
        node.scan_received_ = std::chrono::steady_clock::now();
        node.local_obstacles_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        node.local_received_ = std::chrono::steady_clock::now();
    }
    /**
     * @brief 调用真实动态障碍融合与膨胀实现。
     *
     * @param node 被测规划节点。
     * @return 重建是否成功。
     */
    static void SetCloud(CostmapPublisherNode & node, double age = 0.0)
    {
        node.require_cloud_ = true;
        auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
        cloud->header.frame_id = "map";
        cloud->header.stamp = node.now() - rclcpp::Duration::from_seconds(age);
        sensor_msgs::PointCloud2Modifier modifier(*cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(1);
        sensor_msgs::PointCloud2Iterator<float> x(*cloud, "x"), y(*cloud, "y"), z(*cloud, "z");
        *x = 0.5F; *y = 0.5F; *z = 0.2F;
        node.latest_cloud_ = cloud;
        node.cloud_received_ = std::chrono::steady_clock::now();
    }
    static bool Rebuild(CostmapPublisherNode & node) { return node.rebuildPlanningMap(); }
    /**
     * @brief 读取规划图指定单元的内部代价。
     *
     * @param node 被测节点。
     * @param x 格 x 下标。
     * @param y 格 y 下标。
     * @return 未压缩的 0..255 代价。
     */
    static unsigned char Cost(const CostmapPublisherNode & node, unsigned int x, unsigned int y)
    { return node.planning_costmap_->GetCost(x, y); }
};

/**
 * @brief 验证原始端点单次栅格化不虚假侵占转弯起点。
 */
TEST_F(CostmapPublisherNodeTest, OneRasterizationKeepsTheTurningRobotCellFree)
{
    // Geometry reduced from the Gazebo turn failure: expanding an already rasterized
    // odom cell made map cell (34,42) inscribed, although raw laser/body clearance was safe.
    CostmapPublisherNode node(40, 30, 0.1, 0.0, 0.0, 0, 1000);
    SetScan(node);
    ASSERT_TRUE(Rebuild(node));
    EXPECT_EQ(Cost(node, 36, 48), 254);
    EXPECT_LT(Cost(node, 34, 42), 253);
    EXPECT_TRUE(node.PlanAndPublish({34, 42}, {27, 42}));
}

/**
 * @brief 验证旧扫描无法用于更新动态规划。
 */
TEST_F(CostmapPublisherNodeTest, StaleLaserCannotProduceAnUpdatedPlan)
{
    CostmapPublisherNode node(40, 30, 0.1, 0.0, 0.0, 0, 1000);
    SetScan(node, 1.0);
    EXPECT_FALSE(Rebuild(node));
    EXPECT_FALSE(node.PlanAndPublish({34, 42}, {27, 42}));
}
TEST_F(CostmapPublisherNodeTest, HeightCloudMarksObstacleOnceAndStaleCloudRejectsPlan)
{
    CostmapPublisherNode node(40, 30, 0.1, 0.0, 0.0, 0, 1000);
    SetScan(node);
    SetCloud(node);
    ASSERT_TRUE(Rebuild(node));
    EXPECT_EQ(Cost(node, 29, 51), 254);
    SetCloud(node, 1.0);
    EXPECT_FALSE(Rebuild(node));
}

TEST(CollisionCloudLayout, RejectsTruncatedOrWrongTypedData)
{
    sensor_msgs::msg::PointCloud2 cloud;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(1);
    EXPECT_TRUE(ValidCollisionCloud(cloud));
    cloud.data.pop_back();
    EXPECT_FALSE(ValidCollisionCloud(cloud));
    cloud.data.push_back(0);
    cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT64;
    EXPECT_FALSE(ValidCollisionCloud(cloud));
}

}

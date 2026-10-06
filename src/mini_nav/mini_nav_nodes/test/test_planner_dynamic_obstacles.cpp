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
#include "mini_nav_nodes/collision_map.hpp"
#include <limits>

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
    static void SetStaticPolicy(CostmapPublisherNode & node)
    { node.dynamic_policy_ = "static_then_stable"; }
    static void Publish(CostmapPublisherNode & node) { node.publishMap(); }
    static mini_nav_core::PathPoint Endpoint(const CostmapPublisherNode & node) {
        const auto & p = node.global_path_.poses.back().pose.position;
        return {p.x, p.y};
    }
    static bool CurrentClear(const CostmapPublisherNode & node, const mini_nav_core::PathPoint & p) {
        const double radius = node.inflation_parameters_.robot_radius +
            node.inflation_parameters_.safety_margin + node.localization_uncertainty_;
        return mini_nav_core::CollisionGeometry{*node.collision_source_, node.dynamic_points_,
            radius, node.observation_uncertainty_}.IsClear(p, p);
    }
    static void CheckRelativeFrame(const CostmapPublisherNode & node) {
        EXPECT_EQ(node.collision_points_frame_, "odom");
        ASSERT_EQ(node.collision_points_.size(), 1u);
        EXPECT_NEAR(node.collision_points_[0].x, .8297640675872109, 1e-6);
        EXPECT_NEAR(node.collision_points_[0].y, -.86565062252723, 1e-6);
    }
    static void SetRelativeScan(CostmapPublisherNode & node) {
        SetScan(node);
        auto scan = std::make_shared<sensor_msgs::msg::LaserScan>(*node.latest_scan_);
        scan->header.frame_id = "odom";
        scan->angle_min = std::atan2(-.86565062252723, .8297640675872109);
        scan->ranges = {static_cast<float>(std::hypot(.8297640675872109, -.86565062252723))};
        node.latest_scan_ = scan;
        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = "map";
        transform.child_frame_id = "odom";
        transform.transform.rotation.w = 1.;
        transform.transform.translation.x = .582574028517864 - .6165778430923;
        transform.transform.translation.y = -.934695848070459 + 1.092706860297;
        node.tf_buffer_->setTransform(transform, "relative_observation_test", true);
        node.actual_start_ = mini_nav_core::PathPoint{.582574028517864, -.934695848070459};
    }
    static bool Rebuild(CostmapPublisherNode & node) { return node.rebuildPlanningMap(); }
    static void SetLocalizationBudget(CostmapPublisherNode & node, double uncertainty)
    { node.localization_uncertainty_ = uncertainty; }
    static void CheckResetClearsStableCache(CostmapPublisherNode & node) {
        node.stable_observations_[1] = {1., 2., 5};
        node.stable_scan_stamp_ = 2.;
        auto pose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
        pose->header.frame_id = "map";
        node.initialPoseCallback(pose);
        EXPECT_TRUE(node.stable_observations_.empty());
        node.stable_observations_[1] = {1., 2., 5};
        nav_msgs::msg::OccupancyGrid map;
        map.header.frame_id = "map";
        map.info.width = map.info.height = 20;
        map.info.resolution = .1;
        map.info.origin.orientation.w = 1.;
        map.data.assign(400, 0);
        node.loadMapMessage(map);
        EXPECT_TRUE(node.stable_observations_.empty());
    }
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
TEST_F(CostmapPublisherNodeTest, OneRasterizationRespectsTheExplicitUncertaintyBudget)
{
    // Geometry reduced from the Gazebo turn failure: expanding an already rasterized
    // odom cell made map cell (34,42) inscribed, although raw laser/body clearance was safe.
    CostmapPublisherNode node(40, 30, 0.1, 0.0, 0.0, 0, 1000);
    SetScan(node);
    ASSERT_TRUE(Rebuild(node));
    EXPECT_EQ(Cost(node, 36, 48), 254);
    EXPECT_LT(Cost(node, 34, 42), 253);
    // 显示空闲不代表有足够误差余量；默认 map 预算下应明确拒绝。
    EXPECT_FALSE(node.PlanAndPublish({34, 42}, {27, 42}));
    // 单独验证理想配准时，端点精查不再引入二次栅格面积。
    SetLocalizationBudget(node, 0.0);
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

TEST_F(CostmapPublisherNodeTest, ResetDoesNotReuseStableDynamicEvidence)
{
    CostmapPublisherNode node(40, 30, .1, 0., 0., 0, 1000);
    CheckResetClearsStableCache(node);
}

TEST(CollisionSnapshot, RejectsSoftMapsAndInvalidMetadataAtomically)
{
    msg::CollisionMap message;
    message.valid = true;
    message.clearance_radius = .26;
    message.observation_uncertainty = .03;
    message.grid.header.frame_id = "odom";
    message.grid.info.width = message.grid.info.height = 10;
    message.grid.info.resolution = .1;
    message.grid.info.origin.orientation.w = 1.;
    message.grid.data.assign(100, 0);
    geometry_msgs::msg::Point point;
    point.x = .29;
    message.points.push_back(point);
    std::vector<mini_nav_core::PathPoint> points;
    auto grid = DecodeCollisionMap(message, "odom", points);
    ASSERT_EQ(points.size(), 1u);
    EXPECT_EQ(grid->GetCost(2, 0), 0);
    message.grid.data[2] = 99;
    EXPECT_THROW(DecodeCollisionMap(message, "odom", points), std::invalid_argument);
    EXPECT_DOUBLE_EQ(points.front().x, .29);
    message.grid.data[2] = 0;
    message.points.front().x = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(DecodeCollisionMap(message, "odom", points), std::invalid_argument);
    message.points.front().x = .29;
    EXPECT_THROW(DecodeCollisionMap(message, "map", points), std::invalid_argument);
    message.points_frame_id = "unexpected_frame";
    EXPECT_THROW(DecodeCollisionMap(message, "odom", points), std::invalid_argument);
    message.points_frame_id = "odom";
    EXPECT_NO_THROW(DecodeCollisionMap(message, "odom", points));
    message.points_frame_id.clear();
    message.observation_uncertainty = -.01;
    EXPECT_THROW(DecodeCollisionMap(message, "odom", points), std::invalid_argument);
    message.observation_uncertainty = .03;
    message.valid = false;
    EXPECT_THROW(DecodeCollisionMap(message, "odom", points), std::invalid_argument);
}

TEST_F(CostmapPublisherNodeTest, StaticRouteStillSelectsAnExecutableTerminal) {
    auto node = std::make_shared<CostmapPublisherNode>(40, 30, .1, 0., 0., 0, 1000);
    SetScan(*node);
    SetStaticPolicy(*node);
    ASSERT_TRUE(node->PlanAndPublish({27, 42}, {36, 48}));
    const auto end = Endpoint(*node);
    EXPECT_TRUE(CurrentClear(*node, end));
    for (int i = 0; i < 16; ++i) {
        const double angle = i * 2. * std::acos(-1.) / 16;
        EXPECT_TRUE(CurrentClear(*node, {end.x + .12 * std::cos(angle),
            end.y + .12 * std::sin(angle)}));
    }
    unsigned int x, y;
    ASSERT_TRUE(node->GetCostmap().WorldToMap(end.x, end.y, x, y));
    EXPECT_TRUE(node->PlanAndPublish({x, y}, {27, 42}));
}

TEST_F(CostmapPublisherNodeTest, PublishedGeometryMatchesPlannerAndFailsClosedOnStaleScan) {
    auto node = std::make_shared<CostmapPublisherNode>(40, 30, .1, 0., 0., 0, 1000);
    auto probe = std::make_shared<rclcpp::Node>("collision_snapshot_probe");
    msg::CollisionMap::ConstSharedPtr snapshot;
    auto sub = probe->create_subscription<msg::CollisionMap>("/mini_nav/static_collision_map",
        rclcpp::QoS(1).reliable().transient_local(), [&](msg::CollisionMap::ConstSharedPtr m) { snapshot = m; });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(probe);
    SetScan(*node);
    ASSERT_TRUE(Rebuild(*node));
    for (int i = 0; i < 30 && !snapshot; ++i) {
        Publish(*node);
        executor.spin_once(std::chrono::milliseconds(20));
    }
    ASSERT_TRUE(snapshot);
    ASSERT_TRUE(snapshot->valid);
    EXPECT_EQ(snapshot->points.size(), 1u);
    EXPECT_DOUBLE_EQ(snapshot->observation_uncertainty, .03);
    std::vector<mini_nav_core::PathPoint> points;
    auto grid = DecodeCollisionMap(*snapshot, "map", points);
    mini_nav_core::CollisionGeometry controller{*grid, points, snapshot->clearance_radius,
        snapshot->observation_uncertainty};
    const mini_nav_core::PathPoint nearby{.859 - .30, .345};
    EXPECT_EQ(controller.IsClear(nearby, nearby), CurrentClear(*node, nearby));
    SetScan(*node, 1.0);
    snapshot.reset();
    for (int i = 0; i < 30 && (!snapshot || snapshot->valid); ++i) {
        Publish(*node);
        executor.spin_once(std::chrono::milliseconds(20));
    }
    ASSERT_TRUE(snapshot);
    EXPECT_FALSE(snapshot->valid);
}


TEST_F(CostmapPublisherNodeTest, RelativeLaserDoesNotAcquireAbsoluteMapLocalizationError) {
    CostmapPublisherNode node(40, 30, .1, 0., 0., 0, 1000);
    SetRelativeScan(node);
    ASSERT_TRUE(Rebuild(node));
    EXPECT_TRUE(node.PlanAndPublish({30,22}, {24,32}));
    CheckRelativeFrame(node);
}

}

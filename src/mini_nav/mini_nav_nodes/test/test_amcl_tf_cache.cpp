/**
 * @file test_amcl_tf_cache.cpp
 * @brief 验证 amcl_tf_cache 模块行为及边界的测试。
 * @author Antinomy
 * @date 2026-10-01
 */
#include <gtest/gtest.h>

#include "mini_nav_nodes/amcl/amcl_node.hpp"

namespace mini_nav_nodes
{

class AmclNodeTfCacheTest : public ::testing::Test
{
protected:
  /**
   * @brief 在测试套件启动前初始化 ROS context。
   */
  static void SetUpTestSuite()
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  /**
   * @brief 在套件结束时关闭 ROS context。
   */
  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  /**
   * @brief 直接设置测试所需帧名和 TF 前推量。
   *
   * @param node 被测 AMCL 节点。
   */
  static void ConfigureNode(AmclNode & node)
  {
    node.global_frame_id_ = "map";
    node.odom_frame_id_ = "odom";
    node.transform_tolerance_ = 0.5;
  }

  /**
   * @brief 调用节点私有入口缓存真实二维 map→odom 几何。
   *
   * @param node 被测节点。
   * @param estimate map 系估计。
   * @param odom_pose 同一机器人 odom 位姿。
   */
  static void CacheTransform(
    AmclNode & node,
    const mini_nav_core::localization::PoseEstimate & estimate,
    const mini_nav_core::localization::Pose2D & odom_pose)
  {
    node.cacheMapToOdom(estimate, odom_pose);
  }

  /**
   * @brief 调用私有缓存消息生成接口。
   *
   * @param node 被测节点。
   * @param scan_stamp 模拟扫描时间。
   * @param message 输出 TF 消息。
   * @return 缓存有效性。
   */
  static bool MakeCachedTransform(
    const AmclNode & node,
    const rclcpp::Time & scan_stamp,
    geometry_msgs::msg::TransformStamped & message)
  {
    return node.makeCachedMapToOdomTransform(scan_stamp, message);
  }

  /**
   * @brief 调用真实缓存失效逻辑。
   *
   * @param node 被测节点。
   */
  static void InvalidateCache(AmclNode & node)
  {
    node.invalidateMapToOdom();
  }

  /**
   * @brief 设置测试定位质量与模拟扫描年龄。
   *
   * @param node 被测节点。
   * @param mass 主簇权重占比。
   * @param scan_age 扫描相对当前 ROS 时间的年龄，秒。
   */
  static void SetQuality(AmclNode & node, double mass, double scan_age = 0.0)
  {
    node.active_ = true;
    node.initial_pose_known_ = true;
    node.quality_estimate_.valid = true;
    node.quality_estimate_.hypothesis_mass = mass;
    node.quality_estimate_.covariance = mini_nav_core::localization::Covariance3::Diagonal(0.01, 0.01, 0.01);
    node.quality_scan_received_ = std::chrono::steady_clock::now();
    node.quality_scan_stamp_ = node.now() - rclcpp::Duration::from_seconds(scan_age);
  }
  /**
   * @brief 查询节点实际定位质量判定。
   *
   * @param node 被测节点。
   * @return 是否满足运动许可前提。
   */
  static bool QualityValid(const AmclNode & node) { return node.localizationQualityValid(); }
  /**
   * @brief 模拟节点不活跃状态，验证定位许可撤销。
   *
   * @param node 被测节点。
   */
  static void Deactivate(AmclNode & node) { node.active_ = false; }

  /**
   * @brief 读取 TF 缓存有效标志。
   *
   * @param node 被测节点。
   * @return 缓存是否可重发。
   */
  static bool IsCacheValid(const AmclNode & node)
  {
    return node.map_to_odom_valid_;
  }
};

/**
 * @brief 验证首次有效估计前不生成 map→odom。
 */
TEST_F(AmclNodeTfCacheTest, DoesNotCreateTransformBeforeFirstEstimate)
{
  AmclNode node;
  geometry_msgs::msg::TransformStamped message;

  EXPECT_FALSE(MakeCachedTransform(node, rclcpp::Time(10, 0, RCL_ROS_TIME), message));
}

/**
 * @brief 验证缓存重发只刷新时间戳而不改变几何。
 */
TEST_F(AmclNodeTfCacheTest, RestampsCachedTransformWithoutChangingValue)
{
  AmclNode node;
  ConfigureNode(node);

  mini_nav_core::localization::PoseEstimate estimate;
  estimate.valid = true;
  estimate.pose = {2.0, 3.0, 0.3};
  const mini_nav_core::localization::Pose2D odom_pose{1.0, 1.0, 0.1};
  CacheTransform(node, estimate, odom_pose);

  geometry_msgs::msg::TransformStamped first_message;
  geometry_msgs::msg::TransformStamped second_message;
  ASSERT_TRUE(MakeCachedTransform(
      node, rclcpp::Time(10, 0, RCL_ROS_TIME), first_message));
  ASSERT_TRUE(MakeCachedTransform(
      node, rclcpp::Time(11, 0, RCL_ROS_TIME), second_message));

  EXPECT_EQ(first_message.header.frame_id, "map");
  EXPECT_EQ(first_message.child_frame_id, "odom");
  EXPECT_EQ(first_message.header.stamp.sec, 10);
  EXPECT_EQ(first_message.header.stamp.nanosec, 500000000U);
  EXPECT_EQ(second_message.header.stamp.sec, 11);
  EXPECT_EQ(second_message.header.stamp.nanosec, 500000000U);

  EXPECT_DOUBLE_EQ(
    first_message.transform.translation.x, second_message.transform.translation.x);
  EXPECT_DOUBLE_EQ(
    first_message.transform.translation.y, second_message.transform.translation.y);
  EXPECT_DOUBLE_EQ(
    first_message.transform.translation.z, second_message.transform.translation.z);
  EXPECT_DOUBLE_EQ(
    first_message.transform.rotation.x, second_message.transform.rotation.x);
  EXPECT_DOUBLE_EQ(
    first_message.transform.rotation.y, second_message.transform.rotation.y);
  EXPECT_DOUBLE_EQ(
    first_message.transform.rotation.z, second_message.transform.rotation.z);
  EXPECT_DOUBLE_EQ(
    first_message.transform.rotation.w, second_message.transform.rotation.w);
}

/**
 * @brief 验证缓存失效后旧定位 TF 不可重发。
 */
TEST_F(AmclNodeTfCacheTest, InvalidationPreventsReuseOfOldTransform)
{
  AmclNode node;
  ConfigureNode(node);

  mini_nav_core::localization::PoseEstimate estimate;
  estimate.valid = true;
  CacheTransform(node, estimate, mini_nav_core::localization::Pose2D{});
  ASSERT_TRUE(IsCacheValid(node));

  InvalidateCache(node);

  geometry_msgs::msg::TransformStamped message;
  EXPECT_FALSE(IsCacheValid(node));
  EXPECT_FALSE(MakeCachedTransform(
      node, rclcpp::Time(12, 0, RCL_ROS_TIME), message));
}


/**
 * @brief 验证新鲜 TF 不掩盖扫描过期或主簇质量不足。
 */
TEST_F(AmclNodeTfCacheTest, FreshTransformCannotHideStaleScanOrAmbiguousHypothesis)
{
    AmclNode node;
    ConfigureNode(node);
    mini_nav_core::localization::PoseEstimate estimate;
    estimate.valid = true;
    CacheTransform(node, estimate, {});
    SetQuality(node, 0.9);
    EXPECT_TRUE(QualityValid(node));
    SetQuality(node, 0.5);
    EXPECT_FALSE(QualityValid(node));
    SetQuality(node, 0.9, 1.0);
    EXPECT_FALSE(QualityValid(node));
    geometry_msgs::msg::TransformStamped tf;
    EXPECT_TRUE(MakeCachedTransform(node, node.now(), tf));
}

/**
 * @brief 验证重定位与去激活撤销定位有效性。
 */
TEST_F(AmclNodeTfCacheTest, ResetAndDeactivationRevokeLocalizationQuality)
{
    AmclNode node;
    ConfigureNode(node);
    mini_nav_core::localization::PoseEstimate estimate;
    estimate.valid = true;
    CacheTransform(node, estimate, {});
    SetQuality(node, 0.9);
    EXPECT_TRUE(QualityValid(node));
    InvalidateCache(node);
    EXPECT_FALSE(QualityValid(node));
    CacheTransform(node, estimate, {});
    SetQuality(node, 0.9);
    Deactivate(node);
    EXPECT_FALSE(QualityValid(node));
}

}  // namespace mini_nav_nodes

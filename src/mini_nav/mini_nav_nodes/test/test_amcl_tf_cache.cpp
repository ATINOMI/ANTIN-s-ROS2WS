#include <gtest/gtest.h>

#include "mini_nav_nodes/amcl_node.hpp"

namespace mini_nav_nodes
{

class AmclNodeTfCacheTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  static void ConfigureNode(AmclNode & node)
  {
    node.global_frame_id_ = "map";
    node.odom_frame_id_ = "odom";
    node.transform_tolerance_ = 0.5;
  }

  static void CacheTransform(
    AmclNode & node,
    const mini_nav_core::localization::PoseEstimate & estimate,
    const mini_nav_core::localization::Pose2D & odom_pose)
  {
    node.cacheMapToOdom(estimate, odom_pose);
  }

  static bool MakeCachedTransform(
    const AmclNode & node,
    const rclcpp::Time & scan_stamp,
    geometry_msgs::msg::TransformStamped & message)
  {
    return node.makeCachedMapToOdomTransform(scan_stamp, message);
  }

  static void InvalidateCache(AmclNode & node)
  {
    node.invalidateMapToOdom();
  }

  static bool IsCacheValid(const AmclNode & node)
  {
    return node.map_to_odom_valid_;
  }
};

TEST_F(AmclNodeTfCacheTest, DoesNotCreateTransformBeforeFirstEstimate)
{
  AmclNode node;
  geometry_msgs::msg::TransformStamped message;

  EXPECT_FALSE(MakeCachedTransform(node, rclcpp::Time(10, 0, RCL_ROS_TIME), message));
}

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

}  // namespace mini_nav_nodes
